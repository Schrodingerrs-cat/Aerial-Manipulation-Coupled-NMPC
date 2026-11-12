"""R10: the wall task for the end-to-end policy.  Same plant with the wall on, same start (hover equilibrium, pad 5-8 cm from the wall), same Task state machine and contact logic as the NMPC harness (ocrl/am3_task, am3_episode).
The base holds its fixed hover reference; the joint reference is the IK of the Task's pad reference, plus the policy's residual.  Limits mirror the Episode failsafe."""
import numpy as np
from gymnasium import spaces
from ocrl.am3_rl_direct import Track9Env, BASE, Q_NOM, Sensors
from ocrl.am3_plant import WALL_X, R_PAD
from ocrl.am3_task import Task, TASK
from ocrl.am3_episode import initial_state

PHASES = ["hover", "position", "approach", "settle", "trace", "endhold", "release", "retreat", "settle_resume", "recover", "done"]
CONTACT = ("settle", "settle_resume", "trace", "endhold")
PH_OBS = dict(lift="retreat", travel="position")              # writing (am3_write) pen-up phases reuse the closest existing one-hot slot, so the observation size is unchanged
HOLD = dict(T=30.0, Tr=2.5, laps=1, Ax=0.0, Ay=0.0, Az=0.0, nx=1, ny=1, nz=1, px=0.0, py=0.0, pz=0.0)
# Extra wall shape (2026-11-10, user request, outside the frozen task file): letter M as one stroke, same 8 x 6 cm patch as the frozen shapes.
TASK["paths"].setdefault("M", dict(kind="polygon", vertices=[[-0.04, -0.025], [-0.03, 0.03], [0.0, -0.005], [0.03, 0.03], [0.04, -0.025]], fillet_m=0.006, T_s=12.0))
SHAPES = ("line", "line_fast", "line_T1", "circle", "circle_T4", "rounded_triangle", "rounded_triangle_T5", "M")


def sample_mismatch(g):
    """Unknown model mismatch for training (R13, H1): payload on the hand, thrust and motor-lag error, joint damping, wind and gusts."""
    dist = []
    if g.random() < 0.6: dist += [dict(kind="sine", axis=ax, amp=float(g.uniform(0, 1.0)), freq=float(g.uniform(0.1, 1.0)), t0=float(g.uniform(0, 5))) for ax in (1, 2)]
    if g.random() < 0.4: dist.append(dict(kind="step", axis=int(g.integers(0, 3)), amp=float(g.uniform(-1.2, 1.2)), t0=float(g.uniform(2, 15)), dur=float(g.uniform(0.5, 3))))
    return dict(payload=dict(kg=float(g.uniform(0, 0.25)) if g.random() < 0.6 else 1e-9, pos_hand_m=[0.0, 0.0, 0.0]), kf_scale=float(g.uniform(0.88, 1.08)), motor_tau_scale=float(g.uniform(1.0, 2.0)),
                joint_damping_scale=float(np.exp(g.uniform(np.log(0.5), np.log(4.0)))), disturbance=dist)


def scale_mismatch(mm, s):
    """Curriculum (2026-11-11): s = 0 is the nominal model, s = 1 the full sample from sample_mismatch."""
    return dict(payload=dict(kg=max(1e-9, s * mm["payload"]["kg"]), pos_hand_m=mm["payload"]["pos_hand_m"]), kf_scale=1 + s * (mm["kf_scale"] - 1), motor_tau_scale=1 + s * (mm["motor_tau_scale"] - 1),
                joint_damping_scale=float(mm["joint_damping_scale"] ** s), disturbance=[dict(d, amp=s * d["amp"]) for d in mm["disturbance"]])


class WallEnv(Track9Env):
    def __init__(self, *args, paths=("line", "line_fast", "line_T1"), arm_res=0.25, dt_p=0.0005, pad_w=1.0, contact_w=1.0, done_bonus=100.0, vel_ff=False, write_p=0.0, dr=False, dr_ramp=0, hist=1, dob=False, real_sens=False, pad_fine_w=0.0, **kw):
        super().__init__(*args, ref=HOLD, arm_res=arm_res, dt_p=dt_p, **kw)
        self.hist, self.H, self.pad_fine_w = hist, None, pad_fine_w; self.inner.dob = dob          # R15 (Phase 1): observation history, disturbance observer, fine pad reward
        if real_sens: self.sens = Sensors(kw.get("seed", 0), Sensors.REAL)
        self.pl.m.geom_contype[self.pl.gwall] = 1; self.pl.m.geom_conaffinity[self.pl.gwall] = 1; self.latency = True
        self.inner.kp_j, self.inner.kd_j = 1.0, 0.05          # the track-rung joint gains (1.57, 0.016) go unstable with the harness latency: arm limit cycle at 12 rad/s; chosen from a sweep, see Findings/RL/03
        self.paths, self.pad_w, self.contact_w, self.done_bonus = paths, pad_w, contact_w, done_bonus; self.drive_task = True; self.Fm = 0.0; self.scenario = None; self.vel_ff = vel_ff; self.write_p = write_p; self.dr = dr; self.dr_ramp, self.n_dr = dr_ramp, 0; self.damp0 = self.pl.m.dof_damping.copy()
        self.observation_space = spaces.Box(-np.inf, np.inf, ((self.observation_space.shape[0] + 6 + len(PHASES) + 3) * hist,), np.float32)

    def _obs(self, xm):
        o = super()._obs(xm)
        if self.hist == 1: return o
        self.H = [o] * self.hist if self.H is None else [o] + self.H[:-1]                       # newest first, so a widened warm start keeps the old input in place
        return np.concatenate(self.H)

    def _state0(self):
        s = self.scenario
        gap0, dyz = (s["gap0"], s["dyz"]) if s else (float(self.rng.uniform(0.05, 0.08)), self.rng.uniform(-0.008, 0.008, 2))
        return initial_state(gap0, (dyz[0], 1.04 + dyz[1]))[0]

    def _new_episode(self):
        self.H = None; self.p = dict(HOLD); self.ei = np.zeros(3); self.max_steps = int(25.0 / self.dt_c); self.path = self.scenario["path"] if self.scenario else str(self.rng.choice(self.paths))
        p0 = np.array(self.inner.pr.pad_f(self.x0)[0]).ravel()
        if self.scenario is None and self.rng.random() < self.write_p:            # writing episode: 1-2 random capital letters, random size, moving base
            from ocrl.am3_write import WriteTask, text_strokes, FONT
            text = "".join(self.rng.choice(sorted(FONT), size=1 if self.rng.random() < 0.7 else 2)); h = float(self.rng.uniform(0.03, 0.10))
            self.task = WriteTask(p0, text_strokes(text, h), h, v_write=float(self.rng.uniform(0.025, 0.075))); self.path = "write:" + text          # pen speed 25-75 mm/s
            self.max_steps = int((sum(self.task.Ts) + 4.0 * len(self.task.Ts) + 6.0) / self.dt_c)
        else: self.task = Task(self.path, p0)
        if self.dr: self.domain_rand()
        self.qref = np.array(self.x0[13:16]); self.qdref = np.zeros(3); self.Fm = 0.0; self.q0 = self.qref.copy()

    def domain_rand(self):
        """Domain randomization (R13); every episode starts from the nominal model."""
        self.pl.m.dof_damping[:] = self.damp0; self.mm = sample_mismatch(self.rng); self.n_dr += 1
        if self.dr_ramp: self.mm = scale_mismatch(self.mm, min(1.0, self.n_dr / self.dr_ramp))          # ramp over the first dr_ramp episodes of this env
        x = self.pl.state(); self.pl.apply_mismatch(self.mm); self.pl.set_state(x)          # apply_mismatch -> mj_setConst overwrites the simulation state

    def _set_ref(self, t):
        pr = self.inner.pr
        if self.drive_task and self.n > 0:
            self.Fm = float(self.pl.pad_force()[0] + 0.01 * self.rng.standard_normal()); self.task.advance(self.dt_c, self.Fm, WALL_X - R_PAD - float(self.pl.pad_pos()[0]))
        r = self.task.ref(); self.pref, self.vref, self.aref = np.array(r.get("pB", BASE), float), np.array(r.get("vB", np.zeros(3)), float), np.zeros(3); self.rp, self.rv = r["p"], r["v"]          # moving base (writing) or the fixed hover point
        for _ in range(8): self.qref = self.qref + np.linalg.solve(np.array(pr.Jpad_f(self.qref)), self.rp - self.pref - np.array(pr.fk_f(self.qref)).ravel())
        self.qdref = np.linalg.solve(np.array(pr.Jpad_f(self.qref)), self.rv - self.vref)
        if self.vel_ff: self.inner.qd_ref = self.qdref        # R10c: without it the joint D-term makes the pad lag the reference by kd/kp = 0.05 s

    def _extra_obs(self):
        ph = np.zeros(len(PHASES)); ph[PHASES.index(PH_OBS.get(self.task.phase, self.task.phase))] = 1.0; pad = np.array(self.inner.pr.pad_f(self.pl.state())[0]).ravel()
        return np.concatenate([super()._extra_obs(), np.clip((self.rp - pad) / 0.02, -5, 5), self.rv / 0.1, ph, [self.task.gap_ref / 0.04, np.clip(self.Fm / 2.0, -2, 3), float(self.task.contact)]])

    def step(self, a):
        obs, r, bad, trunc, info = super().step(a)
        x = self.pl.state(); pad = np.array(self.inner.pr.pad_f(x)[0]).ravel(); ep = float(np.linalg.norm(pad - self.rp)); F = self.Fm; ph = self.task.phase
        r -= self.pad_w * min(ep, 0.02) / 0.002
        if ph == "trace": r += self.pad_fine_w * np.exp(-(ep / 0.001) ** 2)              # R15: sharp term at the mm scale, only while writing
        if ph in CONTACT: r += self.contact_w * (1.0 if F > 0.08 else -1.0)
        r -= 0.5 * max(F - 2.0, 0.0)
        tilt = np.degrees(2 * np.arcsin(min(1.0, np.hypot(x[4], x[5])))); stop = tilt > 40 or np.linalg.norm(x[:3] - self.pref) > 0.25 or F > 4.0 or bool(self.task.failed) or len(self.pl.other_contacts()) > 0
        if stop and not bad: r -= self.bad_pen; bad = True
        done = ph == "done" and not bad
        if done: r += self.done_bonus
        info.update(pad_err=ep, F=F, phase=ph, completed=done)
        return obs, float(r), bool(bad), bool((trunc or done) and not bad), info


KIND_KW = dict(write15=dict(hist=4, dob=True, real_sens=True, pad_fine_w=1.0))         # env options a policy was trained with, by training env kind (Phase 1)


class RLCtrl:
    """The trained policy as an Episode controller, so it is scored by the same harness as N and B.  attach(ep) gives it the episode plant (read-only) and task."""
    period = 0.01; name = "RL"; task_uses_state = True

    def __init__(self, policy, vel_ff=False, **kw): self.env = WallEnv(seed=0, vel_ff=vel_ff, **kw); self.env.drive_task = False; self.pol = policy

    def attach(self, ep): self.env.pl = ep.pl

    def reset(self, x0, u0, task):
        e = self.env; e.x0 = np.array(x0); e.task = task; e.a_prev = np.zeros(10); e.ei = np.zeros(3); e.p = dict(HOLD); e.qref = np.array(x0[13:16]); e.Fm = 0.0; e.n = 0; e.inner.dob_state = None; e.H = None

    def compute(self, t, xm, Fm, task):
        e = self.env; e.n = int(round(t / self.period)); e.Fm = float(Fm); e._set_ref(t); obs = e._obs(xm); a = self.pol.predict(obs, deterministic=True)[0] if self.pol != "pd" else e.expert_action(xm)
        a = np.clip(np.asarray(a, float), -1, 1); a_d, w_d, psi, q_d = e._decode(a); e.a_prev = a.copy(); return e.inner(xm, a_d, w_d, psi, q_d), dict(ok=True)
