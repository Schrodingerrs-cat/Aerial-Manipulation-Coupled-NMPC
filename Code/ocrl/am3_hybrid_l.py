"""NMPC + learned model correction "L" (proposal: RL outputs a correction to the NMPC prediction model; NMPC still computes every command).  2026-11-19.
Base controller: NDF (ND + state filter, observer on the filtered state; ocrl/am3_nmpc_filt.py).  Each 20 ms cycle a policy outputs a in [-1, 1]^n:
  n = 2 ("L2", the proposal's channels): d = a * D_MAX -> N's model correction (normal force, pitch torque), the same slot the frozen N reserves for D/L.
  n = 9 ("L9", stated deviation): a * S9 is added to the observer estimate (base force, body moment, joint torques) in the prediction.
The correction enters only the NMPC prediction over the horizon (never the plant, never the motor commands); a = 0 reproduces NDF exactly.
Observations are measured quantities and controller internals only (no plant parameters)."""
import time, faulthandler, numpy as np, gymnasium as gym
from gymnasium import spaces
from ocrl.am3_nmpc_filt import NMPCDF
from ocrl.am3_nmpc_dob import K_DOB
from ocrl.am3_ctrl_n import NMPC
from ocrl.am3_nmpc import D_MAX
from ocrl.am3_write import WriteEpisode, WriteTask, text_strokes, FONT
from ocrl.am3_rl_wall import PHASES, PH_OBS, sample_mismatch

STEP_LIMIT_S = 120.0          # a step normally takes ~45 ms; a step that never returns (L9 froze twice inside one step, 2026-11-19/21) dumps its stack and kills the worker so the job resumes
S9 = np.array([1.5, 1.5, 1.5, 0.15, 0.15, 0.15, 0.3, 0.3, 0.3])          # 9-channel correction scale: N, N m (about a 150 g payload, its moment, joint torque)
S_DG = np.array([2.0, 2.0, 2.0, 0.1, 0.1, 0.1, 0.3, 0.3, 0.3])
HIST = 4
NF1 = 3 + 3 + 3 + 3 + 3 + 3 + 3 + 4 + 4 + 3 + 3 + len(PHASES) + 9          # one frame without the previous action


def k_ndf():
    k = K_DOB.copy(); k[6:9] = 10.0; return k                                # as NDF:0.5,f (ND:10,10 gains)


class NMPCDFL(NMPCDF):
    period = 0.02

    def __init__(self, nch=2, policy=None, alpha=0.5):
        super().__init__(alpha, k_ndf(), obs_on_filtered=True); self.name = f"L{nch}"; self.nch, self.policy = nch, policy; self.a = np.zeros(nch); self.ext_action = np.zeros(nch)

    def reset(self, x0, u0, task):
        super().reset(x0, u0, task); self.a = np.zeros(self.nch); self.ext_action = np.zeros(self.nch); self.H = None; self.c9 = np.zeros(9)

    def params(self, task):
        P = NMPC.params(self, task); return np.r_[P, np.tile((self.dg + self.c9)[:, None], (1, P.shape[1]))]

    def make_obs(self, xm, Fm, task):
        """Called once per cycle, before compute.  Pad/base errors from the measured state, the lag signature (measured rotor speed minus the last
        applied rotor command), observer estimate, phase, previous correction; 4 frames, newest first."""
        pr = self.pr; ref = task.ref(); pB = np.asarray(ref.get("pB", np.zeros(3)), float); vB = np.asarray(ref.get("vB", np.zeros(3)), float)
        pT = np.array(pr.pad_f(xm)[0]).ravel(); vT = (np.array(pr.JC_f(xm)[1]) @ np.r_[xm[7:10], xm[10:13], xm[16:19]]).ravel(); Q = xm[3:7]
        ph = np.zeros(len(PHASES)); ph[PHASES.index(PH_OBS.get(task.phase, task.phase))] = 1.0
        f = np.r_[(pT - ref["p"]) / 0.003, (vT - ref["v"]) / 0.02, (xm[0:3] - pB) / 0.005, (xm[7:10] - vB) / 0.05, 2 * Q[1:4] * np.sign(Q[0]) / 0.1, xm[10:13] / 0.5, xm[16:19] / 1.0,
                  (xm[19:23] - self.u_act[0:4]) / 100.0, (self.u_act[0:4] - 1050.0) / 300.0, self.u_act[4:7] / 1.0, [Fm / 2.0, float(task.contact), task.gap_ref / 0.04], ph, self.dg / S_DG]
        f = np.clip(np.nan_to_num(np.r_[f, self.a]), -10, 10).astype(np.float32)
        self.H = [f] * HIST if self.H is None else [f] + self.H[:-1]; return np.concatenate(self.H)

    def compute(self, t, xm, Fm, task):
        if self.policy is not None: a = self.policy.predict(self.make_obs(xm, Fm, task), deterministic=True)[0]
        else: a = self.ext_action
        self.a = np.clip(np.asarray(a, float), -1, 1)
        if self.nch == 2: self.d = self.a * D_MAX
        else: self.c9 = self.a * S9
        u, info = super().compute(t, xm, Fm, task); info["corr"] = self.a.copy(); return u, info


class WriteStepEpisode(WriteEpisode):
    """Episode.run (ocrl/am3_episode.py) split into observe / commit so a learner acts between them; same latency, failsafe and end condition."""
    def start(self):
        self.k = 0; self.dt_c = self.ctrl.period; self.n_sub = int(round(self.dt_c / self.pl.dt)); self.u_pend = self.u0.copy(); self.u_app = self.u0.copy(); self.stop = False; return self.observe()

    def observe(self):
        pl, task = self.pl, self.task; self.x = x = pl.state()
        if not np.isfinite(x).all(): self.stop = True; self.abort = "nan"; return None, None
        self.F, _ = pl.pad_force(); self.xm, self.Fm = self.sens.measure(x, self.F, pl.d.time)
        if self.k: task.advance(self.dt_c, self.Fm, self.gap_meas(self.xm))
        tilt = np.degrees(2 * np.arcsin(min(1.0, np.hypot(self.xm[4], self.xm[5])))); pref = np.asarray(task.ref().get("pB"), float)
        if task.failed or tilt > 40 or np.linalg.norm(self.xm[0:3] - pref) > 0.25 or pl.d.time > self.T_max: self.stop = True; self.abort = task.failed or "failsafe"
        return self.xm, self.Fm

    def commit(self):
        pl = self.pl; u_new, info = self.ctrl.compute(pl.d.time, self.xm, self.Fm, self.task); u_new = np.asarray(u_new, float)
        self.u_app, self.u_pend = self.u_pend, u_new
        for _ in range(self.n_sub): pl.step(self.u_app)
        self.k += 1; return info


class HybridEnv(gym.Env):
    """One NMPC cycle (20 ms) per step.  Episodes: 1-3 random capital letters, letter height 4-6 cm, pen speed 40-60 mm/s, sensor noise (see noise),
    unknown mismatch from sample_mismatch (payload <= 250 g, motor lag 1-2.5x, thrust, damping, wind, gusts).
    Curriculum: the motor-lag upper bound starts at lag0 = 1.4 (NDF still writes; it fails from 1.5) and reaches lag_max over the first `ramp` episodes of each env.
    wall=True (2026-11-20): also randomize wall stiffness (0.4-3.5x, log-uniform) and friction (0.4-2.5x); the soft wall broke N and NDF (12-15 mm) in the stress bench.
    noise: sensor level, 'real' (stress, L2/L9) or 'lab' (spec-based, Code/scripts/am3_write_bench.py; 2026-11-20).
    An episode slower than the bench's 400 s wall-clock guard is truncated: one stuck episode froze all 30 envs of L9 for 22 h (2026-11-19)."""
    metadata = {}

    def __init__(self, nch=2, seed=0, lag_max=2.5, pl_max=0.25, lag0=1.4, ramp=300, wall=False, noise="real"):
        super().__init__(); self.nch, self.rng, self.lag_max, self.pl_max, self.lag0, self.ramp, self.n_ep, self.wall, self.noise = nch, np.random.default_rng(seed), lag_max, pl_max, lag0, ramp, 0, wall, noise; self.ctrl = NMPCDFL(nch)
        self.observation_space = spaces.Box(-10, 10, ((NF1 + nch) * HIST,), np.float32); self.action_space = spaces.Box(-1, 1, (nch,), np.float32)

    def reset(self, seed=None, options=None):
        faulthandler.dump_traceback_later(STEP_LIMIT_S, exit=True)
        out = self._reset(seed)
        faulthandler.cancel_dump_traceback_later(); return out

    def step(self, a):
        faulthandler.dump_traceback_later(STEP_LIMIT_S, exit=True)
        out = self._step(a)
        faulthandler.cancel_dump_traceback_later(); return out

    def _reset(self, seed=None):
        from am3_write_bench import RealSensors, LabSensors
        if seed is not None: self.rng = np.random.default_rng(seed)
        g = self.rng; text = "".join(g.choice(sorted(FONT), size=int(g.integers(1, 4)))); h = float(g.uniform(0.04, 0.06))
        mm = sample_mismatch(g, pl_max=self.pl_max, lag_max=self.lag0 + (self.lag_max - self.lag0) * min(1.0, self.n_ep / max(self.ramp, 1)))
        if self.wall: mm.update(k_scale=float(np.exp(g.uniform(np.log(0.4), np.log(3.5)))), mu_scale=float(g.uniform(0.4, 2.5)))
        ep = WriteStepEpisode(self.ctrl, text, height=h, v_write=float(g.uniform(0.04, 0.06)), seed=int(g.integers(0, 2 ** 30)), mismatch=mm); self.t_wall0 = time.time()
        self.n_ep += 1
        ep.sens = (LabSensors if self.noise == "lab" else RealSensors)(int(g.integers(0, 2 ** 30))); self.ep = ep; self.n = 0; self.a_prev = np.zeros(self.nch); self.nrec = 0
        self.max_steps = int((sum(ep.task.Ts) + 4.0 * len(ep.task.Ts) + 10.0) / self.ctrl.period); xm, Fm = ep.start()
        return self.ctrl.make_obs(xm, Fm, ep.task), dict(text=text)

    def _step(self, a):
        ep, c = self.ep, self.ctrl; a = np.clip(np.asarray(a, float), -1, 1); c.ext_action = a; ep.commit(); self.n += 1; xm, Fm = ep.observe(); task = ep.task
        x = ep.x; ref = task.ref(); pad = np.array(c.pr.pad_f(x)[0]).ravel(); e_pad = float(np.linalg.norm(pad - ref["p"])); e_base = float(np.linalg.norm(x[0:3] - np.asarray(ref["pB"], float)))
        r = 0.5 - 0.3 * min(e_pad / 0.003, 3.0) - 0.1 * min(e_base / 0.01, 3.0) - 0.02 * float(a @ a) - 0.1 * float((a - self.a_prev) @ (a - self.a_prev)) - 0.5 * max(ep.F - 2.0, 0.0)
        if task.phase == "trace": r += np.exp(-(e_pad / 0.001) ** 2) + (0.5 if ep.F > 0.08 else -0.5)
        if len(task.recoveries) > self.nrec: r -= 5.0 * (len(task.recoveries) - self.nrec); self.nrec = len(task.recoveries)
        self.a_prev = a; done = task.phase == "done"; fail = ep.stop and not done
        if fail: r -= 100.0
        if done: r += 100.0
        obs = c.make_obs(xm, Fm, task) if xm is not None else np.zeros(self.observation_space.shape, np.float32)
        slow = time.time() - self.t_wall0 > ep.wall_limit
        return obs, float(r), bool(fail or done), bool((self.n >= self.max_steps or slow) and not (fail or done)), dict(completed=bool(done), phase=task.phase, wall_timeout=bool(slow))
