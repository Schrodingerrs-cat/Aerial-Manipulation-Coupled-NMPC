"""NMPC + RL (H1, 2026-11-10): the coupled NMPC computes the command, a policy adds a bounded residual.  u = u_N + [60 rad/s * a(4) rotor speeds, 0.25 tau_c * a(3) joint torques].
Zero action = plain NMPC, and PPO starts with near-zero mean actions, so training starts from the NMPC.  Purpose: cover what the NMPC's model gets wrong
(unknown payload, thrust and motor-lag error, damping, wind), measured in scripts/am3_write_mismatch.py.
HybridCtrl is an Episode controller (scored by the same harness as N, B and RL); HybridEnv steps the same loop as Episode.run (sensors, failsafe, one-period
latency, 50 Hz) with writing / shape tasks and domain randomization as in am3_rl_wall."""
import numpy as np, gymnasium as gym
from gymnasium import spaces
from ocrl.am3_ctrl_n import NMPC
from ocrl.am3_episode import Episode, initial_state, BASE
from ocrl.am3_write import WriteEpisode, FONT
from ocrl.am3_rl_wall import PHASES, PH_OBS, CONTACT, SHAPES, sample_mismatch
from ocrl.am3_pred import AMPred3
from ocrl.am3_pred import quat2R

PR = AMPred3(); D_OM, D_TAU = 60.0, 0.25 * float(np.max(PR.tau_c)); OM_H = 1100.0
NOBS = 3 + 3 + 3 + 3 + 6 + 3 + 3 + 3 + 4 + 3 + 7 + len(PHASES) + 3


class HybridCtrl:
    """NMPC + residual policy.  policy=None: the residual comes from set_action (training)."""
    period = 0.02; name = "H"; task_uses_state = True

    def __init__(self, policy=None, nmpc=None):
        self.n = nmpc or NMPC(); self.pol = policy; self.a = np.zeros(7); self.uN = None

    def reset(self, x0, u0, task): self.n.reset(x0, u0, task); self.a = np.zeros(7); self.q0 = np.array(x0[13:16])

    def held(self, u): self.n.held(u)

    def obs(self, xm, Fm, task):
        r = task.ref(); pad = np.array(PR.pad_f(xm)[0]).ravel(); R = np.array(quat2R(xm[3:7])); pB, vB = np.asarray(r.get("pB", BASE), float), np.asarray(r.get("vB", np.zeros(3)), float)
        ph = np.zeros(len(PHASES)); ph[PHASES.index(PH_OBS.get(r["phase"], r["phase"]))] = 1.0
        o = np.r_[(r["p"] - pad) / 0.02, r["v"] / 0.1, (pB - xm[0:3]) / 0.05, (xm[7:10] - vB) / 0.2, R[:, 0], R[:, 1], xm[10:13] / 4.0, xm[13:16] - self.q0, xm[16:19] / 2.0,
                  (self.uN[:4] - OM_H) / 200.0, self.uN[4:7] / D_TAU, self.a, ph, (r["gap"] or 0.0) / 0.04, np.clip(Fm / 2.0, -2, 3), float(task.contact)]
        return np.clip(np.nan_to_num(o), -10, 10).astype(np.float32)

    def residual(self, a): a = np.clip(np.asarray(a, float), -1, 1); self.a = a; return np.r_[D_OM * a[:4], D_TAU * a[4:7]]

    def compute(self, t, xm, Fm, task):
        uN, info = self.n.compute(t, xm, Fm, task); self.uN = np.asarray(uN, float)
        if self.pol is None: return self.uN, info                               # env applies the residual itself
        a = self.pol.predict(self.obs(xm, Fm, task), deterministic=True)[0]; return self.uN + self.residual(a), info


class HybridEnv(gym.Env):
    """One step = one 50 Hz control period of Episode.run, with u = u_N + residual(action)."""
    def __init__(self, seed=0, write_p=0.7, dr=True, bad_pen=100.0, done_bonus=100.0):
        self.rng = np.random.default_rng(seed); self.write_p, self.dr, self.bad_pen, self.done_bonus = write_p, dr, bad_pen, done_bonus
        self.ctrl = HybridCtrl(); self.observation_space = spaces.Box(-10, 10, (NOBS,), np.float32); self.action_space = spaces.Box(-1, 1, (7,), np.float32)

    @property
    def pl(self): return self.ep.pl

    def expert_action(self, x): return np.zeros(7)                              # reference controller = plain NMPC (zero residual)

    def reset(self, seed=None, options=None):
        g = self.rng; mm = sample_mismatch(g) if self.dr else None; x0 = initial_state(float(g.uniform(0.05, 0.08)), (float(g.uniform(-0.008, 0.008)), 1.04 + float(g.uniform(-0.008, 0.008))))
        if g.random() < self.write_p:
            text = "".join(g.choice(sorted(FONT), size=1 if g.random() < 0.7 else 2)); self.ep = WriteEpisode(self.ctrl, text, height=float(g.uniform(0.03, 0.10)), v_write=float(g.uniform(0.025, 0.075)), mismatch=mm, x0=x0, seed=int(g.integers(1 << 30)))
        else: self.ep = Episode(self.ctrl, str(g.choice(SHAPES)), mismatch=mm, x0=x0, seed=int(g.integers(1 << 30)))
        self.u_pend = self.ep.u0.copy(); self.u_app = self.ep.u0.copy(); self.k = 0; self.n_sub = int(round(self.ctrl.period / self.ep.pl.dt))
        return self._observe()[0], {}

    def _observe(self):
        """Measure, advance the task and run the failsafe, as at the top of Episode.run; then the NMPC command and the observation."""
        ep, pl, task = self.ep, self.ep.pl, self.ep.task; x = pl.state()
        if not np.isfinite(x).all(): return np.zeros(NOBS, np.float32), "numerical instability"
        F, _ = pl.pad_force(); xm, Fm = ep.sens.measure(x, F, pl.d.time); self.xm, self.Fm = xm, Fm
        if self.k: task.advance(self.ctrl.period, Fm, ep.gap_meas(xm))
        if task.failed: return np.zeros(NOBS, np.float32), task.failed
        tilt = np.degrees(2 * np.arcsin(min(1.0, np.hypot(xm[4], xm[5])))); pB = np.asarray(task.ref().get("pB", BASE), float)
        if tilt > 40 or np.linalg.norm(xm[0:3] - pB) > 0.25: return np.zeros(NOBS, np.float32), "loss of control"
        self.ctrl.compute(pl.d.time, xm, Fm, task); return self.ctrl.obs(xm, Fm, task), None

    def step(self, a):
        ep, pl, task = self.ep, self.ep.pl, self.ep.task; u_new = self.ctrl.uN + self.ctrl.residual(a)
        self.u_app, self.u_pend = self.u_pend, u_new
        for _ in range(self.n_sub): pl.step(self.u_app)
        self.k += 1; obs, fail = self._observe()
        if fail: return obs, -self.bad_pen, True, False, dict(fail=fail)
        r = task.ref(); pad = np.array(PR.pad_f(self.xm)[0]).ravel(); e = float(np.linalg.norm(pad - r["p"])); eB = float(np.linalg.norm(self.xm[0:3] - np.asarray(r.get("pB", BASE))))
        rew = 2.0 - min(e, 0.02) / 0.002 - min(eB, 0.05) / 0.01 - 0.5 * max(self.Fm - 2.0, 0.0) - 0.1 * float(np.sum(np.square(a)))
        if r["phase"] in CONTACT: rew += 1.0 if self.Fm > 0.08 else -1.0
        if r["phase"] in ("lift", "travel", "position", "retreat") and self.Fm > 0.15: rew -= 1.0          # pen drag while it should be up
        done = task.phase == "done"; trunc = pl.d.time > ep.T_max
        return obs, float(rew + (self.done_bonus if done else 0.0)), done, bool(trunc and not done), dict(phase=task.phase, completed=done)
