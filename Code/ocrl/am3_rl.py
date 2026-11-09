"""Learned model correction L (Plan s8): observation schema, step-wise episode, gymnasium environment (one NMPC control cycle per step; action a in [-1,1]^2 -> d = a * D_MAX held over the horizon) and the
policy-driven corrector used in evaluation.  Observations are measured or nominal-model quantities only (no plant parameters, no future disturbance, no privileged contact information); action 0 reproduces N exactly.
Reward (no alignment term, no force-target term): positive baseline for staying in the task, penalties for base-position error, pad-reference error, contact-loss events, force above the declared bound, correction effort,
terminal penalty for failure and bonus for completing the sequence."""
import numpy as np, gymnasium as gym
from gymnasium import spaces
from ocrl.am3_episode import Episode, DT_C, BASE
from ocrl.am3_ctrl_corr import CorrectedNMPC
from ocrl.am3_nmpc import D_MAX
from ocrl.am3_mismatch import sample_scenario
from ocrl.am3_pred import C

PH = ("hover", "position", "approach", "recover", "settle", "settle_resume", "trace", "endhold", "release", "retreat", "done")
NOBS = len(PH) + 2 + 3 + 2 + 3 * 7 + 2 + 3 + 4          # phases, contact/progress, forces, gap/closing speed, 7 three-vectors (pad err, pad v err, base err, base v, attitude, omega, qd), correction, solver, history
F_BOUND = C["contact"]["F_max_N"]

def build_obs(xm, Fm, task, pr, d_prev, info, hist):
    """Normalised features: phase one-hot, contact flag/progress, measured force vs predictor force, gap and closing speed, pad error/velocity error, base error/velocity, attitude, rates, joint rates, previous correction, solver diagnostics, force-residual history."""
    ref = task.ref(); g, vc, Fn, ft, vC = (np.array(a).ravel() for a in pr.contact_f(xm, np.zeros(7), np.zeros(2))); pT = np.array(pr.pad_f(xm)[0]).ravel(); vT = np.array(pr.JC_f(xm)[1]) @ np.r_[xm[7:10], xm[10:13], xm[16:19]]; vT = vT.ravel()
    Q = xm[3:7]; att = 2 * Q[1:4] * np.sign(Q[0]); ph = np.zeros(len(PH)); ph[PH.index(task.phase)] = 1
    o = np.r_[ph, float(task.contact), min(task.tau_p / task.T, 1.0), Fm, float(Fn[0]), Fm - float(Fn[0]), np.clip(g[0] / 0.01, -3, 10), float(vc[0]) / 0.02, (pT - ref["p"]) / 0.003, (vT.ravel() - ref["v"]) / 0.01, (xm[0:3] - BASE) / 0.005, xm[7:10] / 0.03,
              att / 0.1, xm[10:13] / 0.3, xm[16:19] / 1.0, np.asarray(d_prev) / D_MAX, float(info.get("slack", 0.0)) / 5.0, float(info.get("status", 0) != 0), float(info.get("fallback", False)), np.pad(np.array(hist[-4:][::-1]) / 0.3, (0, 4 - len(hist[-4:])))]
    return np.clip(np.nan_to_num(o), -10, 10).astype(np.float32)

class PolicyCorrector:
    """Evaluation-time corrector driven by a trained SAC policy (deterministic mean action) on the same observation as in training."""
    def __init__(self, policy, pr): self.policy, self.pr = policy, pr; self.hist = []
    def reset(self): self.hist = []
    def update(self, xm, Fm, u_applied, contact, task=None, d_prev=None, info=None):
        o = build_obs(xm, Fm, task, self.pr, d_prev, info, self.hist); a, _ = self.policy.predict(o, deterministic=True); self.hist.append(Fm - float(np.array(self.pr.contact_f(xm, np.zeros(7), np.zeros(2))[2]))); return np.clip(a, -1, 1) * D_MAX

class ExternalCorrector:
    def __init__(self): self.next_d = np.zeros(2)
    def reset(self): self.next_d = np.zeros(2)
    def update(self, xm, Fm, u_applied, contact, **ctx): return self.next_d

class StepEpisode(Episode):
    """Episode with the control cycle split in observe (measure, advance task, failsafe) and commit (controller, log, plant steps) so that a learner can act between them."""
    def observe(self):
        pl, task = self.pl, self.task; self.x = pl.state(); F, self.ft = pl.pad_force(); self.F = F; self.xm, self.Fm = self.sens.measure(self.x, F, pl.d.time); gm = self.gap_meas(self.xm)
        if self.k: task.advance(self.dt_c, self.Fm, gm)
        tilt = np.degrees(2 * np.arcsin(min(1.0, np.hypot(self.xm[4], self.xm[5])))); self.stop = bool(task.failed or not np.isfinite(self.x).all() or tilt > 40 or np.linalg.norm(self.xm[0:3] - BASE) > 0.25 or pl.d.time > self.T_max)
        return self.xm, self.Fm
    def start(self): self.k = 0; self.dt_c = getattr(self.ctrl, "period", DT_C); self.n_sub = int(round(self.dt_c / self.pl.dt)); self.u_pend = self.u0.copy(); self.u_app = self.u0.copy(); return self.observe()
    def commit(self):
        pl = self.pl; t0 = __import__("time").perf_counter(); u_new, info = self.ctrl.compute(pl.d.time, self.xm, self.Fm, self.task); u_new = np.asarray(u_new, float); cyc = __import__("time").perf_counter() - t0; ref = self.task.ref()
        self.rows.append(dict(t=pl.d.time, x=self.x, xm=self.xm, F=self.F, Fm=self.Fm, ft=self.ft.copy(), phase=self.task.phase, ref=ref, pad=np.array(self.kin.pad_f(self.x)[0]).ravel(), gap=pl.gap(), u_cmd=u_new.copy(), u_app=self.u_app.copy(), com=pl.com(), cycle_s=cyc, missed=False, contact=self.task.contact, other=len(pl.other_contacts()), info=info))
        self.u_app, self.u_pend = self.u_pend, u_new
        for _ in range(self.n_sub): pl.step(self.u_app)
        self.k += 1; return info

class CorrectionEnv(gym.Env):
    metadata = {}
    def __init__(self, seed=0, max_steps=900, pool=None):
        super().__init__(); self.rng = np.random.default_rng(seed); self.max_steps = max_steps; self.pool = pool; self.observation_space = spaces.Box(-10, 10, (NOBS,), np.float32); self.action_space = spaces.Box(-1, 1, (2,), np.float32); self.ep = None
    def reset(self, seed=None, options=None):
        from am3_run import free_waypoints
        from ocrl.am3_episode import initial_state
        if seed is not None: self.rng = np.random.default_rng(seed)
        sc = sample_scenario(self.rng, int(self.rng.integers(0, 10 ** 6)), "train", int(self.rng.integers(1, 2 ** 30))) if self.pool is None else self.pool[int(self.rng.integers(0, len(self.pool)))]
        self.ext = ExternalCorrector(); ctrl = CorrectedNMPC(self.ext, "L_train"); self.ep = StepEpisode(ctrl, sc["path"], seed=sc["seed"], mismatch=sc["mismatch"], T_max=40.0, x0=initial_state(sc["gap0"], (sc["dyz"][0], 1.04 + sc["dyz"][1]))); ctrl.reset(self.ep.x0, self.ep.u0, self.ep.task)
        self.hist = []; self.n = 0; self.nloss = 0; self.info = {}; xm, Fm = self.ep.start(); self.pr = self.ep.kin; return self._obs(xm, Fm), {"scenario": sc["id"], "family": sc["family"]}
    def _obs(self, xm, Fm): return build_obs(xm, Fm, self.ep.task, self.pr, self.ep.ctrl.d, self.ep.ctrl.last_info, self.hist)
    def step(self, a):
        ep = self.ep; a = np.clip(np.asarray(a, float), -1, 1); self.ext.next_d = a * D_MAX; self.hist.append(ep.Fm - float(np.array(self.pr.contact_f(ep.xm, np.zeros(7), np.zeros(2))[2]))); ep.commit(); self.n += 1
        r_ = ep.rows[-1]; x = r_["x"]; e_base = np.linalg.norm(x[0:3] - BASE); e_pad = np.linalg.norm(r_["pad"] - r_["ref"]["p"]); xm, Fm = ep.observe(); task = ep.task; ph = task.phase; fail = ep.stop or bool(task.failed)
        peak = max(ep.pl.log["Fn"][-ep.n_sub:]); r = 1.0 - 0.3 * min(e_base / 0.005, 2.0) - 0.3 * min(e_pad / 0.003, 2.0) - 0.05 * float(np.sum(a ** 2)) - 1.5 * min(max(0.0, (peak - 0.5 * F_BOUND) / F_BOUND), 2.0)
        if len(task.recoveries) > self.nloss: r -= 2.0 * (len(task.recoveries) - self.nloss); self.nloss = len(task.recoveries)
        success = ph == "done" and not fail
        if fail: r -= 20.0
        if success: r += 50.0 + float(self.max_steps - self.n)          # credit the UNUSED episode time on completion: without it a stalled episode collects +1/step to the cap and outscores a finished one
        terminated = bool(fail or success); truncated = bool(self.n >= self.max_steps and not terminated); return self._obs(xm, Fm), float(r), terminated, truncated, {"phase": ph}
