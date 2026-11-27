"""Learned 'cheetah tail' (2026-11-27, capstone): RL policy that flies multi-waypoint routes while carrying a block in the hand, with the arm either free to swing (tail) or held.
Same policy interface, inner loop and reward as the R9 moving-base rung (Track9Env: commanded acceleration, body rates, joint targets), changed only where the task needs it:
  - reference: random waypoint routes (3-5 points, 0.3-0.8 m hops, min-jerk segments of T_rng s, dwells) instead of Lissajous figures; look-ahead from the route
  - block: random mass 0..mb_max kg at the hand origin (plant payload body); its mass is an INPUT (load-aware: the capstone's analytical estimate gives it)
  - arm: home posture Q_NOM as the joint reference; 'free' gives the policy +-arm_res rad around it and a weak home term (jq_w), 'rigid' holds the arm at home (no arm action)
  - inner-loop disturbance observer on (as R15-R21)
  - acceleration limit 4 m/s^2 (a 0.6 m min-jerk hop in 1 s peaks at 3.5 m/s^2); free flight, no wall contact (as every Track rung)."""
import numpy as np
from gymnasium import spaces
from ocrl.am3_rl_direct import Track9Env, BASE, Q_NOM, QLO, QHI
from ocrl.am3_path import min_jerk


class TailEnv(Track9Env):
    def __init__(self, *args, arm="free", mb_max=0.2, acc_lim=4.0, T_rng=(1.0, 2.0), fixed=None, **kw):
        super().__init__(*args, acc_lim=acc_lim, arm_res=0.8 if arm == "free" else 0.0, jq_w=0.05 if arm == "free" else 0.3, p0_sigma=0.0, v0_sigma=0.0, **kw)
        self.arm, self.mb_max, self.T_rng, self.fixed, self.mb = arm, mb_max, T_rng, fixed, 0.0
        self.inner.dob = True; self.inner.k_dob[6:9] = 4.0                     # inner-loop disturbance observer with the R21 joint channel: without it the block's constant moment tips the drone (scripted PD: 1 m off with 150 g)
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.observation_space.shape[0] + 1,), np.float32)

    def _new_episode(self):
        g = self.rng; self.ei = np.zeros(3)
        if self.fixed: W, T, dw, self.mb = [np.asarray(w, float) for w in self.fixed["route"]], self.fixed["T"], self.fixed["dwell"], self.fixed["mb"]
        else:
            W = [BASE.copy()]; T = float(g.uniform(*self.T_rng)); dw = float(g.uniform(0.5, 1.5)); self.mb = float(g.uniform(0, self.mb_max)) if g.random() < 0.8 else 0.0
            for _ in range(int(g.integers(3, 6))):
                while True:
                    d = g.standard_normal(3) * np.array([1.0, 1.0, 0.3]); d *= g.uniform(0.3, 0.8) / np.linalg.norm(d); w = W[-1] + d
                    if np.all(np.abs(w[:2] - BASE[:2]) <= 0.8) and abs(w[2] - BASE[2]) <= 0.3: break
                W.append(w)
        self.route, self.T, self.dwell, self.t0 = np.array(W), T, dw, 1.0
        self.max_steps = int((self.t0 + (len(W) - 1) * (T + dw) + 0.5) / self.dt_c)
        m, gw = self.pl.m, self.pl.gwall; m.geom_contype[gw] = m.geom_conaffinity[gw] = 1                 # apply_mismatch recalibrates the wall contact, so the wall must collide while it runs
        x = self.pl.state(); self.pl.apply_mismatch(dict(payload=dict(kg=max(self.mb, 1e-9), pos_hand_m=[0.0, 0.0, 0.0]))); x[0:3] = self.route[0]; self.pl.set_state(x)          # mj_setConst resets the state
        m.geom_contype[gw] = m.geom_conaffinity[gw] = 0                                                 # free flight again

    def ref_at(self, t):
        k = int(np.clip((t - self.t0) // (self.T + self.dwell), 0, len(self.route) - 2)); ts = t - self.t0 - k * (self.T + self.dwell)
        if t < self.t0: return self.route[0], np.zeros(3), np.zeros(3)
        if ts < self.T:
            f, f1, f2 = min_jerk(ts / self.T); d = self.route[k + 1] - self.route[k]; return self.route[k] + d * f, d * f1 / self.T, d * f2 / self.T ** 2
        return self.route[k + 1], np.zeros(3), np.zeros(3)

    def _set_ref(self, t):
        self.pref, self.vref, self.aref = self.ref_at(t); self.qref, self.qdref = Q_NOM.copy(), np.zeros(3)

    def _extra_obs(self):
        t = self.n * self.dt_c; x = self.pl.state(); err = self.pref - x[0:3]
        self.ei += (err - self.ei) * self.dt_c / self.tau_i
        look = [(self.ref_at(t + h)[0] - self.pref) / 0.5 for h in self.LOOK]
        return np.concatenate([self.vref / 0.8, *look, np.clip(err / 0.02, -5, 5), np.clip(self.ei / 0.02, -5, 5),
                               np.clip((self.qref - x[13:16]) / 0.1, -5, 5), (self.qref - Q_NOM) / 0.5, self.qdref / 1.0, [self.mb / 0.25]])

    def _decode(self, a):
        a_d, w_d, psi_d, q_d = super()._decode(a); return a_d, w_d, psi_d, np.clip(q_d, QLO, QHI)
