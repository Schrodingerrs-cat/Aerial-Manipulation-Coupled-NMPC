"""End-to-end RL control of the aerial manipulator (track 2: the policy IS the controller, no NMPC anywhere).

Architecture follows Singh et al., arXiv 2606.16621 (see Findings/RL/03_endtoend_rl_design.md): the policy outputs motion REFERENCES, not motor commands, and a small
geometric inner loop turns them into rotor speeds and joint torques.  That is what makes the problem learnable without the 8192-environment budget that paper uses.

    policy  (100 Hz):  observation -> [ a_b,d (3), omega_b,d (3), yaw_d (1), q_d (3) ]
    inner loop:        thrust-vector decomposition + SO(3) attitude PD + joint PD -> [ Omega_cmd (4), tau_joint (3) ]

**No acados.** This module needs only mujoco, numpy, casadi (for the arm's gravity feedforward) and gymnasium, so it runs anywhere - including a cluster node.
"""
import os, json, numpy as np, mujoco, gymnasium as gym
from gymnasium import spaces
from ocrl.am3_plant import AMPlant3, KF, OM_MIN, OM_MAX, CFG, servo_limit
from ocrl.am3_pred import AMPred3, G0, quat2R

P = CFG["propulsion"]; TAU_C = CFG["servo"]["continuous_torque_Nm"]
BASE = np.array(json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "am3_task_v1.json")))["base_ref_world"])
Q_NOM = np.radians([0.0, -37.0, -78.0])                       # nominal arm posture, inside the operational joint ranges
OP = CFG["joint_limits_deg"]["operational"]
QLO, QHI = np.radians([OP[k][0] for k in ("q1", "q2", "q3")]), np.radians([OP[k][1] for k in ("q1", "q2", "q3")])


def vee(S): return np.array([S[2, 1], S[0, 2], S[1, 0]])


class InnerLoop:
    """Geometric thrust/attitude tracking plus joint PD - the paper's RL+PID baseline, which the TDE estimator later replaces.

    Structure mirrors the validated baseline in am3_ctrl_b: gains are set by a natural frequency against the NOMINAL LOCKED-POSTURE INERTIA ABOUT THE COM, and the
    thrust moment is transferred from the COM to the base origin.  Both matter: the arm puts the system COM ~2 cm from the base origin, so thrust applied at the rotor
    plane produces a standing pitch moment.  Without that term the vehicle tips over even when commanded to do nothing."""
    def __init__(self, wn=8.0, zeta=1.0, kp_j=1.57, kd_j=0.016, aware=False):    # wn 11.6 (the baseline's value) fails 4/20 here: no motor-lag inversion at 100 Hz; 8.0 chosen from a sweep, see Findings/RL/03
        from ocrl.am3_ref import Equilibrium
        rot = np.array(P["rotor_xy_m"]); s_ = np.array(P["spin_reaction_sign"])
        self.A = np.vstack([np.ones(4), rot[:, 1], -rot[:, 0], s_ * P["km_over_kf_m"]]); self.Ainv = np.linalg.inv(self.A)
        self.pr = AMPred3(); self.m = self.pr.m_tot
        pad0 = np.array(self.pr.fk_f(Q_NOM)).ravel() + BASE
        e = Equilibrium(self.pr, BASE).solve(pad0, [0.0, 0.0, 0.0]); self.eq = e
        Jo = np.array(self.pr.M_f(e["x"]))[3:6, 3:6]
        self.rc = np.array(self.pr.com_f(e["x"])).ravel() - BASE                      # body-frame COM offset at the nominal posture
        self.J = Jo - self.m * (self.rc @ self.rc * np.eye(3) - np.outer(self.rc, self.rc))
        wrench = self.A @ (KF * np.asarray(e["u"][:4], float) ** 2)
        self.Mbias = wrench[1:4] - np.cross(self.rc, [0.0, 0.0, wrench[0]])            # residual trim the equilibrium rotors deliver
        self.Mq = np.diag(np.array(self.pr.M_f(e["x"]))[6:9, 6:9]).copy()             # joint inertias at the nominal posture (for the disturbance observer)
        self.dob, self.k_dob, self.dt, self.lag, self.dob_state = False, np.array([10.0] * 6 + [0.0] * 3), 0.01, 2, None   # gains (1/s): base force, body moment, joints; sweep in Findings/RL/04 (joint part off: oscillates)
        self.wn, self.zeta, self.kp_j, self.kd_j, self.aware = wn, zeta, kp_j, kd_j, aware; self.qd_ref = np.zeros(3)   # joint velocity feedforward; only the wall rung sets it, the free-flight rungs keep 0

    def __call__(self, x, a_d, w_d, psi_d, q_d):
        R = np.array(quat2R(x[3:7])); w = x[10:13]
        rc = self.rc
        if self.aware: xc = self.eq["x"].copy(); xc[13:16] = x[13:16]; rc = np.array(self.pr.com_f(xc)).ravel() - BASE          # COM offset at the measured posture (R6)
        d = self.observe(x, R, w, rc) if self.dob else np.zeros(9)
        Fd = self.m * (a_d + np.array([0.0, 0.0, G0])) - d[0:3]
        nF = np.linalg.norm(Fd)
        b3 = Fd / nF if nF > 1e-6 else np.array([0.0, 0.0, 1.0])
        b1c = np.array([np.cos(psi_d), np.sin(psi_d), 0.0])
        b2 = np.cross(b3, b1c); n2 = np.linalg.norm(b2)
        b2 = b2 / n2 if n2 > 1e-6 else np.array([0.0, 1.0, 0.0])
        Rd = np.column_stack([np.cross(b2, b3), b2, b3])
        eR = 0.5 * vee(Rd.T @ R - R.T @ Rd); ew = w - R.T @ Rd @ w_d
        Mb = -self.J @ (self.wn ** 2 * eR + 2 * self.zeta * self.wn * ew) + np.cross(w, self.J @ w)
        T = float(Fd @ R[:, 2])
        Mb = Mb + np.cross(rc, [0.0, 0.0, T]) + self.Mbias                        # moment about the COM -> about the thrust-plane origin
        xr = x.copy(); xr[13:16] = q_d; xr[16:19] = 0.0
        g_j = np.array(self.pr.grav_f(xr)).ravel()[6:9]
        tau = g_j + self.kp_j * (q_d - x[13:16]) - self.kd_j * (x[16:19] - self.qd_ref)
        Mb = Mb - d[3:6]; tau = tau - d[6:9]
        f = np.clip(self.Ainv @ np.r_[T, Mb], P["f_min_N"], P["f_max_N"])
        tau = np.clip(tau, -TAU_C, TAU_C)
        if self.dob: self.dob_state["tau"] = self.dob_state["tau"][1:] + [tau]
        return np.r_[np.clip(np.sqrt(f / KF), OM_MIN, OM_MAX), tau]

    def observe(self, x, R, w, rc):
        """Generalized-momentum disturbance observer on the full coupled model (2026-11-11; the TDE idea of Singh et al., without differentiating noisy rates).
        M(x) nu' + b(x, nu) + G(x) = tau_rot(x) + [0, tau_j - D qd] + d,  nu = [v (world), w (body), qd].  With p = M nu:  p' = (rhs above) + M' nu.
        Each call integrates the interval that just ended (trapezoid; rotor wrench from the measured rotor speeds, so motor lag and latency are inside; the joint command
        that acted on it is `lag` calls old), then d_hat = K (p - p0 - integral(rhs + M' nu + d_hat)).  Arm-base coupling is in M, so d is only what the model lacks:
        contact, payload, thrust/lag/damping error, wind.  d[0:3] world force, d[3:6] body moment about the base origin, d[6:9] joint torques."""
        nu = np.r_[x[7:13], x[16:19]].astype(float); Mx = np.array(self.pr.M_f(x)); pmom = Mx @ nu
        f = np.array(self.pr.tau_rot_f(x)).ravel() - np.array(self.pr.bias_f(x)).ravel() - np.array(self.pr.grav_f(x)).ravel(); f[6:9] -= self.pr.D * nu[6:9]
        if self.dob_state is None: self.dob_state = dict(p0=pmom, I=np.zeros(9), d=np.zeros(9), f=f, M=Mx, nu=nu, tau=[np.array(self.pr.grav_f(x)).ravel()[6:9] for _ in range(self.lag)]); return self.dob_state["d"]
        S = self.dob_state; fa = 0.5 * (f + S["f"]); fa[6:9] += S["tau"][0]
        S["I"] = S["I"] + (fa + S["d"]) * self.dt + (Mx - S["M"]) @ (0.5 * (nu + S["nu"])); S["f"], S["M"], S["nu"] = f, Mx, nu
        S["d"] = self.k_dob * (pmom - S["p0"] - S["I"]); return S["d"]


class Sensors:
    """Same noise model as the NMPC harness (ocrl/am3_episode.Sensors), so RL and NMPC see measurements of equal quality."""
    SIG = dict(p=1e-4, att=np.radians(0.01), v=1e-3, w=5e-4, q=5e-5, qd=1e-3)
    REAL = dict(p=1e-3, att=np.radians(0.5), v=0.01, w=0.02, q=1e-3, qd=0.02, Om=5.0)      # mocap + IMU class (Phase 1, 2026-11-11; same as scripts/am3_write_bench.SIG_REAL)
    def __init__(self, seed=0, sig=None): self.rng = np.random.default_rng(seed); self.SIG = sig or Sensors.SIG
    def __call__(self, x):
        s, r = self.SIG, self.rng.standard_normal; xm = x.copy()
        xm[0:3] += s["p"] * r(3); dth = s["att"] * r(3); Q = x[3:7]
        dq = 0.5 * np.array([-Q[1] * dth[0] - Q[2] * dth[1] - Q[3] * dth[2], Q[0] * dth[0] + Q[2] * dth[2] - Q[3] * dth[1],
                             Q[0] * dth[1] - Q[1] * dth[2] + Q[3] * dth[0], Q[0] * dth[2] + Q[1] * dth[1] - Q[2] * dth[0]])
        xm[3:7] = (Q + dq) / np.linalg.norm(Q + dq)
        xm[7:10] += s["v"] * r(3); xm[10:13] += s["w"] * r(3); xm[13:16] += s["q"] * r(3); xm[16:19] += s["qd"] * r(3)
        if "Om" in s: xm[19:23] += s["Om"] * r(4)
        return xm


# ---- reward shaping (paper eqs 4-6): two scales so the gradient survives far away and sharpens near the goal
def rw(x, r1, r2, r3, r4): return r1 * np.exp(-r2 * x * x) + r3 * np.exp(-r4 * x * x)
def pen(x, p1, p2): return p1 * (np.exp(-p2 * x * x) - 1.0)                    # always <= 0


class HoverEnv(gym.Env):
    """R1: hold the hover reference with the arm held at its nominal posture.  Action is the 10-dim motion reference; the inner loop closes the loop on it.

    Reward uses the paper's multiplicative gate: the secondary terms (smoothness, low velocity, upright) are multiplied by the positional reward, so they cannot be
    collected by a policy that ignores the task.  There is NO unconditional per-step survival bonus - that is what let our earlier correction policy profit from stalling."""
    metadata = {}
    ACT_LO = np.array([-5.0, -5.0, -5.0, -1.5, -1.5, -1.5, -0.3, *(QLO - Q_NOM)])      # body-rate feedforward +-1.5 rad/s: white exploration noise on +-4 acted directly as a rate error
    ACT_HI = np.array([5.0, 5.0, 5.0, 1.5, 1.5, 1.5, 0.3, *(QHI - Q_NOM)])

    def __init__(self, seed=0, max_steps=1000, dt_c=0.01, dt_p=0.001, p0_sigma=0.15, v0_sigma=0.2, randomize=True, arm_frac=0.05, fine_w=0.0, fine_s=0.01, lin_w=0.0, tilt_w=0.0, bad_pen=20.0):
        super().__init__()
        self.rng = np.random.default_rng(seed); self.max_steps = max_steps
        self.dt_c, self.dt_p = dt_c, dt_p; self.n_sub = int(round(dt_c / dt_p))
        self.p0_sigma, self.v0_sigma, self.randomize = p0_sigma, v0_sigma, randomize
        self.pref, self.vref, self.aref = BASE.copy(), np.zeros(3), np.zeros(3)      # reference the base tracks: fixed hover point here, a moving trajectory in TrackEnv
        self.bad_pen = bad_pen                                     # one-off penalty on termination (original reward: 20)
        self.tilt_w = tilt_w                                       # penalty per step per 10 deg of tilt beyond 20 deg (gentler recovery); 0 keeps the reward unchanged
        self.lin_w = lin_w                                         # near-target linear penalty weight (per 1 cm of error, error capped at 3 cm); 0 keeps the original reward
        self.fine_w, self.fine_s = fine_w, fine_s                  # optional third reward scale (fine_s metres, weight fine_w); 0 keeps the original reward
        self.qdref = np.zeros(3); self.latency = False
        self.arm_frac = arm_frac                                   # joint-action authority as a fraction of the operational range; small for hover, larger for the arm rungs
        self.pl = AMPlant3(dt=dt_p); self.pl.m.geom_contype[self.pl.gwall] = 0; self.pl.m.geom_conaffinity[self.pl.gwall] = 0   # free flight: no wall
        self.inner = InnerLoop(); self.sens = Sensors(seed)
        self.observation_space = spaces.Box(-np.inf, np.inf, (31,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (10,), np.float32)
        self.hover_u = np.asarray(self.inner.eq["u"], float); self.hover_om = self.hover_u[:4].copy()

    # ---- helpers
    def _state0(self):
        x = np.zeros(23); x[0:3] = BASE + self.p0_sigma * self.rng.standard_normal(3) * (1.0 if self.randomize else 0.0)
        ang = 0.1 * self.rng.standard_normal(3) * (1.0 if self.randomize else 0.0); th = np.linalg.norm(ang)
        x[3:7] = np.r_[np.cos(th / 2), np.sin(th / 2) * (ang / max(th, 1e-9))]
        x[7:10] = self.v0_sigma * self.rng.standard_normal(3) * (1.0 if self.randomize else 0.0)
        x[10:13] = 0.2 * self.rng.standard_normal(3) * (1.0 if self.randomize else 0.0)
        x[13:16] = Q_NOM; x[19:23] = self.hover_om
        return x

    def _extra_obs(self): return np.zeros(0)
    def _new_episode(self): pass
    def _set_ref(self, t): pass

    def _obs(self, xm):
        R = np.array(quat2R(xm[3:7]))
        return np.clip(np.nan_to_num(np.r_[(self.pref - xm[0:3]) / 0.5, R[:, 0], R[:, 1], (xm[7:10] - self.vref) / 2.0, xm[10:13] / 4.0,
                                           (xm[13:16] - Q_NOM) / 0.5, xm[16:19] / 2.0, self.a_prev, self._extra_obs()]), -10, 10).astype(np.float32)

    def expert_action(self, x, kp=5.0, kd=3.5):
        """Scripted outer-loop PD (with reference-acceleration feedforward) through the same action interface: the bar a learned policy has to clear."""
        a = np.zeros(10); a[0:3] = np.clip((self.aref + kp * (self.pref - x[0:3]) + kd * (self.vref - x[7:10])) / 5.0, -1, 1); return a

    def _decode(self, a):
        a = np.clip(np.asarray(a, float), -1, 1)
        u = self.ACT_LO + (a + 1.0) * 0.5 * (self.ACT_HI - self.ACT_LO)
        return u[0:3], u[3:6], float(u[6]), Q_NOM + self.arm_frac * u[7:10]

    # ---- gym API
    def reset(self, seed=None, options=None):
        if seed is not None: self.rng = np.random.default_rng(seed); self.sens = Sensors(seed, self.sens.SIG)
        self.x0 = self._state0(); self.pl.set_state(self.x0); self.pl.reset_log()
        self.a_prev = np.zeros(10); self.n = 0; self.inner.dob_state = None
        self.u_pend = np.r_[self.x0[19:23], np.asarray(self.inner.eq["u"], float)[4:]]
        self._new_episode(); self._set_ref(0.0)
        self.e_prev = float(np.linalg.norm(self.x0[0:3] - self.pref))
        return self._obs(self.sens(self.x0)), {}

    def step(self, a):
        a = np.clip(np.asarray(a, float), -1, 1)
        a_d, w_d, psi_d, q_d = self._decode(a)
        x = self.pl.state(); u = self.inner(self.sens(x), a_d, w_d, psi_d, q_d)
        if self.latency: u, self.u_pend = self.u_pend, u          # one control period of computation latency, as in the NMPC Episode harness (wall rung)
        for _ in range(self.n_sub): self.pl.step(u)
        self.n += 1; x = self.pl.state(); self._set_ref(self.n * self.dt_c)
        e = float(np.linalg.norm(x[0:3] - self.pref)); R = np.array(quat2R(x[3:7]))
        tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        bad = (not np.isfinite(x).all()) or tilt > 60.0 or e > 1.5 or x[2] < 0.3
        da = a - self.a_prev; self.a_prev = a.copy()

        r_pos = 10.0 * np.exp(-(e / 0.5) ** 2) + 10.0 * np.exp(-(e / 0.05) ** 2) + self.fine_w * np.exp(-(e / self.fine_s) ** 2)       # two scales in METRES: wide (0.5 m) keeps a gradient far away, narrow (5 cm) sharpens near the goal
        r_up = rw(abs(1.0 - R[2, 2]), 2.5, 5, 2.5, 2)
        r_vel = rw(np.linalg.norm(x[7:10] - self.vref) / 0.5, 1, 5, 0, 0) + rw(np.linalg.norm(x[10:13]) / 2.0, 1.5, 2, 0, 0)
        r_jvel = rw(np.linalg.norm(x[16:19] - self.qdref) / 1.0, 2, 5, 0, 0)
        p_act = pen(np.linalg.norm(a[0:3]), 4, 1) + pen(np.linalg.norm(a[3:6]), 4, 5) + pen(abs(a[6]), 2, 5)
        p_rate = pen(np.linalg.norm(da[0:3]), 4, 5) + pen(np.linalg.norm(da[3:6]), 4, 12) + pen(abs(da[6]), 4, 12) + pen(np.linalg.norm(da[7:10]), 4, 12)
        r_near = r_up + r_vel + r_jvel + p_act + p_rate                                  # gated by r_pos: unreachable without being near the goal
        de = self.e_prev - e; self.e_prev = e
        r_prog = 50.0 * de if de > 0 else 100.0 * de                                     # asymmetric: receding costs twice what approaching pays
        r = r_prog + (r_pos * r_near + 50.0 * r_pos + 2.5 * (p_act + p_rate)) / 100.0
        r -= self.lin_w * min(e, 0.03) / 0.01 + self.tilt_w * max(0.0, tilt - 20.0) / 10.0
        if bad: r -= self.bad_pen

        trunc = bool(self.n >= self.max_steps and not bad)
        return self._obs(self.sens(x)), float(r), bool(bad), trunc, dict(e=e, tilt=tilt, steps=self.n)


# ---------------------------------------------------------------------------------------------------------------------------------------------------------
def _ramped(u, laps, T, Tr):
    """Cumulative angle and angular rate after u seconds of the smooth-rate profile: ramps 0 -> 1 -> 0 and completes exactly `laps` revolutions in T seconds.
    Same maths as LoopTask._ramped in ocrl/am3_traj.py, so the reference family is the one the NMPC demos used."""
    Tr = min(Tr, T / 2)
    if u <= Tr: x = u / Tr; te, rt = Tr * (x ** 3 - x ** 4 / 2), 3 * x ** 2 - 2 * x ** 3
    elif u <= T - Tr: te, rt = 0.5 * Tr + (u - Tr), 1.0
    else: y = max(0.0, (T - u) / Tr); te, rt = (T - 1.5 * Tr) + (0.5 * Tr - Tr * (y ** 3 - y ** 4 / 2)), 3 * y ** 2 - 2 * y ** 3
    w = 2 * np.pi * laps / max(T - Tr, 1e-9); return w * te, w * rt


SHAPES = dict(lemniscate=(1, 2, 0.0), lissajous=(2, 3, np.pi / 2), sine=(1, 3, 0.0))      # (ny, nz, phase): y = A sin(ny th), z = C (sin(nz th + phase) - sin(phase))


def ref_point(t, p):
    """Base reference position and velocity at time t for reference parameters p: a Lissajous figure in the y-z plane, starting and ending at rest at the hover point."""
    th, thd = _ramped(t, p["laps"], p["T"], p["Tr"]); ny, nz, ph = SHAPES[p["shape"]]
    pos = BASE + np.array([0.0, p["Ay"] * np.sin(ny * th), p["Az"] * (np.sin(nz * th + ph) - np.sin(ph))])
    vel = np.array([0.0, p["Ay"] * ny * np.cos(ny * th) * thd, p["Az"] * nz * np.cos(nz * th + ph) * thd])
    return pos, vel


def peak_accel(p, n=400):
    v = np.array([ref_point(t, p)[1] for t in np.linspace(0, p["T"], n)]); return float(np.linalg.norm(np.diff(v, axis=0) / (p["T"] / n), axis=1).max())


class TrackEnv(HoverEnv):
    """R2: the base follows a moving reference trajectory (a Lissajous figure in the y-z plane) with the arm held still.  Same policy, action interface, inner loop and reward
    as R1, with the position/velocity terms now measured against the moving reference.  Observation adds the reference velocity and two look-ahead points (0.25 s, 0.5 s),
    which is how the policy learns acceleration feed-forward without being handed the acceleration.  Reference parameters are re-drawn every episode, rejecting figures whose
    peak acceleration exceeds `max_acc` (the NMPC demos peak at about 0.1 m/s^2; this keeps training inside what the tilt limit allows).  Fine reward term on by default."""
    LOOK = (0.25, 0.5)

    def __init__(self, *args, ref=None, max_acc=2.0, fine_w=10.0, **kw):
        super().__init__(*args, fine_w=fine_w, **kw)
        self.fixed_ref, self.max_acc = ref, max_acc
        self.observation_space = spaces.Box(-np.inf, np.inf, (31 + 3 + 3 * len(self.LOOK),), np.float32)

    def _new_episode(self):
        if self.fixed_ref is not None: self.p = dict(self.fixed_ref)
        else:
            while True:
                T = float(self.rng.uniform(12.0, 30.0))
                p = dict(Ay=float(self.rng.uniform(0.03, 0.20)), Az=float(self.rng.uniform(0.01, 0.08)), T=T, Tr=2.5, laps=int(self.rng.integers(1, 4)),
                         shape=str(self.rng.choice(list(SHAPES))))
                if peak_accel(p) <= self.max_acc: break
            self.p = p
        self.max_steps = int((self.p["T"] + 1.0) / self.dt_c)                      # one second of hold at the end

    def _set_ref(self, t):
        self.pref, self.vref = ref_point(t, self.p)
        self.aref = (ref_point(t + self.dt_c, self.p)[1] - self.vref) / self.dt_c

    def _extra_obs(self):
        t = self.n * self.dt_c
        look = [(ref_point(t + h, self.p)[0] - self.pref) / 0.2 for h in self.LOOK]
        return np.concatenate([self.vref / 0.3, *look])


# ---------------------------------------------------------------------------------------------------------------------------------------------------------
def ref_point3(t, p):
    """3-D Lissajous reference (x forward/back, y left/right, z up/down), starting and ending at rest at the hover point:  axis = A (sin(n th + phase) - sin(phase))."""
    th, thd = _ramped(t, p["laps"], p["T"], p["Tr"]); pos, vel = BASE.copy(), np.zeros(3)
    for i, ax in enumerate("xyz"):
        A, n, ph = p["A" + ax], p["n" + ax], p["p" + ax]; pos[i] += A * (np.sin(n * th + ph) - np.sin(ph)); vel[i] = A * n * np.cos(n * th + ph) * thd
    return pos, vel


def peak_accel3(p, n=400):
    v = np.array([ref_point3(t, p)[1] for t in np.linspace(0, p["T"], n)]); return float(np.linalg.norm(np.diff(v, axis=0) / (p["T"] / n), axis=1).max())


class Track3Env(TrackEnv):
    """R2b: larger 3-D trajectories (all six directions), a fine-scaled error input (position error / 2 cm) and a leaky error integral (tau 3 s) so a persistent offset is visible,
    and a linear near-target penalty on top of the two-scale reward.  Diagnosed from R2: the policy sat ~10 mm below the reference in z while the position error entered the
    network as error/0.5 m (a 10 mm offset was a 0.02 input) and the 1 cm reward term has almost no gradient at 10 mm."""
    def __init__(self, *args, ref=None, max_acc=2.0, fine_w=20.0, lin_w=0.4, tau_i=3.0, **kw):
        super().__init__(*args, ref=ref, max_acc=max_acc, fine_w=fine_w, lin_w=lin_w, **kw)
        self.tau_i = tau_i; self.ei = np.zeros(3)
        self.observation_space = spaces.Box(-np.inf, np.inf, (31 + 3 + 3 * len(self.LOOK) + 6,), np.float32)

    def _new_episode(self):
        self.ei = np.zeros(3)
        if self.fixed_ref is not None: self.p = dict(self.fixed_ref)
        else:
            while True:
                p = dict(T=float(self.rng.uniform(15.0, 35.0)), Tr=2.5, laps=int(self.rng.integers(1, 4)),
                         Ax=float(self.rng.choice([0.0, self.rng.uniform(0.1, 0.4)])), Ay=float(self.rng.uniform(0.1, 0.5)), Az=float(self.rng.uniform(0.03, 0.25)))
                for ax in "xyz": p["n" + ax] = int(self.rng.integers(1, 4)); p["p" + ax] = float(self.rng.uniform(0, 2 * np.pi))
                if peak_accel3(p) <= self.max_acc: break
            self.p = p
        self.max_steps = int((self.p["T"] + 1.0) / self.dt_c)

    def _set_ref(self, t):
        self.pref, self.vref = ref_point3(t, self.p)
        self.aref = (ref_point3(t + self.dt_c, self.p)[1] - self.vref) / self.dt_c

    def _extra_obs(self):
        t = self.n * self.dt_c; err = self.pref - self.pl.state()[0:3]
        self.ei += (err - self.ei) * self.dt_c / self.tau_i
        look = [(ref_point3(t + h, self.p)[0] - self.pref) / 0.5 for h in self.LOOK]
        return np.concatenate([self.vref / 0.8, *look, np.clip(err / 0.02, -5, 5), np.clip(self.ei / 0.02, -5, 5)])


class Track4Env(Track3Env):
    """R4: Track3Env with a bounded action envelope and the survival costs built in.  R3 diagnosis: in every lost start the policy saturated its body-rate command (1.5 rad/s) and
    flipped to 60 deg tilt within one second, while the scripted expert never needs more than ~2.5 m/s^2 and zero body rate.  Commanded acceleration is limited to +-acc_lim and
    body rate to +-rate_lim (like the input constraints the NMPC has); tilt_w / bad_pen penalise high tilt and termination.  Everything else is Track3Env."""
    def __init__(self, *args, acc_lim=3.5, rate_lim=0.6, tilt_w=3.0, bad_pen=60.0, **kw):
        super().__init__(*args, tilt_w=tilt_w, bad_pen=bad_pen, **kw)
        self.ACT_LO, self.ACT_HI = self.ACT_LO.copy(), self.ACT_HI.copy()
        self.ACT_LO[0:3], self.ACT_HI[0:3], self.ACT_LO[3:6], self.ACT_HI[3:6] = -acc_lim, acc_lim, -rate_lim, rate_lim

    def expert_action(self, x, kp=5.0, kd=3.5):
        a = np.zeros(10); a[0:3] = np.clip((self.aref + kp * (self.pref - x[0:3]) + kd * (self.vref - x[7:10])) / self.ACT_HI[0], -1, 1); return a


class Track5Env(Track4Env):
    """R5: big 3-D base trajectory AND a moving arm at the same time.  The arm has a joint reference (sinusoid per joint about the nominal posture, starting at it; amplitude 10-35% of the smaller half-range, because the PD baseline loses the drone at 50-100%); the policy
    outputs a residual +-arm_res rad on top of it.  Reward adds a joint-tracking term; the joint-velocity penalty is taken about the reference velocity.  The scripted expert
    (PD+ff, zero residual) follows the same arm reference and ignores the arm-base coupling: the baseline to beat."""
    def __init__(self, *args, arm_res=0.1, jq_w=0.3, arm_amp=(0.1, 0.35), **kw):
        super().__init__(*args, **kw)
        self.arm_res, self.jq_w, self.arm_amp = arm_res, jq_w, arm_amp
        self.qref, self.qdref = Q_NOM.copy(), np.zeros(3)
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.observation_space.shape[0] + 9,), np.float32)

    def _new_episode(self):
        super()._new_episode()
        half = np.minimum(Q_NOM - QLO, QHI - Q_NOM)
        self.qa = self.rng.uniform(*self.arm_amp, 3) * half; self.qw = 2 * np.pi / self.rng.uniform(4.0, 10.0, 3); self.qph = self.rng.uniform(0, 2 * np.pi, 3)

    def _set_ref(self, t):
        super()._set_ref(t)
        self.qref = Q_NOM + self.qa * (np.sin(self.qw * t + self.qph) - np.sin(self.qph))
        self.qdref = self.qa * self.qw * np.cos(self.qw * t + self.qph)

    def _extra_obs(self):
        q = self.pl.state()[13:16]
        return np.concatenate([super()._extra_obs(), np.clip((self.qref - q) / 0.1, -5, 5), (self.qref - Q_NOM) / 0.5, self.qdref / 1.0])

    def _decode(self, a):
        a_d, w_d, psi_d, _ = super()._decode(a)
        return a_d, w_d, psi_d, self.qref + self.arm_res * np.clip(np.asarray(a, float)[7:10], -1, 1)

    def step(self, a):
        obs, r, bad, trunc, info = super().step(a)
        ej = float(np.linalg.norm(self.pl.state()[13:16] - self.qref)); info["ej"] = ej
        return obs, r - self.jq_w * min(ej, 0.1) / 0.02, bad, trunc, info


class Track6Env(Track5Env):
    """R6: R5 with a posture-aware inner loop: the thrust-moment transfer uses the COM at the measured joint angles instead of the nominal posture.  Diagnosed from R5: a 20 deg yaw-joint
    offset moves the COM about 2 cm sideways; with the fixed offset the attitude PD (no integral) needs more commanded tilt than the +-3.5 m/s^2 action limit allows."""
    def __init__(self, *args, arm_amp=(0.1, 0.5), **kw):                    # 0.5: the sinusoid reaches 2 x amplitude from nominal, so this is the largest swing that stays in the operational range
        super().__init__(*args, arm_amp=arm_amp, **kw); self.inner.aware = True



class Track7Env(Track6Env):
    """R7: R6 with the yaw reference fixed at zero; the policy's yaw output is ignored.  No path has a yaw target, and overriding only this channel on trained R6 policies removed all early crashes on 300 starts (Findings/RL/03)."""
    def _decode(self, a):
        a_d, w_d, _, q_d = super()._decode(a)
        return a_d, w_d, 0.0, q_d


class Track8Env(Track7Env):
    """R8: R7 with the commanded acceleration limited to +-1.75 m/s^2 (half of R4's +-3.5).  Clipping trained R7 policies to this removed every early crash on 350 starts (Findings/RL/03).  The paths peak at 0.43 m/s^2."""
    def __init__(self, *args, acc_lim=1.75, **kw):
        super().__init__(*args, acc_lim=acc_lim, **kw)


class Track9Env(Track8Env):
    """R9: R8 with the acceleration limit at +-2.5 m/s^2, between R8 (survives, 4.8-8.6 mm) and R7's +-3.5 (4.4-4.8 mm, loses starts)."""
    def __init__(self, *args, acc_lim=2.5, **kw):
        super().__init__(*args, acc_lim=acc_lim, **kw)
