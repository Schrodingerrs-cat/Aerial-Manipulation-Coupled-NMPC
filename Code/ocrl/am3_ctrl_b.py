"""Baseline B (Plan s7.1), strictly separated: two independent controllers that exchange nothing at run time.  The physical plant stays fully coupled; only the information boundary is cut.
DroneController: input = drone measurements only (position, attitude, velocity, angular velocity, rotor speeds); output = four rotor-speed commands.  Fixed design constants only: total mass, nominal locked-posture COM and
  inertia, hover-trim moment initialised once from the rotor telemetry at t0.  It receives no arm joint state, no arm/task reference or phase, no wall-force measurement, and no force feedforward.
ArmController: input = joint measurements (q, qd) and ITS OWN reference (pad trajectory and contact reference from the task); output = three joint torques.  It receives no drone state, no rotor command, nothing from the
  drone controller.  Fixed mounting/world-frame assumption (documented): the drone sits at the fixed hover reference BASE with a pitch derived only from the arm's own contact reference (a nominal pitch atan(F_ref / m g) for
  the wall force F_ref it intends to apply, 0 in free flight); there is no feedback of the measured pad position, because that would need the drone pose.
BaselineB is only plumbing: it slices the measurement vector into the two interfaces and concatenates the outputs.  The information boundary is verified by scripts/am3_check_b_separation.py."""
import numpy as np
from ocrl.am3_pred import AMPred3, C, G0, quat2R
from ocrl.am3_task import BASE
from ocrl.am3_ref import Equilibrium, quat_from_rp
from ocrl.am3_plant import OM_MIN, OM_MAX, KF

GAINS = dict(kp_pos=4.7908, kd_pos=5.1545, ki_pos=1.2254, w_att=11.5777, zeta_att=1.0, ki_att=454.269, i_pos_max=0.3, i_att_max=0.6, lag_gain=0.0, tau_motor=0.04, kp_joint=0.9952, kd_joint=0.0185)  # B v5: config/am3_b_gains_v5.json

def vee(S): return np.array([S[2, 1], S[0, 2], S[1, 0]])
def hat(a): return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
def expm(w):
    th = np.linalg.norm(w)
    if th < 1e-12: return np.eye(3)
    K = hat(w / th); return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
def Rmat(Q): return np.array(quat2R(Q))

class DroneController:
    """Geometric hover controller about the fixed reference BASE.  compute(t, y): y = [p(3), Q(4), v(3), omega(3), rotor speeds(4)] -> rotor-speed commands (4)."""
    def __init__(self, gains, period):
        self.g, self.period = gains, period; self.t_lat = 1.5 * period; pr = AMPred3(); P = C["propulsion"]; self.m = pr.m_tot
        rot = np.array(P["rotor_xy_m"]); s = np.array(P["spin_reaction_sign"]); self.A = np.vstack([np.ones(4), rot[:, 1], -rot[:, 0], s * P["km_over_kf_m"]]); self.Ainv = np.linalg.inv(self.A)          # [T, Mx, My, Mz] = A f
        e = Equilibrium(pr, BASE).solve(np.array(pr.fk_f(np.radians([0, -37, -78]))).ravel() + BASE, [0, 0, 0]); Jo = np.array(pr.M_f(e["x"]))[3:6, 3:6]
        self.rc = np.array(pr.com_f(e["x"])).ravel() - BASE; self.Jn = Jo - self.m * (self.rc @ self.rc * np.eye(3) - np.outer(self.rc, self.rc))          # declared nominal locked-posture COM and inertia about the COM (design constants)
    def reset(self, y0):
        self.ip = np.zeros(3); self.ia = np.zeros(3); self.t_prev = None; self.om_prev = np.asarray(y0[13:17], float).copy()
        w = self.A @ (KF * self.om_prev ** 2); self.Mbias = w[1:4] - np.cross(self.rc, [0, 0, w[0]])          # hover trim from the rotor telemetry at t0: moment the rotors deliver beyond the nominal-COM model
    def held(self, om): self.om_prev = np.asarray(om, float).copy()
    def compute(self, t, y):
        g, T0 = self.g, self.period; dt = T0 if self.t_prev is None else t - self.t_prev; self.t_prev = t; p, Q, v, w, Om = y[0:3], y[3:7], y[7:10], y[10:13], y[13:17]
        R = Rmat(Q) @ expm(w * 1.5 * T0); ep = p + 1.5 * T0 * v - BASE          # state predicted over the command latency
        self.ip = np.clip(self.ip + ep * dt, -g["i_pos_max"], g["i_pos_max"]); a_d = -g["kp_pos"] * ep - g["kd_pos"] * v - g["ki_pos"] * self.ip
        Fd = self.m * (a_d + np.array([0, 0, G0])); b3 = Fd / np.linalg.norm(Fd); b2 = np.cross(b3, [1.0, 0, 0]); b2 /= np.linalg.norm(b2); Rd = np.column_stack([np.cross(b2, b3), b2, b3])
        eR = 0.5 * vee(Rd.T @ R - R.T @ Rd); self.ia = np.clip(self.ia + eR * dt, -g["i_att_max"], g["i_att_max"]); J = self.Jn
        Mb = -J @ (g["w_att"] ** 2 * eR + 2 * g["zeta_att"] * g["w_att"] * w + g["ki_att"] * self.ia) + np.cross(w, J @ w); T = Fd @ R[:, 2]; Mb = Mb + np.cross(self.rc, [0, 0, T]) + self.Mbias          # moment about the COM -> about the thrust-plane origin
        f = np.clip(self.Ainv @ np.r_[T, Mb], C["propulsion"]["f_min_N"], C["propulsion"]["f_max_N"]); om_des = np.sqrt(f / KF)
        e_lag = np.exp(-T0 / g["tau_motor"]); om_next = self.om_prev + (Om - self.om_prev) * e_lag; gg = g["tau_motor"] / T0 * (1 - e_lag)           # partial inversion of the motor lag (gain 0 in v5 = off)
        om = np.clip(om_des + g["lag_gain"] * gg / (1 - gg) * (om_des - om_next), OM_MIN, OM_MAX); self.om_prev = om.copy(); return om

class ArmController:
    """Joint-space controller: IK reference of the pad trajectory under the fixed mounting assumption, joint PD + feedforward of the arm's own model.  compute(t, q, qd, ref, a_ref): ref = its own reference dict from the task
    (pad position p, velocity v, gap, contact flag), a_ref = reference acceleration -> joint torques (3)."""
    def __init__(self, gains, period):
        self.g, self.period = gains, period; self.pr = AMPred3(); self.dec = AMPred3(coupling="decoupled"); self.m = self.pr.m_tot; self.k = C["contact"]["k_N_m"]
    def reset(self, q0): self.q_ref = np.asarray(q0, float).copy()
    def compute(self, t, q, qd, ref, a_ref):
        g = self.g; gap = ref["gap"]; F_ref = self.k * max(-gap, 0.0) if gap is not None else 0.0; Fc = np.array([-F_ref, 0, 0])          # wall force the arm itself intends to apply (its own contact reference)
        th = np.arctan(F_ref / (self.m * G0)); Rn_T = Rmat(quat_from_rp(0.0, th)).T; tgt = Rn_T @ (ref["p"] - BASE)          # fixed mounting assumption: drone at BASE with the pitch implied by the arm's own contact reference
        for _ in range(8): self.q_ref = self.q_ref + np.linalg.solve(np.array(self.pr.Jpad_f(self.q_ref)), tgt - np.array(self.pr.fk_f(self.q_ref)).ravel())
        Jb = np.array(self.pr.Jpad_f(self.q_ref)); qd_ref = np.linalg.solve(Jb, Rn_T @ ref["v"]); eps = 1e-5
        dJ = (np.array(self.pr.Jpad_f(self.q_ref + eps * qd_ref)) - np.array(self.pr.Jpad_f(self.q_ref - eps * qd_ref))) / (2 * eps); qdd_ref = np.linalg.solve(Jb, Rn_T @ a_ref - dJ @ qd_ref)
        xr = np.zeros(23); xr[3:7] = quat_from_rp(0.0, th); xr[13:16] = self.q_ref; xr[16:19] = qd_ref          # model terms at the reference
        Md = np.array(self.dec.M_f(xr))[6:9, 6:9]; h = (np.array(self.dec.bias_f(xr)) + np.array(self.dec.grav_f(xr))).ravel()[6:9]; JC = np.array(self.pr.JC_f(xr)[0])
        tau = Md @ qdd_ref + h + self.pr.D * qd_ref - JC[:, 6:9].T @ Fc + g["kp_joint"] * (self.q_ref - q) + g["kd_joint"] * (qd_ref - qd)
        return np.clip(tau, -C["servo"]["continuous_torque_Nm"], C["servo"]["continuous_torque_Nm"])

class BaselineB:
    name = "B"; period = 0.005; task_uses_state = False          # task_uses_state False: the shared task receives no drone-derived pad gap (see Episode)
    def __init__(self, gains=None):
        self.g = dict(GAINS, **(gains or {})); self.drone = DroneController(self.g, self.period); self.arm = ArmController(self.g, self.period)
    def reset(self, x0, u0, task): self.drone.reset(np.r_[x0[0:13], x0[19:23]]); self.arm.reset(x0[13:16])
    def held(self, u): self.drone.held(u[:4])
    def _a_ref(self, task): h = task.horizon(1, self.period, self.period); return (h[1]["v"] - h[0]["v"]) / self.period
    def compute(self, t, xm, Fm, task):
        om = self.drone.compute(t, np.r_[xm[0:13], xm[19:23]]); tau = self.arm.compute(t, xm[13:16], xm[16:19], task.ref(), self._a_ref(task)); return np.r_[om, tau], {}
