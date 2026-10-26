"""Baseline B (Plan s7.1): competent coupling-neglecting controller on the same plant, sensors, actuator limits and common contact policy.  It runs at its own natural rate (200 Hz, one-period latency): its
computation takes microseconds, whereas the NMPC needs most of a 20 ms period.
Base: geometric hover controller: position PID with the correct total mass and gravity support, SO(3) attitude PD+I about the declared nominal locked-posture COM with its inertia, moment trim initialised from rotor
telemetry at hover, kinematic prediction over the command latency, partial inversion of the motor lag, rotor allocation.
Arm: separate joint-space control: position-only IK of the pad reference at the nominal base pose, joint PD plus feedforward from the arm's OWN model (joint block of the decoupled predictor: inertia, gravity,
velocity terms at the reference), slow task-space correction of the pad reference from the measured pad position.  Both receive the nominal wall force implied by the contact policy as feedforward.
B omits the arm-base inertial coupling (M_bq, reaction of arm motion on the base, configuration-dependent COM shift beyond the nominal posture) and optimises nothing jointly; hover errors it causes are left to feedback."""
import numpy as np
from ocrl.am3_pred import AMPred3, C, G0, quat2R
from ocrl.am3_task import BASE
from ocrl.am3_ref import Equilibrium, quat_from_rp
from ocrl.am3_plant import OM_MIN, OM_MAX, KF

GAINS = dict(kp_pos=7.6669, kd_pos=5.6958, ki_pos=1.5503, w_att=11.5931, zeta_att=1.0, ki_att=1160.701, i_pos_max=0.3, i_att_max=0.9136, lag_gain=0.8762, tau_motor=0.05, kp_joint=1.5695, kd_joint=0.0161, k_corr=0.1653, corr_max=0.01)  # B v3: config/am3_b_gains_v3.json

def vee(S): return np.array([S[2, 1], S[0, 2], S[1, 0]])
def hat(a): return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
def expm(w):
    th = np.linalg.norm(w)
    if th < 1e-12: return np.eye(3)
    K = hat(w / th); return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
def Rmat(Q): return np.array(quat2R(Q))

class BaselineB:
    name = "B"; period = 0.005
    def __init__(self, gains=None):
        self.g = dict(GAINS, **(gains or {})); self.pr = AMPred3(); self.dec = AMPred3(coupling="decoupled"); P = C["propulsion"]; self.m = self.pr.m_tot; self.t_lat = 1.5 * self.period
        rot = np.array(P["rotor_xy_m"]); s = np.array(P["spin_reaction_sign"]); self.A = np.vstack([np.ones(4), rot[:, 1], -rot[:, 0], s * P["km_over_kf_m"]]); self.Ainv = np.linalg.inv(self.A)          # [T, Mx, My, Mz] = A f
        e = Equilibrium(self.pr, BASE).solve(np.array(self.pr.fk_f(np.radians([0, -37, -78]))).ravel() + BASE, [0, 0, 0]); Jo = np.array(self.pr.M_f(e["x"]))[3:6, 3:6]
        self.rc = np.array(self.pr.com_f(e["x"])).ravel() - BASE; self.Jn = Jo - self.m * (self.rc @ self.rc * np.eye(3) - np.outer(self.rc, self.rc))          # declared nominal locked-posture COM and inertia about the COM
        self.k = C["contact"]["k_N_m"]; self.reset(e["x"], e["u"], None)
    def reset(self, x0, u0, task):
        self.ip = np.zeros(3); self.ia = np.zeros(3); self.corr = np.zeros(3); self.q_ref = np.asarray(x0[13:16], float).copy(); self.t_prev = None; self.om_prev = np.asarray(x0[19:23], float).copy()
        w = self.A @ (KF * np.asarray(x0[19:23]) ** 2); self.Mbias = w[1:4] - np.cross(self.rc, [0, 0, w[0]])          # trim from rotor telemetry: moment the rotors deliver beyond the nominal-COM model
    def held(self, u): self.om_prev = np.asarray(u[:4], float).copy()
    def _a_ref(self, task): h = task.horizon(1, self.period, self.period); return (h[1]["v"] - h[0]["v"]) / self.period
    def compute(self, t, xm, Fm, task):
        g, T0 = self.g, self.period; dt = T0 if self.t_prev is None else t - self.t_prev; self.t_prev = t; ref = task.ref(); gap = ref["gap"]
        F_ff = self.k * max(-gap, 0.0) if gap is not None else 0.0; Fc = np.array([-F_ff, 0, 0])          # wall force implied by the contact policy's indentation reference (spring law), whether or not contact is detected yet
        # ---- base: geometric position/attitude control on the state predicted over the command latency
        R = Rmat(xm[3:7]) @ expm(xm[10:13] * self.t_lat); ep = xm[0:3] + self.t_lat * xm[7:10] - BASE
        self.ip = np.clip(self.ip + ep * dt, -g["i_pos_max"], g["i_pos_max"]); a_d = -g["kp_pos"] * ep - g["kd_pos"] * xm[7:10] - g["ki_pos"] * self.ip
        Fd = self.m * (a_d + np.array([0, 0, G0])) - Fc; b3 = Fd / np.linalg.norm(Fd); b2 = np.cross(b3, [1.0, 0, 0]); b2 /= np.linalg.norm(b2); Rd = np.column_stack([np.cross(b2, b3), b2, b3])
        eR = 0.5 * vee(Rd.T @ R - R.T @ Rd); self.ia = np.clip(self.ia + eR * dt, -g["i_att_max"], g["i_att_max"]); w = xm[10:13]; J = self.Jn
        Mb = -J @ (g["w_att"] ** 2 * eR + 2 * g["zeta_att"] * g["w_att"] * w + g["ki_att"] * self.ia) + np.cross(w, J @ w); T = Fd @ R[:, 2]; Mb = Mb + np.cross(self.rc, [0, 0, T]) + self.Mbias          # moment about the COM -> about the thrust-plane origin
        f = np.clip(self.Ainv @ np.r_[T, Mb], C["propulsion"]["f_min_N"], C["propulsion"]["f_max_N"]); om_des = np.sqrt(f / KF)
        e_lag = np.exp(-T0 / g["tau_motor"]); om_next = self.om_prev + (xm[19:23] - self.om_prev) * e_lag; gg = g["tau_motor"] / T0 * (1 - e_lag)           # rotor speed when the new command starts; mean-speed factor over a period
        om = np.clip(om_des + g["lag_gain"] * gg / (1 - gg) * (om_des - om_next), OM_MIN, OM_MAX); self.om_prev = om.copy()
        # ---- arm: IK references at the nominal base pose, joint PD + feedforward of the arm's own model, slow task-space correction
        th = np.arctan(F_ff / (self.m * G0)); Rn_T = Rmat(quat_from_rp(0.0, th)).T; pad = np.array(self.pr.pad_f(xm)[0]).ravel(); e_pad = ref["p"] - pad
        self.corr = np.clip(self.corr + g["k_corr"] * e_pad * dt, -g["corr_max"], g["corr_max"]); tgt = Rn_T @ (ref["p"] + self.corr - BASE)
        for _ in range(8): self.q_ref = self.q_ref + np.linalg.solve(np.array(self.pr.Jpad_f(self.q_ref)), tgt - np.array(self.pr.fk_f(self.q_ref)).ravel())
        Jb = np.array(self.pr.Jpad_f(self.q_ref)); qd_ref = np.linalg.solve(Jb, Rn_T @ ref["v"]); eps = 1e-5
        dJ = (np.array(self.pr.Jpad_f(self.q_ref + eps * qd_ref)) - np.array(self.pr.Jpad_f(self.q_ref - eps * qd_ref))) / (2 * eps); qdd_ref = np.linalg.solve(Jb, Rn_T @ self._a_ref(task) - dJ @ qd_ref)
        xr = np.zeros(23); xr[3:7] = quat_from_rp(0.0, th); xr[13:16] = self.q_ref; xr[16:19] = qd_ref          # model terms at the reference, not at the delayed measurement
        Md = np.array(self.dec.M_f(xr))[6:9, 6:9]; h = (np.array(self.dec.bias_f(xr)) + np.array(self.dec.grav_f(xr))).ravel()[6:9]; JC = np.array(self.pr.JC_f(xr)[0])
        tau = Md @ qdd_ref + h + self.pr.D * qd_ref - JC[:, 6:9].T @ Fc + g["kp_joint"] * (self.q_ref - xm[13:16]) + g["kd_joint"] * (qd_ref - xm[16:19])
        tau = np.clip(tau, -C["servo"]["continuous_torque_Nm"], C["servo"]["continuous_torque_Nm"]); return np.r_[om, tau], dict(T=T, pad_err=e_pad, base_err=ep)
