"""Classical model correction D (Plan s8): augmented error-state EKF estimating the two correction channels d = (F_hat, tau_hat) with the shared sensors.
Error state (24) = [dp, dtheta (right attitude perturbation), dv, domega, dq, dqd, dOmega, dd(2)]; random-walk correction; the force sensor reads F_model(x) + F_hat (an unmodelled extra normal force on the pad is
exactly what the F_hat channel represents).  The estimator state is internal: the NMPC of every variant uses the same raw measured state; only d_hat is passed on (zero = nominal).  Fixed conventions: multiplicative right
attitude error, three-dimensional attitude innovation, inverse innovation covariance in the gain, covariance reset after attitude injection."""
import numpy as np, casadi as ca
from ocrl.am3_episode import Sensors

def tangent_basis(Q):
    """T (23x22): d x = T d xi with xi = (dp, dtheta_body, dv, dw, dq, dqd, dOmega)."""
    w, x, y, z = Q; L = np.array([[w, -x, -y, -z], [x, w, -z, y], [y, z, w, -x], [z, -y, x, w]]); T = np.zeros((23, 22)); T[0:3, 0:3] = np.eye(3); T[3:7, 3:6] = 0.5 * L[:, 1:4]; T[7:23, 6:22] = np.eye(16); return T

class ClassicalCorrector:
    def __init__(self, pred, step_d, h=0.02, q_state=None, q_d=(0.01, 0.001), p0_d=(0.1, 0.02), channels=(1, 1)):
        self.pr, self.step_d, self.h = pred, step_d, h; x = ca.MX.sym("x", 23); u = ca.MX.sym("u", 7); d = ca.MX.sym("d", 2); f = pred.xdot_mx()
        self.jac = ca.Function("ekf_jac", [x, u, d], [ca.jacobian(f(x, u, d), x), ca.jacobian(f(x, u, d), d)]); g, vc, Fn, ft, vC = pred.contact_f(x, ca.DM.zeros(7), ca.DM.zeros(2)); self.Fn = ca.Function("ekf_Fn", [x], [Fn, ca.jacobian(Fn, x)])
        S = Sensors.SIG; self.Rdiag = np.r_[[S["p"]] * 3, [S["att"]] * 3, [S["v"]] * 3, [S["w"]] * 3, [S["q"]] * 3, [S["qd"]] * 3, [S["Om"]] * 4, S["F"]] ** 2
        qs = q_state if q_state is not None else np.r_[[1.5e-4] * 3, [1e-3] * 3, [5e-3] * 3, [8e-2] * 3, [3e-3] * 3, [0.26] * 3, [1.3] * 4]          # per-cycle process-noise std: replaced by the empirical one-step residual RMS (scripts/am3_ekf_tune.py)
        self.Qx = np.asarray(qs) ** 2; self.Qd = np.array(q_d) ** 2; self.P0d = np.array(p0_d) ** 2; self.mask = np.array(channels, float); self.reset()
    def reset(self): self.xh = None; self.dh = np.zeros(2); self.P = None; self.u_prev = None; self.nis = []
    def update(self, xm, Fm, u_apply, contact, **ctx):
        h = self.h
        if self.xh is None:
            self.xh = xm.copy(); self.P = np.diag(np.r_[1e-6 * np.ones(18), 1.0 * np.ones(4), self.P0d]); self.u_prev = np.asarray(u_apply, float).copy(); return self.dh * self.mask
        A, Bd = (np.array(a) for a in self.jac(self.xh, self.u_prev, self.dh)); T = tangent_basis(self.xh[3:7]); Tp = np.linalg.pinv(T)
        Ad = Tp @ (np.eye(23) + h * A) @ T; Bdd = Tp @ (h * Bd); F = np.block([[Ad, Bdd], [np.zeros((2, 22)), np.eye(2)]]); self.P = F @ self.P @ F.T + np.diag(np.r_[self.Qx, self.Qd])
        xp = np.array(self.step_d(self.xh, self.u_prev, self.dh)).ravel(); Fmod, JF = self.Fn(xp); Fmod = float(Fmod); JF = np.array(JF).ravel() @ T
        Qh, Qm = xp[3:7], xm[3:7]; qe = np.array([Qh[0] * Qm[1] - Qh[1] * Qm[0] - Qh[2] * Qm[3] + Qh[3] * Qm[2], Qh[0] * Qm[2] + Qh[1] * Qm[3] - Qh[2] * Qm[0] - Qh[3] * Qm[1], Qh[0] * Qm[3] - Qh[1] * Qm[2] + Qh[2] * Qm[1] - Qh[3] * Qm[0]]) * 2 * np.sign(Qh @ Qm)
        contact = bool(contact) or Fm > 0.05
        if not contact: self.dh[0] = 0.0; self.P[22, :] = 0.0; self.P[:, 22] = 0.0; self.P[22, 22] = self.P0d[0]          # F_hat is a contact-force correction: not estimated while the pad is free
        z = np.r_[xm[0:3] - xp[0:3], qe, xm[7:23] - xp[7:23], Fm - (Fmod + self.dh[0])]; H = np.zeros((23, 24)); H[:22, :22] = np.eye(22); H[22, :22] = JF; H[22, 22] = 1.0 if contact else 0.0
        Rm = np.diag(self.Rdiag); S = H @ self.P @ H.T + Rm; K = self.P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0])); dx = K @ z; IKH = np.eye(24) - K @ H; self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T; self.nis.append(float(z @ np.linalg.solve(S, z)))
        dth = dx[3:6]; ang = np.linalg.norm(dth); dq = np.r_[np.cos(ang / 2), np.sin(ang / 2) * dth / ang] if ang > 1e-12 else np.r_[1.0, 0.5 * dth]; Q = xp[3:7]
        Qn = np.array([Q[0] * dq[0] - Q[1] * dq[1] - Q[2] * dq[2] - Q[3] * dq[3], Q[0] * dq[1] + Q[1] * dq[0] + Q[2] * dq[3] - Q[3] * dq[2], Q[0] * dq[2] - Q[1] * dq[3] + Q[2] * dq[0] + Q[3] * dq[1], Q[0] * dq[3] + Q[1] * dq[2] - Q[2] * dq[1] + Q[3] * dq[0]]); Qn /= np.linalg.norm(Qn)
        xn = xp.copy(); xn[0:3] += dx[0:3]; xn[3:7] = Qn; xn[7:23] += dx[6:22]; self.xh = xn; self.dh = self.dh + dx[22:24]
        G = np.eye(24); G[3:6, 3:6] = np.eye(3) - 0.5 * np.array([[0, -dth[2], dth[1]], [dth[2], 0, -dth[0]], [-dth[1], dth[0], 0]]); self.P = G @ self.P @ G.T; self.u_prev = np.asarray(u_apply, float).copy(); return self.dh * self.mask
