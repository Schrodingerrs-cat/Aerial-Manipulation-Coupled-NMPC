"""NMPC with a state filter "NF" (2026-11-19): controller N fed a filtered state instead of the raw measurement.
Under realistic sensor noise N re-solves on raw measurements and its rotor commands jump ~360 rad/s per step (Findings/RL/04, Smoothness).
Filter: fixed-gain predictor-corrector (steady-state Kalman form) with N's own predictor as the model:
    x_pred = step(x_hat, u_applied);   x_hat = x_pred + alpha * (x_meas - x_pred)
alpha in (0, 1]: 1 = raw measurement (N exactly), smaller = more filtering.  Costs, constraints and solver are N's (frozen files unchanged)."""
import numpy as np
from ocrl.am3_ctrl_n import NMPC


class NMPCF(NMPC):
    def __init__(self, alpha=0.3):
        super().__init__("full"); self.name = "NF"; self.alpha = float(alpha)

    def reset(self, x0, u0, task):
        super().reset(x0, u0, task); self.xh = None; self.u_act = self.u_pend.copy()

    def filter(self, xm):
        if self.xh is None: self.xh = np.asarray(xm, float).copy(); return self.xh
        xp = np.array(self.form.step(self.xh, self.u_act)).ravel()
        qm = xm[3:7] if np.dot(xm[3:7], xp[3:7]) >= 0 else -xm[3:7]                       # same quaternion hemisphere before blending
        xh = xp + self.alpha * (np.r_[xm[0:3], qm, xm[7:]] - xp); xh[3:7] /= np.linalg.norm(xh[3:7]); self.xh = xh; return xh

    def compute(self, t, xm, Fm, task):
        xh = self.filter(np.asarray(xm, float)); self.u_act = self.u_pend.copy()            # u_pend acts on the coming period
        u, info = super().compute(t, xh, Fm, task); info["xh_minus_xm_p_mm"] = float(1e3 * np.linalg.norm(xh[0:3] - xm[0:3])); return u, info
