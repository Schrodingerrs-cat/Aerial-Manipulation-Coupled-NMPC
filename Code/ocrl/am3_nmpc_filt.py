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


from ocrl.am3_nmpc_dob import NMPCD, K_DOB


class NMPCDF(NMPCD):
    """ND + the same filter, predicting with the observer's disturbance estimate (offset-free filter).  With the nominal model alone the filter
    is biased under model mismatch (NF lost control with thrust -10 %, payload, slow motors; 2026-11-19).  Observer runs on the raw measurement, as in ND."""
    def __init__(self, alpha=0.3, k=K_DOB, obs_on_filtered=False):
        super().__init__(k); self.name = "NDF"; self.alpha = float(alpha); self.obs_on_filtered = obs_on_filtered

    def reset(self, x0, u0, task):
        super().reset(x0, u0, task); self.xh = None

    def compute(self, t, xm, Fm, task):
        xm = np.asarray(xm, float)
        if self.xh is None: self.xh = xm.copy()
        else:
            xp = np.array(self.form.step_g(self.xh, self.u_pend_prev, self.dg)).ravel(); qm = xm[3:7] if np.dot(xm[3:7], xp[3:7]) >= 0 else -xm[3:7]
            self.xh = xp + self.alpha * (np.r_[xm[0:3], qm, xm[7:]] - xp); self.xh[3:7] /= np.linalg.norm(self.xh[3:7])
        self.u_pend_prev = self.u_pend.copy()
        self.observe(self.xh if self.obs_on_filtered else xm, self.u_act, float(Fm)); self.u_act = self.u_pend.copy()
        xh = np.array(self.form.step_g(self.xh, self.u_pend, self.dg)).ravel(); P = self.params(task); Xg = self.Xg.copy(); Xg[:, 0] = xh
        r = self.solver.solve(xh, self.u_pend, P, Xg, self.Ug); info = dict(solve_s=r["solve_s"], status=r["status"], ok=r["ok"], slack=r.get("slack_max", 0.0), fallback=False, iters=r.get("iters", -1), d=self.dg.copy())
        if not r["ok"]:
            self.n_fail += 1; self.solver.reset()
            r = self.solver.solve(xh, self.u_pend, P, np.tile(xh[:, None], (1, self.N + 1)), np.tile(self.u_pend[:, None], (1, self.N))); info.update(retry=True, ok=r["ok"], status=r["status"])
        if r["ok"]: u = r["U"][:, 0]; self.Xg = np.c_[r["X"][:, 1:], r["X"][:, -1]]; self.Ug = np.c_[r["U"][:, 1:], r["U"][:, -1]]
        else: u = self.u_pend.copy(); info["fallback"] = True
        self.u_pend = u.copy(); info["pad_pred_err"] = None; return u, info
