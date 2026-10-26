"""N with a model-correction provider (Plan s8): the NMPC of am3_ctrl_n with the two prediction-correction channels d = (F_hat, tau_hat) supplied each cycle by a corrector (zero -> identical to N, classical EKF -> D,
learned policy -> L).  The correction is held over the horizon, rate- and magnitude-limited identically for every provider (am3_nmpc.D_RATE / D_MAX), and enters only the prediction (never a plant force or a motor residual).
Frozen files are untouched: this is a new module."""
import numpy as np
from ocrl.am3_ctrl_n import NMPC
from ocrl.am3_nmpc import D_MAX, D_RATE

class ZeroCorrector:
    def reset(self): pass
    def update(self, xm, Fm, u_applied, contact, **ctx): return np.zeros(2)

class CorrectedNMPC(NMPC):
    def __init__(self, corrector, name, **kw): super().__init__(name=name, **kw); self.corrector = corrector
    def reset(self, x0, u0, task): super().reset(x0, u0, task); self.corrector.reset(); self.u_old = np.asarray(u0, float).copy(); self.d = np.zeros(2); self.last_info = {}
    def compute(self, t, xm, Fm, task):
        d = np.asarray(self.corrector.update(xm, Fm, self.u_old, task.contact, task=task, d_prev=self.d, info=self.last_info), float); self.d = np.clip(np.clip(d, self.d - D_RATE, self.d + D_RATE), -D_MAX, D_MAX)
        self.u_old = self.u_pend.copy(); u, info = super().compute(t, xm, Fm, task); info["d"] = self.d.copy(); self.last_info = info; return u, info
    def held(self, u): super().held(u)
