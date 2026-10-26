"""Static references from the predictor only (no MuJoCo queries): equilibrium with the base at its fixed hover position, the pad centre at a target point and the wall force Fc (on the robot, world) applied
at the pad contact point.  Unknowns z = [q(3), roll, pitch, Omega(4), tau(3)]; equations: nu_dot = 0 (9) and pad position (3).  Base yaw is zero."""
import numpy as np, casadi as ca
from scipy.optimize import least_squares
from ocrl.am3_pred import AMPred3

def quat_from_rp(roll, pitch):
    cr, sr, cp, sp = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2)      # R = Ry(pitch) Rx(roll)
    return np.array([cp * cr, cp * sr, sp * cr, -sp * sr])

class Equilibrium:
    def __init__(self, pr: AMPred3, base_pos):
        self.pr, self.base = pr, np.asarray(base_pos, float); xs = ca.SX.sym("x", 23); us = ca.SX.sym("u", 7); fs = ca.SX.sym("f", 3)
        self.res_f = ca.Function("res", [xs, us, fs], [ca.vertcat(pr.nud_force_f(xs, us, fs), pr.pad_f(xs)[0])]); self.last = None
    def unpack(self, z):
        x = np.zeros(23); x[0:3] = self.base; x[3:7] = quat_from_rp(z[3], z[4]); x[13:16] = z[0:3]; x[19:23] = z[5:9]; return x, np.r_[z[5:9], z[9:12]]
    def solve(self, pad_target, Fc, z0=None):
        """Fc: force on the robot at the pad contact point (world); the pad-centre target already includes the indentation."""
        tgt = np.asarray(pad_target, float); Fc = np.asarray(Fc, float)
        def fun(z): x, u = self.unpack(z); r = np.array(self.res_f(x, u, Fc)).ravel(); r[9:] = 1e3 * (r[9:] - tgt); return r
        guess = z0 if z0 is not None else self.last if self.last is not None else np.r_[0, np.radians(-37), np.radians(-78), 0, 0.04, [1200] * 4, 0, 0, 0]
        s = least_squares(fun, guess, x_scale=np.r_[0.05, 0.05, 0.05, 0.01, 0.01, [100] * 4, 0.1, 0.1, 0.1], xtol=1e-14, ftol=1e-14, gtol=1e-14); self.last = s.x; x, u = self.unpack(s.x)
        return dict(x=x, u=u, res=float(np.linalg.norm(s.fun)), z=s.x)
