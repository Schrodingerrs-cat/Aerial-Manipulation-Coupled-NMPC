"""Load-aware tracker: time-varying LQR around a plan, from the same model as the planner (qp_ocp.model: drone + load + attitude loop + motor lag).
State x (17) = [p(3), a, b, pd(3), ad, bd, phi, theta, phid, thetad, angular accel(2), T] (a, b = swing angles), input = commanded thrust vector.  100 Hz, RK4.
Measured: drone and load states, tilt and tilt rate (attitude, gyro).  Not measured: the motor-lag state and T; the controller runs the model's
own equations for those on its commands.  After the plan it holds the end point with the steady-state LQR, which also damps the swing."""
import numpy as np, casadi as ca
from ocrl.qp_ocp import model, rest, tilt_cmd, WN, ZETA, TAU_M, TAU_T, NX
from ocrl.qp_plant import L

DT = 0.01
Q = np.diag([60, 60, 60, 10, 10, 8, 8, 8, 1, 1, 0.1, 0.1, 0.01, 0.01, 1e-4, 1e-4, 0.01]); R = np.eye(3) * 0.1


def measure(s):
    """Swing angles, tilt and their rates from the measured drone and load states."""
    d = (s["r"] - s["p"]) / L; dd = (s["rv"] - s["v"]) / L; a, b = np.arctan2(d[0], -d[2]), np.arcsin(np.clip(d[1], -1, 1))
    Jd = np.array([[np.cos(a) * np.cos(b), -np.sin(a) * np.sin(b)], [0.0, np.cos(b)], [np.sin(a) * np.cos(b), np.cos(a) * np.sin(b)]])
    z = s["R"][:, 2]; zd = s["R"] @ np.array([s["w"][1], -s["w"][0], 0.0]); ph, th = np.arctan2(-z[1], np.hypot(z[0], z[2])), np.arctan2(z[0], z[2])
    Jz = np.array([[-np.sin(th) * np.sin(ph), np.cos(th) * np.cos(ph)], [-np.cos(ph), 0.0], [-np.cos(th) * np.sin(ph), -np.sin(th) * np.cos(ph)]])
    return np.r_[s["p"], a, b, s["v"], np.linalg.lstsq(Jd, dd, rcond=None)[0], ph, th, np.linalg.lstsq(Jz, zd, rcond=None)[0]]


class Estimator:
    """Unmeasured model states (angular accel, T), propagated with the model's own equations on the applied commands."""
    def __init__(self, x0): self.z = np.array(x0[14:17], float)

    def full(self, s): return np.r_[measure(s), self.z]

    def update(self, x, f):
        acmd = WN ** 2 * (np.array(tilt_cmd(f, np)) - x[10:12]) - 2 * ZETA * WN * x[12:14]
        self.z[:2] = acmd + (self.z[:2] - acmd) * np.exp(-DT / TAU_M); self.z[2] = np.linalg.norm(f) + (self.z[2] - np.linalg.norm(f)) * np.exp(-DT / TAU_T)


def discrete():
    ode = model(); x = ca.SX.sym("x", NX); u = ca.SX.sym("u", 3)
    k1 = ode(x, u); k2 = ode(x + DT / 2 * k1, u); k3 = ode(x + DT / 2 * k2, u); k4 = ode(x + DT * k3, u); xn = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return ca.Function("AB", [x, u], [ca.jacobian(xn, x), ca.jacobian(xn, u)])


class TVLQR:
    def __init__(self, x_ref, f_ref, t_end):
        AB = discrete(); A, B = (np.array(m) for m in AB(x_ref[-1], f_ref[-1])); P = Q.copy()
        for _ in range(3000): K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A); P = Q + A.T @ P @ (A - B @ K)
        self.K_end, self.P_end = K, P.copy(); self.K = [None] * len(x_ref)
        for k in range(len(x_ref) - 1, -1, -1):                                             # backward Riccati along the plan
            A, B = (np.array(m) for m in AB(x_ref[k], f_ref[k])); K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A); P = Q + A.T @ P @ (A - B @ K); self.K[k] = K
        self.x_ref, self.f_ref, self.t_end, self.est = x_ref, f_ref, t_end, Estimator(x_ref[0])

    def __call__(self, t, s):
        k = min(int(round(t / DT)), len(self.x_ref) - 1); x = self.est.full(s); K = self.K[k] if t <= self.t_end else self.K_end
        f = self.f_ref[k] - K @ (x - self.x_ref[k]); self.est.update(x, f); return f


def resample(pl):
    """A qp_ocp plan on the 100 Hz grid: states and thrust commands."""
    t = np.arange(0, pl["T"] + 1e-9, DT); I = lambda Y: np.column_stack([np.interp(t, pl["t"], Y[:, j]) for j in range(Y.shape[1])])
    return I(pl["x"]), I(pl["f"])


def from_plan(pl):
    return TVLQR(*resample(pl), pl["T"])
