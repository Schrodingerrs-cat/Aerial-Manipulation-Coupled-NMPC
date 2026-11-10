"""Load-aware tracker: time-varying LQR around a plan, from the same drone+load model as the planner (qp_ocp: dynamics and thrust lag TAU_F).
State x = [p(3), a, b, pd(3), ad, bd, f_act(3)] (a, b = swing angles of the string), input = commanded thrust vector.  100 Hz, RK4 discretisation.
f_act is not measured: the controller runs the same first-order lag on its own commands.  After the plan it holds the end point with the
steady-state LQR, which also damps the swing."""
import numpy as np, casadi as ca
from ocrl.qp_ocp import dynamics, TAU_F
from ocrl.qp_plant import L

DT = 0.01
Q = np.diag([60, 60, 60, 10, 10, 8, 8, 8, 1, 1, 0.01, 0.01, 0.01]); R = np.eye(3) * 0.1          # 0.01-0.03 end in a small limit cycle (the attitude loop is not exactly first order); 0.1 is the stiffest stable setting tested


def swing_state(s):
    """Swing angles and rates from the measured drone and load states."""
    d = (s["r"] - s["p"]) / L; dd = (s["rv"] - s["v"]) / L; a, b = np.arctan2(d[0], -d[2]), np.arcsin(np.clip(d[1], -1, 1))
    Jd = np.array([[np.cos(a) * np.cos(b), -np.sin(a) * np.sin(b)], [0.0, np.cos(b)], [np.sin(a) * np.cos(b), np.cos(a) * np.sin(b)]])
    return np.r_[s["p"], a, b, s["v"], np.linalg.lstsq(Jd, dd, rcond=None)[0]]


class TVLQR:
    def __init__(self, x_ref, f_ref, t_end):
        dyn = dynamics(False); x = ca.SX.sym("x", 13); u = ca.SX.sym("u", 3); ode = lambda x: ca.vertcat(x[5:10], dyn(x[:5], x[5:10], x[10:13]), (u - x[10:13]) / TAU_F)
        k1 = ode(x); k2 = ode(x + DT / 2 * k1); k3 = ode(x + DT / 2 * k2); k4 = ode(x + DT * k3); xn = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        AB = ca.Function("AB", [x, u], [ca.jacobian(xn, x), ca.jacobian(xn, u)])
        A, B = (np.array(m) for m in AB(x_ref[-1], f_ref[-1])); P = Q.copy()
        for _ in range(3000): K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A); P = Q + A.T @ P @ (A - B @ K)
        self.K_end = K; self.K = [None] * len(x_ref)
        for k in range(len(x_ref) - 1, -1, -1):                                             # backward Riccati along the plan
            A, B = (np.array(m) for m in AB(x_ref[k], f_ref[k])); K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A); P = Q + A.T @ P @ (A - B @ K); self.K[k] = K
        self.x_ref, self.f_ref, self.t_end, self.f_act = x_ref, f_ref, t_end, x_ref[0][10:13].copy()

    def __call__(self, t, s):
        k = min(int(round(t / DT)), len(self.x_ref) - 1); x = np.r_[swing_state(s), self.f_act]; K = self.K[k] if t <= self.t_end else self.K_end
        f = self.f_ref[k] - K @ (x - self.x_ref[k]); self.f_act = f + (self.f_act - f) * np.exp(-DT / TAU_F); return f


def from_plan(pl):
    """Resample a qp_ocp plan onto the 100 Hz grid."""
    t = np.arange(0, pl["T"] + 1e-9, DT); I = lambda Y: np.column_stack([np.interp(t, pl["t"], Y[:, j]) for j in range(Y.shape[1])])
    x = np.column_stack([I(pl["p"]), I(pl["swing"]), I(pl["v"]), I(pl["swing_d"]), I(pl["f_act"])]); return TVLQR(x, I(pl["f"]), pl["T"])
