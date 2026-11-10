"""Load-aware tracking NMPC (acados, SQP-RTI) around a qp_ocp plan: same model as the planner and the TVLQR (qp_ocp.model, state layout in qp_lqr).
Input = commanded thrust vector.  Horizon N steps of H s, called at 100 Hz.
Cost: same Q, R as qp_lqr (rescaled per step), terminal cost = the LQR's steady-state Riccati matrix at B.
Hard limits (same as the planner): thrust norm in [F_MIN, F_MAX], tilt under TILT_MAX.  Unmeasured states from qp_lqr.Estimator, as in the TVLQR."""
import os, ctypes, hashlib, numpy as np, casadi as ca
from ocrl.qp_ocp import model, F_MIN, F_MAX, TILT_MAX, NX
from ocrl.qp_lqr import Q, R, DT, Estimator, resample, TVLQR
HERE = os.path.dirname(os.path.abspath(__file__)); GEN = os.path.join(HERE, "..", "results", "acados_codegen")
N, H = 30, 0.04                                                   # 1.2 s look-ahead (pendulum period 1.43 s)


def solver(P_end):
    lib = os.path.abspath(os.path.join(HERE, "..", "third_party", "acados", "lib")); os.environ["ACADOS_SOURCE_DIR"] = os.path.dirname(lib)
    for f in ("libqpOASES_e.so", "libblasfeo.so", "libhpipm.so", "libacados.so"): ctypes.CDLL(os.path.join(lib, f), mode=ctypes.RTLD_GLOBAL)
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
    x = ca.SX.sym("x", NX); u = ca.SX.sym("u", 3)
    m = AcadosModel(); m.name = "qp_nmpc"; m.x, m.u = x, u; m.f_expl_expr = model()(x, u)
    n2 = ca.sumsqr(u); m.con_h_expr = ca.vertcat(n2, u[2] - np.cos(TILT_MAX) * ca.sqrt(n2 + 1e-9))
    ocp = AcadosOcp(); ocp.model = m; so = ocp.solver_options; so.N_horizon, so.tf = N, N * H
    ocp.cost.cost_type, ocp.cost.cost_type_e = "LINEAR_LS", "LINEAR_LS"
    ocp.cost.Vx = np.vstack([np.eye(NX), np.zeros((3, NX))]); ocp.cost.Vu = np.vstack([np.zeros((NX, 3)), np.eye(3)]); ocp.cost.Vx_e = np.eye(NX)
    ocp.cost.W = np.diag(np.r_[np.diag(Q), np.diag(R)]) * (H / DT); ocp.cost.W_e = P_end; ocp.cost.yref = np.zeros(NX + 3); ocp.cost.yref_e = np.zeros(NX)
    so.cost_scaling = np.ones(N + 1)                               # weights already per step
    ocp.constraints.lh = np.array([F_MIN ** 2, 0.0]); ocp.constraints.uh = np.array([F_MAX ** 2, 1e3]); ocp.constraints.x0 = np.zeros(NX)
    so.integrator_type = "ERK"; so.sim_method_num_stages = 4; so.sim_method_num_steps = 2
    so.nlp_solver_type = "SQP_RTI"; so.hessian_approx = "GAUSS_NEWTON"; so.qp_solver = "PARTIAL_CONDENSING_HPIPM"; so.qp_solver_iter_max = 100; so.print_level = 0
    key = hashlib.sha1(open(__file__, "rb").read() + open(os.path.join(HERE, "qp_ocp.py"), "rb").read() + P_end.tobytes()).hexdigest()[:10]; d = os.path.abspath(os.path.join(GEN, f"qp_nmpc_{key}")); os.makedirs(d, exist_ok=True); ocp.code_export_directory = d
    built = os.path.exists(os.path.join(d, "built")); s = AcadosOcpSolver(ocp, json_file=os.path.join(d, "qp_nmpc.json"), build=not built, generate=not built); open(os.path.join(d, "built"), "w").write("ok")
    return s


class NMPC:
    def __init__(self, x_ref, f_ref, P_end):
        """x_ref, f_ref on the 100 Hz grid (qp_lqr.from_plan layout); after the end they hold the last point."""
        self.x_ref, self.f_ref, self.s = x_ref, f_ref, solver(P_end); self.est = Estimator(x_ref[0]); self.status = []; self.warm = False

    def ref(self, t):
        k = min(int(round(t / DT)), len(self.x_ref) - 1); return self.x_ref[k], self.f_ref[k]

    def __call__(self, t, s):
        x0 = self.est.full(s); S = self.s; S.set(0, "lbx", x0); S.set(0, "ubx", x0)
        for j in range(N + 1):
            xr, fr = self.ref(t + j * H)
            if not self.warm: S.set(j, "x", xr)
            if j < N:
                S.set(j, "yref", np.r_[xr, fr])
                if not self.warm: S.set(j, "u", fr)
            else: S.set(j, "yref", xr)
        self.status.append(S.solve()); self.warm = True; f = S.get(0, "u")
        self.est.update(x0, f); return f


def from_plan(pl):
    x, f = resample(pl); return NMPC(x, f, TVLQR(x, f, pl["T"]).P_end)          # terminal cost = the LQR's Riccati matrix at B
