"""Offset-free coupled NMPC "ND" (2026-11-12): controller N plus a generalized-momentum disturbance observer.  The estimate d (9: world force on the base,
body moment, joint torques) is what N's own predictor lacks (payload, thrust/lag error, wind, contact-model error); it enters the prediction as a constant over
the horizon:  M(x) nu' = rhs(x, u) + d.  Costs, constraints, weights and solver settings are N's (am3_nmpc, frozen, unchanged); only the dynamics get d.
Observer: same equations as InnerLoop.observe (ocrl/am3_rl_direct.py), with N's full predictor (contact model included) as the model."""
import os, time, hashlib, fcntl, numpy as np, casadi as ca
from ocrl.am3_pred import AMPred3, quat_rate
from ocrl.am3_nmpc import Formulation, AcadosNMPC, NP, NH, RHO, GEN_DIR, HERE, default_params
from ocrl.am3_ctrl_n import NMPC

ND = 9
K_DOB = np.array([10.0] * 6 + [0.0] * 3)            # 1/s: base force, body moment, joints (joint channel off, as in the RL inner loop)


class FormulationD(Formulation):
    def __init__(self, pred, N=10, h=0.02, nsub=4):
        super().__init__(pred, N, h, nsub); pr = pred
        x = ca.MX.sym("x", 23); u = ca.MX.sym("u", 7); d2 = ca.MX.sym("d2", 2); g = ca.MX.sym("g", ND)
        nud = ca.solve(pr.M_f(x), pr.rhs_f(x, u, d2) + g, "qr")
        f = ca.Function("xdot_g", [x, u, d2, g], [ca.vertcat(x[7:10], quat_rate(x[3:7], x[10:13]), nud[0:3], nud[3:6], x[16:19], nud[6:9], pr.Om_dot_f(x, u))])
        xs = x; hh = h / nsub
        for _ in range(nsub):
            k1 = f(xs, u, d2, g); k2 = f(xs + hh / 2 * k1, u, d2, g); k3 = f(xs + hh / 2 * k2, u, d2, g); k4 = f(xs + hh * k3, u, d2, g); xs = xs + hh / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        xs = ca.vertcat(xs[0:3], xs[3:7] / ca.norm_2(xs[3:7]), xs[7:23]); self.step_dg = ca.Function("step_dg", [x, u, d2, g], [xs])
        self.step_g = ca.Function("step_g", [x, u, g], [self.step_dg(x, u, ca.DM.zeros(2), g)])


class AcadosNMPCD(AcadosNMPC):
    """am3_nmpc.AcadosNMPC with the parameter vector extended by d (NP + 9); solve/reset inherited unchanged."""
    def __init__(self, form, rti=True, name=None, sqp_iters=30):
        import ctypes
        flavour = os.environ.get("OCRL_ACADOS", "acados")
        ACADOS_LIB = os.path.abspath(os.path.join(HERE, "..", "third_party", flavour, "lib")); os.environ["ACADOS_SOURCE_DIR"] = os.path.abspath(os.path.join(ACADOS_LIB, ".."))
        for lib in ("libqpOASES_e.so", "libblasfeo.so", "libhpipm.so", "libacados.so"): ctypes.CDLL(os.path.join(ACADOS_LIB, lib), mode=ctypes.RTLD_GLOBAL)
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        self.f, self.rti, N, h = form, rti, form.N, form.h; pr = form.pr; self.N = N
        self.Sx = np.r_[np.ones(19), [1000.0] * 4]; self.Su = np.r_[[1000.0] * 4, [0.5] * 3]
        zs = ca.MX.sym("z", 30); v = ca.MX.sym("v", 7); p = ca.MX.sym("p", NP + ND); x = zs[0:23] * ca.DM(self.Sx); up = zs[23:30] * ca.DM(self.Su); u = v * ca.DM(self.Su); pn = p[0:NP]
        res = form.res(x, u, up, pn); res_e = form.res_e(x, pn); hc = form.hcon(x, pn)
        m = AcadosModel(); m.name = name; m.x = zs; m.u = v; m.p = p; m.disc_dyn_expr = ca.vertcat(form.step_dg(x, u, pn[11:13], p[NP:]) / ca.DM(self.Sx), v); m.cost_y_expr = res; m.cost_y_expr_e = res_e
        m.con_h_expr_0 = hc; m.con_h_expr = hc; m.con_h_expr_e = hc
        ocp = AcadosOcp(); ocp.model = m; ocp.solver_options.N_horizon = N; ocp.solver_options.tf = N * h; ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = 2 * np.eye(res.shape[0]); ocp.cost.W_e = 2 * np.eye(res_e.shape[0]); ocp.cost.yref = np.zeros(res.shape[0]); ocp.cost.yref_e = np.zeros(res_e.shape[0])
        ocp.constraints.x0 = np.zeros(30); ocp.constraints.lbu = np.r_[[pr.om_min] * 4, [-pr.tau_c] * 3] / self.Su; ocp.constraints.ubu = np.r_[[pr.om_max] * 4, [pr.tau_c] * 3] / self.Su; ocp.constraints.idxbu = np.arange(7)
        for suf in ("_0", "", "_e"):
            setattr(ocp.constraints, "lh" + suf, -1e6 * np.ones(NH)); setattr(ocp.constraints, "uh" + suf, np.zeros(NH)); setattr(ocp.constraints, "idxsh" + suf, np.arange(NH))
            for z in ("zl", "zu"): setattr(ocp.cost, z + suf, RHO * np.ones(NH))
            for Z in ("Zl", "Zu"): setattr(ocp.cost, Z + suf, 2 * RHO * np.ones(NH))
        ocp.parameter_values = np.r_[default_params(), np.zeros(ND)]; so = ocp.solver_options; so.cost_scaling = np.ones(N + 1); so.integrator_type = "DISCRETE"
        so.nlp_solver_type = "SQP_RTI" if rti else "SQP"; so.hessian_approx = "GAUSS_NEWTON"; so.qp_solver = "PARTIAL_CONDENSING_HPIPM"; so.nlp_solver_max_iter = sqp_iters; so.qp_solver_iter_max = 100
        so.nlp_solver_tol_stat = so.nlp_solver_tol_eq = so.nlp_solver_tol_ineq = so.nlp_solver_tol_comp = 1e-4; so.globalization = "MERIT_BACKTRACKING" if not rti else "FIXED_STEP"; so.print_level = 0; so.levenberg_marquardt = 1e-4; so.ext_fun_compile_flags = "-O3 -march=native"
        src = b"".join(open(os.path.join(HERE, f), "rb").read() for f in ("am3_pred.py", "am3_nmpc.py", "am3_nmpc_dob.py")) + open(os.path.join(HERE, "..", "config", "am3_design_v3.json"), "rb").read() + repr((flavour, N, h, form.nsub, rti, sqp_iters, pr.k, pr.c, pr.mu, pr.coupling)).encode()
        d = os.path.abspath(os.path.join(GEN_DIR, f"{name}_{hashlib.sha1(src).hexdigest()[:12]}")); os.makedirs(d, exist_ok=True); ocp.code_export_directory = d; stamp = os.path.join(d, "built"); self.build_dir = d
        with open(os.path.join(GEN_DIR, ".build.lock"), "w") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX); fresh = os.path.exists(stamp)
            self.solver = AcadosOcpSolver(ocp, json_file=os.path.join(d, name + ".json"), build=not fresh, generate=not fresh); open(stamp, "w").write("ok")
        for key in ("qp_tol_stat", "qp_tol_eq", "qp_tol_ineq", "qp_tol_comp"): self.solver.options_set(key, 1e-8)


class NMPCD(NMPC):
    """N + disturbance observer.  k = observer gains (1/s, 9); k = 0 gives N's behaviour with the extended solver (parity check)."""
    def __init__(self, k=K_DOB, N=10, h=0.02, nsub=4):
        self.name = "ND"; self.N, self.h = N, h; self.form = FormulationD(AMPred3(coupling="full"), N=N, h=h, nsub=nsub)
        self.solver = AcadosNMPCD(self.form, name=f"am3_nd_N{N}"); self.is_acados = True; self.d = np.zeros(2); self.k = np.asarray(k, float); self.pr = self.form.pr

    def reset(self, x0, u0, task):
        super().reset(x0, u0, task); self.obs = None; self.dg = np.zeros(ND); self.u_act = self.u_pend.copy()

    def params(self, task):
        P = super().params(task); return np.r_[P, np.tile(self.dg[:, None], (1, P.shape[1]))]

    def observe(self, xm, u_applied):
        """p = M nu;  p' = rhs(x, u) + M' nu + d.  Trapezoid over the period that just ended; u_applied is the command that acted on it."""
        nu = np.r_[xm[7:13], xm[16:19]].astype(float); Mx = np.array(self.pr.M_f(xm)); pm = Mx @ nu
        f = np.array(self.pr.rhs_f(xm, u_applied, np.zeros(2))).ravel()
        if self.obs is None: self.obs = dict(p0=pm, I=np.zeros(ND), f=f, M=Mx, nu=nu); return
        S = self.obs; S["I"] = S["I"] + (0.5 * (f + S["f"]) + self.dg) * self.period + (Mx - S["M"]) @ (0.5 * (nu + S["nu"])); S["f"], S["M"], S["nu"] = f, Mx, nu
        self.dg = self.k * (pm - S["p0"] - S["I"])

    def compute(self, t, xm, Fm, task):
        self.observe(xm, self.u_act); self.u_act = self.u_pend.copy()                   # one-period latency: u_pend acts on the coming period, u_act acted on the one that ended
        xh = np.array(self.form.step_g(xm, self.u_pend, self.dg)).ravel(); P = self.params(task); Xg = self.Xg.copy(); Xg[:, 0] = xh
        r = self.solver.solve(xh, self.u_pend, P, Xg, self.Ug); info = dict(solve_s=r["solve_s"], status=r["status"], ok=r["ok"], slack=r.get("slack_max", 0.0), fallback=False, iters=r.get("iters", -1), d=self.dg.copy())
        if not r["ok"]:
            self.n_fail += 1; self.solver.reset()
            r = self.solver.solve(xh, self.u_pend, P, np.tile(xh[:, None], (1, self.N + 1)), np.tile(self.u_pend[:, None], (1, self.N))); info.update(retry=True, ok=r["ok"], status=r["status"])
        if r["ok"]: u = r["U"][:, 0]; self.Xg = np.c_[r["X"][:, 1:], r["X"][:, -1]]; self.Ug = np.c_[r["U"][:, 1:], r["U"][:, -1]]
        else: u = self.u_pend.copy(); info["fallback"] = True
        self.u_pend = u.copy(); info["pad_pred_err"] = None; return u, info
