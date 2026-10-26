"""Coupled nominal NMPC N of the aerial manipulator (Plan s7.2): 23-state predictor (am3_pred), multiple shooting over N stages of h seconds, normalised least-squares cost, soft state constraints with named
normalised slacks, hard rotor-speed and joint-torque bounds.  One CasADi formulation (residuals, constraints, dynamics) feeds both the IPOPT reference (Opti) and the generated acados solver, so cost, constraints and
parameter ordering are identical by construction.  Objectives: fixed base position, timed pad position/velocity along the task reference (no orientation, no force setpoint), small attitude/rate/joint-rate regularisation,
input variation.  Augmented acados state [x(23); u_prev(7)] carries the input-rate term.  Per-stage parameters: see PARAM_NAMES."""
import os, time, json, hashlib, fcntl, numpy as np, casadi as ca
from ocrl.am3_pred import AMPred3, C, quat2R

HERE = os.path.dirname(os.path.abspath(__file__)); GEN_DIR = os.path.join(HERE, "..", "results", "acados_codegen")
GROUPS = ("tp", "tv", "B", "vB", "att", "w", "qd", "du")
SIG = dict(tp=(1e-3, 2e-3, 2e-3), tv=5e-3, B=5e-3, vB=0.02, att=0.05, w=0.2, qd=0.5)
PARAM_NAMES = ["pad(3)", "vpad(3)", "pB(3)", "F_cap", "vc_max", "d(2)", "sqrt_w(8)"]
NP = 3 + 3 + 3 + 1 + 1 + 2 + 8
I_P = dict(pad=slice(0, 3), vpad=slice(3, 6), pB=slice(6, 9), Fcap=9, vcmax=10, d=slice(11, 13), w=slice(13, 21))
OP = C["joint_limits_deg"]["operational"]; QLO, QHI = np.radians([OP[k][0] for k in ("q1", "q2", "q3")]), np.radians([OP[k][1] for k in ("q1", "q2", "q3")])
LIM = dict(tilt_deg=15.0, qd_max=3.0, z_clear=-0.045, sig_tilt=0.1, sig_q=0.05, sig_qd=0.5, sig_F=0.1, sig_vc=5e-3, sig_z=5e-3)
NH = 1 + 6 + 6 + 1 + 1 + 2                       # tilt, q range (lo/hi x3), qd (+/- x3), force cap, closing speed, elbow/wrist height below the rotor plane
RHO = 1e3                                        # penalty on normalised slack (quadratic and linear)
# sqrt-weights per phase group: tp tv B vB att w qd du
W = {"free":    (2.0, 1.0, 6.0, 2.0, 0.3, 1.0, 0.2, 0.3),
     "approach": (2.0, 1.0, 6.0, 2.0, 0.3, 1.0, 0.2, 0.3),
     "contact":  (2.0, 1.0, 6.0, 2.0, 0.3, 1.0, 0.2, 0.3),
     "trace":    (3.0, 1.5, 6.0, 2.0, 0.3, 1.0, 0.2, 0.3)}
PH_W = dict(hover="free", position="free", approach="approach", recover="approach", settle="contact", settle_resume="contact", trace="trace", endhold="contact", release="contact", retreat="free", done="free")
D_MAX = np.array([0.5, 0.10]); D_RATE = np.array([0.25, 0.05])            # correction channels (used by D/L only)

class Formulation:
    def __init__(self, pred: AMPred3, N=10, h=0.02, nsub=4):
        self.pr, self.N, self.h, self.nsub = pred, N, h, nsub; pr = pred; self.us = np.r_[[100.0] * 4, [0.2] * 3]
        f = pr.xdot_mx(); x = ca.MX.sym("x", 23); u = ca.MX.sym("u", 7); d = ca.MX.sym("d", 2); xs = x; hh = h / nsub
        for _ in range(nsub):
            k1 = f(xs, u, d); k2 = f(xs + hh / 2 * k1, u, d); k3 = f(xs + hh / 2 * k2, u, d); k4 = f(xs + hh * k3, u, d); xs = xs + hh / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        xs = ca.vertcat(xs[0:3], xs[3:7] / ca.norm_2(xs[3:7]), xs[7:23]); self.step_d = ca.Function("step_d", [x, u, d], [xs]); self.step = ca.Function("step", [x, u], [self.step_d(x, u, ca.DM.zeros(2))])
        up = ca.MX.sym("up", 7); p = ca.MX.sym("p", NP); self.res = ca.Function("res", [x, u, up, p], [self._residual(x, u, up, p)]); self.nres_state = 21
        self.res_e = ca.Function("res_e", [x, p], [self._residual(x, ca.MX.zeros(7), ca.MX.zeros(7), p)[:self.nres_state]]); self.hcon = ca.Function("hcon", [x, p], [self._constraints(x, p)])
    def _outputs(self, x):
        pr = self.pr; g, vc, Fn, ft, vC = pr.contact_f(x, ca.DM.zeros(7), ca.DM.zeros(2)); JC, JT, JwT = pr.JC_f(x); nu = ca.vertcat(x[7:10], x[10:13], x[16:19]); pT, aT = pr.pad_f(x); return vc, Fn, JT @ nu, pT
    def _residual(self, x, u, up, p):
        pad_r, vpad_r, pB, sw = p[0:3], p[3:6], p[6:9], p[13:21]; S = dict(zip(GROUPS, [sw[i] for i in range(8)])); vc, Fn, vT, pT = self._outputs(x); R = quat2R(x[3:7])
        att = ca.vertcat(R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]) / 2          # rotation vector (small angle) of the base from level; yaw included (weak weight), no pointing reference
        rs = ca.vertcat(S["tp"] * (pT - pad_r) / ca.DM(SIG["tp"]), S["tv"] * (vT - vpad_r) / SIG["tv"], S["B"] * (x[0:3] - pB) / SIG["B"], S["vB"] * x[7:10] / SIG["vB"], S["att"] * att / SIG["att"], S["w"] * x[10:13] / SIG["w"], S["qd"] * x[16:19] / SIG["qd"])
        return ca.vertcat(rs, S["du"] * (u - up) / ca.DM(self.us))
    def _constraints(self, x, p):
        F_cap, vc_max = p[9], p[10]; vc, Fn, vT, pT = self._outputs(x); R33 = quat2R(x[3:7])[2, 2]; q, qd = x[13:16], x[16:19]; o3, o4, rp = self.pr.pts_f(x); Rm = quat2R(x[3:7])
        zb = lambda o: (Rm.T @ ca.vertcat(0, 0, 1)).T @ o                                    # point height along world up expressed in the base frame
        return ca.vertcat((np.cos(np.radians(LIM["tilt_deg"])) - R33) / LIM["sig_tilt"], (q - QHI) / LIM["sig_q"], (QLO - q) / LIM["sig_q"], (qd - LIM["qd_max"]) / LIM["sig_qd"], (-qd - LIM["qd_max"]) / LIM["sig_qd"],
                          (Fn + p[11] - F_cap) / LIM["sig_F"], (vc - vc_max) / LIM["sig_vc"], (zb(o3) - LIM["z_clear"]) / LIM["sig_z"], (zb(o4) - LIM["z_clear"]) / LIM["sig_z"])

def default_params():
    p = np.zeros(NP); p[9:11] = [C["contact"]["F_max_N"], 0.05]; return p

# ------------------------------------------------------------------ IPOPT reference
class IpoptNMPC:
    def __init__(self, form: Formulation, max_iter=100):
        self.f, N = form, form.N; o = ca.Opti(); X = o.variable(23, N + 1); U = o.variable(7, N); S = o.variable(NH, N + 1); x0, up = o.parameter(23), o.parameter(7); P = o.parameter(NP, N + 1)
        o.subject_to(X[:, 0] == x0); cost = 0
        for j in range(N + 1):
            if j < N:
                o.subject_to(X[:, j + 1] == form.step_d(X[:, j], U[:, j], P[11:13, j])); o.subject_to(o.bounded(form.pr.om_min, U[0:4, j], form.pr.om_max)); o.subject_to(o.bounded(-form.pr.tau_c, U[4:7, j], form.pr.tau_c))
                r = form.res(X[:, j], U[:, j], up if j == 0 else U[:, j - 1], P[:, j])
            else: r = form.res_e(X[:, j], P[:, j])
            cost += ca.sumsqr(r) + RHO * ca.sumsqr(S[:, j]) + RHO * ca.sum1(S[:, j]); o.subject_to(S[:, j] >= 0); o.subject_to(form.hcon(X[:, j], P[:, j]) <= S[:, j])
        o.minimize(cost); o.solver("ipopt", {"print_time": False, "ipopt.print_level": 0, "ipopt.sb": "yes", "ipopt.max_iter": max_iter, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4})
        self.o, self.v = o, dict(X=X, U=U, S=S, x0=x0, up=up, P=P)
    def solve(self, x0, u_prev, P, Xg, Ug):
        o, v = self.o, self.v; o.set_value(v["x0"], x0); o.set_value(v["up"], u_prev); o.set_value(v["P"], P); o.set_initial(v["X"], Xg); o.set_initial(v["U"], Ug); o.set_initial(v["S"], 0.0); t0 = time.perf_counter()
        try: sol = o.solve(); st = sol.stats(); ok = True
        except RuntimeError: sol = o.debug; st = o.stats(); ok = False
        return dict(ok=ok or st["return_status"] == "Solved_To_Acceptable_Level", X=sol.value(v["X"]), U=sol.value(v["U"]), S=sol.value(v["S"]), cost=float(sol.value(o.f)), status=st["return_status"], iters=st.get("iter_count", -1), solve_s=time.perf_counter() - t0)

# ------------------------------------------------------------------ acados production solver
class AcadosNMPC:
    def __init__(self, form: Formulation, rti=True, name=None, sqp_iters=30):
        import ctypes
        flavour = os.environ.get("OCRL_ACADOS", "acados")                  # "acados": single-thread build; "acados_omp": OpenMP build tree (threads via OMP_NUM_THREADS)
        ACADOS_LIB = os.path.abspath(os.path.join(HERE, "..", "third_party", flavour, "lib")); os.environ["ACADOS_SOURCE_DIR"] = os.path.abspath(os.path.join(ACADOS_LIB, ".."))
        for lib in ("libqpOASES_e.so", "libblasfeo.so", "libhpipm.so", "libacados.so"): ctypes.CDLL(os.path.join(ACADOS_LIB, lib), mode=ctypes.RTLD_GLOBAL)
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        self.f, self.rti, N, h = form, rti, form.N, form.h; pr = form.pr; self.N = N
        self.Sx = np.r_[np.ones(19), [1000.0] * 4]; self.Su = np.r_[[1000.0] * 4, [0.5] * 3]                  # solver variables scaled so that the QP is well conditioned (z = x / Sx, v = u / Su)
        zs = ca.MX.sym("z", 30); v = ca.MX.sym("v", 7); p = ca.MX.sym("p", NP); x = zs[0:23] * ca.DM(self.Sx); up = zs[23:30] * ca.DM(self.Su); u = v * ca.DM(self.Su)
        res = form.res(x, u, up, p); res_e = form.res_e(x, p); hc = form.hcon(x, p)
        m = AcadosModel(); m.name = name = name or "am3_nmpc"; m.x = zs; m.u = v; m.p = p; m.disc_dyn_expr = ca.vertcat(form.step_d(x, u, p[11:13]) / ca.DM(self.Sx), v); m.cost_y_expr = res; m.cost_y_expr_e = res_e
        m.con_h_expr_0 = hc; m.con_h_expr = hc; m.con_h_expr_e = hc
        ocp = AcadosOcp(); ocp.model = m; ocp.solver_options.N_horizon = N; ocp.solver_options.tf = N * h; ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = 2 * np.eye(res.shape[0]); ocp.cost.W_e = 2 * np.eye(res_e.shape[0]); ocp.cost.yref = np.zeros(res.shape[0]); ocp.cost.yref_e = np.zeros(res_e.shape[0])
        ocp.constraints.x0 = np.zeros(30); ocp.constraints.lbu = np.r_[[pr.om_min] * 4, [-pr.tau_c] * 3] / self.Su; ocp.constraints.ubu = np.r_[[pr.om_max] * 4, [pr.tau_c] * 3] / self.Su; ocp.constraints.idxbu = np.arange(7)
        for suf in ("_0", "", "_e"):
            setattr(ocp.constraints, "lh" + suf, -1e6 * np.ones(NH)); setattr(ocp.constraints, "uh" + suf, np.zeros(NH)); setattr(ocp.constraints, "idxsh" + suf, np.arange(NH))
            for z in ("zl", "zu"): setattr(ocp.cost, z + suf, RHO * np.ones(NH))
            for Z in ("Zl", "Zu"): setattr(ocp.cost, Z + suf, 2 * RHO * np.ones(NH))       # 2*Z for the quadratic part so that cost = RHO*s^2 + RHO*s (matches the IPOPT expression)
        ocp.parameter_values = default_params(); so = ocp.solver_options; so.cost_scaling = np.ones(N + 1); so.integrator_type = "DISCRETE"
        so.nlp_solver_type = "SQP_RTI" if rti else "SQP"; so.hessian_approx = "GAUSS_NEWTON"; so.qp_solver = "PARTIAL_CONDENSING_HPIPM"; so.nlp_solver_max_iter = sqp_iters; so.qp_solver_iter_max = 100
        so.nlp_solver_tol_stat = so.nlp_solver_tol_eq = so.nlp_solver_tol_ineq = so.nlp_solver_tol_comp = 1e-4; so.globalization = "MERIT_BACKTRACKING" if not rti else "FIXED_STEP"; so.print_level = 0; so.levenberg_marquardt = 1e-4; so.ext_fun_compile_flags = "-O3 -march=native"
        src = b"".join(open(os.path.join(HERE, f), "rb").read() for f in ("am3_pred.py", "am3_nmpc.py")) + open(os.path.join(HERE, "..", "config", "am3_design_v3.json"), "rb").read() + repr((flavour, N, h, form.nsub, rti, sqp_iters, form.pr.k, form.pr.c, form.pr.mu, form.pr.coupling)).encode()
        d = os.path.abspath(os.path.join(GEN_DIR, f"{name}_{hashlib.sha1(src).hexdigest()[:12]}")); os.makedirs(d, exist_ok=True); ocp.code_export_directory = d; stamp = os.path.join(d, "built"); self.build_dir = d
        with open(os.path.join(GEN_DIR, ".build.lock"), "w") as lk:                        # serial builds only
            fcntl.flock(lk, fcntl.LOCK_EX); fresh = os.path.exists(stamp); t0 = time.time()
            self.solver = AcadosOcpSolver(ocp, json_file=os.path.join(d, name + ".json"), build=not fresh, generate=not fresh); open(stamp, "w").write("ok"); self.build_s = 0.0 if fresh else time.time() - t0
        for key in ("qp_tol_stat", "qp_tol_eq", "qp_tol_ineq", "qp_tol_comp"): self.solver.options_set(key, 1e-8)
    def reset(self): self.solver.reset()
    def solve(self, x0, u_prev, P, Xg, Ug, max_iter=None):
        s, N = self.solver, self.N; xa0 = np.r_[x0 / self.Sx, u_prev / self.Su]
        for j in range(N + 1): s.set(j, "p", P[:, j]); s.set(j, "x", np.r_[Xg[:, j] / self.Sx, (Ug[:, j - 1] if j else u_prev) / self.Su])
        for j in range(N): s.set(j, "u", Ug[:, j] / self.Su)
        s.set(0, "lbx", xa0); s.set(0, "ubx", xa0); t0 = time.perf_counter(); status = s.solve(); dt = time.perf_counter() - t0
        X = np.stack([s.get(j, "x")[:23] * self.Sx for j in range(N + 1)], 1); U = np.stack([s.get(j, "u") * self.Su for j in range(N)], 1)
        sl = np.stack([np.r_[s.get(j, "su") if s.get(j, "su").size else np.zeros(NH)] for j in range(N)], 1)
        return dict(ok=status == 0 or (not self.rti and status == 2), truncated=bool(status == 2), status=int(status), X=X, U=U, slack_max=float(np.abs(sl).max()) if sl.size else 0.0, solve_s=dt, cost=float(s.get_cost()), iters=int(s.get_stats("sqp_iter")))
