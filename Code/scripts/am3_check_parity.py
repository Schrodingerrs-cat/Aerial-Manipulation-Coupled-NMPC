"""IPOPT (CasADi Opti) vs acados SQP parity on the same OCP (Plan s7.2): identical state/input conventions, dynamics, residuals, constraints, slacks and parameters by construction; here compared numerically on
representative stage parameters (hover, approach, contact settle, trace): (1) cost of an identical trajectory evaluated by both formulations, (2) converged optimum cost and first input.  -> results/am3_predictor/solver_parity.json"""
import sys, os, json, numpy as np, casadi as ca
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ocrl.am3_episode import initial_state
from ocrl.am3_ctrl_n import NMPC
from ocrl.am3_task import Task, BASE
from ocrl.am3_pred import AMPred3
from ocrl.am3_nmpc import IpoptNMPC, RHO, NH
n = NMPC(rti=False, name=None); form = n.form; ip = IpoptNMPC(form); x0, u0 = initial_state(); p0 = np.array(AMPred3().pad_f(x0)[0]).ravel(); out = {}
for ph_name, steps in (("hover", 0), ("position", 100), ("approach", 250), ("trace", 420)):
    task = Task("line", p0, None); n.reset(x0, u0, task); x = x0.copy()
    for _ in range(steps): task.advance(0.02, 0.0, 0.0)
    task.phase = task.phase; P = n.params(task); Xg = np.tile(x[:, None], (1, n.N + 1)); Ug = np.tile(u0[:, None], (1, n.N))
    r_a = n.solver.solve(x, u0, P, Xg, Ug); r_i = ip.solve(x, u0, P, Xg, Ug)
    # identical-trajectory cost: acados trajectory evaluated with the CasADi residual function
    X, U = r_a["X"], r_a["U"]; c = 0.0
    for j in range(n.N + 1):
        c += float(ca.sumsqr(form.res(X[:, j], U[:, j], u0 if j == 0 else U[:, j - 1], P[:, j]))) if j < n.N else float(ca.sumsqr(form.res_e(X[:, j], P[:, j])))
    out[ph_name] = dict(phase=task.phase, acados_ok=r_a["ok"], ipopt_ok=r_i["ok"], cost_acados_reported=r_a["cost"], cost_ipopt=r_i["cost"], cost_casadi_on_acados_traj=c, u0_diff_inf=float(np.abs(r_a["U"][:, 0] - r_i["U"][:, 0]).max()), u0_scale=[float(abs(v)) for v in r_i["U"][:, 0]], ipopt_status=r_i["status"], acados_iters=r_a["iters"])
    print(ph_name, {k: v for k, v in out[ph_name].items() if k != "u0_scale"}, flush=True)
json.dump(out, open(os.path.join(os.path.dirname(__file__), "..", "results", "am3_predictor", "solver_parity.json"), "w"), indent=1)
