"""Controller N (and its model ablation): the coupled NMPC of am3_nmpc on the common task.  Delay compensation: the measured state is propagated one control period with the pending command before solving, so the
optimised first input starts when it is applied.  Reference preview comes from the shared task (nominal, event-free timeline).  `coupling="decoupled"` gives the NMPC model ablation (same costs, constraints, solver settings;
only the predictor's base-arm inertial coupling differs)."""
import numpy as np
from ocrl.am3_pred import AMPred3, C
from ocrl.am3_nmpc import Formulation, AcadosNMPC, IpoptNMPC, NP, I_P, W, PH_W, default_params, NH
from ocrl.am3_task import BASE, CP

class NMPC:
    period = 0.02
    def __init__(self, coupling="full", N=10, h=0.02, nsub=4, solver="acados", rti=True, sqp_iters=30, name=None):
        self.name = name or ("N" if coupling == "full" else "N_decoupled"); self.N, self.h = N, h; self.form = Formulation(AMPred3(coupling=coupling), N=N, h=h, nsub=nsub)
        self.solver = AcadosNMPC(self.form, rti=rti, name=f"am3_{coupling}_N{N}", sqp_iters=sqp_iters) if solver == "acados" else IpoptNMPC(self.form); self.is_acados = solver == "acados"; self.d = np.zeros(2)
    def reset(self, x0, u0, task):
        self.u_pend = np.asarray(u0, float).copy(); self.Xg = np.tile(np.asarray(x0, float)[:, None], (1, self.N + 1)); self.Ug = np.tile(self.u_pend[:, None], (1, self.N)); self.n_fail = 0
        if self.is_acados: self.solver.reset()
    def held(self, u): self.u_pend = np.asarray(u, float).copy()          # the computed command missed its deadline; the previous one stays in force
    def params(self, task):
        refs = task.horizon(self.N + 1, self.h)[1:]; P = np.zeros((NP, self.N + 1))
        for j, r in enumerate(refs):
            p = default_params(); p[I_P["pad"]] = r["p"]; p[I_P["vpad"]] = r["v"]; p[I_P["pB"]] = r.get("pB", BASE); p[I_P["vB"]] = r.get("vB", 0.0); p[I_P["d"]] = self.d; p[I_P["w"]] = W[PH_W[r["phase"]]]
            p[I_P["vcmax"]] = 0.04 if r["phase"] in ("approach", "recover", "position") else 0.5; P[:, j] = p
        return P
    def compute(self, t, xm, Fm, task):
        xh = np.array(self.form.step(xm, self.u_pend)).ravel(); P = self.params(task); Xg = self.Xg.copy(); Xg[:, 0] = xh
        r = self.solver.solve(xh, self.u_pend, P, Xg, self.Ug); info = dict(solve_s=r["solve_s"], status=r["status"], ok=r["ok"], slack=r.get("slack_max", 0.0), fallback=False, iters=r.get("iters", -1))
        if not r["ok"]:
            self.n_fail += 1
            if self.is_acados: self.solver.reset()
            r = self.solver.solve(xh, self.u_pend, P, np.tile(xh[:, None], (1, self.N + 1)), np.tile(self.u_pend[:, None], (1, self.N))); info.update(retry=True, ok=r["ok"], status=r["status"])
        if r["ok"]: u = r["U"][:, 0]; self.Xg = np.c_[r["X"][:, 1:], r["X"][:, -1]]; self.Ug = np.c_[r["U"][:, 1:], r["U"][:, -1]]
        else: u = self.u_pend.copy(); info["fallback"] = True
        self.u_pend = u.copy(); info["pad_pred_err"] = None; return u, info
