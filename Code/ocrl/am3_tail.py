"""Arm as a 'cheetah tail' while carrying a block (2026-11-27, capstone; professor's suggestion via Sahil).
The hand holds a rigid block of known mass (the capstone's analytical mass estimate gives it).  The drone flies a multi-waypoint route.  Question: can the coupled NMPC
use the arm + block as a tail (reaction torque and centre-of-mass shift) to move faster / smoother than when the arm is held still?
Controllers (same coupled NMPC, same solver, same limits; only the arm terms and the model differ):
  rigid  : joints frozen relative to the body (heavy joint-rate penalty, no block-position term): the block is dead weight   - load-aware model
  hold   : the block is held at a fixed offset from the base reference in the WORLD (arm counter-rotates when the body tilts) - load-aware model
  tail   : the block offset is only a weak spring (TAIL_TP), so the optimiser may swing the arm                              - load-aware model
  unaware: rigid, but the model does not know the block (what load awareness buys)
('hold' was the first 'locked' baseline: keeping the block still in the world makes the arm rotate against the body's pitch, i.e. an anti-tail; 'rigid' is the fair one.)
The block is added to a copy of the frozen hand parameters at solver-build time; the frozen files are not edited."""
import copy, numpy as np
import ocrl.am3_pred as P
from ocrl.am3_pred import AMPred3
from ocrl.am3_nmpc import Formulation, AcadosNMPC, NP, I_P, W, default_params
from ocrl.am3_ctrl_n import NMPC
from ocrl.am3_ref import Equilibrium
from ocrl.am3_task import BASE
from ocrl.am3_path import min_jerk

TAIL_TP = 0.05                                       # sqrt-weight of the block-offset term in tail mode (hold: W["free"] = 2.0)
RIGID_QD = 20.0                                      # sqrt-weight of the joint-rate term in rigid mode (free: 0.2; 1 sigma = 0.025 rad/s)
R_BLOCK = 0.02                                       # m, block treated as a solid sphere at the hand origin (as the plant's payload body)


def pred_with_block(mb, coupling="full"):
    """AMPred3 whose hand carries a block of mb kg at the hand origin (combined mass, centre of mass and inertia by the parallel-axis rule)."""
    C0 = copy.deepcopy(P.C); h = P.C["supplied"]["hand"]; m0, c0, I0 = h["mass_kg"], np.array(h["com_m"], float), np.array(h["I_diag"], float)
    if mb > 0:
        m = m0 + mb; c = m0 * c0 / m; d0, db = c0 - c, -c
        par = lambda mi, d: mi * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
        h.update(mass_kg=m, com_m=list(c), I_diag=list(I0 + par(m0, d0) + 0.4 * mb * R_BLOCK ** 2 + par(mb, db)))
    try: return AMPred3(coupling=coupling)
    finally: P.C.clear(); P.C.update(C0)


class WaypointTask:
    """Base flies W0 -> W1 -> ... with min-jerk segments of T_move s and dwell s pauses; the block reference is the base reference + a fixed offset."""
    def __init__(self, waypoints, T_move, dwell, block_off, t0=1.0):
        self.W = np.asarray(waypoints, float); self.T, self.dwell, self.off, self.t0 = T_move, dwell, np.asarray(block_off, float), t0
        self.t = 0.0; self.phase = "dwell"; self.failed = None; self.contact = False; self.gap_ref = 0.2; self.t_done = None
        self.t_end = t0 + (len(self.W) - 1) * (T_move + dwell)

    def ref_at(self, t):
        k = int(np.clip((t - self.t0) // (self.T + self.dwell), 0, len(self.W) - 2)); ts = t - self.t0 - k * (self.T + self.dwell)
        if t < self.t0: pB, vB, ph = self.W[0], np.zeros(3), "dwell"
        elif ts < self.T: f, f1, _ = min_jerk(ts / self.T); d = self.W[k + 1] - self.W[k]; pB, vB, ph = self.W[k] + d * f, d * f1 / self.T, "move"
        else: pB, vB, ph = self.W[k + 1], np.zeros(3), "dwell"
        return dict(phase=ph if t < self.t_end else "done", p=pB + self.off, v=vB, pB=pB, vB=vB, gap=self.gap_ref, contact_expected=False, s=0.0, stroke=0)

    def ref(self): return self.ref_at(self.t)
    def horizon(self, N, h): return [self.ref_at(self.t + j * h) for j in range(N + 1)]
    def advance(self, dt, F_meas, gap_meas, nominal=False):
        self.t += dt; self.phase = self.ref()["phase"]
        if self.phase == "done" and self.t_done is None: self.t_done = self.t


class TailNMPC(NMPC):
    """Coupled NMPC with the block in its model (mb kg; 0 = unaware) and the arm rigid, holding the block still in the world, or free (tail)."""
    def __init__(self, mode="tail", mb=0.15, N=10, h=0.02, nsub=4):
        self.mode, self.name, self.N, self.h = mode, f"{mode}_{int(mb * 1000)}g", N, h; self.d = np.zeros(2); self.is_acados = True
        self.form = Formulation(pred_with_block(mb), N=N, h=h, nsub=nsub); self.solver = AcadosNMPC(self.form, rti=True, name=f"am3_tail_b{int(mb * 1000)}_N{N}")
        w = np.array(W["free"], float)
        if mode.startswith("tail"): w[0], w[1] = float(mode[4:] or TAIL_TP), 0.0      # block position: weak spring only ("tail0.3": sqrt-weight 0.3); block velocity: free
        if mode == "rigid": w[0], w[1], w[6] = 0.0, 0.0, RIGID_QD                   # no block term; joints frozen by the joint-rate penalty
        self.w = w

    def params(self, task):
        P_ = np.zeros((NP, self.N + 1))
        for j, r in enumerate(task.horizon(self.N, self.h)[1:]):
            p = default_params(); p[I_P["pad"]] = r["p"]; p[I_P["vpad"]] = r["v"]; p[I_P["pB"]] = r["pB"]; p[I_P["vB"]] = r["vB"]; p[I_P["d"]] = self.d; p[I_P["w"]] = self.w; p[I_P["vcmax"]] = 0.5
            P_[:, j] = p
        return P_


def start_state(mb, base, block_off):
    """Hover equilibrium at `base` with the block at base + block_off, for the model with the block."""
    e = Equilibrium(pred_with_block(mb), base).solve(np.asarray(base) + block_off, [0, 0, 0]); return e["x"], e["u"]
