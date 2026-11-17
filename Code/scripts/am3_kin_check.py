"""Check ocrl/am3_kin_poe.py (capstone analytical kinematics) against the CasADi predictor AMPred3 and the numerical IK (am3_static).
Usage: am3_kin_check.py  -> prints errors and timings, writes results/capstone_kin/kin_check.json"""
import sys, os, json, time, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from ocrl.am3_kin_poe import ArmPoE
from ocrl.am3_pred import AMPred3

A = ArmPoE(); pr = AMPred3(coupling="full"); rng = np.random.default_rng(0)
lim = np.radians([[-35, 35], [-80, -5], [-125, -30]]); Q = rng.uniform(lim[:, 0], lim[:, 1], (1000, 3))
x0 = np.zeros(23); x0[3] = 1.0
def pad_pred(q): x = x0.copy(); x[13:16] = q; return np.array(pr.pad_f(x)[0]).ravel()
fk_err = max(np.linalg.norm(A.fk(q)[1] - pad_pred(q)) for q in Q)
t = time.time(); IK = [A.ik_pad(A.fk(q)[1]) for q in Q]; t_ik = (time.time() - t) / len(Q)
ik_err = max(np.abs(np.arctan2(np.sin(qi - q), np.cos(qi - q))).max() for (qi, ok), q in zip(IK, Q)); n_ok = sum(ok for _, ok in IK)
ik_pad_err = max(np.linalg.norm(A.fk(qi)[1] - A.fk(q)[1]) for (qi, ok), q in zip(IK, Q))
h = 1e-6; jac_err = 0.0
for q in Q[:200]:
    p = A.fk(q)[1]; T = A.fk(q)[0]; pc = T[:3, :3].T @ (p - T[:3, 3])                        # pad as a point fixed in the hand
    Jn = np.column_stack([(A.fk(q + h * e)[1] - A.fk(q - h * e)[1]) / (2 * h) for e in np.eye(3)])
    jac_err = max(jac_err, np.abs(A.jac_point(q, p) - Jn).max())
out = dict(n=len(Q), fk_vs_AMPred3_max_m=fk_err, ik_roundtrip_max_rad=ik_err, ik_pad_err_max_m=ik_pad_err, ik_reachable=int(n_ok), ik_us=t_ik * 1e6, jac_vs_fd_max=jac_err)
try:
    from ocrl.am3_static import Rig
    S = Rig(); pB = np.array([0.0, 0.0, 1.2]); t = time.time(); r = [S.ik(pB + A.fk(q)[1], pB, q_ref=np.radians([0, -37, -78])) for q in Q[:50]]; out["numeric_ik_ms"] = (time.time() - t) / 50 * 1e3
    out["numeric_vs_analytic_max_rad"] = max(np.abs(ri[0] - qi).max() for ri, (qi, _) in zip(r, IK[:50]))
except Exception as e: out["numeric_ik_ms"] = f"not timed ({type(e).__name__}: {e})"
print(json.dumps(out, indent=1, default=float)); json.dump(out, open(os.path.join(os.path.dirname(__file__), "..", "results", "capstone_kin", "kin_check.json"), "w"), indent=1, default=float)
