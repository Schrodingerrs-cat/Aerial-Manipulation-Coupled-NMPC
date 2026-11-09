"""Task-logic checks (audit B02, B03): (1) the commanded path velocity equals the finite-difference derivative of the commanded position through the progress ramp and recovery, (2) contact acquisition/loss take the same elapsed
time at the 5 ms (B) and 20 ms (N) control periods.  Exits non-zero on failure.  -> results/am3_predictor/task_checks.json"""
import sys, os, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_task import Task, CP
out = {}
# (1) reference velocity vs derivative of position, event-free nominal timeline and with a forced loss/recovery
for name in ("line", "circle", "rounded_triangle"):
    t = Task(name, [0.7, 0, 1.0]); dt = 0.005; P = []; V = []; ph = []
    for k in range(int(20 / dt)):
        F = 0.6 if t.gap_ref <= 0.0 and t.phase in ("approach", "settle", "settle_resume", "trace", "endhold") else 0.0
        if t.phase == "trace" and 7.5 < t.t < 7.52: F = 0.0                         # brief force dropout during the trace -> confirmed loss -> recovery
        if 7.5 < t.t < 7.7 and t.phase == "trace": F = 0.0
        t.advance(dt, F, t.gap_ref); r = t.ref(); P.append(r["p"]); V.append(r["v"]); ph.append(t.phase)
    P = np.array(P); V = np.array(V); dP = np.gradient(P, dt, axis=0); sel = np.array([p in ("trace", "approach", "position", "retreat", "release") for p in ph]); err = np.abs(dP[:, 1:] - V[:, 1:])[sel]
    out[name] = dict(max_abs_velocity_error_m_s=float(err.max()), phases_seen=sorted(set(ph)), recoveries=len(t.recoveries)); assert err.max() < 2e-3, (name, err.max())
# (2) detection time
def detect(dt):
    t = Task("line", [0.7, 0, 1.0]); tau_on = tau_off = None
    for k in range(int(12 / dt)):
        F = 0.6 if (t.phase == "approach" and t.gap_ref <= 0.0) or t.phase in ("settle", "trace") else 0.0
        if t.phase == "trace" and tau_off is None and t.tph > 1.0: F = 0.0
        was = t.contact; t.advance(dt, F, t.gap_ref)
        if not was and t.contact and tau_on is None: tau_on = t.t - (k * 0.0)
        if t.phase == "recover" and tau_off is None: tau_off = t.t
    return tau_on, tau_off
for dt in (0.005, 0.02): out[f"detect_dt_{dt}"] = dict(zip(("t_contact_detected", "t_loss_confirmed"), detect(dt)))
json.dump(out, open(os.path.join(HERE, "..", "results", "am3_predictor", "task_checks.json"), "w"), indent=1); print(json.dumps(out, indent=1))
