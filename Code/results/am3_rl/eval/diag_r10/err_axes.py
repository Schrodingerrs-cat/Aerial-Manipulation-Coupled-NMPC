"""Trace-phase pad error by axis (mean and RMS) for one wall scenario: is the RL tracking error a bias or lag?"""
import sys, numpy as np
sys.path.insert(0, "scripts")
from am3_eval_wall import SC, run
s = next(x for x in SC if x["id"] == sys.argv[2])
for c in (sys.argv[1], "N"):
    ep, m, p = run(c, s); tr = np.array([r["phase"] == "trace" for r in ep.rows]); pad = np.array([r["pad"] for r in ep.rows])[tr]; ref = np.array([np.asarray(r["ref"]["p"]) for r in ep.rows])[tr]
    v = np.array([np.asarray(r["ref"]["v"]) for r in ep.rows])[tr]; d = (pad - ref) * 1e3
    print(c[-30:], s["path"], "mean x y z", d.mean(0).round(2), "rms", np.sqrt((d ** 2).mean(0)).round(2), "track", round(m["trace"]["track_rms_mm"], 2), "| err along ref vel", round(float(np.mean((d * v).sum(1) / np.maximum(np.linalg.norm(v, axis=1), 1e-6))), 2), "ref speed max mm/s", round(float(np.linalg.norm(v, axis=1).max() * 1e3), 1))
