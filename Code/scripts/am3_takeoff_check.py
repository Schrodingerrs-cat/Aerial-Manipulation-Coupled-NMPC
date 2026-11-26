"""One take-off-then-write run with per-phase errors (2026-11-26).  Usage: am3_takeoff_check.py CTRL [TEXT] [CASE]   (lab noise, seed 0)
Prints, per phase (ground, takeoff, unfold, hover): max base and pen error, max tilt, height at the end, non-pad contacts; then the liftoff time and the writing metrics."""
import sys, numpy as np
from stable_baselines3 import PPO          # torch before acados
from ocrl.am3_takeoff import TakeoffWriteEpisode, START
from am3_write_eval import make, write_metrics
from am3_write_bench import LabSensors
from am3_write_mismatch import CASES

ctrl = sys.argv[1]; text = sys.argv[2] if len(sys.argv) > 2 else "HI"; case = sys.argv[3] if len(sys.argv) > 3 else "nominal"
ep = TakeoffWriteEpisode(make(ctrl), text, height=0.05, v_write=0.05, mismatch=CASES[case]); ep.sens = LabSensors(0); ep.run()
R = ep.rows; ph = [r["phase"] for r in R]
for p in ("ground", "takeoff", "unfold", "hover"):
    idx = [i for i, q in enumerate(ph) if q == p]
    if not idx: continue
    eb = max(np.linalg.norm(R[i]["x"][0:3] - R[i]["ref"]["pB"]) for i in idx) * 1e3; epn = max(np.linalg.norm(R[i]["pad"] - R[i]["ref"]["p"]) for i in idx) * 1e3
    tilt = max(np.degrees(2 * np.arcsin(min(1, np.hypot(R[i]["x"][4], R[i]["x"][5])))) for i in idx)
    print(f"{p:8s} t {R[idx[0]]['t']:.2f}-{R[idx[-1]]['t']:.2f}  base err max {eb:.1f} mm  pen err max {epn:.1f} mm  tilt max {tilt:.1f} deg  z end {R[idx[-1]]['x'][2]:.3f}")
print("liftoff t", next((r["t"] for r in R if r["x"][2] > START[2] + 0.002), None), " abort", ep.abort, " non-stand contacts", sorted({c for c in ep.pl.other_names if "stand" not in "".join(c)}) if hasattr(ep.pl, "other_names") else "")
print({k: v for k, v in write_metrics(ep).items() if k in ("completed", "abort", "strokes", "trace_rms_mm", "coverage", "F_peak_N", "tilt_max_deg", "drag")})
