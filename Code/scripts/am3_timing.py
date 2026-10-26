"""Timing and deadline study on a quiet machine (one process at a time): N full-cycle time (estimator-free, reference preview + solve + wrapper) over three paths; the same runs with the deadline ENFORCED (a result later than 20 ms is discarded and the previous
command stays); injected random misses (2 %, 10 %) for N and B.  -> results/am3_timing/summary.json"""
import sys, os, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_run import run
os.environ["OMP_NUM_THREADS"] = "1"; out = {}
def row(m): return dict(completed=m["completed"], abort=str(m["abort"]), base_max_mm=m["base"]["max_mm"], track_rms=m.get("trace", {}).get("track_rms_mm"), losses=m["contact"]["losses"], timing=m["timing"], peak_Fn=m["contact"]["peak_Fn_N"])
for p in ("line", "circle", "rounded_triangle"):
    for tag, kw in (("free_running", {}), ("deadline_enforced", dict(enforce_deadline=True)), ("miss_2pct", dict(miss_p=0.02)), ("miss_10pct", dict(miss_p=0.10))):
        ep, m = run(f"timing/N_{p}_{tag}", "N", p, **kw); out[f"N_{p}_{tag}"] = row(m); t = m["timing"]; print("N", p, tag, "ok" if m["completed"] else m["abort"], "cycle med %.1f p95 %.1f p99 %.1f max %.1f ms, over %.3f, misses %d, base max %.2f" % (t["median_ms"], t["p95_ms"], t["p99_ms"], t["max_ms"], t["over_period_fraction"], t["injected_misses"], m["base"]["max_mm"]), flush=True)
    for tag, kw in (("miss_2pct", dict(miss_p=0.02)), ("miss_10pct", dict(miss_p=0.10))):
        ep, m = run(f"timing/B_{p}_{tag}", "B", p, **kw); out[f"B_{p}_{tag}"] = row(m); print("B", p, tag, "ok" if m["completed"] else m["abort"], "base max %.1f" % m["base"]["max_mm"], flush=True)
os.makedirs(os.path.join(HERE, "..", "results", "am3_timing"), exist_ok=True); json.dump(out, open(os.path.join(HERE, "..", "results", "am3_timing", "summary.json"), "w"), indent=1, default=str)
