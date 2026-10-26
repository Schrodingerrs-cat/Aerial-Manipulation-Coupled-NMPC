"""Apply the acceptance criteria to a batch table and print joint successes, failed criteria and summary statistics.  Usage: am3_gate.py TAG [ACCEPTANCE_JSON]  -> results/am3_runs/<TAG>/gate.json"""
import sys, os, json, collections, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_eval import passes
tag = sys.argv[1]; acc = json.load(open(sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "..", "config", "am3_acceptance_draft.json")))["criteria"]
rows = json.load(open(os.path.join(HERE, "..", "results", "am3_runs", tag, "table.json"))); res = []; fails = collections.Counter(); stubs = [m for m in rows if m.get("stub")]; rows_all = rows; rows = [m for m in rows if not m.get("stub")]
for m in rows:
    p = passes(m, acc); res.append(dict(id=m["scenario"]["id"], path=m["scenario"]["path"], family=m["scenario"].get("family"), all=p["all"], failed=[k for k, v in p.items() if not v and k != "all"], abort=m["abort"])); fails.update(res[-1]["failed"])
g = lambda f: [f(m) for m in rows]
stat = dict(n=len(rows_all), solver_hang_stubs=len(stubs), joint_successes=sum(r["all"] for r in res), completed=sum(m["completed"] for m in rows), failed_criteria=dict(fails), base_max_mm=dict(median=float(np.median(g(lambda m: m["base"]["max_mm"]))), max=float(np.max(g(lambda m: m["base"]["max_mm"])))),
            trace_track_rms_mm=(lambda v: dict(median=float(np.median(v)), max=float(np.max(v))) if v else None)([m["trace"]["track_rms_mm"] for m in rows if "trace" in m]), peak_Fn_N_max=float(np.max(g(lambda m: m["contact"]["peak_Fn_N"]))),
            timing=dict(median_ms=float(np.median(g(lambda m: m["timing"]["median_ms"]))), max_ms=float(np.max(g(lambda m: m["timing"]["max_ms"]))), over_period_fraction_mean=float(np.mean(g(lambda m: m["timing"]["over_period_fraction"])))))
json.dump(dict(criteria_file=sys.argv[2] if len(sys.argv) > 2 else "draft", stat=stat, scenarios=res), open(os.path.join(HERE, "..", "results", "am3_runs", tag, "gate.json"), "w"), indent=1, default=str)
res += [dict(id=m["scenario"]["id"], path=m["scenario"]["path"], family=None, all=False, failed=["solver_hang"], abort=m["abort"]) for m in stubs]; print(tag, json.dumps(stat, indent=1)); [print(" FAIL", r["id"], r["path"], r["failed"], r["abort"]) for r in res if not r["all"]]
