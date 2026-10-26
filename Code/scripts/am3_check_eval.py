"""Evaluator counterexamples (Plan s11.1) on real closed-loop logs of N modified after the fact: (a) completed but drifting base, (b) detached tool tracing the curve (no contact force), (c) partial path, (d) missed deadlines,
(e) detached tool via zero force at the wall.  Each must flip the corresponding pass flag.  -> results/am3_predictor/evaluator_counterexamples.json"""
import sys, os, json, copy, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..")); sys.path.insert(0, os.path.dirname(__file__))
from am3_run import make_ctrl
from ocrl.am3_episode import Episode, initial_state
from ocrl.am3_eval import metrics, passes
crit = json.load(open(os.path.join(os.path.dirname(__file__), "..", "config", "am3_acceptance_draft.json")))["criteria"]
ep = Episode(make_ctrl("N"), "line", T_max=40.0).run(); base = metrics(ep); out = dict(baseline=dict(passes=passes(base, crit), completed=base["completed"]))
def variant(name, mod):
    e2 = copy.copy(ep); e2.rows = [dict(r) for r in ep.rows]; e2.pl = copy.copy(ep.pl); e2.pl.log = {k: list(v) for k, v in ep.pl.log.items()}; e2.task = copy.copy(ep.task); mod(e2); m = metrics(e2); p = passes(m, crit); out[name] = dict(passes={k: v for k, v in p.items() if not v}, all=p["all"], key=dict(base_max=m["base"]["max_mm"], coverage=m.get("trace", {}).get("coverage_5mm"), valid_fraction=m["contact"]["valid_fraction"], completed=m["completed"], over_period=m["timing"]["over_period_fraction"]))
def drift(e):
    for r in e.rows: r["x"] = r["x"].copy(); r["x"][0] += 0.03
def detached(e):
    for r in e.rows:
        if r["phase"] in ("trace", "endhold"): r["F"] = 0.0
    e.pl.log["Fn"] = [0.0 if (k > len(e.pl.log["Fn"]) // 3) else v for k, v in enumerate(e.pl.log["Fn"])]
def partial(e): e.rows = [r for r in e.rows if r["phase"] in ("hover", "position", "approach", "settle")]; e.task = copy.copy(e.task); e.task.phase = "settle"
def late(e):
    for k, r in enumerate(e.rows): r["cycle_s"] = 0.03 if k % 5 == 0 else r["cycle_s"]
for n, f in (("drifting_base", drift), ("detached_tool_in_trace", detached), ("partial_path", partial), ("deadline_overruns", late)): variant(n, f)
json.dump(out, open(os.path.join(os.path.dirname(__file__), "..", "results", "am3_predictor", "evaluator_counterexamples.json"), "w"), indent=1, default=str); print(json.dumps(out, indent=1, default=str))
