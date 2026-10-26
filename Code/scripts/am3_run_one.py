"""Run one scenario of a domain list (used to complete a batch that was interrupted).  Usage: am3_run_one.py TAG DOMAIN CONTROLLER SCENARIO_ID"""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_run import run
tag, dom, c, sid = sys.argv[1:5]; sc = next(s for s in json.load(open(os.path.join(HERE, "..", "config", f"am3_domain_{dom}.json")))["scenarios"] if s["id"] == sid)
ep, m = run(f"{tag}/{sid}", c, sc["path"], seed=sc["seed"], gap0=sc["gap0"], dyz=tuple(sc["dyz"]), outdir=os.path.join(HERE, "..", "results", "am3_runs", tag, sid)); print(sid, m["completed"], m["abort"])
