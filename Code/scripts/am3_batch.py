"""Run a scenario list for one controller in parallel (one process per scenario, single-thread solver).  Usage: am3_batch.py TAG DOMAIN CONTROLLER [--workers W] [--enforce] [--miss P]
Writes results/am3_runs/<TAG>/<scenario id>/ and results/am3_runs/<TAG>/table.json"""
import sys, os, json, argparse, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
def one(args):
    tag, c, sc, enforce, miss = args; os.environ["OMP_NUM_THREADS"] = "1"
    from am3_run import run; ep, m = run(os.path.join(tag, sc["id"]), c, sc["path"], seed=sc["seed"], gap0=sc["gap0"], dyz=tuple(sc["dyz"]), enforce_deadline=enforce, mismatch=sc.get("mismatch"), miss_p=miss, outdir=os.path.join(HERE, "..", "results", "am3_runs", tag, sc["id"])); m["scenario"] = sc; return m
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("domain"); ap.add_argument("controller"); ap.add_argument("--workers", type=int, default=4); ap.add_argument("--enforce", action="store_true"); ap.add_argument("--miss", type=float, default=0.0); a = ap.parse_args()
    dom = json.load(open(os.path.join(HERE, "..", "config", f"am3_domain_{a.domain}.json")))["scenarios"]
    if a.controller != "B":                                                   # build the acados solver once, serially, before the workers start
        from am3_run import make_ctrl; make_ctrl(a.controller)
    with mp.get_context("spawn").Pool(a.workers) as pool: rows = pool.map(one, [(a.tag, a.controller, sc, a.enforce, a.miss) for sc in dom], chunksize=1)
    json.dump(rows, open(os.path.join(HERE, "..", "results", "am3_runs", a.tag, "table.json"), "w"), indent=1, default=str)
    for m in rows: print(m["scenario"]["id"], m["scenario"]["path"], m["scenario"].get("family", ""), "done" if m["completed"] else f"NOT done {m['abort']}", "base rms %.2f max %.1f" % (m["base"]["rms_mm"], m["base"]["max_mm"]))
