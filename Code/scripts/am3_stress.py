"""Headless B-versus-N search over a pre-registered grid of tasks (config/am3_stress_v1.json).  Both controllers run each scenario unchanged (same model, plant, sensors, seed, task logic, acceptance criteria); the grid's extra
paths are injected into TASK['paths'] at run time, nothing else is modified.  Usage: [AM3_STRESS_GRID=am3_stress_v2] am3_stress.py TAG [--workers W] [--ctrl B,N] -> results/am3_stress/<TAG>/{<ctrl>_<id>/summary.json, table.json}"""
import sys, os, json, argparse, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
CFG = json.load(open(os.path.join(HERE, "..", "config", os.environ.get("AM3_STRESS_GRID", "am3_stress_v1") + ".json")))
def init():
    os.environ["OMP_NUM_THREADS"] = "1"; from ocrl.am3_task import TASK; TASK["paths"].update(CFG["paths"])
def one(a):
    tag, c, sc = a; from am3_run import run; ep, m = run(f"{tag}/{c}_{sc['id']}", c, sc["path"], seed=sc["seed"], gap0=sc["gap0"], dyz=tuple(sc["dyz"]), outdir=os.path.join(HERE, "..", "results", "am3_stress", tag, f"{c}_{sc['id']}")); m["scenario"] = sc; m["ctrl"] = c; return m
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--workers", type=int, default=8); ap.add_argument("--ctrl", default="B,N"); a = ap.parse_args()
    init(); from am3_run import make_ctrl
    if "N" in a.ctrl: make_ctrl("N")                                          # build the acados solver once before the workers start
    with mp.get_context("spawn").Pool(a.workers, initializer=init) as pool: rows = pool.map(one, [(a.tag, c, sc) for sc in CFG["scenarios"] for c in a.ctrl.split(",")], chunksize=1)
    os.makedirs(os.path.join(HERE, "..", "results", "am3_stress", a.tag), exist_ok=True); json.dump(rows, open(os.path.join(HERE, "..", "results", "am3_stress", a.tag, "table.json"), "w"), indent=1, default=str)
