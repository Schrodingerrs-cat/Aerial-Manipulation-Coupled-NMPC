"""Development tuning of the decoupled-model NMPC Nd (same weights-only freedom as N) (no recompilation: weights are solver parameters).  Scenarios: fast paths with several noise seeds (dev seeds only).  Score = trace tracking RMS (mm) + 10 per contact loss +
1e3 per failure.  Every configuration is kept in results/am3_nd_tuning/weights.json.  Usage: am3_tune_n.py"""
import sys, os, json, itertools, numpy as np, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
CONFIGS = {"base": None, "tp_tv_x0.3": dict(tp=0.3, tv=0.3), "tp_tv_x0.1": dict(tp=0.1, tv=0.1), "du x5": dict(du=5.0), "du x20": dict(du=20.0), "w_qd x5": dict(qd=5.0, w=3.0), "att_w x3 du x5": dict(att=3.0, w=3.0, du=5.0), "all soft": dict(tp=0.2, tv=0.2, B=0.5, vB=0.5, du=5.0, qd=3.0, w=3.0), "all softer": dict(tp=0.1, tv=0.1, B=0.3, vB=0.3, du=15.0, qd=5.0, w=5.0, att=3.0)}
JOBS = [("line", 1), ("line", 2), ("circle", 1), ("free", 1)]
def one(args):
    name, mult, path, seed = args; import ocrl.am3_nmpc as nm
    if mult: 
        idx = {g: i for i, g in enumerate(nm.GROUPS)}
        for k in nm.W: nm.W[k] = tuple(w * mult.get(g, 1.0) for g, w in zip(nm.GROUPS, nm.W[k]))
    from am3_run import run; ep, m = run(f"tune_nd/{name}_{path}_{seed}", "Nd", path, seed=seed, outdir=os.path.join(HERE, "..", "results", "am3_nd_tuning", "runs", f"{name}_{path}_{seed}")); tr = m.get("trace", {})
    return dict(cfg=name, path=path, seed=seed, completed=m["completed"], track_rms=tr.get("track_rms_mm", float("nan")), losses=m["contact"]["losses"], base_max=m["base"]["max_mm"])
if __name__ == "__main__":
    from am3_run import make_ctrl; make_ctrl("Nd")
    with mp.get_context("spawn").Pool(8) as pool: rows = pool.map(one, [(n, c, p, s) for n, c in CONFIGS.items() for p, s in JOBS], chunksize=1)
    os.makedirs(os.path.join(HERE, "..", "results", "am3_nd_tuning"), exist_ok=True); json.dump(rows, open(os.path.join(HERE, "..", "results", "am3_nd_tuning", "weights.json"), "w"), indent=1)
    for n in CONFIGS:
        r = [x for x in rows if x["cfg"] == n]; print(n.ljust(12), "failures", sum(not x["completed"] for x in r), "losses", sum(x["losses"] for x in r), "track rms mean %.2f max %.2f" % (np.mean([x["track_rms"] for x in r]), np.max([x["track_rms"] for x in r])), "base max %.2f" % max(x["base_max"] for x in r))
