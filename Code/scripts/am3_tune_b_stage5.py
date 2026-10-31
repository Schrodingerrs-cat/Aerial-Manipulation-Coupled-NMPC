"""Stage 5b of the B tuning: the 12 gain sets with the smallest linearised spectral radius (stability_search.json) are run in MuJoCo with noise: 4 stationary postures x {rest, kick} for 30 s (must not lose control; 20-30 s window
reported) and the six development scenarios.  Score = hover failures (1e3 each) + mean dev base RMS (mm) + 0.5 mean dev pad RMS.  All results kept in results/am3_b_tuning/stage5.json."""
import sys, os, json, numpy as np, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
SC = ("free", "free_big", "line", "line_T1", "circle", "rounded_triangle")
def evaluate(g):
    from am3_hover_check import run as hover, POSTURES
    from am3_run import free_waypoints
    from ocrl.am3_ctrl_b import BaselineB
    from ocrl.am3_episode import Episode, BASE
    hov = {}
    for n, (gap, yz) in POSTURES.items():
        for kick in (False, True): r = hover(BaselineB(g), gap, yz, 30.0, kick=kick); hov[f"{n}{'_kick' if kick else ''}"] = dict(abort=r["abort"] != "None", late=r["windows"].get("20-30s"), final_tilt=r["final_tilt_deg"])
    dev = []
    for sc in SC:
        wp, seg = free_waypoints(sc) if sc.startswith("free") else (None, None); ep = Episode(BaselineB(g), sc, T_max=40.0, free_waypoints=wp, segment_s=seg).run(); R = ep.rows; X = np.array([r["x"][0:3] for r in R]) - BASE; pe = np.array([r["pad"] - r["ref"]["p"] for r in R])
        dev.append(dict(sc=sc, ok=bool(ep.abort is None and ep.task.phase == "done"), base_rms=float(np.sqrt((X ** 2).sum(1).mean()) * 1e3), pad_rms=float(np.sqrt((pe ** 2).sum(1).mean()) * 1e3)))
    fails = sum(h["abort"] or (h["final_tilt"] is not None and h["final_tilt"] > 2.0) for h in hov.values()) + sum(not d["ok"] for d in dev)
    return dict(g=g, hover=hov, dev=dev, score=1e3 * fails + float(np.mean([d["base_rms"] for d in dev]) + 0.5 * np.mean([d["pad_rms"] for d in dev])))
if __name__ == "__main__":
    S = sorted(json.load(open(os.path.join(HERE, "..", "results", "am3_b_tuning", "stability_search.json"))), key=lambda o: o["rho_max"])[:12]
    with mp.get_context("spawn").Pool(8) as pool: out = pool.map(evaluate, [o["g"] for o in S], chunksize=1)
    for o, s in zip(out, S): o["rho_max"] = s["rho_max"]
    json.dump(out, open(os.path.join(HERE, "..", "results", "am3_b_tuning", "stage5.json"), "w"), indent=1)
    for o in sorted(out, key=lambda o: o["score"])[:5]: print(round(o["score"], 1), "rho", round(o["rho_max"], 5), {k: round(v, 3) for k, v in o["g"].items()}, [(d["sc"], d["ok"], round(d["base_rms"], 1)) for d in o["dev"]])
