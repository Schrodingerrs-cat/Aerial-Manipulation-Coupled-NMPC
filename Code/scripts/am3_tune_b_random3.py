"""Development tuning of baseline B, fourth stage (v3 was tuned on four scenarios; a vigorous free-space arm motion `free_big` was then found to excite a growing oscillation): seeded random local search, 8 processes, scored on six development
scenarios (free, free_big, line, line_T1, circle, rounded_triangle; fixed seed 0): score = mean over scenarios of base RMS [mm] + 0.5 pad RMS, failures 1e3 each.  All samples kept in results/am3_b_tuning/random_search3.json."""
import sys, os, json, numpy as np, multiprocessing as mp
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..")); sys.path.insert(0, os.path.dirname(__file__))
N_SAMPLES = 120; SC = ("free", "free_big", "line", "line_T1", "circle", "rounded_triangle")
def sample(rng, center):
    kp = center["kp_pos"] * np.exp(rng.uniform(-0.4, 0.4)); return dict(w_att=center["w_att"] * np.exp(rng.uniform(-0.25, 0.25)), ki_att=center["ki_att"] * np.exp(rng.uniform(-0.8, 0.8)), i_att_max=rng.uniform(0.6, 3.0), kp_pos=kp, kd_pos=2 * kp ** 0.5 * rng.uniform(0.8, 1.5),
        ki_pos=center["ki_pos"] * np.exp(rng.uniform(-0.8, 0.8)), lag_gain=rng.uniform(0.2, 0.9), kp_joint=center["kp_joint"] * np.exp(rng.uniform(-0.5, 0.5)), kd_joint=rng.uniform(0.015, 0.04), k_corr=rng.uniform(0.0, 0.2))
def evaluate(g):
    from ocrl.am3_episode import Episode, initial_state, BASE
    from ocrl.am3_ctrl_b import BaselineB
    from am3_run import free_waypoints
    res = []
    for sc in SC:
        wp, seg = free_waypoints(sc) if sc.startswith("free") else (None, None); ep = Episode(BaselineB(g), sc, T_max=40.0, free_waypoints=wp, segment_s=seg).run(); R = ep.rows; X = np.array([r["x"][0:3] for r in R]) - BASE; pe = np.array([r["pad"] - r["ref"]["p"] for r in R]); ok = ep.abort is None and ep.task.phase == "done"
        res.append(dict(sc=sc, ok=bool(ok), base_rms=float(np.sqrt((X ** 2).sum(1).mean()) * 1e3), pad_rms=float(np.sqrt((pe ** 2).sum(1).mean()) * 1e3)))
    return dict(g=g, per=res, score=float(np.mean([1e3 if not r["ok"] else r["base_rms"] + 0.5 * r["pad_rms"] for r in res])))
if __name__ == "__main__":
    center = json.load(open(os.path.join(os.path.dirname(__file__), "..", "config", "am3_b_gains_v3.json")))["gains"]; rng = np.random.default_rng(2); samples = [center] + [sample(rng, center) for _ in range(N_SAMPLES - 1)]
    with mp.get_context("spawn").Pool(8) as pool: out = pool.map(evaluate, samples, chunksize=1)
    json.dump(out, open(os.path.join(os.path.dirname(__file__), "..", "results", "am3_b_tuning", "random_search3.json"), "w"), indent=1)
    best = sorted(out, key=lambda r: r["score"])[:3]
    for b in best: print(round(b["score"], 1), {k: round(v, 3) for k, v in b["g"].items()}, [(r["sc"], r["ok"], round(r["base_rms"], 1)) for r in b["per"]])
    print("center", round(out[0]["score"], 1), "n_ok_all", sum(all(r["ok"] for r in o["per"]) for o in out), "of", len(out))
