"""Stage 6 of the B tuning: local search around the best stationary-stable candidate of stage 5 (results/am3_b_tuning/stage5.json).  600 log-normal perturbations of all gains; the linearised spectral radius
over the four postures must be < 0.9993; the 16 best-conditioned survivors are run through the same MuJoCo evaluation as stage 5 (30 s stationary hover x 4 postures x {rest, kick}; six development scenarios).
All samples are kept: results/am3_b_tuning/stage6.json."""
import sys, os, json, numpy as np, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_b_stability_search import evaluate as lin_eval
from am3_tune_b_stage5 import evaluate as mj_eval
def perturb(rng, g, s=0.25):
    q = lambda v, lo, hi: float(np.clip(v * np.exp(rng.normal(0, s)), lo, hi)); kp = q(g["kp_pos"], 1.0, 14.0)
    return dict(w_att=q(g["w_att"], 4, 16), ki_att=q(g["ki_att"], 50, 1800), kp_pos=kp, kd_pos=float(np.clip(g["kd_pos"] * np.exp(rng.normal(0, s)), 1.0, 8.0)), ki_pos=q(g["ki_pos"], 0.05, 2.5), lag_gain=float(np.clip(g["lag_gain"] + rng.normal(0, 0.2), 0, 1)), tau_motor=float(np.clip(g["tau_motor"] + rng.normal(0, 0.004), 0.04, 0.06)),
                kp_joint=q(g["kp_joint"], 0.2, 2.5), kd_joint=float(np.clip(g["kd_joint"] * np.exp(rng.normal(0, s)), 0.01, 0.045)), k_corr=float(np.clip(g["k_corr"] + rng.normal(0, 0.04), 0, 0.25)))
if __name__ == "__main__":
    best = sorted(json.load(open(os.path.join(HERE, "..", "results", "am3_b_tuning", "stage5.json"))), key=lambda o: o["score"])[0]["g"]; rng = np.random.default_rng(6); S = [best] + [perturb(rng, best) for _ in range(599)]
    with mp.get_context("spawn").Pool(8) as pool:
        lin = pool.map(lin_eval, S, chunksize=4)
        for o in lin: o["rho_max"] = max(o["rho"].values())
        ok = sorted([o for o in lin if o["rho_max"] < 0.9993], key=lambda o: o["rho_max"])[:16]; print("linear survivors", sum(o["rho_max"] < 0.9993 for o in lin), "of", len(lin), flush=True)
        out = pool.map(mj_eval, [o["g"] for o in ok], chunksize=1)
    for o, s in zip(out, ok): o["rho_max"] = s["rho_max"]
    json.dump(dict(linear=lin, mujoco=out), open(os.path.join(HERE, "..", "results", "am3_b_tuning", "stage6.json"), "w"), indent=1)
    for o in sorted(out, key=lambda o: o["score"])[:4]: print(round(o["score"], 1), "rho", round(o["rho_max"], 5), {k: round(v, 3) for k, v in o["g"].items()}, [(d["sc"], d["ok"], round(d["base_rms"], 1)) for d in o["dev"]])
