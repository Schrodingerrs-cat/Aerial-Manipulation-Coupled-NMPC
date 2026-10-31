"""Stage 5 of the B tuning: linearised closed-loop spectral radius (per 5 ms, predictor plant, noise-free) at four stationary arm postures; random search over all gains; keeps every sample.
The stationary-hover stability was missing from stages 1-4 (audit B01).  -> results/am3_b_tuning/stability_search.json"""
import sys, os, json, numpy as np, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
POST = {"hover_gap60": (0.06, (0.0, 1.04)), "off_right": (0.06, (0.04, 1.07)), "off_left_low": (0.03, (-0.04, 1.01)), "near_contact": (0.003, (0.0, 1.04))}
def sample(rng):
    kp = float(np.exp(rng.uniform(np.log(1.5), np.log(12)))); return dict(w_att=float(rng.uniform(4, 14)), ki_att=float(np.exp(rng.uniform(np.log(50), np.log(1500)))), kp_pos=kp, kd_pos=float(2 * kp ** 0.5 * rng.uniform(0.7, 1.6)), ki_pos=float(rng.uniform(0.05, 2.0)),
        lag_gain=float(rng.uniform(0, 1)), tau_motor=float(rng.uniform(0.04, 0.06)), kp_joint=float(np.exp(rng.uniform(np.log(0.2), np.log(2.0)))), kd_joint=float(rng.uniform(0.01, 0.04)), k_corr=float(rng.uniform(0, 0.2)))
def evaluate(g):
    import am3_b_linear as L
    return dict(g=g, rho={n: float(L.eigs(g, a, yz)[0]) for n, (a, yz) in POST.items()})
if __name__ == "__main__":
    center = json.load(open(os.path.join(HERE, "..", "config", "am3_b_gains_v4.json")))["gains"]; rng = np.random.default_rng(5); S = [center] + [sample(rng) for _ in range(399)]
    with mp.get_context("spawn").Pool(8) as pool: out = pool.map(evaluate, S, chunksize=4)
    for o in out: o["rho_max"] = max(o["rho"].values())
    json.dump(out, open(os.path.join(HERE, "..", "results", "am3_b_tuning", "stability_search.json"), "w"), indent=1); out.sort(key=lambda o: o["rho_max"])
    print("center rho_max", [o for o in out if o["g"] == center][0]["rho_max"]); print("stable (<0.999):", sum(o["rho_max"] < 0.999 for o in out), "of", len(out))
    for o in out[:5]: print(round(o["rho_max"], 5), {k: round(v, 3) for k, v in o["g"].items()})
