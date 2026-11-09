"""Stationary-arm hover check (no wall contact, constant pad target): for several arm postures, noise on, 30 s, plus a finite-perturbation recovery run.  Reports base-position error and tilt per time window.
Usage: am3_hover_check.py CONTROLLER [T [GAINS_JSON]] -> results/am3_physics/hover_check_<controller>[_<gains>].json   (GAINS_JSON: B gains file, e.g. config/am3_b_gains_v5.json)"""
import sys, os, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_run import make_ctrl
from ocrl.am3_episode import Episode, initial_state, BASE
from ocrl.am3_pred import AMPred3
POSTURES = {"hover_gap60": (0.06, (0.0, 1.04)), "hover_off_right": (0.06, (0.04, 1.07)), "hover_off_left_low": (0.03, (-0.04, 1.01)), "near_contact_posture": (0.003, (0.0, 1.04))}
def run(ctrl, gap0, yz, T, seed=0, kick=False):
    x0, u0 = initial_state(gap0, yz)
    if kick: x0 = x0.copy(); x0[7:10] += [0.05, 0.03, 0.0]; x0[10:13] += [0.15, 0.15, 0.0]
    p0 = np.array(AMPred3().pad_f(x0)[0]).ravel(); ep = Episode(ctrl, "hover", seed=seed, T_max=T, free_waypoints=[p0] * (int(T / 5) + 2), segment_s=5.0, x0=(x0, u0)).run(); R = ep.rows; t = np.array([r["t"] for r in R]); X = np.array([r["x"] for r in R])
    e = np.linalg.norm(X[:, 0:3] - BASE, axis=1) * 1e3; tilt = np.degrees(2 * np.arcsin(np.minimum(1, np.hypot(X[:, 4], X[:, 5]))))
    win = [(0, 2), (2, 5), (5, 10), (10, 20), (20, 30)]; out = {f"{a}-{b}s": dict(base_max_mm=float(e[(t >= a) & (t < b)].max()) if ((t >= a) & (t < b)).any() else None, tilt_max_deg=float(tilt[(t >= a) & (t < b)].max()) if ((t >= a) & (t < b)).any() else None) for a, b in win}
    return dict(abort=str(ep.abort), windows=out, final_base_mm=float(e[-1]), final_tilt_deg=float(tilt[-1]))
if __name__ == "__main__":
    c = sys.argv[1]; T = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0; res = {}; gf = sys.argv[3] if len(sys.argv) > 3 else None; tagx = "_" + os.path.basename(gf).replace(".json", "") if gf else ""
    if gf:
        from ocrl.am3_ctrl_b import BaselineB; G = dict(tau_motor=0.05, **json.load(open(gf))["gains"]); make_ctrl = lambda name: BaselineB(G)
    for name, (g, yz) in POSTURES.items():
        res[name] = run(make_ctrl(c), g, yz, T); res[name + "_kick"] = run(make_ctrl(c), g, yz, T, kick=True)
        for k in (name, name + "_kick"): w = res[k]["windows"]; print(c, k.ljust(28), res[k]["abort"][:40], " ".join(f"{a}: {v['base_max_mm']:.1f}mm/{v['tilt_max_deg']:.2f}deg" for a, v in w.items() if v["base_max_mm"] is not None), flush=True)
    os.makedirs(os.path.join(HERE, "..", "results", "am3_physics"), exist_ok=True); json.dump(res, open(os.path.join(HERE, "..", "results", "am3_physics", f"hover_check_{c}{tagx}.json"), "w"), indent=1)
