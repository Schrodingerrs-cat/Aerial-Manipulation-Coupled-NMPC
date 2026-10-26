"""Development tuning of baseline B on the free-space arm motion (fixed waypoints, no wall): grid over attitude/position integral gains and joint gains; score = RMS base error + RMS pad error (mm), failures scored 1e3.
Usage: am3_tune_b.py -> results/am3_b_tuning/free_arm_grid.json (every evaluated set is kept)"""
import sys, os, json, itertools, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ocrl.am3_episode import *
from ocrl.am3_ctrl_b import BaselineB
x0, u0 = initial_state(); p0 = np.array(AMPred3().pad_f(x0)[0]).ravel(); WPS = [p0 + [0, 0.06, 0.03], p0 + [0.02, -0.06, 0.03], p0 + [0, 0, -0.03], p0]
def score(g, seed=0):
    ep = Episode(BaselineB(g), "free", free_waypoints=WPS, T_max=12.0, seed=seed).run(); R = ep.rows; X = np.array([r["x"][0:3] for r in R]) - BASE; pe = np.array([r["pad"] - r["ref"]["p"] for r in R])
    return dict(abort=ep.abort is not None, base_rms=float(np.sqrt((X ** 2).sum(1).mean()) * 1e3), base_max=float(np.linalg.norm(X, axis=1).max() * 1e3), pad_rms=float(np.sqrt((pe ** 2).sum(1).mean()) * 1e3), pad_max=float(np.linalg.norm(pe, axis=1).max() * 1e3))
if __name__ == "__main__":
    out = []
    for kia, kip, kpp, w in itertools.product((16, 60, 150, 300), (0.3, 1.0), (2.0, 4.0), (6.0, 8.0)):
        g = dict(ki_att=kia, ki_pos=kip, kp_pos=kpp, kd_pos=1.4 * kpp ** 0.5 * 1.2, w_att=w); r = score(g); out.append(dict(g=g, **r))
        print(g, "abort" if r["abort"] else "", "base rms %.1f max %.1f  pad rms %.1f max %.1f mm" % (r["base_rms"], r["base_max"], r["pad_rms"], r["pad_max"]), flush=True)
    os.makedirs(os.path.join(os.path.dirname(__file__), "..", "results", "am3_b_tuning"), exist_ok=True); json.dump(out, open(os.path.join(os.path.dirname(__file__), "..", "results", "am3_b_tuning", "free_arm_grid.json"), "w"), indent=1)
