"""Which channel spins the body on crashing starts, and what do crashing starts have in common (200-step window, big_fig8)."""
import sys
import numpy as np
from stable_baselines3 import PPO
sys.path.insert(0, "."); sys.path.insert(0, "scripts")
from ocrl.am3_rl_direct import Track6Env, quat2R
from am3_eval_track import REFS

M = {s: PPO.load(f"results/am3_rl/R6_turing_s{s}/model_final.zip", device="cpu") for s in (0, 1)}
env = Track6Env(seed=0, ref=REFS["big_fig8"])


def feat(seed):
    env.reset(seed=seed); x = env.x0; R = np.array(quat2R(x[3:7])); d = x[:3] - env.pref
    return np.array([np.linalg.norm(d), d[2], np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))), np.linalg.norm(x[10:13]), np.linalg.norm(x[7:10])])


obs, _ = env.reset(seed=1008); print("s0 seed 1008: k, a_acc(xyz), a_rate(xyz), a_yaw, w_body(xyz), arm q-Qnom")
for k in range(65):
    a = M[0].predict(obs, deterministic=True)[0]; obs, r, term, trunc, info = env.step(a); x = env.pl.state()
    if k % 6 == 0 or term: print(f" {k:3d}", np.round(a[:3], 2), np.round(a[3:6], 2), round(float(a[6]), 2), np.round(x[10:13], 1), np.round(x[13:16] - env.eq_q if hasattr(env, 'eq_q') else x[13:16], 2), f"tilt {info['tilt']:.0f}")
    if term: break

names = "dist z_off tilt w v".split()
all_f = np.array([feat(k) for k in range(2000, 2300)])
print("population (seeds 2000-2299) median / p95:", {n: (round(float(np.median(all_f[:, i])), 2), round(float(np.percentile(all_f[:, i], 95)), 2)) for i, n in enumerate(names)})
for k, who in [(1008, "s0"), (2019, "s0"), (2174, "s0"), (2188, "s0"), (2217, "s0"), (2220, "s0,s1"), (2073, "s1"), (2170, "s1")]:
    print(k, who, dict(zip(names, np.round(feat(k), 2))))
