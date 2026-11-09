"""Which action channel causes R6's early crashes?  Override one channel of the trained policy for the first 2 s on 300 unseen starts (seeds 2000-2299, big_fig8).
Diagnostic only, not a controller.  Usage: intervene.py SEED VARIANT R6|R7 (now also runs seeds 1000-1049)"""
import sys, json
import numpy as np
from stable_baselines3 import PPO
sys.path.insert(0, "."); sys.path.insert(0, "scripts")
from ocrl.am3_rl_direct import Track6Env, Track7Env
from am3_eval_track import REFS

s, var, run = int(sys.argv[1]), sys.argv[2], sys.argv[3]
pol = PPO.load(f"results/am3_rl/{run}_turing_s{s}/model_final.zip", device="cpu"); env = (Track7Env if run == "R7" else Track6Env)(seed=0, ref=REFS["big_fig8"])


def edit(a):
    a = np.array(a, float)
    if var in ("yaw0", "yawrate0"): a[6] = 0.0
    if var in ("rate0", "yawrate0"): a[3:6] = 0.0
    if var == "rate05": a[3:6] *= 0.5
    if var == "acc07": a[:3] = np.clip(a[:3], -0.7, 0.7)
    if var == "acc05": a[:3] = np.clip(a[:3], -0.5, 0.5)
    return a


crash, big, wmax = [], [], []
for k in [*range(1000, 1050), *range(2000, 2300)]:
    obs, _ = env.reset(seed=k); e, w = [], []; term = False
    for n in range(200):
        obs, r, term, trunc, info = env.step(edit(pol.predict(obs, deterministic=True)[0])); e.append(info["e"]); w.append(np.linalg.norm(env.pl.state()[10:13]))
        if term: break
    if term: crash.append(k)
    elif max(e[50:]) > 0.6: big.append(k)
    wmax.append(max(w))
print(json.dumps(dict(seed=s, var=var, crash=len(crash), big_excursion=len(big), wmax_p50=round(float(np.median(wmax)), 2), wmax_p99=round(float(np.percentile(wmax, 99)), 2), crash_seeds=crash, big_seeds=big)))
