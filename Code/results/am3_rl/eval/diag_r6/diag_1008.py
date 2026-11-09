"""R6 early-crash diagnosis: start features of seed 1008, replay of s0/s1/PD on it, early-crash rate on 300 unseen starts, and what drives s1's bad-RMS tail."""
import sys, json
import numpy as np
from stable_baselines3 import PPO
sys.path.insert(0, "."); sys.path.insert(0, "scripts")
from ocrl.am3_rl_direct import Track6Env, quat2R, Q_NOM, BASE
from am3_eval_track import REFS

M = {s: PPO.load(f"results/am3_rl/R6_turing_s{s}/model_final.zip", device="cpu") for s in (0, 1)}
env = Track6Env(seed=0, ref=REFS["big_fig8"])


def features(seed):
    env.reset(seed=seed); x = env.x0; R = np.array(quat2R(x[3:7]))
    return dict(p=np.linalg.norm(x[:3] - env.pref), v=np.linalg.norm(x[7:10]), tilt=np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))), w=np.linalg.norm(x[10:13]), qa1=abs(env.qa[0]), qa2=abs(env.qa[1]), qa3=abs(env.qa[2]))


F = {k: features(k) for k in range(1000, 1050)}
print("seed 1008 start, value (percentile among 1000-1049):")
for k in F[1008]:
    v = np.array([F[s][k] for s in F]); print(f"  {k:5s} {F[1008][k]:7.3f}  ({(v < F[1008][k]).mean() * 100:3.0f}%)")
env.reset(seed=1008); print("  raw p offset", np.round(env.x0[:3] - env.pref, 3), "v", np.round(env.x0[7:10], 3), "w", np.round(env.x0[10:13], 3))


def replay(pol, seed, n=120, log=True):
    obs, _ = env.reset(seed=seed); rows = []
    for k in range(n):
        a = env.expert_action(env.pl.state()) if pol == "pd" else pol.predict(obs, deterministic=True)[0]
        obs, r, term, trunc, info = env.step(a); x = env.pl.state(); rows.append((k, info["e"] * 1e3, info["tilt"], np.abs(a[:3]).max(), np.abs(a[3:6]).max(), np.linalg.norm(x[10:13]), x[2]))
        if term: break
    if log:
        for r in rows[::max(1, len(rows) // 14)] + rows[-1:]: print("   k=%3d e=%7.1f mm tilt=%5.1f |acc|max=%.2f |rate|max=%.2f |w|=%5.2f z=%.2f" % r)
    return term, rows


for name, p in (("s0", M[0]), ("s1", M[1]), ("PD", "pd")):
    t, rows = replay(p, 1008); print(f"{name} seed 1008: crashed={t} after {len(rows)} steps ({len(rows) * env.dt_c:.2f} s)")

print("\nearly-crash rate, first 2 s, seeds 2000-2299 (not used anywhere before):")
for s in (0, 1):
    bad = [k for k in range(2000, 2300) if replay(M[s], k, n=200, log=False)[0]]; print(f"  s{s}: {len(bad)}/300 crash", bad[:20])
bad = [k for k in range(2000, 2300) if replay("pd", k, n=200, log=False)[0]]; print(f"  PD: {len(bad)}/300 crash", bad[:20])

print("\ncorrelation of RMS with start features over survivors (big_fig8, n=50):")
d = json.load(open("results/am3_rl/eval/r6_eval_big_fig8_n50.json"))["big_fig8"]
for name, rows in d.items():
    if name == "zero": continue
    ok = [r for r in rows if r["survived"]]; y = np.log([r["rms_mm"] for r in ok]); c = {k: np.corrcoef([F[r["seed"]][k] for r in ok], y)[0, 1] for k in F[1000]}
    top = sorted(ok, key=lambda r: -r["rms_mm"])[:4]
    print(f"  {name[:14]:14s}", " ".join(f"{k}={v:+.2f}" for k, v in c.items()), "| worst:", [(r["seed"], round(r["rms_mm"], 1)) for r in top])
