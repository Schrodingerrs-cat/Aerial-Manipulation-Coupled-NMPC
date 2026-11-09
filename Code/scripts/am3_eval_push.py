"""Push-recovery test: a force step on the base (0.5 s, in each of six directions, several magnitudes) while flying big_fig8, RL policy vs scripted PD+ff on the same start.
Reported per policy and force: survival, peak error during/after the push, and time after the push ends until the error is back under 20 mm and stays there for 1 s.
Usage: am3_eval_push.py results/am3_rl/R4_x/model_1000k.zip [--env track4] [--forces 1,2,4] [--start 100]"""
import sys, os, json, argparse
import numpy as np
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
from ocrl.am3_rl_direct import Track3Env, Track4Env, Track5Env, Track6Env, Track7Env
from am3_eval_track import REFS

T_PUSH, DUR = 8.0, 0.5
DIRS = [("+x", 0, 1), ("-x", 0, -1), ("+y", 1, 1), ("-y", 1, -1), ("+z", 2, 1), ("-z", 2, -1)]


def run(env, pol, seed, axis, amp):
    obs, _ = env.reset(seed=seed); env.pl.dist = [dict(kind="step", t0=T_PUSH, dur=DUR, axis=axis, amp=amp)]; E = []
    while True:
        a = env.expert_action(env.pl.state()) if pol == "pd" else pol.predict(obs, deterministic=True)[0]
        obs, r, term, trunc, info = env.step(a); E.append(info["e"])
        if term or trunc: break
    e = np.array(E) * 1e3; k0 = int(T_PUSH / env.dt_c); k1 = k0 + int(DUR / env.dt_c); w = int(1.0 / env.dt_c)
    peak = float(e[k0:].max()) if len(e) > k0 else float("nan"); rec = None
    for k in range(k1, len(e) - w):
        if e[k:k + w].max() < 20.0: rec = (k - k1) * env.dt_c; break
    return dict(survived=not term, peak_mm=peak, recovery_s=rec)


def main(a):
    Env = dict(track3=Track3Env, track4=Track4Env, track5=Track5Env, track6=Track6Env, track7=Track7Env)[a.env]; env = Env(seed=0, ref=REFS["big_fig8"]); pol = PPO.load(a.model, device="cpu")
    out = {}; print(f"{'force':>6s} {'dir':>4s} | {'PD+ff survived / peak mm / recovery s':>40s} | {'RL survived / peak mm / recovery s':>36s}")
    for F in [float(f) for f in a.forces.split(",")]:
        for name, ax, sg in DIRS:
            rows = {}
            for who, p in (("pd", "pd"), ("rl", pol)): rows[who] = run(env, p, a.start, ax, sg * F)
            out[f"{F}N{name}"] = rows
            f = lambda r: f"{'yes' if r['survived'] else 'LOST'} / {r['peak_mm']:7.1f} / {('%.2f' % r['recovery_s']) if r['recovery_s'] is not None else '  - '}"
            print(f"{F:5.1f}N {name:>4s} | {f(rows['pd']):>40s} | {f(rows['rl']):>36s}", flush=True)
    od = os.path.join(HERE, "..", "results", "am3_rl", "eval"); json.dump(out, open(os.path.join(od, a.out), "w"), indent=1); print("written", a.out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("model"); ap.add_argument("--env", default="track4"); ap.add_argument("--forces", default="1,2,4"); ap.add_argument("--start", type=int, default=100); ap.add_argument("--out", default="push_eval.json"); main(ap.parse_args())
