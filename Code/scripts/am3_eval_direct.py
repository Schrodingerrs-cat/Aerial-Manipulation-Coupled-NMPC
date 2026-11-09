"""Held-out evaluation of an end-to-end RL controller against two fixed references on IDENTICAL episodes.

Each episode k is reset with seed k, so every policy sees the same initial offset, velocity and attitude.  References:
  zero    zero action (commanded acceleration 0, nominal arm): the floor
  PD      scripted outer-loop PD through the same action interface: the bar a learned policy has to clear
Reported per policy: survival (reached the step cap without terminating), position error over the last 200 steps, peak tilt, and action roughness.
Training never uses seeds >= 100, so these are unseen starts.

Usage: am3_eval_direct.py results/am3_rl/R1_hover_v2_s0/model_final.zip [more.zip ...] [--n 40] [--steps 1000]
"""
import sys, os, json, argparse
import numpy as np
from stable_baselines3 import SAC, PPO          # first: torch must load before anything else heavy
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_rl_direct import HoverEnv, BASE


def pd_action(x, kp=5.0, kd=3.5):
    a = np.zeros(10); a[0:3] = np.clip((-kp * (x[0:3] - BASE) - kd * x[7:10]) / 5.0, -1, 1); return a


def rollout(env, policy, seed, steps):
    obs, _ = env.reset(seed=seed); E, T, dA = [], [], []; a_prev = np.zeros(10); terminated = False
    for i in range(steps):
        if policy == "zero": a = np.zeros(10)
        elif policy == "pd": a = pd_action(env.pl.state())
        else: a = policy.predict(obs, deterministic=True)[0]
        obs, r, term, trunc, info = env.step(a); E.append(info["e"]); T.append(info["tilt"]); dA.append(np.linalg.norm(np.asarray(a) - a_prev)); a_prev = np.asarray(a)
        if term: terminated = True; break
        if trunc: break
    return dict(seed=seed, steps=len(E), survived=not terminated, err_last_mm=float(np.mean(E[-200:]) * 1e3), err_max_mm=float(np.max(E) * 1e3),
                tilt_max=float(np.max(T)), rough=float(np.mean(dA)))


def main(a):
    env = HoverEnv(seed=0, max_steps=a.steps)
    pols = {"zero": "zero", "PD expert": "pd"}
    for p in a.models:
        pols[os.path.basename(os.path.dirname(p)) + "/" + os.path.basename(p)] = PPO.load(p, device="cpu")
    res = {}
    for name, pol in pols.items():
        res[name] = [rollout(env, pol, 100 + k, a.steps) for k in range(a.n)]
    print(f"\n{a.n} held-out episodes (seeds 100..{99 + a.n}), {a.steps} steps each\n")
    print(f"{'policy':36s} {'survived':>9s} {'err last200 mm':>15s} {'median':>8s} {'max err mm':>11s} {'tilt max':>9s} {'roughness':>10s}")
    for name, rows in res.items():
        s = sum(r["survived"] for r in rows); el = np.array([r["err_last_mm"] for r in rows if r["survived"]])
        print(f"{name:36s} {s:>6d}/{len(rows)} {(el.mean() if len(el) else float('nan')):>15.1f} {(np.median(el) if len(el) else float('nan')):>8.1f} "
              f"{np.mean([r['err_max_mm'] for r in rows]):>11.1f} {np.mean([r['tilt_max'] for r in rows]):>8.1f}° {np.mean([r['rough'] for r in rows]):>10.3f}")
    od = os.path.join(HERE, "..", "results", "am3_rl", "eval"); os.makedirs(od, exist_ok=True)
    json.dump(res, open(os.path.join(od, a.out), "w"), indent=1); print(f"\nwritten results/am3_rl/eval/{a.out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("models", nargs="+"); ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--steps", type=int, default=1000); ap.add_argument("--out", default="r1_eval.json"); main(ap.parse_args())
