"""Learned tail policies on the tail study's square route (2026-11-27, capstone).  Same route, dwell and metrics as scripts/am3_tail_run.py; lab sensing.
Usage: am3_tail_rl_eval.py NAME MODEL_ZIP:arm [MODEL_ZIP:arm ...] [--T 2.0 1.5 1.2 1.0 0.9] [--mb 0 0.15 0.2] [--seeds 3]
arm is 'free' or 'rigid' (the env the policy was trained in).  Writes Code/results/tail/NAME.jsonl (one row per run)."""
import sys, os, json, argparse, numpy as np
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
from ocrl.am3_rl_tail import TailEnv
from ocrl.am3_rl_direct import LabSensors
from ocrl.am3_plant import KF
from am3_tail_run import ROUTE, DWELL


def run(job):
    from stable_baselines3 import PPO
    model, arm, T, mb, seed = job
    pol = PPO.load(model, device="cpu")
    env = TailEnv(seed=seed, max_steps=1000, arm=arm, fixed=dict(route=ROUTE, T=T, dwell=DWELL, mb=mb)); env.sens = LabSensors(seed)
    o, _ = env.reset(); X, P, bad = [env.pl.state()], [env.pref.copy()], False
    for _ in range(env.max_steps):
        o, r, bad, trunc, info = env.step(pol.predict(o, deterministic=True)[0]); X.append(env.pl.state()); P.append(env.pref.copy())
        if bad or trunc: break
    X, P = np.array(X), np.array(P); dt = env.dt_c; t = np.arange(len(X)) * dt; e = np.linalg.norm(X[:, 0:3] - P, axis=1)
    settle = []
    for k in range(1, len(ROUTE)):
        ta, tb = env.t0 + k * (T + DWELL) - DWELL, env.t0 + k * (T + DWELL); m = (t >= ta) & (t < tb)
        ok = (np.linalg.norm(X[m, 0:3] - ROUTE[k], axis=1) < 0.02) & (np.linalg.norm(X[m, 7:10], axis=1) < 0.05)
        bd = np.where(~ok)[0]; settle.append(None if not len(ok) or (len(bd) and bd[-1] == len(ok) - 1) else float(t[m][bd[-1] + 1] - ta) if len(bd) else 0.0)
    acc = np.gradient(X[:, 7:10], dt, axis=0); jerk = np.gradient(acc, dt, axis=0)
    F = KF * X[:, 19:23] ** 2; hover = env.pl.m.body_subtreemass[1] * 9.81 / 4; tilt = np.degrees(2 * np.arcsin(np.clip(np.hypot(X[:, 4], X[:, 5]), 0, 1)))
    return dict(model=model, arm=arm, T=T, mb=mb, seed=seed, completed=not bad, t_end=float(t[-1]),
                base_rms_mm=float(np.sqrt(np.mean(e ** 2)) * 1e3), base_max_mm=float(e.max() * 1e3), settle_s=settle,
                settle_mean_s=float(np.mean(settle)) if all(s is not None for s in settle) and not bad else None,
                tilt_max_deg=float(tilt.max()), jerk_rms=float(np.sqrt(np.mean(np.sum(jerk ** 2, 1)))), energy_rel_hover=float(np.sum(F ** 1.5) / (4 * hover ** 1.5 * len(F))),
                joint_range_deg=list(np.degrees(X[:, 13:16].max(0) - X[:, 13:16].min(0)).round(1)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("name"); ap.add_argument("models", nargs="+"); ap.add_argument("--T", type=float, nargs="+", default=[2.0, 1.5, 1.2, 1.0, 0.9])
    ap.add_argument("--mb", type=float, nargs="+", default=[0.0, 0.15, 0.2]); ap.add_argument("--seeds", type=int, default=3); ap.add_argument("--procs", type=int, default=8); a = ap.parse_args()
    jobs = [(*m.rsplit(":", 1), T, mb, s) for m in a.models for T in a.T for mb in a.mb for s in range(a.seeds)]
    out = os.path.join(HERE, "..", "results", "tail"); os.makedirs(out, exist_ok=True)
    with Pool(a.procs, maxtasksperchild=1) as pool, open(os.path.join(out, a.name + ".jsonl"), "w") as f:
        for i, r in enumerate(pool.imap_unordered(run, jobs)):
            f.write(json.dumps(r, default=float) + "\n"); f.flush()
            print(i + 1, "/", len(jobs), r["arm"], r["T"], r["mb"], r["seed"], r["completed"], round(r["base_rms_mm"], 1), round(r["tilt_max_deg"], 1), flush=True)
