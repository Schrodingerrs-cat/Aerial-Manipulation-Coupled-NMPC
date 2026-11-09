"""Paired evaluation of a learned correction against the zero-action baseline.

Zero action reproduces nominal N bit-for-bit (verified, results/am3_runs/zparity_Z), so "policy vs zero action" is exactly "L vs N" measured inside the training environment.
Episodes are paired: episode k uses reset(seed=k) for every policy, so both see the identical scenario, plant mismatch and sensor noise.  A policy that cannot beat zero action
on the family it was trained for has not learned anything useful, whatever its training curve looks like.

Usage:
  am3_eval_L.py --domain mm2_val_contact --n 20 --policies zero results/am3_rl/PILOT_contact_s0/model_40k.zip
  -> results/am3_rl/eval/<name>.json and a printed table
"""
import sys, os, json, argparse, numpy as np
from stable_baselines3 import SAC          # imported BEFORE anything loads acados: importing torch after the acados shared libraries crashes the process
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))


def run_policy(name, domain, n, max_steps):
    """Returns per-episode return and task outcome for one policy over n paired episodes."""
    from ocrl.am3_rl import CorrectionEnv
    pool = json.load(open(os.path.join(HERE, "..", "config", f"am3_domain_{domain}.json")))["scenarios"]
    env = CorrectionEnv(seed=0, max_steps=max_steps, pool=pool)
    policy = None if name == "zero" else SAC.load(name, device="cpu")
    rows = []
    for k in range(n):
        obs, info = env.reset(seed=k); R = 0.0; steps = 0
        while True:
            a = np.zeros(2, np.float32) if policy is None else policy.predict(obs, deterministic=True)[0]
            obs, r, term, trunc, _ = env.step(a); R += r; steps += 1
            if term or trunc: break
        task = env.ep.task
        rows.append(dict(scenario=info["scenario"], ret=float(R), steps=steps,
                         completed=bool(task.phase == "done" and not env.ep.stop),
                         losses=len(task.recoveries), failed=task.failed))
        print(f"  [{name.split('/')[-1]:18s}] ep {k:3d} {info['scenario']:22s} return {R:8.1f}  steps {steps:4d}  "
              f"{'done' if rows[-1]['completed'] else 'NOT done'}  losses {rows[-1]['losses']}", flush=True)
    return rows


def main(a):
    res = {}
    for p in a.policies:
        res[p] = run_policy(p, a.domain, a.n, a.max_steps)
    base = res[a.policies[0]]
    print(f"\n{'policy':34s} {'mean return':>12s} {'vs zero':>9s} {'completed':>10s} {'losses/ep':>10s}")
    for p, rows in res.items():
        R = np.array([r["ret"] for r in rows]); comp = sum(r["completed"] for r in rows); los = np.mean([r["losses"] for r in rows])
        d = R - np.array([r["ret"] for r in base])
        wins = int((d > 0).sum()); name = p if p == "zero" else "/".join(p.split("/")[-2:])
        delta = "" if p == a.policies[0] else f"{d.mean():+9.1f}"
        print(f"{name:34s} {R.mean():12.1f} {delta:>9s} {comp:>7d}/{len(rows)} {los:10.2f}"
              + ("" if p == a.policies[0] else f"   better on {wins}/{len(rows)} paired episodes"))
    od = os.path.join(HERE, "..", "results", "am3_rl", "eval"); os.makedirs(od, exist_ok=True)
    out = os.path.join(od, f"{a.name or a.domain}.json")
    json.dump(dict(domain=a.domain, n=a.n, policies={k: v for k, v in res.items()}), open(out, "w"), indent=1, default=str)
    print(f"\nwritten: {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--domain", default="mm2_val_contact"); ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--max-steps", type=int, default=900); ap.add_argument("--name", default=None)
    ap.add_argument("--policies", nargs="+", required=True, help="first one is the reference; use 'zero' for nominal N")
    main(ap.parse_args())
