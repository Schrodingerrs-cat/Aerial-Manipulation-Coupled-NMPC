"""Payload study under realistic sensing (2026-11-11): the A->B (2 m) and zigzag-obstacle tasks rerun with sensor noise + one-period delay
(qp_run.SIG), and with noise + wind.  Same plans, trackers, plant and settle test as qp_run / qp_obstacle; truth is used only for scoring.
Every case is flown with 10 noise seeds.  The unaware drone (U) again gets its best duration, now chosen on the mean over the seeds (and, with posts, no hit in any seed).
Usage: qp_noise.py [--seeds 10] [--procs 20]"""
import sys, os, json, time, argparse
import numpy as np
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from qp_run import fly, metrics, pd
from qp_obstacle import POSTS, VIA, N as N_OBS, clearance
from ocrl.qp_ocp import plan, minjerk
from ocrl.qp_lqr import from_plan

A = np.array([0.0, 0.0, 1.5]); B_AB = A + [2.0, 0.0, 0.0]; B_OB = A + [3.0, 0.0, 0.0]
LEVELS = dict(exact=None, noise=dict(wind=False), noise_wind=dict(wind=True))


def run(job):
    task, case, pl, kind, level, seed = job; posts = POSTS if task == "obstacle" else (); B = B_OB if task == "obstacle" else B_AB
    if kind == "nmpc":
        from ocrl.qp_nmpc import from_plan as nmpc_from_plan; ctrl = nmpc_from_plan(pl)
    else: ctrl = pd(pl) if kind == "pd" else from_plan(pl)
    real = None if LEVELS[level] is None else dict(LEVELS[level], seed=seed)
    rows = fly(ctrl, A, 10.0, posts=posts, real=real); m = metrics(rows, B, pl["T"])
    if posts: m.update(clearance(rows)); m["hit"] = bool(min(m["drone_clear_cm"], m["load_clear_cm"]) < 0)
    return (task, case, level, seed), m


def summary(ms):
    st = np.array([m["settle_s"] for m in ms]); fin = np.isfinite(st)
    out = dict(n=len(ms), settled=int(fin.sum()), settle_mean=float(st[fin].mean()) if fin.any() else None, settle_max=float(st.max()), swing_max=float(max(m["swing_max_deg"] for m in ms)))
    if "hit" in ms[0]: out.update(hits=int(sum(m["hit"] for m in ms)), drone_clear_min_cm=float(min(m["drone_clear_cm"] for m in ms)), load_clear_min_cm=float(min(m["load_clear_cm"] for m in ms)))
    return out


def main(a):
    t0 = time.time(); seeds = range(a.seeds); P = {}
    for k in ("S", "E"): P[("ab", k)] = plan(A, B_AB, k); P[("obstacle", k)] = plan(A, B_OB, k, N=N_OBS, posts=POSTS, via=VIA)
    for T in np.arange(1.5, 5.01, 0.25): P[("ab", f"U{T:.2f}")] = minjerk(A, B_AB, T)
    for T in np.arange(2.0, 6.01, 0.5):
        try: P[("obstacle", f"U{T:.2f}")] = plan(A, B_OB, "U", N=N_OBS, posts=POSTS, via=VIA, T_fix=float(T))
        except RuntimeError: pass
    print(f"plans done {time.time() - t0:.0f} s", flush=True)
    jobs = [(task, c, pl, "pd" if c.startswith("U") else "lqr", lv, s) for (task, c), pl in P.items() for lv in LEVELS for s in (seeds if LEVELS[lv] else [0])]
    nm = [("ab", "ENMPC", P[("ab", "E")], "nmpc", lv, s) for lv in LEVELS for s in (seeds if LEVELS[lv] else [0])]
    with Pool(a.procs) as pool: res = dict(pool.map(run, jobs, chunksize=1))
    res.update(dict(map(run, nm)))                                                       # acados runs one at a time (shared generated solver)
    print(f"flights done {time.time() - t0:.0f} s", flush=True)
    out = {}
    for task in ("ab", "obstacle"):
        for lv in LEVELS:
            ss = [s for s in (seeds if LEVELS[lv] else [0])]; cases = sorted({c for (t, c, l, _) in res if t == task and l == lv})
            sm = {c: summary([res[(task, c, lv, s)] for s in ss]) for c in cases}
            us = {c: v for c, v in sm.items() if c.startswith("U") and v["settled"] == v["n"] and v.get("hits", 0) == 0}
            best = min(us, key=lambda c: us[c]["settle_mean"]) if us else None
            out[f"{task}/{lv}"] = dict(U_best=best, cases={c: v for c, v in sm.items() if not c.startswith("U") or c == best}, U_sweep={c: v for c, v in sm.items() if c.startswith("U")})
            for c, v in out[f"{task}/{lv}"]["cases"].items(): print(f"{task:8} {lv:10} {c:7}", {k: (round(x, 2) if isinstance(x, float) else x) for k, x in v.items()}, flush=True)
    json.dump(dict(sig=__import__("qp_run").SIG, seeds=a.seeds, results=out), open(os.path.join(HERE, "..", "results", "payload", "noise_study.json"), "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--seeds", type=int, default=10); ap.add_argument("--procs", type=int, default=20); main(ap.parse_args())
