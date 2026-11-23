"""Energy check for the hanging-load A->B move (2026-11-23, professor's question: does using the swing cost more power?).
Same cases, plans and controllers as qp_run.py (U best duration, S, E + LQR), re-flown with every rotor's actual thrust logged (plant ctrl, after the motor lag).
Fair window: every flight is scored over the SAME 5 s (all are settled by then; the faster one just hovers at B for the rest).
Rotor power from momentum theory, P_i = T_i^1.5 / sqrt(2 rho A): reported relative to hovering for the same time, so the rotor disc area cancels.
Also: thrust impulse over the window, peak total thrust, peak single-rotor thrust, energy until settled.
Usage: qp_energy.py [--window 5] [--seeds 10]  (seeds: noisy-sensing repeats as in qp_noise.py; 0 = exact state only)"""
import sys, os, json, argparse
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.qp_plant import MQ, ML, G
from ocrl.qp_ocp import plan, minjerk
from ocrl.qp_lqr import from_plan
from qp_run import fly, pd, metrics

U_BEST_T = 4.75                       # best normal-drone duration from qp_run (ab_2m.json, exact state)


def score(ctrl, A, B, T, win, real=None):
    log = []; rows = fly(ctrl, A, max(win, 10.0), on_step=lambda P, t: log.append((t, P.d.ctrl[:4].copy())), real=real)
    t = np.array([x[0] for x in log]); F = np.array([x[1] for x in log]); dt = t[1] - t[0]; w = t <= win + 1e-9
    hover = (MQ + ML) * G / 4; e_hover = 4 * hover ** 1.5 * win             # 4 rotors hovering for the whole window
    m = metrics(rows, B, T); ts = m["settle_s"]
    return dict(settle_s=ts, energy_rel_hover=float(np.sum(F[w] ** 1.5) * dt / e_hover), impulse_Ns=float(F[w].sum() * dt),
                energy_to_settle_rel=float(np.sum(F[t <= ts] ** 1.5) * dt / (4 * hover ** 1.5)) if np.isfinite(ts) else None,   # in "seconds of hover"
                peak_total_N=float(F.sum(1).max()), peak_rotor_N=float(F.max()), hover_rotor_N=hover)


def main(a):
    A = np.array([0.0, 0.0, 1.5]); B = A + [2.0, 0.0, 0.0]
    plans = dict(U=minjerk(A, B, U_BEST_T), S=plan(A, B, "S"), E=plan(A, B, "E"))
    ctrls = dict(U=lambda: pd(plans["U"]), S=lambda: from_plan(plans["S"]), E=lambda: from_plan(plans["E"]))
    out = dict(window_s=a.window, note=__doc__.splitlines()[0])
    for c in ctrls:
        ex = score(ctrls[c](), A, B, plans[c]["T"], a.window); out[c] = dict(exact=ex)
        if a.seeds:
            rs = [score(ctrls[c](), A, B, plans[c]["T"], a.window, real=dict(seed=k)) for k in range(a.seeds)]
            out[c]["noise_mean"] = {k: float(np.mean([r[k] for r in rs if r[k] is not None])) for k in ex}
        print(c, json.dumps(out[c]), flush=True)
    od = os.path.join(HERE, "..", "results", "payload"); json.dump(out, open(os.path.join(od, a.name + ".json"), "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--window", type=float, default=5.0); ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--name", default="energy_ab_2m"); main(ap.parse_args())
