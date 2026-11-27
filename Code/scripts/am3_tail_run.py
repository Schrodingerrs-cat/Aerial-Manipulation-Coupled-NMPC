"""Tail study (2026-11-27, capstone): arm rigid vs block held still vs arm as a tail vs load-unaware, carrying a block, on a square waypoint route at decreasing move times.
Usage: am3_tail_run.py NAME [--mb 0.15] [--T 2.0 1.5 1.2 1.0 0.8] [--modes rigid hold tail unaware] [--seeds 1] [--procs 12]
Writes Code/results/tail/NAME.json.  Metrics per run: completed, base tracking RMS/max (mm), settle time after each move (within 2 cm and 5 cm/s, s), max tilt, base jerk RMS,
rotor energy over the route relative to hovering (momentum theory, P ~ T^1.5), max joint torque / servo limit, joint range used."""
import sys, os, json, argparse, numpy as np
import time, queue, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
from ocrl.am3_tail import TailNMPC, WaypointTask, start_state
from ocrl.am3_episode import Episode
from ocrl.am3_task import BASE
from ocrl.am3_plant import KF
from am3_write_bench import LabSensors

OFF = np.array([0.20, 0.0, -0.15])                   # block (pad point) offset from the base: arm half folded, low shoulder torque
W0 = BASE + np.array([-0.15, 0.0, 0.0])
ROUTE = [W0, W0 + [-0.6, 0, 0], W0 + [-0.6, 0.6, 0], W0 + [0, 0.6, 0], W0]      # square, 0.6 m sides: fore-aft (arm pitch plane) and sideways moves
DWELL = 1.5
HANG_S = 300                                         # a 14 s route takes ~20 s; acados SQP-RTI can stall after the drone has lost control (2026-11-27: tail, T 0.9, seed 2)


def run(job):
    mode, T, mb, seed = job
    ctrl = TailNMPC("rigid" if mode == "unaware" else mode, mb=0.0 if mode == "unaware" else mb)
    x0, u0 = start_state(mb, W0, OFF)
    ep = Episode(ctrl, "free", seed=seed, mismatch=dict(payload=dict(kg=mb, pos_hand_m=[0.0, 0.0, 0.0])), free_waypoints=[[0.0, 0.0, 0.0]], segment_s=1.0, x0=(x0, u0), T_max=60.0)
    ep.task = WaypointTask(ROUTE, T, DWELL, OFF); ctrl.reset(ep.x0, ep.u0, ep.task); ep.sens = LabSensors(seed); ep.run()
    R = ep.rows; t = np.array([r["t"] for r in R]); X = np.array([r["x"] for r in R]); pB = np.array([r["ref"]["pB"] for r in R]); e = np.linalg.norm(X[:, 0:3] - pB, axis=1)
    settle = []
    for k in range(1, len(ROUTE)):
        ta, tb = 1.0 + k * (T + DWELL) - DWELL, 1.0 + k * (T + DWELL); m = (t >= ta) & (t < tb)
        ok = (np.linalg.norm(X[m, 0:3] - ROUTE[k], axis=1) < 0.02) & (np.linalg.norm(X[m, 7:10], axis=1) < 0.05)
        bad = np.where(~ok)[0]; settle.append(None if not len(ok) or (len(bad) and bad[-1] == len(ok) - 1) else float(t[m][bad[-1] + 1] - ta) if len(bad) else 0.0)          # None: not settled by the end of the dwell
    dt = t[1] - t[0]; acc = np.gradient(X[:, 7:10], dt, axis=0); jerk = np.gradient(acc, dt, axis=0)
    F = KF * X[:, 19:23] ** 2; hover = (ep.pl.m.body_subtreemass[1]) * 9.81 / 4
    tau = np.array([r["u_app"][4:7] for r in R]); tilt = np.degrees(2 * np.arcsin(np.clip(np.hypot(X[:, 4], X[:, 5]), 0, 1)))
    return dict(mode=mode, T=T, mb=mb, seed=seed, completed=ep.abort is None, abort=None if ep.abort is None else ep.abort[1], t_end=float(t[-1]),
                base_rms_mm=float(np.sqrt(np.mean(e ** 2)) * 1e3), base_max_mm=float(e.max() * 1e3), settle_s=settle,
                settle_mean_s=float(np.mean(settle)) if all(s is not None for s in settle) and ep.abort is None else None,
                tilt_max_deg=float(tilt.max()), jerk_rms=float(np.sqrt(np.mean(np.sum(jerk ** 2, 1)))), energy_rel_hover=float(np.sum(F ** 1.5) / (4 * hover ** 1.5 * len(F))),
                tau_max_frac=float(np.abs(tau).max() / 1.0), joint_range_deg=list(np.degrees(X[:, 13:16].max(0) - X[:, 13:16].min(0)).round(1)), solve_fail=int(sum(1 for r in R if r["info"].get("fallback"))))


def main(a):
    for m in sorted({0.0 if x == "unaware" else a.mb for x in a.modes}): TailNMPC("rigid", mb=m)      # build the solvers once before the workers start
    jobs = [(m, T, a.mb, s) for T in a.T for m in a.modes for s in range(a.seeds)]
    out = os.path.join(HERE, "..", "results", "tail"); os.makedirs(out, exist_ok=True); rows, todo, live = [], list(jobs), []
    with open(os.path.join(out, a.name + ".jsonl"), "w") as f:          # one process per run with a wall-clock limit, rows written as they finish
        while todo or live:
            while todo and len(live) < a.procs:
                q = mp.Queue(); j = todo.pop(0); pr = mp.Process(target=lambda j=j, q=q: q.put(run(j))); pr.start(); live.append((pr, q, j, time.time()))
            for it in list(live):
                pr, q, j, t0 = it
                try: r = q.get(timeout=0.2)
                except queue.Empty:
                    if time.time() - t0 < HANG_S: continue
                    pr.kill(); r = dict(mode=j[0], T=j[1], mb=j[2], seed=j[3], completed=False, abort=f"solver hang: no return within {HANG_S} s (seen only after loss of control)", settle_mean_s=None)
                pr.join(); live.remove(it); rows.append(r); f.write(json.dumps(r, default=float) + "\n"); f.flush()
                print(len(rows), "/", len(jobs), {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items() if k not in ("settle_s",)}, flush=True)
    json.dump(dict(route=[list(w) for w in ROUTE], dwell=DWELL, block_offset=list(OFF), noise="lab", rows=rows), open(os.path.join(out, a.name + ".json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("name"); ap.add_argument("--mb", type=float, default=0.15); ap.add_argument("--T", type=float, nargs="+", default=[2.0, 1.5, 1.2, 1.0, 0.8])
    ap.add_argument("--modes", nargs="+", default=["rigid", "hold", "tail", "unaware"]); ap.add_argument("--seeds", type=int, default=1); ap.add_argument("--procs", type=int, default=12); main(ap.parse_args())
