"""Phase-1 writing benchmark (fixed 2026-11-11, before any Phase-1 result): sentences that together use all 26 letters, letter height 5 cm
(about 20 cm per 4-letter word), pen speed 50 mm/s, each written with several sensor-noise seeds.
Sensing: 'real' = mocap + IMU class noise (SIG_REAL below); 'nominal' = the frozen harness noise (0.1 mm, 0.01 deg).  Latency is the Episode's one control period.
Pass (Phase 1): every episode completed, mean trace RMS <= 2 mm, no pen drag (drag <= 0.02).
CTRL: an RL .zip, 'pd' (the scripted outer loop through the same inner loop), or an NMPC name.  --dob turns on the inner-loop disturbance observer; --case adds a model mismatch.
Usage: am3_write_bench.py CTRL [--noise real] [--seeds 3] [--procs 15] [--dob] [--case NAME] --name NAME"""
import sys, os, json, argparse, time
import numpy as np
from stable_baselines3 import PPO          # torch must load before acados and the renderer
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_write import WriteEpisode
from ocrl.am3_episode import Sensors
from am3_write_eval import make, write_metrics
from am3_write_mismatch import CASES

SENTENCES = ["HELLO WPI", "AERIAL ROBOT", "THE QUICK BROWN", "FOX JUMPS OVER", "THE LAZY DOG"]
HEIGHT, SPEED = 0.05, 0.05
SIG_REAL = dict(p=1e-3, att=np.radians(0.5), v=0.01, w=0.02, q=1e-3, qd=0.02, Om=5.0, F=0.02)   # 1 mm / 0.5 deg pose, gyro 0.02 rad/s, 12-bit encoder, rotor telemetry, force


class RealSensors(Sensors):
    SIG = SIG_REAL


def run(job):
    ctrl, text, seed, noise, dob, case = job; t0 = time.time()
    if ctrl == "pd": from ocrl.am3_rl_wall import RLCtrl; c = RLCtrl("pd", vel_ff=True)
    else: c = make(ctrl)
    if dob: c.env.inner.dob = True
    ep = WriteEpisode(c, text, height=HEIGHT, v_write=SPEED, seed=seed, mismatch=CASES[case] if case else None)
    if noise == "real": ep.sens = RealSensors(seed)
    ep.run(); return dict(text=text, seed=seed, **write_metrics(ep), wall_s=round(time.time() - t0))


def summary(rows):
    ok = [r for r in rows if r["completed"]]; rms = [r["trace_rms_mm"] for r in ok]
    return dict(n=len(rows), completed=len(ok), trace_rms_mean=round(float(np.mean(rms)), 3) if rms else None, trace_rms_max=round(float(np.max(rms)), 3) if rms else None,
                drag_max=max(r["drag"] for r in rows), coverage_mean=round(float(np.mean([r["coverage"] for r in ok])), 3) if ok else None,
                aborts=sorted({r["abort"] for r in rows if r["abort"]}))


def main(a):
    jobs = [(a.ctrl, s, seed, a.noise, a.dob, a.case) for s in SENTENCES for seed in range(a.seeds)]
    if a.procs > 1:
        with Pool(a.procs) as pool: rows = pool.map(run, jobs, chunksize=1)
    else: rows = [run(j) for j in jobs]
    for r in rows: print({k: r[k] for k in ("text", "seed", "completed", "abort", "strokes", "trace_rms_mm", "trace_max_mm", "coverage", "drag", "tilt_max_deg", "wall_s")}, flush=True)
    s = summary(rows); print("SUMMARY", s, flush=True)
    out = os.path.join(HERE, "..", "results", "am3_write", "bench"); os.makedirs(out, exist_ok=True)
    json.dump(dict(ctrl=a.ctrl, noise=a.noise, dob=a.dob, case=a.case, height=HEIGHT, speed=SPEED, sig=SIG_REAL if a.noise == "real" else Sensors.SIG, summary=s, rows=rows),
              open(os.path.join(out, f"{a.name}_{a.noise}{'_' + a.case if a.case else ''}.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ctrl"); ap.add_argument("--noise", choices=("real", "nominal"), default="real"); ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--procs", type=int, default=15); ap.add_argument("--name", required=True); ap.add_argument("--dob", action="store_true"); ap.add_argument("--case", default=None); main(ap.parse_args())
