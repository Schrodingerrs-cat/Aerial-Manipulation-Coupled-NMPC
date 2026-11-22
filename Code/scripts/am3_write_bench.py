"""Phase-1 writing benchmark (fixed 2026-11-11, before any Phase-1 result): sentences that together use all 26 letters, letter height 5 cm
(about 20 cm per 4-letter word), pen speed 50 mm/s, each written with several sensor-noise seeds.
Sensing: 'real' = mocap + IMU class noise (SIG_REAL below, white; the stress level since 2026-11-12); 'lab' = spec-based level (SIG_LAB, pose-estimate error
correlated, 2026-11-20); 'nominal' = the frozen harness noise (0.1 mm, 0.01 deg).  Latency is the Episode's one control period.
Pass (Phase 1): every episode completed, mean trace RMS <= 2 mm, no pen drag (drag <= 0.02).
CTRL: an RL .zip, 'pd' (the scripted outer loop through the same inner loop), or an NMPC name.  --dob on|off overrides the inner-loop disturbance observer (default: as the policy was trained); --case adds a model mismatch.
Usage: am3_write_bench.py CTRL [--noise real|lab|nominal] [--seeds 3] [--procs 15] [--dob on|off] [--case NAME] [--kj GAIN] [--force F_REF] --name NAME"""
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
WALL_LIMIT_S = 1500.0          # stuck-solver guard (the Episode default 400 s aborted completed-looking NMPC runs on slow Turing nodes: 350-440 s per sentence; 2026-11-21)
SMOOTH = ("rotor_tv", "tau_tv", "angacc_rms", "jointacc_rms", "pen_vel_err_rms")          # smoothness metrics (am3_write_eval.smooth_metrics), averaged over completed episodes (2026-11-19)
SIG_REAL = dict(p=1e-3, att=np.radians(0.5), v=0.01, w=0.02, q=1e-3, qd=0.02, Om=5.0, F=0.02)   # 1 mm / 0.5 deg pose, gyro 0.02 rad/s, 12-bit encoder, rotor telemetry, force


# 'lab' (2026-11-20).  Sources: OptiTrack <= 0.3 mm / <= 0.05 deg (vendor), third-party +-0.2-0.3 mm, 0.13-0.16 deg -> p 0.3 mm, att 0.1 deg;
# BMI088 gyro 0.014 deg/s/sqrt(Hz) x sqrt(1.22 x 100 Hz) = 0.15 deg/s -> w 0.003 rad/s; 12-bit encoder quantization 2pi/4096/sqrt(12) -> q 4.4e-4 rad.
# Assumed (no datasheet): estimator velocity 5 mm/s, joint speed 0.01 rad/s, rotor telemetry 5 rad/s (~0.5 %), force 0.02 N.
SIG_LAB = dict(p=3e-4, att=np.radians(0.1), v=5e-3, w=3e-3, q=4.4e-4, qd=0.01, Om=5.0, F=0.02)
TAU_LAB = 0.05          # s; pose-estimate error (p, att, v) is first-order Gauss-Markov: an estimator output drifts, it does not jump every 5 ms


class RealSensors(Sensors):
    SIG = SIG_REAL


class LabSensors(Sensors):
    """SIG_LAB; p/att/v errors correlated (TAU_LAB), gyro/encoders/rotor/force white.  Deterministic in (seed, 5 ms grid index) like Sensors: every controller sees the same noise."""
    SIG = SIG_LAB

    def __init__(self, seed=0):
        super().__init__(seed); self.k, self.e = -1, np.zeros(9)

    def measure(self, x, F, t=0.0):
        s, k, a = self.SIG, int(round(t / 0.005)), np.exp(-0.005 / TAU_LAB); sd = np.repeat([s["p"], s["att"], s["v"]], 3)
        if self.k < 0: self.k, self.e = k, sd * np.random.default_rng([self.seed, k, 1]).standard_normal(9)          # stationary start
        while self.k < k: self.k += 1; self.e = a * self.e + np.sqrt(1 - a * a) * sd * np.random.default_rng([self.seed, self.k, 1]).standard_normal(9)
        r = np.random.default_rng([self.seed, k]).standard_normal; xm = x.copy(); xm[0:3] += self.e[0:3]; dth = self.e[3:6]; Q = x[3:7]
        dq = 0.5 * np.array([-Q[1] * dth[0] - Q[2] * dth[1] - Q[3] * dth[2], Q[0] * dth[0] + Q[2] * dth[2] - Q[3] * dth[1], Q[0] * dth[1] - Q[1] * dth[2] + Q[3] * dth[0], Q[0] * dth[2] + Q[1] * dth[1] - Q[2] * dth[0]])
        xm[3:7] = (Q + dq) / np.linalg.norm(Q + dq); xm[7:10] += self.e[6:9]; xm[10:13] += s["w"] * r(3); xm[13:16] += s["q"] * r(3); xm[16:19] += s["qd"] * r(3); xm[19:23] += s["Om"] * r(4)
        return xm, F + s["F"] * r()


def run(job):
    ctrl, text, seed, noise, dob, case, kj, force = job; t0 = time.time()
    if ctrl == "pd": from ocrl.am3_rl_wall import RLCtrl; c = RLCtrl("pd", vel_ff=True)
    else: c = make(ctrl)
    if dob is not None: c.env.inner.dob = dob                                                     # None: as trained
    if kj is not None: c.env.inner.k_dob[6:9] = kj                                                # joint-channel observer gain override (2026-11-17)
    mm = dict(CASES[case]) if case else {}
    if "v_approach" in mm: from ocrl.am3_task import CP; CP["v_approach_m_s"] = mm.pop("v_approach"); CP["v_final_m_s"] = mm.pop("v_final", CP["v_final_m_s"])          # stress case: faster approach and touch-down (impact), this process only
    ep = WriteEpisode(c, text, height=HEIGHT, v_write=SPEED, seed=seed, mismatch=mm or None, f_ref=force)
    ep.wall_limit = WALL_LIMIT_S
    if noise == "real": ep.sens = RealSensors(seed)
    if noise == "lab": ep.sens = LabSensors(seed)
    ep.run(); return dict(text=text, seed=seed, **write_metrics(ep), wall_s=round(time.time() - t0))


def summary(rows):
    ok = [r for r in rows if r["completed"]]; rms = [r["trace_rms_mm"] for r in ok]
    return dict(n=len(rows), completed=len(ok), trace_rms_mean=round(float(np.mean(rms)), 3) if rms else None, trace_rms_max=round(float(np.max(rms)), 3) if rms else None,
                drag_max=max(r["drag"] for r in rows), coverage_mean=round(float(np.mean([r["coverage"] for r in ok])), 3) if ok else None,
                aborts=sorted({r["abort"] for r in rows if r["abort"]}), F_peak_max=round(max(r.get("F_peak_N", 0.0) for r in rows), 3),
                **{k: round(float(np.mean([r[k] for r in ok if r.get(k) is not None])), 3) if ok and any(r.get(k) is not None for r in ok) else None for k in SMOOTH})


def main(a):
    dob = None if a.dob is None else a.dob == "on"; jobs = [(a.ctrl, s, seed, a.noise, dob, a.case, a.kj, a.force) for s in SENTENCES for seed in range(a.seeds)]
    if a.procs > 1:
        with Pool(a.procs) as pool: rows = pool.map(run, jobs, chunksize=1)
    else: rows = [run(j) for j in jobs]
    for r in rows: print({k: r[k] for k in ("text", "seed", "completed", "abort", "strokes", "trace_rms_mm", "trace_max_mm", "coverage", "drag", "tilt_max_deg", "F_peak_N", "wall_s") + SMOOTH if k in r}, flush=True)
    s = summary(rows); print("SUMMARY", s, flush=True)
    out = os.path.join(HERE, "..", "results", "am3_write", "bench"); os.makedirs(out, exist_ok=True)
    json.dump(dict(ctrl=a.ctrl, noise=a.noise, dob=a.dob, case=a.case, force_ref=a.force, height=HEIGHT, speed=SPEED, sig=dict(real=SIG_REAL, lab=SIG_LAB).get(a.noise, Sensors.SIG), tau_lab=TAU_LAB if a.noise == "lab" else None, summary=s, rows=rows),
              open(os.path.join(out, f"{a.name}{'_dob' + a.dob if a.dob else ''}_{a.noise}{'_' + a.case if a.case else ''}.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ctrl"); ap.add_argument("--noise", choices=("real", "lab", "nominal"), default="real"); ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--procs", type=int, default=15); ap.add_argument("--name", required=True); ap.add_argument("--dob", choices=("on", "off"), default=None); ap.add_argument("--kj", type=float, default=None); ap.add_argument("--case", default=None)
    ap.add_argument("--force", type=float, default=None, help="force-controlled writing with this setpoint (N); default: depth-controlled"); main(ap.parse_args())
