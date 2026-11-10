"""Coupled NMPC (N, 50 Hz, unchanged) on the RL task: same plant, start states, sensor noise, base path, joint motion and survival limits as am3_eval_track.py --env track8.
The joint reference is given to N as the pad reference pB + FK(qref).  Keeps the Episode convention of one control period of computation latency.  The Episode 25 cm failsafe is NOT used, because RL has none.
RL (--policy, default R8_turing_s1) and PD+ff are run on the same environment object.  --no-offset starts on the reference (no random position, attitude or velocity error).
Usage: am3_rl_vs_nmpc.py big_fig8 [--start 1000] [--n 20] [--no-offset] [--out nmpc_same_task.json]"""
import sys, os, json, time, argparse
import numpy as np
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
from ocrl.am3_rl_direct import Track8Env, Track9Env, ref_point3, Q_NOM, quat2R
from ocrl.am3_ctrl_n import NMPC
from am3_eval_track import REFS, rollout

SHIFT = np.array([0.5, 0.0, 0.0])          # N's internal model contains the wall, the free-flight plant does not; N works in a frame 0.5 m away from it so the model wall stays clear of the flight volume


class SameTask:
    """Reference at time t for N's preview: base path and joint sinusoids of the RL episode."""
    def __init__(self, env): self.env, self.t = env, 0.0

    def at(self, t):
        e = self.env; pB, vB = ref_point3(t, e.p)
        q = Q_NOM + e.qa * (np.sin(e.qw * t + e.qph) - np.sin(e.qph)); qd = e.qa * e.qw * np.cos(e.qw * t + e.qph); pr = e.inner.pr
        return dict(phase="trace", p=pB + np.array(pr.fk_f(q)).ravel() - SHIFT, v=vB + np.array(pr.Jpad_f(q)) @ qd, pB=pB - SHIFT, vB=vB, gap=None, contact_expected=False, s=0.0, q=q)

    def ref(self): return self.at(self.t)
    def horizon(self, N, h, dt=0.02): return [self.at(self.t + j * h) for j in range(N + 1)]


def run(env, ctrl, seed):
    env.reset(seed=seed); task = SameTask(env); x0 = env.x0; u_app = u_pend = np.asarray(env.inner.eq["u"], float).copy(); x0n = x0.copy(); x0n[:3] -= SHIFT; ctrl.reset(x0n, u_pend, task)
    n_sub = int(round(ctrl.period / env.pl.dt)); steps = int(round(env.max_steps * env.dt_c / ctrl.period)); E, J, T, cyc = [], [], [], []; bad = False; fails = 0
    for k in range(steps):
        task.t = k * ctrl.period; x = env.pl.state(); t0 = time.perf_counter(); xm = env.sens(x); xm[:3] -= SHIFT; u_new, info = ctrl.compute(task.t, xm, 0.0, task); cyc.append(time.perf_counter() - t0); fails += int(not info["ok"])
        u_app, u_pend = u_pend, np.asarray(u_new, float).copy()
        for _ in range(n_sub): env.pl.step(u_app)
        x = env.pl.state(); r = task.at((k + 1) * ctrl.period); e = float(np.linalg.norm(x[:3] - r["pB"] - SHIFT)); R = np.array(quat2R(x[3:7])); tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        E.append(e); J.append(float(np.linalg.norm(x[13:16] - r["q"]))); T.append(tilt); bad = (not np.isfinite(x).all()) or tilt > 60.0 or e > 1.5 or x[2] < 0.3
        if bad: break
    k0 = int(2.0 / ctrl.period); b = np.array(E[k0:]) if len(E) > k0 else np.array(E)
    return dict(seed=seed, steps=len(E), survived=not bad, rms_mm=float(np.sqrt(np.mean(b ** 2)) * 1e3), max_mm=float(b.max() * 1e3), tilt_max=float(max(T)), joint_rms_deg=float(np.degrees(np.sqrt(np.mean(np.array(J[k0:]) ** 2)))) if len(J) > k0 else 0.0,
                solver_fails=fails, cycle_median_ms=float(np.median(cyc) * 1e3), cycle_max_ms=float(np.max(cyc) * 1e3))


def main(a):
    env = dict(R8=Track8Env, R9=Track9Env)[a.policy[:2]](seed=0, ref=REFS[a.ref], randomize=not a.no_offset); ctrl = NMPC("full"); pol = PPO.load(os.path.join(HERE, "..", "results", "am3_rl", a.policy, "model_final.zip"), device="cpu"); out = dict(N=[], PD=[], RL=[])
    for k in range(a.n):
        s = a.start + k; out["N"].append(run(env, ctrl, s)); out["PD"].append(rollout(env, "pd", s)); out["RL"].append(rollout(env, pol, s))
        print(s, {c: (v[-1]["survived"], v[-1]["steps"], round(v[-1]["rms_mm"], 2), round(v[-1]["joint_rms_deg"], 2)) for c, v in out.items()}, "N fails", out["N"][-1]["solver_fails"], flush=True)
    json.dump(dict(ref=a.ref, no_offset=a.no_offset, **out), open(os.path.join(HERE, "..", "results", "am3_rl", "eval", a.out), "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ref"); ap.add_argument("--start", type=int, default=1000); ap.add_argument("--n", type=int, default=20); ap.add_argument("--no-offset", action="store_true"); ap.add_argument("--policy", default="R8_turing_s1"); ap.add_argument("--out", default="nmpc_same_task.json"); main(ap.parse_args())
