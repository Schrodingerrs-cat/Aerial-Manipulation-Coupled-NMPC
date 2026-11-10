"""Side-by-side video on one identical start: coupled NMPC | PD+feedforward | RL (--policy), free flight, same path and arm motion.  Writes Media_results/<TAG>/compare.mp4 plus nmpc.mp4, pd.mp4, rl.mp4.
Usage: am3_rl_vs_nmpc_video.py TAG --ref big_fig8 --seed 1000 [--policy R9_turing_s0]"""
import sys, os, shutil, subprocess, argparse
import numpy as np
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
from am3_rl_vs_nmpc import SameTask, SHIFT, NMPC, Track8Env, Track9Env, REFS, quat2R
from am3_rl_video import run, OUT


class NMPCEnv(Track8Env):
    """Runs N through the env interface so the shared renderer can draw it; the action passed to step is ignored."""
    def __init__(self, ctrl, **kw):
        super().__init__(seed=0, dt_c=0.02, **kw); self.ctrl = ctrl

    def reset(self, seed=None, options=None):
        out = super().reset(seed=seed); self.task = SameTask(self); self.u_app = self.u_pend = np.asarray(self.inner.eq["u"], float).copy()
        x0 = self.x0.copy(); x0[:3] -= SHIFT; self.ctrl.reset(x0, self.u_pend, self.task); return out

    def step(self, a):
        self.task.t = self.n * self.dt_c; xm = self.sens(self.pl.state()); xm[:3] -= SHIFT; u, _ = self.ctrl.compute(self.task.t, xm, 0.0, self.task)
        self.u_app, self.u_pend = self.u_pend, np.asarray(u, float).copy()
        for _ in range(self.n_sub): self.pl.step(self.u_app)
        self.n += 1; self._set_ref(self.n * self.dt_c); x = self.pl.state(); e = float(np.linalg.norm(x[:3] - self.pref)); R = np.array(quat2R(x[3:7])); tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        bad = (not np.isfinite(x).all()) or tilt > 60.0 or e > 1.5 or x[2] < 0.3
        return None, 0.0, bad, bool(self.n >= self.max_steps and not bad), dict(e=e, tilt=tilt, ej=float(np.linalg.norm(x[13:16] - self.qref)))


def main(a):
    out = os.path.join(OUT, a.tag); shutil.rmtree(out, ignore_errors=True); os.makedirs(out); size = (960, 600); kw = dict(ref=REFS[a.ref])
    pol = PPO.load(os.path.join(HERE, "..", "results", "am3_rl", a.policy, "model_final.zip"), device="cpu"); ns, lines = [], []; Env = dict(R8=Track8Env, R9=Track9Env)[a.policy[:2]]
    panels = [("nmpc", "Coupled NMPC (acados, 50 Hz)", NMPCEnv(NMPC("full"), **kw), "zero", 2), ("pd", "PD + feedforward (scripted)", Env(seed=0, **kw), "pd", 4), ("rl", f"RL policy {a.policy[:2]} (end-to-end PPO)", Env(seed=0, **kw), pol, 4)]
    for name, who, env, p, every in panels:
        n, rms, term = run(env, who, p, a.seed, os.path.join(out, "frames", name), size, every=every); ns.append(n); lines.append(f"{who}: RMS error after 2 s = {rms:.2f} mm, terminated={term}"); print(lines[-1], flush=True)
    m = max(ns)
    for (name, *_), n in zip(panels, ns):          # shorter runs hold their last frame
        d = os.path.join(out, "frames", name); last = os.path.join(d, f"{n - 1:05d}.png")
        for j in range(n, m): shutil.copy(last, os.path.join(d, f"{j:05d}.png"))
    fr = lambda name: os.path.join(out, "frames", name, "%05d.png")
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", "25", "-i", fr("nmpc"), "-framerate", "25", "-i", fr("pd"), "-framerate", "25", "-i", fr("rl"), "-filter_complex", "[0:v][1:v][2:v]hstack=inputs=3,scale=1920:-2[v]", "-map", "[v]", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", os.path.join(out, "compare.mp4")]
    subprocess.run(cmd, check=True)
    for name in ("nmpc", "pd", "rl"): subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "25", "-i", fr(name), "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", os.path.join(out, f"{name}.mp4")], check=True)
    shutil.rmtree(os.path.join(out, "frames")); open(os.path.join(out, "README.txt"), "w").write(f"ref {a.ref}  start seed {a.seed}\nRL model {a.policy}/model_final.zip\n" + "\n".join(lines) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--ref", default="big_fig8"); ap.add_argument("--seed", type=int, default=1000); ap.add_argument("--policy", default="R8_turing_s1"); main(ap.parse_args())
