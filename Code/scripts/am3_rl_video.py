"""Side-by-side video of RL policy vs references on one identical start and reference trajectory (free flight, no wall).
Panels: zero action | PD+feedforward expert | RL policy.  Green = reference path, orange = actual base path.
Usage: am3_rl_video.py TAG results/am3_rl/R2_x/model_final.zip [--ref fig8_fast] [--seed 100] [--env track|track3|track4|track5|hover]; writes compare.mp4 plus zero.mp4, pd.mp4, rl.mp4
"""
import sys, os, shutil, subprocess, argparse
import numpy as np, mujoco
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
from PIL import Image, ImageDraw, ImageFont
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
from ocrl.am3_rl_direct import TrackEnv, Track3Env, Track4Env, Track5Env, Track6Env, Track7Env, Track8Env, HoverEnv
from am3_eval_track import REFS
from am3_eval_direct import pd_action

OUT = os.path.join(HERE, "..", "..", "Media_results")
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)


def seg(scn, a, b, rgba, w=0.0015):
    if scn.ngeom >= scn.maxgeom: return
    g = scn.geoms[scn.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, dtype=np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, np.asarray(a, float), np.asarray(b, float)); scn.ngeom += 1


def run(env, who, policy, seed, fdir, size, every=4):
    obs, _ = env.reset(seed=seed); env.pl.m.vis.global_.offwidth = size[0]; env.pl.m.vis.global_.offheight = size[1]
    ren = mujoco.Renderer(env.pl.m, size[1], size[0]); cam = mujoco.MjvCamera(); cam.lookat[:] = (0.52, 0.0, 1.20); cam.distance = 3.2 if isinstance(env, Track3Env) else 1.9; cam.azimuth = 0; cam.elevation = -6
    os.makedirs(fdir, exist_ok=True); A, R, errs, n, k = [], [], [], 0, 0
    while True:
        a = np.zeros(10) if policy == "zero" else env.expert_action(env.pl.state()) if policy == "pd" else pd_action(env.pl.state()) if policy == "pdh" else policy.predict(obs, deterministic=True)[0]
        obs, r, term, trunc, info = env.step(a); x = env.pl.state(); A.append(x[0:3].copy()); R.append(env.pref.copy()); errs.append(info["e"]); k += 1
        if k % every == 0:
            ren.update_scene(env.pl.d, cam); scn = ren.scene; s = max(1, len(A) // 300)
            for p, q in zip(R[::s][:-1], R[::s][1:]): seg(scn, p, q, (0.1, 0.8, 0.2, 1))
            for p, q in zip(A[::s][:-1], A[::s][1:]): seg(scn, p, q, (1.0, 0.5, 0.0, 1))
            img = Image.fromarray(ren.render()); d = ImageDraw.Draw(img); d.rectangle([0, 0, size[0], 38], fill=(0, 0, 0))
            d.text((6, 3), who, fill=(255, 255, 255), font=FONT); d.text((6, 20), f"t={k * env.dt_c:5.2f}s  error {info['e'] * 1e3:6.1f} mm   tilt {info['tilt']:4.1f} deg" + (f"   arm err {np.degrees(info['ej']):4.2f} deg" if "ej" in info else ""), fill=(255, 255, 255), font=FONT)
            img.save(os.path.join(fdir, f"{n:05d}.png")); n += 1
        if term or trunc: break
    if term:
        last = Image.open(os.path.join(fdir, f"{n - 1:05d}.png")).convert("RGB"); d = ImageDraw.Draw(last); d.rectangle([0, size[1] - 20, size[0], size[1]], fill=(150, 0, 0)); d.text((6, size[1] - 17), f"TERMINATED t={k * env.dt_c:.2f}s", fill=(255, 255, 255), font=FONT)
        for i in range(n, n + 50): last.save(os.path.join(fdir, f"{i:05d}.png"))
        n += 50
    e = np.array(errs); body = e[int(2 / env.dt_c):]; return n, float(np.sqrt(np.mean(body ** 2)) * 1e3), bool(term)


def main(a):
    out = os.path.join(OUT, a.tag); shutil.rmtree(out, ignore_errors=True); os.makedirs(out); size = (960, 600)
    pol = PPO.load(a.model, device="cpu"); mk = {"track": lambda: TrackEnv(seed=0, ref=REFS[a.ref]), "track3": lambda: Track3Env(seed=0, ref=REFS[a.ref]), "track4": lambda: Track4Env(seed=0, ref=REFS[a.ref]), "track5": lambda: Track5Env(seed=0, ref=REFS[a.ref]), "track6": lambda: Track6Env(seed=0, ref=REFS[a.ref]), "track7": lambda: Track7Env(seed=0, ref=REFS[a.ref]), "track8": lambda: Track8Env(seed=0, ref=REFS[a.ref]), "hover": lambda: HoverEnv(seed=0)}[a.env]
    panels = [("zero action (floor)", "zero"), ("PD + feedforward (scripted)" if a.env != "hover" else "PD (scripted)", "pd" if a.env != "hover" else "pdh"), ("RL policy (end-to-end PPO)", pol)]
    ns, lines = [], []
    for i, (who, p) in enumerate(panels):
        n, rms, term = run(mk(), who, p, a.seed, os.path.join(out, "frames", str(i)), size); ns.append(n); lines.append(f"{who}: RMS error after 2 s = {rms:.1f} mm, terminated={term}")
        print(lines[-1], flush=True)
    m = max(ns)
    for i, n in enumerate(ns):          # shorter runs hold their last frame
        d = os.path.join(out, "frames", str(i)); last = os.path.join(d, f"{n - 1:05d}.png")
        for j in range(n, m): shutil.copy(last, os.path.join(d, f"{j:05d}.png"))
    cmd = ["ffmpeg", "-y", "-loglevel", "error"] + sum([["-framerate", "25", "-i", os.path.join(out, "frames", str(i), "%05d.png")] for i in range(3)], []) + \
          ["-filter_complex", "[0:v][1:v][2:v]hstack=inputs=3,scale=1920:-2[v]", "-map", "[v]", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", os.path.join(out, "compare.mp4")]
    subprocess.run(cmd, check=True)
    for i, name in enumerate(("zero", "pd", "rl")):          # same frames, one full-size video per controller
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "25", "-i", os.path.join(out, "frames", str(i), "%05d.png"), "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", os.path.join(out, f"{name}.mp4")], check=True)
    shutil.rmtree(os.path.join(out, "frames"))
    open(os.path.join(out, "README.txt"), "w").write(f"model {a.model}\nref {a.ref if a.env != 'hover' else 'hover'}  start seed {a.seed}  (held-out, >=100)\n" + "\n".join(lines) + "\nffmpeg: " + " ".join(cmd) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("model"); ap.add_argument("--ref", default="fig8_fast"); ap.add_argument("--seed", type=int, default=100); ap.add_argument("--env", default="track"); main(ap.parse_args())
