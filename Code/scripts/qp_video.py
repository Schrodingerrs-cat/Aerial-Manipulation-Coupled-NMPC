"""Video of the A->B slung-load cases, rendered live from the MuJoCo plant while each controller flies (no replay).
Panels: U at E's speed | U at its best duration | S | E.  Writes Media_results/<TAG>/compare.mp4 (2x2) and one mp4 per case, plus README.txt.
Usage: qp_video.py TAG [--dx 2.0] [--tsim 10]"""
import sys, os, shutil, subprocess, argparse, json
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from qp_run import fly, metrics, pd
from ocrl.qp_plant import Plant
from ocrl.qp_ocp import plan, minjerk
from ocrl.qp_lqr import from_plan
MEDIA = os.path.join(HERE, "..", "..", "Media_results"); FPS = 30; W, H = 960, 540
F = lambda sz: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", sz)


def record(ctrl, A, B, tsim, label, d):
    os.makedirs(d); st = dict(r=None, n=0, next=0.0, trail=[])
    def cb(P, t):
        if t + 1e-9 < st["next"]: return
        st["next"] += 1.0 / FPS
        if st["r"] is None: st["r"] = mujoco.Renderer(P.m, H, W); st["cam"] = mujoco.MjvCamera(); st["cam"].lookat[:] = [(A[0] + B[0]) / 2, 0, 1.15]; st["cam"].distance = 3.6; st["cam"].azimuth = 90; st["cam"].elevation = -8
        s = P.state(); st["trail"].append(s["r"].copy()); r = st["r"]; r.update_scene(P.d, st["cam"]); scn = r.scene
        for c, rgba in ((A, (0.1, 0.7, 0.1, 0.6)), (B, (0.9, 0.1, 0.1, 0.6))):
            g = scn.geoms[scn.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.full(3, 0.05), np.asarray(c, float), np.eye(3).ravel(), np.array(rgba, np.float32)); scn.ngeom += 1
        for p in st["trail"][-300::3]:
            if scn.ngeom >= scn.maxgeom: break
            g = scn.geoms[scn.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.full(3, 0.008), p, np.eye(3).ravel(), np.array((0.2, 0.4, 0.9, 0.5), np.float32)); scn.ngeom += 1
        im = Image.fromarray(r.render()); dr = ImageDraw.Draw(im); dr.rectangle([0, 0, W, 70], fill=(20, 20, 30))
        dr.text((12, 6), label, fill=(255, 255, 255), font=F(22)); dr.text((12, 38), f"t = {t:5.2f} s   swing {Plant.swing_deg(s):5.1f} deg   drone to B {np.linalg.norm(s['p'] - B) * 100:5.1f} cm", fill=(255, 220, 120), font=F(18))
        im.save(os.path.join(d, f"{st['n']:05d}.png")); st["n"] += 1
    rows = fly(ctrl, A, tsim, on_step=cb); return rows, st["n"]


def main(a):
    A = np.array([0.0, 0.0, 1.5]); B = A + [a.dx, 0.0, 0.0]; out = os.path.join(MEDIA, a.tag); shutil.rmtree(out, ignore_errors=True); os.makedirs(out)
    E = plan(A, B, "E"); S = plan(A, B, "S"); res = json.load(open(os.path.join(HERE, "..", "results", "payload", "ab_2m.json"))); Tbest = res["U"]["metrics"]["plan_T"]
    cases = [("U_fast", f"Normal drone (load ignored), {E['T']:.1f} s move", pd(minjerk(A, B, round(E["T"], 2))), round(E["T"], 2)),
             ("U_best", f"Normal drone (load ignored), best: {Tbest:.2f} s move", pd(minjerk(A, B, Tbest)), Tbest),
             ("S", f"Load-aware, swing suppressed (<3 deg), {S['T']:.1f} s plan", from_plan(S), S["T"]),
             ("E", f"Load-aware, swing exploited, {E['T']:.1f} s plan", from_plan(E), E["T"])]
    lines = []
    for name, label, ctrl, T in cases:
        rows, n = record(ctrl, A, B, a.tsim, label, os.path.join(out, "frames", name)); m = metrics(rows, B, T)
        lines.append(f"{name}: {label}: settled at {m['settle_s']:.2f} s (drone within 5 cm of B and swing < 2 deg from then on), max swing {m['swing_max_deg']:.1f} deg, overshoot {m['overshoot_cm']:.1f} cm"); print(lines[-1], flush=True)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", os.path.join(out, "frames", name, "%05d.png"), "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", os.path.join(out, f"{name}.mp4")], check=True)
    ins = sum([["-i", os.path.join(out, f"{c[0]}.mp4")] for c in cases], [])
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + ins + ["-filter_complex", "[0:v][1:v]hstack[t];[2:v][3:v]hstack[b];[t][b]vstack[v]", "-map", "[v]", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", os.path.join(out, "compare.mp4")], check=True)
    shutil.rmtree(os.path.join(out, "frames")); open(os.path.join(out, "README.txt"), "w").write(f"A {A.tolist()} -> B {B.tolist()}, 0.2 kg load on 0.6 m string, quad 1.05 kg. Video = simulation time 1.0x. Green = A, red = B, blue dots = load trail.\n" + "\n".join(lines) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--dx", type=float, default=2.0); ap.add_argument("--tsim", type=float, default=10.0); main(ap.parse_args())
