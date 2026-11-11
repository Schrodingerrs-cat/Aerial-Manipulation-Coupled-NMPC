"""Video of the drone writing text on the wall (am3_write), rendered live while the controller flies.  Cameras follow the base along the text:
iso = isometric (drone, arm and writing together), front = facing the wall from below the pad line (the written text, readable).
Ink = red (pad pressing), reference strokes = thin green.  Writes Media_results/<TAG>/<CTRL>.mp4 (iso over front) and, for several controllers, compare.mp4.
Usage: am3_write_video.py TAG TEXT [--height 0.06] [--ctrl N ...]"""
import sys, os, shutil, subprocess, argparse, time
import numpy as np, mujoco
from stable_baselines3 import PPO          # torch must load before acados and the renderer (a later import crashes the process silently)
from PIL import Image
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_compare import ENC, MEDIA
from ocrl.am3_record import Recorder, CAMS, XW
from ocrl.am3_write import WriteEpisode, YZ0
from am3_write_eval import write_metrics, make


class WriteRecorder(Recorder):
    def attach(self, ep):
        super().attach(ep); self.task = ep.task

    def _on_step(self):
        ep = self.ep; d = ep.pl.d
        if d.time < self.next_t or not ep.rows: return
        self.next_t += self.dt; r = ep.rows[-1]; x = ep.pl.state(); pad = np.array(ep.kin.pad_f(x)[0]).ravel(); F = ep.pl.pad_force()[0]
        if F > 0.08: self.trace.append(np.r_[XW - 0.0007, pad[1:]])          # ink: contact point drawn just in front of the (soft) wall surface
        else: self.trace.append(None)
        if self.renderer is None: ep.pl.m.vis.global_.offwidth = max(self.size[0], ep.pl.m.vis.global_.offwidth); ep.pl.m.vis.global_.offheight = max(self.size[1], ep.pl.m.vis.global_.offheight); self.renderer = mujoco.Renderer(ep.pl.m, self.size[1], self.size[0], max_geom=20000)
        e_base = np.linalg.norm(x[0:3] - r["ref"]["pB"]) * 1e3; e_pad = np.linalg.norm(pad - r["ref"]["p"]) * 1e3; tilt = np.degrees(2 * np.arcsin(min(1, np.hypot(x[4], x[5]))))
        for cam in self.cams:
            c = CAMS[cam]; mc = mujoco.MjvCamera(); mc.lookat[:] = (c["lookat"][0], x[1], c["lookat"][2] + x[2] - 1.2); mc.distance = c["distance"]; mc.azimuth = c["azimuth"]; mc.elevation = c["elevation"]
            self.renderer.update_scene(d, mc); scn = self.renderer.scene; self._blades(scn)
            for p in self.task.paths:
                pts = [np.r_[XW - 0.0001, self.task.yz0 + p.at(s)[0]] for s in np.linspace(0, p.length, 40)]
                for a, b in zip(pts[:-1], pts[1:]): self._seg(scn, a, b, (0.1, 0.85, 0.25, 1), 0.0010)
            for a, b in zip(self.trace[:-1], self.trace[1:]):
                if a is not None and b is not None and scn.ngeom < scn.maxgeom - 10: self._seg(scn, a, b, (0.9, 0.05, 0.05, 1), 0.0022)
            img = Image.fromarray(self.renderer.render()); self._hud(img, r, e_base, e_pad, tilt, F, cam); img.save(os.path.join(self.dir, cam, f"{self.n:05d}.png"))
        self.n += 1


CAMS["w_iso"] = dict(lookat=(0.76, 0.0, 1.06), distance=0.7, azimuth=40, elevation=-18)
CAMS["w_front"] = dict(lookat=(0.85, 0.0, 1.06), distance=0.85, azimuth=-25, elevation=20)          # turned 25 deg so the arm does not hide the letter being written
VIEWS = ("w_iso", "w_front")


def main(a):
    out = os.path.join(MEDIA, a.tag); shutil.rmtree(out, ignore_errors=True); os.makedirs(out); lines = []
    names = ["RL" if c.endswith(".zip") else c for c in a.ctrl]                          # file-safe names (a policy is given as a path)
    for c, name in zip(a.ctrl, names):
        label = f"RL ({os.path.basename(os.path.dirname(c))})" if c.endswith(".zip") else {"N": "N: coupled NMPC"}.get(c, c)
        rec = WriteRecorder(os.path.join(out, "frames", name), f"{label}: writing \"{a.text}\"", a.tag, a.text, cams=VIEWS, path=None, rate_label="")
        ep = WriteEpisode(make(c), a.text, height=a.height, v_write=a.speed); rec.attach(ep); t0 = time.time(); ep.run(); m = write_metrics(ep)
        lines.append(f"{label}: " + ", ".join(f"{k}={v}" for k, v in m.items()) + f"  (wall {time.time() - t0:.0f} s)"); print(lines[-1], flush=True)
        for v in VIEWS: subprocess.run(ENC(os.path.join(out, "frames", name, v), os.path.join(out, f"{name}_{v}.mp4")), check=True)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", os.path.join(out, f"{name}_{VIEWS[0]}.mp4"), "-i", os.path.join(out, f"{name}_{VIEWS[1]}.mp4"), "-filter_complex", "[0:v][1:v]vstack=inputs=2[v]", "-map", "[v]", "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", os.path.join(out, f"{name}.mp4")], check=True)
        for v in VIEWS: os.remove(os.path.join(out, f"{name}_{v}.mp4"))
    if len(a.ctrl) > 1:
        ins = sum([["-i", os.path.join(out, f"{c}.mp4")] for c in names], []); n = len(names)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + ins + ["-filter_complex", "".join(f"[{i}:v]" for i in range(n)) + f"hstack=inputs={n}[v]", "-map", "[v]", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", os.path.join(out, "compare.mp4")], check=True)
    shutil.rmtree(os.path.join(out, "frames")); open(os.path.join(out, "README.txt"), "w").write(f"text \"{a.text}\", letter height {a.height} m, moving base (outside the frozen fixed-hover plan). Red = ink (pad pressing), green = reference strokes. Video 1.0x.\n" + "\n".join(lines) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("text"); ap.add_argument("--height", type=float, default=0.06); ap.add_argument("--speed", type=float, default=0.025); ap.add_argument("--ctrl", nargs="+", default=["N"]); main(ap.parse_args())
