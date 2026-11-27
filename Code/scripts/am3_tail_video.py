"""Tail-study videos (2026-11-27, capstone): the drone flies the square route carrying a block; rendered live while the controller flies.
Green = route, blue = base path, orange = block path.  Controllers: NMPC modes (rigid, tail0.3, ...) or RL policies (MODEL_ZIP:free|rigid).
Writes Media_results/TAG/<name>.mp4 and, for several controllers, compare.mp4 (side by side).
Usage: am3_tail_video.py TAG --T 1.0 --mb 0.15 --ctrl rigid tail0.3 path/model.zip:free"""
import sys, os, shutil, subprocess, argparse
from types import SimpleNamespace
import numpy as np, mujoco
from stable_baselines3 import PPO          # torch must load before acados and the renderer
from PIL import Image, ImageDraw
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_compare import ENC, MEDIA
from ocrl.am3_record import Recorder, FONT
from am3_tail_run import ROUTE, DWELL, OFF, W0, TailNMPC, WaypointTask, start_state, Episode, LabSensors as EpLab
from ocrl.am3_rl_tail import TailEnv
from ocrl.am3_rl_direct import LabSensors as EnvLab

CAM = dict(lookat=W0 + np.array([-0.3, 0.3, -0.05]), distance=1.6, azimuth=45, elevation=-30)


class TailRecorder(Recorder):
    """Frames every 1/30 s of simulation time; ref() gives (base reference, phase text)."""
    def __init__(self, out_dir, label, sub, pl, ref):
        super().__init__(out_dir, label, "", sub, cams=("v",)); self.pl, self.ref, self.bt, self.ht = pl, ref, [], []; self.ep = SimpleNamespace(pl=pl)          # _blades reads self.ep.pl
        self.hand = mujoco.mj_name2id(pl.m, mujoco.mjtObj.mjOBJ_BODY, "payload")
        orig = pl.step
        def step(u): orig(u); self._on_step()
        pl.step = step

    def _on_step(self):
        d = self.pl.d
        if d.time < self.next_t: return
        self.next_t += self.dt; x = self.pl.state(); pB, phase = self.ref(); self.bt.append(x[0:3].copy()); self.ht.append(d.xpos[self.hand].copy())
        if self.renderer is None: self.renderer = mujoco.Renderer(self.pl.m, self.size[1], self.size[0], max_geom=20000)
        mc = mujoco.MjvCamera(); mc.lookat[:] = CAM["lookat"]; mc.distance, mc.azimuth, mc.elevation = CAM["distance"], CAM["azimuth"], CAM["elevation"]
        self.renderer.update_scene(d, mc); scn = self.renderer.scene; self._blades(scn)
        for a, b in zip(ROUTE[:-1], ROUTE[1:]): self._seg(scn, a, b, (0.1, 0.85, 0.25, 1), 0.003)
        for tr, col in ((self.bt, (0.15, 0.45, 1.0, 1)), (self.ht, (1.0, 0.55, 0.1, 1))):
            s = tr[-400:]
            for a, b in zip(s[:-1], s[1:]):
                if scn.ngeom < scn.maxgeom - 10: self._seg(scn, a, b, col, 0.003)
        e = np.linalg.norm(x[0:3] - pB) * 1e3; tilt = np.degrees(2 * np.arcsin(min(1, np.hypot(x[4], x[5]))))
        img = Image.fromarray(self.renderer.render()); W, H = self.size; ov = Image.new("RGBA", img.size, (0, 0, 0, 0)); dr = ImageDraw.Draw(ov)
        dr.rectangle([0, 0, W, 100], fill=(8, 10, 16, 190)); dr.text((14, 8), self.label, fill=(255, 255, 255, 255), font=self.f_t); dr.text((14, 42), self.traj, fill=(200, 220, 255, 255), font=self.f_s)
        dr.text((14, 70), f"t = {d.time:5.2f} s   {phase:6}   base error {e:6.1f} mm   tilt {tilt:5.1f} deg   joints {np.degrees(x[13:16]).round(0)} deg", fill=(255, 235, 140, 255), font=self.f_s)
        img.paste(ov, (0, 0), ov); img.save(os.path.join(self.dir, "v", f"{self.n:05d}.png")); self.n += 1


def fly(ctrl, T, mb, out, sub, seed=0):
    """Runs one controller with the recorder attached; returns (name, failure reason or None)."""
    if ctrl.endswith((":free", ":rigid")):
        model, arm = ctrl.rsplit(":", 1); pol = PPO.load(model, device="cpu"); name = f"RL_{arm}"
        env = TailEnv(seed=seed, max_steps=1000, arm=arm, fixed=dict(route=ROUTE, T=T, dwell=DWELL, mb=mb), randomize=False); env.sens = EnvLab(seed)
        o, _ = env.reset(); rec = TailRecorder(os.path.join(out, "frames", name), f"RL policy, arm {arm} ({os.path.basename(os.path.dirname(model))})", sub, env.pl, lambda: (env.pref, ""))
        for _ in range(env.max_steps):
            o, r, bad, trunc, _ = env.step(pol.predict(o, deterministic=True)[0])
            if bad or trunc: break
        fail = "loss of control" if bad else None
    else:
        c = TailNMPC(ctrl, mb=mb); x0, u0 = start_state(mb, W0, OFF); name = f"NMPC_{ctrl}"
        ep = Episode(c, "free", seed=seed, mismatch=dict(payload=dict(kg=mb, pos_hand_m=[0.0, 0.0, 0.0])), free_waypoints=[[0.0, 0.0, 0.0]], segment_s=1.0, x0=(x0, u0), T_max=60.0)
        ep.task = WaypointTask(ROUTE, T, DWELL, OFF); c.reset(ep.x0, ep.u0, ep.task); ep.sens = EpLab(seed)
        label = {"rigid": "NMPC, arm rigid (block = dead weight)", "tail0.3": "NMPC, arm as a tail (spring 0.3)"}.get(ctrl, f"NMPC, arm mode {ctrl}")
        rec = TailRecorder(os.path.join(out, "frames", name), label, sub, ep.pl, lambda: (ep.task.ref()["pB"], ep.task.ref()["phase"])); ep.run()
        fail = None if ep.abort is None else ep.abort[1]
    rec.renderer.close(); subprocess.run(ENC(os.path.join(out, "frames", name, "v"), os.path.join(out, name + ".mp4")), check=True)
    return name, fail


def main(a):
    out = os.path.join(MEDIA, a.tag); shutil.rmtree(out, ignore_errors=True); os.makedirs(out); sub = f"square 0.6 m, {a.T} s per side, {int(a.mb * 1000)} g block, lab sensing"; res = []
    for c in a.ctrl: res.append(fly(c, a.T, a.mb, out, sub, a.seed)); print(res[-1], flush=True)
    if len(res) > 1:
        ins = sum([["-i", os.path.join(out, n + ".mp4")] for n, _ in res], []); k = len(res)
        f = f"hstack=inputs={k}" if k <= 3 else f"xstack=inputs={k}:layout=0_0|w0_0|0_h0|w0_h0:fill=black"
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + ins + ["-filter_complex", "".join(f"[{i}:v]" for i in range(k)) + f + "[v]", "-map", "[v]", "-c:v", "libx264", "-crf", "24", "-pix_fmt", "yuv420p", os.path.join(out, "compare.mp4")], check=True)
    shutil.rmtree(os.path.join(out, "frames"))
    open(os.path.join(out, "README.txt"), "w").write(f"Tail study (capstone): {sub}, dwell {DWELL} s, seed {a.seed}. Green = route, blue = base path, orange = block path. Video 1.0x.\n" + "\n".join(f"{n}: {'completed' if f is None else 'FAILED: ' + f}" for n, f in res) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--T", type=float, default=1.0); ap.add_argument("--mb", type=float, default=0.15); ap.add_argument("--ctrl", nargs="+", default=["rigid", "tail0.3"]); ap.add_argument("--seed", type=int, default=0); main(ap.parse_args())
