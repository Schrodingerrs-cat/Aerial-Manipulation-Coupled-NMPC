"""Closed-loop recording: the callback renders the live plant instance (MuJoCo offscreen, GLFW) at fixed simulation-time intervals while the controller is running; nothing is replayed or smoothed.  The video timeline is
simulation time (playback 1.0x of simulated time; wall-clock compute time is disclosed in the run summary).  Decorations are drawn from true 3D positions: green = reference path on the wall plane, red = actual pad
positions during measured contact (|F| > F_off), blue = actual pad positions while detached.  Overlay: controller, run tag, simulation time, base error, pad-reference error, task phase, contact state, measured normal force."""
import os, numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
from ocrl.am3_task import XW, R_PAD, YZ0, TASK
CAMS = dict(overview=dict(lookat=(0.62, 0.0, 1.05), distance=0.85, azimuth=-58, elevation=-14), contact=dict(lookat=(0.77, 0.0, 1.04), distance=0.30, azimuth=-90, elevation=0), wallview=dict(lookat=(0.78, 0.0, 1.04), distance=0.22, azimuth=-35, elevation=-5))

class Recorder:
    def __init__(self, out_dir, label, tag, fps=25, size=(640, 400), cams=("overview", "contact"), path=None):
        self.dir, self.label, self.tag, self.dt, self.size, self.cams = out_dir, label, tag, 1.0 / fps, size, cams; self.next_t = 0.0; self.n = 0; self.trace = []; self.path = path; self.renderer = None
        for c in cams: os.makedirs(os.path.join(out_dir, c), exist_ok=True)
        try: self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)
        except Exception: self.font = ImageFont.load_default()
    def _seg(self, scn, a, b, rgba, w):
        if scn.ngeom >= scn.maxgeom: return
        g = scn.geoms[scn.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, dtype=np.float32)); mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, a, b); scn.ngeom += 1
    def __call__(self, ep, t):
        r = ep.rows[-1]; pad = r["pad"]; self.trace.append((pad.copy(), r["F"] > 0.08))
        if t < self.next_t: return
        self.next_t += self.dt
        if self.renderer is None: ep.pl.m.vis.global_.offwidth = max(self.size[0], ep.pl.m.vis.global_.offwidth); ep.pl.m.vis.global_.offheight = max(self.size[1], ep.pl.m.vis.global_.offheight); self.renderer = mujoco.Renderer(ep.pl.m, self.size[1], self.size[0])
        e_base = np.linalg.norm(r["x"][0:3] - np.array(TASK["base_ref_world"])) * 1e3; e_pad = np.linalg.norm(pad - r["ref"]["p"]) * 1e3
        txt = [f"{self.label}   run {self.tag}", f"t = {r['t']:6.2f} s   phase: {r['phase']}", f"base error {e_base:5.1f} mm   pad-reference error {e_pad:5.1f} mm", f"contact: {'YES' if r['contact'] else 'no '}   Fn measured {r['Fm']:5.2f} N"]
        for cam in self.cams:
            c = CAMS[cam]; mc = mujoco.MjvCamera(); mc.lookat[:] = c["lookat"]; mc.distance = c["distance"]; mc.azimuth = c["azimuth"]; mc.elevation = c["elevation"]; self.renderer.update_scene(ep.pl.d, mc); scn = self.renderer.scene
            if self.path is not None:
                pts = [np.r_[XW - 0.0015, YZ0 + self.path.at(s)[0]] for s in np.linspace(0, self.path.length, 80)]
                for a, b in zip(pts[:-1], pts[1:]): self._seg(scn, a, b, (0.1, 0.8, 0.2, 1), 0.0012)
            T = self.trace[::max(1, len(self.trace) // 400)]
            for (a, ca), (b, cb) in zip(T[:-1], T[1:]): self._seg(scn, a, b, (0.9, 0.1, 0.1, 1) if (ca and cb) else (0.2, 0.4, 1.0, 1), 0.0015)
            img = Image.fromarray(self.renderer.render()); d = ImageDraw.Draw(img); d.rectangle([0, 0, self.size[0], 4 + 17 * len(txt)], fill=(0, 0, 0))
            for i, line in enumerate(txt): d.text((6, 3 + 17 * i), line, fill=(255, 255, 255), font=self.font)
            img.save(os.path.join(self.dir, cam, f"{self.n:05d}.png"))
        self.n += 1
