"""Closed-loop recording: frames are rendered from the LIVE plant instance (MuJoCo offscreen, 8x MSAA, shadows) at exact simulation-time instants inside the physics loop, while the controller runs; nothing is replayed,
smoothed or scripted.  The video timeline is simulation time (30 fps = 1.0x simulated time; wall-clock compute time is disclosed in the run summary).
Decorations (drawn from true 3D positions): green = reference path of the CONTACT POINT on the wall plane; red = actual pad contact point (pad centre + pad radius toward the wall) while pressing (true Fn > F_off);
blue = the same point while detached; a detached trace is therefore never drawn on the wall.  Rotor blades are drawn from the physical rotor speeds, visually slowed by ROTOR_SLOWDOWN (stated in the overlay).
Overlay: controller, trajectory, simulation time, phase, base-position error, pad-reference error, tilt, contact state, normal force, phase timeline."""
import os, numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
from ocrl.am3_task import XW, R_PAD, YZ0, TASK
CAMS = dict(overview=dict(lookat=(0.62, 0.0, 1.05), distance=0.95, azimuth=-58, elevation=-14), contact=dict(lookat=(0.77, 0.0, 1.04), distance=0.30, azimuth=-90, elevation=0), wallview=dict(lookat=(0.78, 0.0, 1.04), distance=0.28, azimuth=-8, elevation=-4))
ROTOR_SLOWDOWN = 0.01
PHASES = ("hover", "position", "approach", "settle", "trace", "endhold", "release", "retreat", "done")
FONT = lambda sz, bold=True: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else ""), sz)

class Recorder:
    def __init__(self, out_dir, label, tag, traj_label, fps=30, size=(1280, 720), cams=("overview", "wallview", "contact"), path=None, rate_label=""):
        self.dir, self.label, self.tag, self.traj, self.dt, self.size, self.cams, self.path, self.rate = out_dir, label, tag, traj_label, 1.0 / fps, size, cams, path, rate_label
        self.next_t = 0.0; self.n = 0; self.trace = []; self.renderer = None; self.phi = np.zeros(4); self.t_last = 0.0; self.ep = None; self.f_t = FONT(26); self.f_s = FONT(20, False); self.f_x = FONT(16, False)
        for c in cams: os.makedirs(os.path.join(out_dir, c), exist_ok=True)
    def attach(self, ep):
        """Hook the plant's physics step: frames are produced at exact simulation instants."""
        self.ep = ep; pl = ep.pl; orig = pl.step; rec = self
        def step(u):
            orig(u); rec._on_step()
        pl.step = step
    def _seg(self, scn, a, b, rgba, w):
        if scn.ngeom >= scn.maxgeom: return
        g = scn.geoms[scn.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, dtype=np.float32)); mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, a, b); scn.ngeom += 1
    def _blades(self, scn):
        pl = self.ep.pl; m, d = pl.m, pl.d; R = d.xmat[1].reshape(3, 3); dt = d.time - self.t_last; self.t_last = d.time; self.phi += pl.Omega * ROTOR_SLOWDOWN * dt
        for i in range(4):
            if scn.ngeom >= scn.maxgeom: return
            c = np.cos(self.phi[i]); s = np.sin(self.phi[i]); Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]); pos = d.site_xpos[m.site(f"rotor{i}").id] + R[:, 2] * 0.004
            g = scn.geoms[scn.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.array([0.085, 0.007, 0.0012]), pos, (R @ Rz).ravel(), np.array([0.05, 0.05, 0.05, 1.0], dtype=np.float32)); scn.ngeom += 1
    def _on_step(self):
        ep = self.ep; d = ep.pl.d
        if d.time < self.next_t or not ep.rows: return
        self.next_t += self.dt; r = ep.rows[-1]; pad = np.array(ep.kin.pad_f(ep.pl.state())[0]).ravel(); cpt = pad + np.array([R_PAD, 0, 0]); F = ep.pl.pad_force()[0]
        if F > 0.08: cpt[0] = min(cpt[0], XW - 0.0007)          # pressing: the pad indents ~1 mm into the soft wall; draw the contact point just in front of the surface so it is visible
        self.trace.append((cpt, F > 0.08))
        if self.renderer is None: ep.pl.m.vis.global_.offwidth = max(self.size[0], ep.pl.m.vis.global_.offwidth); ep.pl.m.vis.global_.offheight = max(self.size[1], ep.pl.m.vis.global_.offheight); self.renderer = mujoco.Renderer(ep.pl.m, self.size[1], self.size[0]); self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1
        x = ep.pl.state(); e_base = np.linalg.norm(x[0:3] - np.array(TASK["base_ref_world"])) * 1e3; e_pad = np.linalg.norm(pad - r["ref"]["p"]) * 1e3; tilt = np.degrees(2 * np.arcsin(min(1, np.hypot(x[4], x[5]))))
        for cam in self.cams:
            c = CAMS[cam]; mc = mujoco.MjvCamera(); mc.lookat[:] = c["lookat"]; mc.distance = c["distance"]; mc.azimuth = c["azimuth"]; mc.elevation = c["elevation"]; self.renderer.update_scene(d, mc); scn = self.renderer.scene; self._blades(scn)
            if self.path is not None:
                pts = [np.r_[XW - 0.0001, YZ0 + self.path.at(s)[0]] for s in np.linspace(0, self.path.length, 120)]          # reference slightly behind the pressing trace so that both stay visible
                for a, b in zip(pts[:-1], pts[1:]): self._seg(scn, a, b, (0.1, 0.85, 0.25, 1), 0.0016)
            T = self.trace[::max(1, len(self.trace) // 900)]
            for (a, ca), (b, cb) in zip(T[:-1], T[1:]): self._seg(scn, a, b, (0.95, 0.1, 0.1, 1) if (ca and cb) else (0.2, 0.45, 1.0, 1), 0.0019)
            img = Image.fromarray(self.renderer.render()); self._hud(img, r, e_base, e_pad, tilt, F, cam); img.save(os.path.join(self.dir, cam, f"{self.n:05d}.png"))
        self.n += 1
    def _hud(self, img, r, e_base, e_pad, tilt, F, cam):
        W, H = self.size; ov = Image.new("RGBA", img.size, (0, 0, 0, 0)); dr = ImageDraw.Draw(ov); dr.rectangle([0, 0, W, 128], fill=(8, 10, 16, 190)); dr.rectangle([0, H - 30, W, H], fill=(8, 10, 16, 190))
        dr.text((14, 8), f"{self.label}", fill=(255, 255, 255, 255), font=self.f_t); dr.text((W - 14, 10), f"{self.traj}", fill=(200, 220, 255, 255), font=self.f_s, anchor="ra")
        dr.text((14, 46), f"t = {self.ep.pl.d.time:6.2f} s    phase: {r['phase']}", fill=(255, 235, 140, 255), font=self.f_s); cs = "PRESSING" if F > 0.08 else "detached"
        dr.text((14, 76), f"base error {e_base:6.1f} mm   pad-ref error {e_pad:6.1f} mm   tilt {tilt:5.1f} deg   contact: {cs}   Fn {F:4.2f} N", fill=(255, 255, 255, 255), font=self.f_s)
        x0 = 14; w = (W - 28) / len(PHASES); cur = PHASES.index(r["phase"]) if r["phase"] in PHASES else (3 if r["phase"].startswith("settle") else 2)
        for i, nme in enumerate(PHASES): dr.rectangle([x0 + i * w + 1, H - 24, x0 + (i + 1) * w - 1, H - 6], fill=(70, 160, 255, 255) if i == cur else (60, 64, 76, 255)); dr.text((x0 + i * w + 5, H - 25), nme, fill=(255, 255, 255, 255), font=self.f_x)
        dr.text((14, 106), f"video = simulation time 1.0x   |   rotor blades drawn from physical rotor speed, slowed {int(1 / ROTOR_SLOWDOWN)}x   |   {self.rate}", fill=(170, 180, 200, 255), font=self.f_x); img.paste(ov, (0, 0), ov)
