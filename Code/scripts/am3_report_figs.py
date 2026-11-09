"""Report figures for a one-column IEEE layout (3.5 in wide).  system_overview.png: MuJoCo renders with labels projected from the model's own body positions.
arm_reaction.png: free-floating test (no gravity, rotors off): a shoulder torque pulse moves the base while the system centre of mass stays still."""
import os, sys, json
import numpy as np, mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
from scipy.optimize import least_squares
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_plant import XML
from ocrl.am3_path import line_path, circle_path, rounded_polygon

OUT = os.path.join(HERE, "..", "..", "docs", "images"); W = 1200
BASE = np.array([0.508, 0.0, 1.2]); PAD_GOAL = (0.788, 1.04)        # pad centre touching the wall face at x = 0.80 (radius 12 mm)
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)


def pose(m, d, q2, q3):
    d.qpos[:] = 0; d.qpos[0:3] = BASE; d.qpos[3] = 1.0
    d.qpos[7:10] = (0.0, q2, q3); mujoco.mj_kinematics(m, d)
    return d.site("ee").xpos.copy()


def project(cam, fovy, p, H):
    az, el = np.radians(cam.azimuth), np.radians(cam.elevation)
    fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]); right = np.cross(fwd, [0, 0, 1.0]); right /= np.linalg.norm(right); up = np.cross(right, fwd)
    v = np.asarray(p, float) - (np.array(cam.lookat) - cam.distance * fwd); f = 0.5 * H / np.tan(np.radians(fovy) / 2)
    return W / 2 + f * (v @ right) / (v @ fwd), H / 2 - f * (v @ up) / (v @ fwd)


def view(m, d, H, lookat, dist, az, el, labels, wall):
    cam = mujoco.MjvCamera(); cam.lookat[:] = lookat; cam.distance = dist; cam.azimuth = az; cam.elevation = el
    opt = mujoco.MjvOption(); opt.geomgroup[3] = 0; opt.geomgroup[4] = 1 if wall else 0          # floor always hidden, wall only where it matters
    ren = mujoco.Renderer(m, H, W); ren.update_scene(d, cam, scene_option=opt); rgb = ren.render().copy()
    ren.enable_segmentation_rendering(); ren.update_scene(d, cam, scene_option=opt); seg = ren.render()[:, :, 0]; rgb[seg < 0] = 255; u0 = int(project(cam, m.vis.global_.fovy, (0.80, 0, 1.1), H)[0]); wm = seg == m.geom("wall").id; rgb[wm] = (214, 214, 208); rgb[:, :u0][wm[:, :u0]] = 255; ren.disable_segmentation_rendering()      # white background, flat wall
    img = Image.fromarray(rgb).convert("RGB"); dr = ImageDraw.Draw(img)
    for text, p, (dx, dy) in labels:
        u, v = project(cam, m.vis.global_.fovy, p, H); bb = dr.textbbox((u + dx, v + dy), text, font=FONT, anchor="mm"); bb = (bb[0] - 8, bb[1] - 5, bb[2] + 8, bb[3] + 5)
        dr.line([(u, v), (u + dx, v + dy)], fill=(255, 255, 255), width=5); dr.line([(u, v), (u + dx, v + dy)], fill=(0, 0, 0), width=2)
        dr.ellipse((u - 6, v - 6, u + 6, v + 6), fill=(255, 255, 255), outline=(0, 0, 0))
        dr.rectangle(bb, fill=(255, 255, 255), outline=(0, 0, 0)); dr.text((u + dx, v + dy), text, fill=(0, 0, 0), font=FONT, anchor="mm")
    ren.close(); return img


def main():
    m = mujoco.MjModel.from_xml_path(XML); m.vis.global_.offwidth = W; m.vis.global_.offheight = 1000; d = mujoco.MjData(m)
    fl, wl = m.geom("floor").id, m.geom("wall").id; m.geom_group[fl] = 3; m.geom_group[wl] = 4; m.geom_matid[wl] = -1; m.geom_rgba[wl] = (0.72, 0.72, 0.70, 1)
    for i in range(4): m.geom(f"prop{i}").rgba = (0.72, 0.74, 0.77, 1)
    s = least_squares(lambda q: pose(m, d, *q)[[0, 2]] - PAD_GOAL, np.radians([-37.0, -78.0]), bounds=(np.radians([-80, -125]), np.radians([-5, -30])))
    pad = pose(m, d, *s.x); print("q2, q3 deg", np.degrees(s.x).round(1), "pad", pad.round(4), "residual mm", np.abs(s.fun).max() * 1e3)
    b = lambda n: d.body(n).xpos.copy()
    mid = lambda p, q: (b(p) + b(q)) / 2
    side = [("q1 yaw", b("link1"), (-210, 0)), ("q2 shoulder", b("link2"), (-200, 35)), ("q3 elbow", b("link3"), (-230, 10)), ("0.16 m", mid("link2", "link3"), (-175, 0)),
            ("0.14 m", mid("link3", "hand"), (40, -115)), ("wrist 20\u00b0 fixed", b("hand"), (-70, 110)), ("pad", pad, (10, -250)), ("wall", (0.82, 0, 1.33), (10, -40))]
    rear = [("quadrotor, 1.05 kg", b("drone"), (250, -130)), ("arm and hand, 0.47 kg", mid("link2", "link3"), (-210, 115))]
    a = view(m, d, 560, (0.60, 0.0, 1.16), 0.72, 35, 14, rear, False); c = view(m, d, 760, (0.60, 0.0, 1.14), 0.62, 90, 0, side, True)
    fig = Image.new("RGB", (W, 560 + 760 + 16), (255, 255, 255)); fig.paste(a, (0, 0)); fig.paste(c, (0, 576)); dr = ImageDraw.Draw(fig)
    dr.text((12, 8), "(a)", fill=(0, 0, 0), font=FONT); dr.text((12, 584), "(b)", fill=(0, 0, 0), font=FONT); dr.line([(0, 568), (W, 568)], fill=(170, 170, 170), width=2)
    os.makedirs(OUT, exist_ok=True); fig.save(os.path.join(OUT, "system_overview.png"))
    arm_reaction(); wall_forces(); wall_paths()


def arm_reaction(move=20.0, t0=0.05, Tm=0.4, T=0.8, kp=3.0, kd=0.3):
    """Free floating (no gravity, rotors off): joint servos hold q1 and q3 and move q2 by `move` degrees along a minimum-jerk path."""
    m = mujoco.MjModel.from_xml_path(XML); m.opt.gravity[:] = 0; m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT; d = mujoco.MjData(m)
    d.qpos[:] = 0; d.qpos[0:3] = BASE; d.qpos[3] = 1.0; q0 = np.radians([0.0, -37.0, -78.0]); d.qpos[7:10] = q0; mujoco.mj_forward(m, d)
    com = lambda: d.subtree_com[m.body("drone").id].copy(); c0, p0 = com(), d.qpos[0:3].copy(); ids = [m.actuator(n).id for n in ("tau1", "tau2", "tau3")]; L = []
    while d.time < T:
        s_ = np.clip((d.time - t0) / Tm, 0, 1); qr = q0 + np.radians([0, move * (10 * s_ ** 3 - 15 * s_ ** 4 + 6 * s_ ** 5), 0])
        d.ctrl[ids] = kp * (qr - d.qpos[7:10]) - kd * d.qvel[6:9]; mujoco.mj_step(m, d); mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
        w, x, y, z = d.qpos[3:7]; L.append((d.time, d.ctrl[ids[1]], np.degrees(d.qpos[8] - q0[1]), np.linalg.norm(d.qpos[0:3] - p0) * 1e3, np.linalg.norm(com() - c0) * 1e3, np.degrees(2 * np.arctan2(y, w))))
    L = np.array(L); print("final base move mm", L[-1, 3].round(2), "peak CoM move mm", L[:, 4].max().round(5), "final base pitch deg", L[-1, 5].round(2), "final shoulder deg", L[-1, 2].round(2), "peak torque", np.abs(L[:, 1]).max().round(3))
    plt.rcParams.update({"font.size": 8, "axes.grid": True, "grid.alpha": 0.3})
    f, ax = plt.subplots(2, 1, figsize=(3.5, 2.4), sharex=True)
    ax[0].plot(L[:, 0], L[:, 2], color="#235DA0", label="shoulder joint"); ax[0].plot(L[:, 0], L[:, 5], color="#CD5F19", label="base pitch"); ax[0].set_ylabel("Angle\nchange (deg)"); ax[0].legend(frameon=False, fontsize=7)
    ax[1].plot(L[:, 0], L[:, 3], color="#CD5F19", label="base"); ax[1].plot(L[:, 0], L[:, 4], "k--", label="system CoM"); ax[1].set_ylabel("Displacement\n(mm)"); ax[1].set_xlabel("Time (s)"); ax[1].legend(frameon=False, fontsize=7)
    f.tight_layout(pad=0.3); f.subplots_adjust(left=0.2); f.align_ylabels(); f.savefig(os.path.join(OUT, "arm_reaction.png"), dpi=300)


def wall_forces(tilt=12.0):
    """Schematic, tilt exaggerated.  Static balance: T sin(theta) = F_C, T cos(theta) = m g; the wall-side rotors push harder to cancel the moment of F_C about the CoM."""
    th = np.radians(tilt); Rm = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]]); P = lambda x, z: Rm @ np.array([x, z])
    f, ax = plt.subplots(figsize=(3.5, 2.5)); ax.set_aspect("equal"); ax.axis("off"); ax.set_xlim(-0.33, 0.47); ax.set_ylim(-0.33, 0.3)
    arrow = lambda a, b, c, t, o: (ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="-|>", color=c, lw=1.6)), ax.text(b[0] + o[0], b[1] + o[1], t, color=c, fontsize=9, ha="center", va="center"))
    wall = 0.36; ax.fill_between([wall, 0.47], -0.33, 0.3, color="#D6D6D0"); ax.text(0.415, 0.2, "wall", fontsize=8, ha="center")
    a, b = P(-0.14, 0), P(0.14, 0); ax.plot([a[0], b[0]], [a[1], b[1]], color="k", lw=3, solid_capstyle="round")
    arm = np.array([[0, 0], [0.0, -0.09], [0.09, -0.21]]); pad = np.array([wall - 0.012, -0.17]); ax.plot([0, 0.0, 0.12, pad[0]], [0, -0.09, -0.22, pad[1]], color="#3A4466", lw=2.5); ax.plot(*pad, "o", color="#D02020", ms=6)
    arrow(tuple(a), tuple(P(-0.14, 0.1)), "#235DA0", "$f_{\\mathrm{rear}}$", (-0.03, 0.02)); arrow(tuple(b), tuple(P(0.14, 0.15)), "#235DA0", "$f_{\\mathrm{wall}}$", (0.03, 0.02))
    arrow((0, 0), tuple(P(0, 0.23)), "#1B7A3D", "$T$", (0.03, 0.0)); arrow((-0.04, -0.02), (-0.04, -0.24), "#555555", "$mg$", (-0.045, 0)); arrow(tuple(pad), (pad[0] - 0.14, pad[1]), "#CD5F19", "$F_C$", (-0.03, 0.0))
    ax.plot([0, 0], [0, 0.24], color="gray", lw=0.8, ls="--"); ax.text(0.022, 0.262, "$\\theta$", fontsize=9, ha="center"); ax.text(-0.33, -0.31, "Tilt exaggerated", fontsize=7, color="gray")
    f.tight_layout(pad=0.1); f.savefig(os.path.join(OUT, "wall_forces.png"), dpi=300)


def wall_paths():
    """The three wall paths from config/am3_task_v1.json, in wall coordinates relative to the patch centre."""
    T = json.load(open(os.path.join(HERE, "..", "config", "am3_task_v1.json")))["paths"]; c = T["circle"]; v = T["rounded_triangle"]
    paths = [("Line", line_path(*[np.array(p) for p in (T["line"]["a"], T["line"]["b"])])), ("Circle", circle_path(np.array(c["centre"]), c["radius"])), ("Polyline", rounded_polygon(np.array(v["vertices"]), v["fillet_m"]))]
    plt.rcParams.update({"font.size": 8}); f, ax = plt.subplots(1, 3, figsize=(3.5, 1.3), sharex=True, sharey=True)
    for a, (name, P) in zip(ax, paths):
        xy = np.array([P.at(u)[0] for u in np.linspace(0, P.length, 300)]) * 1e3; a.plot(xy[:, 0], xy[:, 1], color="#235DA0", lw=1.6); a.plot(*xy[0], "o", color="#1B7A3D", ms=4); a.plot(*xy[-1], "s", color="#CD5F19", ms=3.5)
        k = 100; a.annotate("", xy=xy[k + 4], xytext=xy[k], arrowprops=dict(arrowstyle="-|>", color="#235DA0", lw=1)); a.set_title(name, fontsize=8, pad=2); a.set_aspect("equal"); a.set_xlim(-50, 50); a.set_ylim(-35, 42); a.set_xticks([-40, 0, 40]); a.grid(alpha=0.3); a.set_xlabel("y (mm)")
    ax[0].set_ylabel("z (mm)"); f.tight_layout(pad=0.2); f.savefig(os.path.join(OUT, "wall_paths.png"), dpi=300)


if __name__ == "__main__":
    main()
