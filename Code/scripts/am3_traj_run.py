"""Moving-base demonstrations (free flight, no wall).  Three shapes from one periodic task, selected by lap counts:
  --mode loop --base-laps L --arm-laps K   base flies L figure-eight laps while the pad circles the base K times
  --arm-laps 0                             the arm ramps out to a fixed offset pose and HOLDS it: a standing COM offset the drone must trim while it flies
  --base-laps 0                            the base holds its hover reference and only the arm moves
With several controllers (--ctrls B,N) each is run on the same plant, reference, seed and sensor noise and composited side by side.
Writes Media_results/<TAG>/: raw per-controller per-camera videos, the side-by-side video, plots, summary.json, compose_commands.txt."""
import sys, os, json, argparse, subprocess, shutil, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from PIL import Image, ImageDraw
from ocrl.am3_traj import TrajEpisode, LoopEpisode, TrajRecorder, base_metrics, down_initial_state
from ocrl.am3_record import CAMS
from am3_run import make_ctrl

MEDIA = os.path.join(HERE, "..", "..", "Media_results")
LABEL = dict(N="N: coupled NMPC (50 Hz)", B="B: decoupled baseline (200 Hz)", Nd="Nd: NMPC, decoupled model (50 Hz)")
BASE_OFFSETS = [[0.0, 0.15, 0.04], [0.0, 0.0, 0.08], [0.0, -0.15, 0.04], [0.0, 0.0, 0.0]]
PAD_OFFSETS = [[0.02, 0.06, 0.03], [0.0, -0.06, 0.03], [0.02, -0.06, -0.03], [0.0, 0.0, 0.0]]


def set_cameras(amp_y, amp_z=0.05, no_wall=False):
    """Frame both demo cameras so the whole swept volume stays in shot (MuJoCo free camera: azimuth 0 looks from -x toward the wall)."""
    d = max(1.30, 2.6 * amp_y + 2.2 * amp_z + 1.05); lz = 1.20 if no_wall else 1.24
    CAMS["traj_front"] = dict(lookat=(0.52, 0.0, lz), distance=d, azimuth=0, elevation=-6)
    CAMS["traj_iso"] = dict(lookat=(0.56, 0.0, lz), distance=d * 1.2, azimuth=-52, elevation=-14)


def encode(frames_dir, out, fps=25):
    return ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", os.path.join(frames_dir, "%05d.png"), "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", out]


def pad_to(frames_dir, rec, ep, n_target, size):
    """An aborted or shorter run keeps its last frame with a banner stating when and why it ended; never looped, never hidden (Plan s10)."""
    for cam in rec.cams:
        d = os.path.join(frames_dir, cam); last = Image.open(os.path.join(d, f"{rec.n - 1:05d}.png")).convert("RGB")
        msg = f"ENDED t = {ep.rows[-1]['t']:.2f} s: " + (f"ABORT: {ep.abort[1]}" if ep.abort else "episode complete")
        for i in range(rec.n, n_target):
            im = last.copy(); dr = ImageDraw.Draw(im); dr.rectangle([0, size[1] - 20, size[0], size[1]], fill=(150, 0, 0) if ep.abort else (0, 90, 0))
            dr.text((6, size[1] - 17), msg, fill=(255, 255, 255), font=rec.font); im.save(os.path.join(d, f"{i:05d}.png"))


def main(tag, ctrls, seed, segment_s, scale, video, mode="diamond", duration=30.0, base_laps=3, arm_laps=10, r_arm=0.05, shape="lemniscate", no_wall=False, arm_down=False):
    out = os.path.join(MEDIA, tag); os.makedirs(out, exist_ok=True)
    bo = (np.array(BASE_OFFSETS) * scale).tolist(); po = (np.array(PAD_OFFSETS) * scale).tolist()
    amp_y, amp_z = 0.15 * scale, 0.05 * scale; set_cameras(amp_y, amp_z, no_wall)
    x0 = down_initial_state()[0] if arm_down else None
    res, logs, recs, cmds, size = {}, {}, {}, [], (640, 400)
    for c in ctrls:
        rec = None
        if video:
            fr = os.path.join(out, "frames", c); shutil.rmtree(fr, ignore_errors=True)
            rec = TrajRecorder(fr, LABEL[c], tag, size=size, cams=("traj_iso", "traj_front"))
        if mode == "loop":
            ep = LoopEpisode(make_ctrl(c), T=duration, base_laps=base_laps, arm_laps=arm_laps, amp_y=amp_y, amp_z=amp_z,
                             r_arm=r_arm, shape=shape, no_wall=no_wall, seed=seed, T_max=duration + 15.0, record=rec, x0=x0).run()
        else:
            ep = TrajEpisode(make_ctrl(c), bo, po, segment_s=segment_s, seed=seed, T_max=40.0, record=rec).run()
        res[c] = base_metrics(ep); logs[c] = ep; recs[c] = rec
        print(f"[{c}] completed={res[c]['completed']} abort={res[c]['abort']} base_rms={res[c]['base_track_rms_mm']:.3f} mm "
              f"pad_rms={res[c]['pad_track_rms_mm']:.3f} mm tilt={res[c]['tilt_max_deg']:.2f} deg")
        if video:
            for cam in rec.cams:
                cm = encode(os.path.join(fr, cam), os.path.join(out, f"raw_{c}_{cam}.mp4")); cmds.append(cm); subprocess.run(cm, check=True)

    if video and len(ctrls) > 1:                                     # matched side-by-side: columns are controllers, rows are cameras
        N = max(recs[c].n for c in ctrls)
        for c in ctrls: pad_to(os.path.join(out, "frames", c), recs[c], logs[c], N, size)
        comp = ["ffmpeg", "-y", "-loglevel", "error"] + sum([["-framerate", "25", "-i", os.path.join(out, "frames", c, cam, "%05d.png")] for cam in ("traj_iso", "traj_front") for c in ctrls], []) + \
               ["-filter_complex", "[0:v][1:v]hstack=inputs=2[top];[2:v][3:v]hstack=inputs=2[bot];[top][bot]vstack=inputs=2[v]", "-map", "[v]",
                "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", os.path.join(out, f"compare_{mode}.mp4")]
        cmds.append(comp); subprocess.run(comp, check=True)
    elif video:
        comp = ["ffmpeg", "-y", "-loglevel", "error", "-i", os.path.join(out, f"raw_{ctrls[0]}_traj_front.mp4"), "-i", os.path.join(out, f"raw_{ctrls[0]}_traj_iso.mp4"),
                "-filter_complex", "[0:v][1:v]hstack=inputs=2", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", os.path.join(out, f"demo_{mode}.mp4")]
        cmds.append(comp); subprocess.run(comp, check=True)

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    col = dict(N="tab:blue", B="tab:orange", Nd="tab:green")
    fig, ax = plt.subplots(5, 1, figsize=(9, 12), sharex=True)
    for c in ctrls:
        ep = logs[c]; t = np.array([r["t"] for r in ep.rows]); X = np.array([r["x"] for r in ep.rows])
        pB = np.array([r["ref"]["pB"] for r in ep.rows]); pad = np.array([r["pad"] for r in ep.rows]); pr = np.array([r["ref"]["p"] for r in ep.rows])
        if c == ctrls[0]:
            ax[0].plot(t, pB[:, 1], "k--", lw=1.2, label="reference"); ax[1].plot(t, pB[:, 2], "k--", lw=1.2)
        ax[0].plot(t, X[:, 1], col[c], label=f"{c} actual"); ax[1].plot(t, X[:, 2], col[c])
        ax[2].plot(t, np.linalg.norm(X[:, 0:3] - pB, axis=1) * 1e3, col[c], label=c)
        ax[3].plot(t, np.linalg.norm(pad - pr, axis=1) * 1e3, col[c])
        ax[4].plot(t, np.degrees(2 * np.arcsin(np.minimum(1, np.hypot(X[:, 4], X[:, 5])))), col[c])
    for a, y in zip(ax, ("base y [m]", "base z [m]", "base vs moving ref [mm]", "pad vs reference [mm]", "tilt [deg]")): a.set_ylabel(y); a.grid(alpha=.3)
    ax[0].legend(); ax[2].legend(); ax[2].set_yscale("log"); ax[3].set_yscale("log"); ax[4].set_xlabel("simulation time [s]")
    fig.suptitle(f"{tag}: base trajectory + arm motion (free flight)"); fig.tight_layout(); fig.savefig(os.path.join(out, "trajectory.png"), dpi=120)

    fig2, bx = plt.subplots(1, 2, figsize=(12, 5.2))
    for c in ctrls:
        ep = logs[c]; X = np.array([r["x"] for r in ep.rows]); pB = np.array([r["ref"]["pB"] for r in ep.rows])
        pad = np.array([r["pad"] for r in ep.rows]); pr = np.array([r["ref"]["p"] for r in ep.rows])
        if c == ctrls[0]:
            bx[0].plot(pB[:, 1], pB[:, 2], "k--", lw=1.4, label="reference"); bx[1].plot(pr[:, 1], pr[:, 2], "k--", lw=1.0, label="reference")
        bx[0].plot(X[:, 1], X[:, 2], col[c], lw=1.0, label=f"{c} actual"); bx[1].plot(pad[:, 1], pad[:, 2], col[c], lw=0.8, label=f"{c} actual")
    for a, ti in zip(bx, ("base path (world y-z)", "pad path (world y-z)")): a.set_title(ti); a.set_xlabel("y [m]"); a.set_ylabel("z [m]"); a.axis("equal"); a.grid(alpha=.3); a.legend()
    fig2.suptitle(f"{tag}: commanded vs actual paths"); fig2.tight_layout(); fig2.savefig(os.path.join(out, "paths.png"), dpi=120)

    setup = dict(mode=mode, seed=seed, scale=scale, controllers=ctrls)
    setup.update(dict(duration_s=duration, base_laps=base_laps, arm_laps=arm_laps, r_arm_m=r_arm, amp_y_m=amp_y, amp_z_m=amp_z, shape=shape, no_wall=no_wall, arm_down=arm_down) if mode == "loop"
                 else dict(base_offsets=bo, pad_offsets=po, segment_s=segment_s))
    json.dump(dict(res, _setup=setup), open(os.path.join(out, "summary.json"), "w"), indent=1, default=str)
    if cmds: open(os.path.join(out, "compose_commands.txt"), "w").write("\n".join(" ".join(c) for c in cmds) + "\n")
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--ctrls", default="N"); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--segment", type=float, default=2.0); ap.add_argument("--scale", type=float, default=1.0); ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--mode", choices=("diamond", "loop"), default="diamond"); ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--base-laps", type=int, default=3); ap.add_argument("--arm-laps", type=int, default=10); ap.add_argument("--r-arm", type=float, default=0.05)
    ap.add_argument("--shape", choices=("lemniscate", "lissajous", "sine"), default="lemniscate"); ap.add_argument("--no-wall", action="store_true"); ap.add_argument("--arm-down", action="store_true")
    a = ap.parse_args(); main(a.tag, a.ctrls.split(","), a.seed, a.segment, a.scale, not a.no_video, a.mode, a.duration, a.base_laps, a.arm_laps, a.r_arm, a.shape, a.no_wall, a.arm_down)
