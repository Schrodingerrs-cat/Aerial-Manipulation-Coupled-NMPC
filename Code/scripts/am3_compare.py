"""Matched B/N comparison with closed-loop recordings: runs the same scenario (model, initial state, reference, sensor seed) for B and N, each rendered live by am3_record, and writes into
Media_results/<TAG>/: raw per-controller per-camera videos, the 2x2 side-by-side video, plots, summary.json and compose_commands.txt (the exact ffmpeg commands).  Both panels show simulation time at 1.0x playback.
Usage: am3_compare.py TAG PATH [--seed S] [--ctrls B,N]"""
import sys, os, json, argparse, subprocess, shutil, glob, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_run import run
from ocrl.am3_record import Recorder
from ocrl.am3_task import make_path
from PIL import Image, ImageDraw, ImageFont
MEDIA = os.path.join(HERE, "..", "..", "Media_results"); LABEL = dict(B="B: decoupled baseline (200 Hz)", N="N: coupled NMPC (50 Hz)", Nd="Nd: NMPC, decoupled model (50 Hz)")
def encode(frames_dir, out, fps=25): return ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", os.path.join(frames_dir, "%05d.png"), "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", out]
def main(tag, path, seed, ctrls):
    out = os.path.join(MEDIA, tag); os.makedirs(out, exist_ok=True); cmds = []; res = {}; eps = {}
    for c in ctrls:
        fr = os.path.join(out, "frames", c); shutil.rmtree(fr, ignore_errors=True)
        rec = Recorder(fr, LABEL[c], tag, path=None if path.startswith("free") else make_path(path)[0]); ep, m = run(f"{tag}_{c}", c, path, seed=seed, record=rec); res[c] = m; eps[c] = (ep, rec)
        for cam in rec.cams: cm = encode(os.path.join(fr, cam), os.path.join(out, f"raw_{c}_{cam}.mp4")); cmds.append(cm); subprocess.run(cm, check=True)          # raw, uninterrupted originals first
    nfr = {c: eps[c][1].n for c in ctrls}; N = max(nfr.values())
    for c in ctrls:                                                             # an aborted controller keeps its last frame with the abort banner (never looped or hidden)
        ep, rec = eps[c]
        for cam in rec.cams:
            d = os.path.join(out, "frames", c, cam); last = Image.open(os.path.join(d, f"{rec.n - 1:05d}.png")).convert("RGB")
            for i in range(rec.n, N):
                im = last.copy(); dr = ImageDraw.Draw(im); msg = f"ENDED t = {ep.rows[-1]['t']:.2f} s: " + (f"ABORT: {ep.abort[1]}" if ep.abort else "episode complete"); dr.rectangle([0, 380, 640, 400], fill=(150, 0, 0) if ep.abort else (0, 90, 0)); dr.text((6, 383), msg, fill=(255, 255, 255)); im.save(os.path.join(d, f"{i:05d}.png"))
    comp = ["ffmpeg", "-y", "-loglevel", "error"] + sum([["-framerate", "25", "-i", os.path.join(out, "frames", c, cam, "%05d.png")] for cam in ("overview", "contact") for c in ctrls], []) + \
           ["-filter_complex", f"[0:v][1:v]hstack=inputs=2[top];[2:v][3:v]hstack=inputs=2[bot];[top][bot]vstack=inputs=2[v]", "-map", "[v]", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "20", os.path.join(out, f"compare_{path}.mp4")]
    cmds.append(comp)
    subprocess.run(comp, check=True)
    open(os.path.join(out, "compose_commands.txt"), "w").write("\n".join(" ".join(c) for c in cmds) + "\n")
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True); col = dict(B="tab:orange", N="tab:blue", Nd="tab:green")
    for c in ctrls:
        L = np.load(os.path.join(HERE, "..", "results", "am3_runs", f"{tag}_{c}", "log.npz"), allow_pickle=True); t = L["t"]; X = L["x"]; base = np.array([0.508, 0.0, 1.2])
        ax[0].plot(t, np.linalg.norm(X[:, 0:3] - base, axis=1) * 1e3, col[c], label=c); ax[1].plot(t, np.linalg.norm(L["pad"] - L["ref"], axis=1) * 1e3, col[c]); ax[2].plot(t, np.degrees(2 * np.arcsin(np.minimum(1, np.hypot(X[:, 4], X[:, 5])))), col[c]); ax[3].plot(t, L["F"], col[c])
    for a, y in zip(ax, ("base position error [mm]", "pad - reference error [mm]", "tilt [deg]", "normal force Fn [N]")): a.set_ylabel(y); a.grid(alpha=.3)
    ax[0].legend(); ax[3].set_xlabel("simulation time [s]"); fig.suptitle(f"{tag}: {path} (same plant, seed, reference)"); fig.tight_layout(); fig.savefig(os.path.join(out, f"compare_{path}.png"), dpi=120)
    json.dump({c: dict(res[c], frames=nfr[c]) for c in ctrls}, open(os.path.join(out, "summary.json"), "w"), indent=1, default=str); return res
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("path"); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--ctrls", default="B,N"); a = ap.parse_args(); main(a.tag, a.path, a.seed, a.ctrls.split(","))
