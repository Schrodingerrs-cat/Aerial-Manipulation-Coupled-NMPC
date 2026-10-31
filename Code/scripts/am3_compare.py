"""Matched B/N comparison with high-quality closed-loop recordings and measurable-metric figures.  Runs the same scenario (model, initial state, reference, sensor seed) for B and N, each rendered live by am3_record
(1280x720 per panel, 30 fps of SIMULATION time), and writes into Media_results/<TAG>/:
  compare_<path>_overview_wall.mp4   2x2: overview (airframe, mount, arm) over wall-front close-up (reference path green, pad contact point red/blue), B left, N right
  compare_<path>_contact_side.mp4    side view of the pad against the wall, B left, N right
  raw_<ctrl>_<camera>.mp4            uninterrupted originals;  compose_commands.txt (exact ffmpeg commands)
  <path>_timeseries.png, <path>_summary.png, <path>_metrics.json   measurable metrics (am3_figures)
Usage: am3_compare.py TAG PATH [--seed S] [--domain NAME --scenario ID]   (with --scenario, PATH/seed/start/mismatch are taken from that scenario of config/am3_domain_NAME.json; unknown to both controllers)"""
import sys, os, json, argparse, subprocess, shutil
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_run import run
from am3_figures import make as make_figures
from ocrl.am3_record import Recorder
from ocrl.am3_task import make_path
from PIL import Image, ImageDraw, ImageFont
MEDIA = os.path.join(HERE, "..", "..", "Media_results"); LABEL = dict(B="B: separate drone and arm controllers, 200 Hz", N="N: coupled NMPC, 50 Hz")
TRAJ = dict(line="horizontal line, 8 cm in 4 s", circle="circle, radius 2.5 cm, 10 s", rounded_triangle="open polyline with rounded corner, 10 s")
TRAJ["free_s1.5_T0.8"] = "arm motion only, no wall: 3 s hover, then 1.5x loop, 0.8 s per leg"
TRAJ["free_circ_A0.06_f2.0"] = "arm motion only, no wall: pad circle, radius 6 cm, 2 Hz, after 3 s hover"
TRAJ["free_osc0.06_T0.25"] = "arm motion only, no wall: pad oscillates +-6 cm at 2 Hz after 3 s hover"
ENC = lambda frames, out: ["ffmpeg", "-y", "-loglevel", "error", "-framerate", "30", "-i", os.path.join(frames, "%05d.png"), "-c:v", "libx264", "-preset", "slow", "-crf", "14", "-pix_fmt", "yuv420p", out]
def main(tag, path, seed, sc=None):
    NAMES = dict(kf_scale="thrust", motor_tau_scale="lag", k_scale="k", c_scale="c", mu_scale="mu"); mm = " | unknown mismatch: " + ", ".join(f"{NAMES.get(k, k)} x{v:.2f}" for k, v in sc["mismatch"].items() if not isinstance(v, (dict, list))) if sc and sc.get("mismatch") else ""
    out = os.path.join(MEDIA, tag); shutil.rmtree(out, ignore_errors=True); os.makedirs(out); cmds = []; eps = {}
    for c in "BN":
        fr = os.path.join(out, "frames", c); rec = Recorder(fr, LABEL[c], tag, TRAJ.get(path, path), path=None if path.startswith("free") else make_path(path)[0], rate_label=LABEL[c].split(", ")[1] + mm); ep, m = run(f"{tag}_{c}", c, path, seed=seed, record=rec, **(dict(gap0=sc["gap0"], dyz=tuple(sc["dyz"]), mismatch=sc.get("mismatch")) if sc else {})); eps[c] = (ep, rec, m)
        for cam in rec.cams: cm = ENC(os.path.join(fr, cam), os.path.join(out, f"raw_{c}_{cam}.mp4")); cmds.append(cm); subprocess.run(cm, check=True)          # raw, uninterrupted originals first
    N = max(r.n for _, r, _ in eps.values())
    for c, (ep, rec, m) in eps.items():                                         # an ended/aborted controller keeps its last frame with a banner (never looped or hidden)
        for cam in rec.cams:
            d = os.path.join(out, "frames", c, cam); last = Image.open(os.path.join(d, f"{rec.n - 1:05d}.png")).convert("RGB")
            for i in range(rec.n, N):
                im = last.copy(); dr = ImageDraw.Draw(im); W, H = im.size; msg = f"ENDED at t = {ep.rows[-1]['t']:.2f} s: " + (f"ABORT: {ep.abort[1]}" if ep.abort else "episode complete"); dr.rectangle([0, H - 62, W, H - 32], fill=(150, 0, 0) if ep.abort else (0, 90, 0)); dr.text((14, H - 60), msg, fill=(255, 255, 255), font=ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)); im.save(os.path.join(d, f"{i:05d}.png"))
    def hstack(cams, name):
        ins = sum([["-framerate", "30", "-i", os.path.join(out, "frames", c, cam, "%05d.png")] for cam in cams for c in "BN"], []); n = 2 * len(cams)
        fc = "".join(f"[{2 * k}:v][{2 * k + 1}:v]hstack=inputs=2[r{k}];" for k in range(len(cams))) + "".join(f"[r{k}]" for k in range(len(cams))) + (f"vstack=inputs={len(cams)}[v]" if len(cams) > 1 else "null[v]")
        return ["ffmpeg", "-y", "-loglevel", "error"] + ins + ["-filter_complex", fc, "-map", "[v]", "-c:v", "libx264", "-preset", "slow", "-crf", "14", "-pix_fmt", "yuv420p", os.path.join(out, name)]
    for cm in (hstack(("overview", "wallview"), f"compare_{path}_overview_wall.mp4"), hstack(("contact",), f"compare_{path}_contact_side.mp4")): cmds.append(cm); subprocess.run(cm, check=True)
    open(os.path.join(out, "compose_commands.txt"), "w").write("\n".join(" ".join(c) for c in cmds) + "\n"); shutil.rmtree(os.path.join(out, "frames"))
    M = make_figures(f"{tag}_B", f"{tag}_N", os.path.join(out, path), f"{tag}: {TRAJ.get(path, path)}", path)
    json.dump({c: dict(wall_clock_s=eps[c][2]["wall_s"], frames=eps[c][1].n, abort=str(eps[c][0].abort)) for c in "BN"}, open(os.path.join(out, "recording_info.json"), "w"), indent=1); return M
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("path"); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--domain"); ap.add_argument("--scenario"); a = ap.parse_args()
    sc = next(s for s in json.load(open(os.path.join(HERE, "..", "config", f"am3_domain_{a.domain}.json")))["scenarios"] if s["id"] == a.scenario) if a.scenario else None; main(a.tag, sc["path"] if sc else a.path, sc["seed"] if sc else a.seed, sc)
