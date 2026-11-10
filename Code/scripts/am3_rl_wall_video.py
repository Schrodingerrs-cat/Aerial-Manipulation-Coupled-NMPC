"""Wall-line video, one held-out scenario, same harness as am3_eval_wall: N | B | RL rendered live by am3_record (isometric view over wall close-up).
Writes Media_results/<TAG>/compare.mp4 (three panels) and N.mp4, B.mp4, RL.mp4, plus README.txt with the metrics of each run.
Usage: am3_rl_wall_video.py TAG POLICY.zip --scenario ID"""
import sys, os, shutil, subprocess, argparse
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_eval_wall import SC, SHAPE_SC, run
from am3_compare import ENC, MEDIA, TRAJ
from ocrl.am3_record import Recorder, CAMS as REC_CAMS
from ocrl.am3_task import make_path
from PIL import Image, ImageDraw, ImageFont

REC_CAMS["iso"] = dict(lookat=(0.74, 0.0, 1.05), distance=0.6, azimuth=45, elevation=-18)          # isometric: drone, arm and the drawing on the wall in one view
CAMS = ("iso", "wallview")


def main(a):
    s = next(x for x in SC + SHAPE_SC if x["id"] == a.scenario); out = os.path.join(MEDIA, a.tag); shutil.rmtree(out, ignore_errors=True); os.makedirs(out)
    who = dict(N=("N", "N: coupled NMPC, 50 Hz"), B=("B", "B: separate drone and arm controllers, 200 Hz"), RL=(a.policy, f"RL: end-to-end PPO ({os.path.basename(os.path.dirname(a.policy))}{' + joint vel. feedforward' if a.ff else ''}), 100 Hz"))
    runs, lines = {}, []
    for c, (ctrl, label) in who.items():
        rec = Recorder(os.path.join(out, "frames", c), label, a.tag, TRAJ.get(s["path"], s["path"]), cams=CAMS, path=make_path(s["path"])[0], rate_label=label.split(", ")[-1])
        ep, m, p = run(ctrl, s, record=rec, ff=a.ff); runs[c] = (ep, rec); t = m.get("trace", {})
        lines.append(f"{c}: completed={m['completed']} abort={ep.abort} pass_all={p['all']} failed={[f for f, v in p.items() if not v and f != 'all']} track_rms_mm={t.get('track_rms_mm')} base_max_mm={m['base']['max_mm']:.1f} contact={m['contact']['valid_fraction']}"); print(lines[-1], flush=True)
    n = max(r.n for _, r in runs.values()); font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    for c, (ep, rec) in runs.items():                   # a run that ended early holds its last frame with a banner
        for cam in CAMS:
            d = os.path.join(out, "frames", c, cam); last = Image.open(os.path.join(d, f"{rec.n - 1:05d}.png")).convert("RGB")
            for i in range(rec.n, n):
                im = last.copy(); dr = ImageDraw.Draw(im); W, H = im.size; dr.rectangle([0, H - 62, W, H - 32], fill=(150, 0, 0) if ep.abort else (0, 90, 0))
                dr.text((14, H - 60), f"ENDED at t = {ep.rows[-1]['t']:.2f} s: " + (f"ABORT: {ep.abort[1]}" if ep.abort else "episode complete"), fill=(255, 255, 255), font=font); im.save(os.path.join(d, f"{i:05d}.png"))
        for cam in CAMS: subprocess.run(ENC(os.path.join(out, "frames", c, cam), os.path.join(out, f"{c}_{cam}.mp4")), check=True)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", os.path.join(out, f"{c}_{CAMS[0]}.mp4"), "-i", os.path.join(out, f"{c}_{CAMS[1]}.mp4"), "-filter_complex", "[0:v][1:v]vstack=inputs=2[v]", "-map", "[v]", "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", os.path.join(out, f"{c}.mp4")], check=True)
        for cam in CAMS: os.remove(os.path.join(out, f"{c}_{cam}.mp4"))
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + sum([["-i", os.path.join(out, f"{c}.mp4")] for c in who], []) + ["-filter_complex", "[0:v][1:v][2:v]hstack=inputs=3,scale=2880:-2[v]", "-map", "[v]", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", os.path.join(out, "compare.mp4")], check=True)
    shutil.rmtree(os.path.join(out, "frames")); open(os.path.join(out, "README.txt"), "w").write(f"scenario {s['id']} ({s['path']}, gap0 {s['gap0']}, dyz {s['dyz']}, seed {s['seed']})\nRL model {a.policy}  joint velocity feedforward forced: {a.ff}\n" + "\n".join(lines) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("policy"); ap.add_argument("--scenario", required=True); ap.add_argument("--ff", action="store_true"); main(ap.parse_args())
