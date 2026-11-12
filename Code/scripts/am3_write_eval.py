"""Score a controller on writing text (am3_write).  Metrics per episode:
completed (all strokes, no abort), trace_rms_mm / trace_max_mm (pad yz vs reference during trace), coverage (share of reference points within 2 mm of ink),
contact (pressing share of trace time), drag (pressing share of pen-up time, should be 0), base_max_mm (vs the moving base reference), tilt_max_deg, time_s.
Usage: am3_write_eval.py CTRL TEXT [TEXT ...] [--height 0.06] [--out NAME.json]"""
import sys, os, json, argparse, time
import numpy as np
from stable_baselines3 import PPO          # torch must load before acados and the renderer (a later import crashes the process silently)
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_write import WriteEpisode


def write_metrics(ep):
    R, task = ep.rows, ep.task; tr = [r for r in R if r["phase"] == "trace"]; up = [r for r in R if r["phase"] in ("lift", "travel")]
    e = np.array([np.linalg.norm(r["pad"][1:] - r["ref"]["p"][1:]) for r in tr]) if tr else np.array([np.nan])
    ink = np.array([r["pad"][1:] for r in R if r["F"] > 0.08]); ref = np.vstack([[task.yz0 + p.at(s)[0] for s in np.linspace(0, p.length, max(2, int(p.length / 0.002)))] for p in task.paths])
    cov = float(np.mean([np.min(np.linalg.norm(ink - q, axis=1)) < 0.002 for q in ref])) if len(ink) else 0.0
    tilt = max(np.degrees(2 * np.arcsin(min(1, np.hypot(r["x"][4], r["x"][5])))) for r in R)
    rel = np.array([r["pad"][1:] - r["x"][1:3] for r in R if r["phase"] in ("trace", "travel")]); q = np.array([r["x"][13:16] for r in R])          # arm share: pad motion relative to the base
    arm = dict(arm_rel_range_mm=[round(float(1e3 * np.ptp(rel[:, j])), 1) for j in range(2)] if len(rel) else None, joint_range_deg=[round(float(np.degrees(np.ptp(q[:, j]))), 1) for j in range(3)])
    return dict(completed=bool(ep.abort is None and task.phase == "done"), abort=None if ep.abort is None else str(ep.abort[1]), strokes=f"{len(task.written)}/{len(task.paths)}",
                trace_rms_mm=round(float(1e3 * np.sqrt(np.mean(e ** 2))), 3), trace_max_mm=round(float(1e3 * e.max()), 3), coverage=round(cov, 3),
                contact=round(float(np.mean([r["F"] > 0.08 for r in tr])), 3) if tr else 0.0, drag=round(float(np.mean([r["F"] > 0.08 for r in up])), 3) if up else 0.0,
                base_max_mm=round(float(1e3 * max(np.linalg.norm(r["x"][:3] - r["ref"]["pB"]) for r in R)), 2), tilt_max_deg=round(float(tilt), 2), time_s=round(float(R[-1]["t"]), 1), **arm)


def make(ctrl):
    if ctrl.endswith(".zip"):
        from stable_baselines3 import PPO, SAC
        from ocrl.am3_rl_wall import RLCtrl, KIND_KW
        cfg = os.path.join(os.path.dirname(ctrl), "config.json"); cfg = json.load(open(cfg)) if os.path.exists(cfg) else {}
        algo = SAC if cfg.get("algo") == "sac" else PPO
        return RLCtrl(algo.load(ctrl, device="cpu"), vel_ff=True, **KIND_KW.get(cfg.get("env"), {}))           # same env options the policy was trained with
    from am3_run import make_ctrl
    return make_ctrl(ctrl)


def main(a):
    rows = []
    for text in a.text:
        t0 = time.time(); ep = WriteEpisode(make(a.ctrl), text, height=a.height, v_write=a.speed); ep.run(); m = dict(text=text, height=a.height, **write_metrics(ep), wall_s=round(time.time() - t0)); rows.append(m); print(m, flush=True)
    if a.out: json.dump(rows, open(os.path.join(HERE, "..", "results", "am3_write", a.out), "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ctrl"); ap.add_argument("text", nargs="+"); ap.add_argument("--height", type=float, default=0.06); ap.add_argument("--speed", type=float, default=0.025); ap.add_argument("--out", default=None); main(ap.parse_args())
