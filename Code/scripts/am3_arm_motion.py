"""Free-flight arm-motion test (no wall, nominal plant): does commanding arm motion disturb the drone?  The arm holds still for HOVER_S seconds (pre-motion baseline), then the pad follows smooth waypoints (min-jerk segments).
Same model, plant, sensors, seed, task logic for B and N; only the pad waypoint amplitude (scale of the standard `free` loop) and segment duration change.  Reports, per case and controller, in the pre-motion window and in the motion window:
base displacement from the hover reference, tilt, rotor-speed deviation from the pre-motion mean, joint speed, pad tracking error.  Usage: am3_arm_motion.py TAG [--workers W] -> results/am3_arm_motion/<TAG>/{summary.json, <ctrl>_<case>.npz}"""
import sys, os, json, argparse, multiprocessing as mp, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
HOVER_S = 3.0; V_MOVE = 0.02          # rad/s joint speed that counts as "arm moving"
CASES = [(s, T) for s in (0.5, 1.0, 1.5) for T in (3.0, 1.5, 0.8)]          # kind 'loop': scale, seconds per leg
MOTG = [(A, f) for A in (0.02, 0.04, 0.06) for f in (1.0, 1.5, 2.0, 3.0)]          # kinds circ/yosc/zosc/reach: amplitude (m), frequency (Hz), 6 s of motion
OSC = [(A, T) for A in (0.03, 0.06) for T in (0.8, 0.5, 0.35, 0.25)]          # kind 'osc': pad oscillates +-A (m) along (y, z/2) with T seconds per half cycle, 12 legs
def one(a):
    tag, c, s, T, kind, seed = a; os.environ["OMP_NUM_THREADS"] = "1"
    from ocrl.am3_task import TASK, BASE; TASK["hover_s"] = HOVER_S
    from am3_run import make_ctrl, free_waypoints
    from ocrl.am3_episode import Episode, initial_state
    MOT = kind in ("circ", "yosc", "zosc", "reach"); name = f"free_{kind}_A{s}_f{T}" if MOT else "free"; wps, _ = free_waypoints(name); x0, u0 = initial_state(0.06); from ocrl.am3_pred import AMPred3; p0 = np.array(AMPred3().pad_f(x0)[0]).ravel(); wps = [p0 + s * (w - p0) for w in wps] if kind == "loop" else wps if MOT else [p0 + (-1) ** k * s * np.array([0, 1.0, 0.5]) for k in range(12)]
    ep = Episode(make_ctrl(c), name, seed=seed, T_max=HOVER_S + (7.5 if MOT else T * len(wps) + 1.5), free_waypoints=wps, segment_s=6.0 if MOT else T, x0=(x0, u0)).run(); R = ep.rows
    t = np.array([r["t"] for r in R]); X = np.array([r["x"] for r in R]); ph = np.array([r["phase"] for r in R]); pad = np.array([r["pad"] for r in R]); ref = np.array([r["ref"]["p"] for r in R])
    e = np.linalg.norm(X[:, 0:3] - BASE, axis=1) * 1e3; tilt = np.degrees(2 * np.arcsin(np.minimum(1, np.hypot(X[:, 4], X[:, 5])))); qd = np.abs(X[:, 16:19]).max(1); Om = X[:, 19:23]
    t_start = float(t[ph == "trace"][0]) if (ph == "trace").any() else float("nan"); mov = (t >= t_start) & (t <= t_start + (6.5 if MOT else T * len(wps) + 0.5)); pre = (t >= t_start - 1.0) & (t < t_start); Om0 = Om[pre].mean(0)
    jm = np.nonzero(qd > V_MOVE)[0]; t_joint = float(t[jm[0]]) if len(jm) else float("nan")
    out = dict(seed=seed, kind=kind, ctrl=c, scale=s, seg_s=T, abort=str(ep.abort), t_motion_start_cmd=t_start, t_joint_moving=t_joint, pre=dict(base_max_mm=float(e[pre].max()), tilt_max_deg=float(tilt[pre].max()), rotor_dev_max_rad_s=float(abs(Om[pre] - Om0).max())),
               motion=dict(base_max_mm=float(e[mov].max()), tilt_max_deg=float(tilt[mov].max()), rotor_dev_max_rad_s=float(abs(Om[mov] - Om0).max()), rotor_dev_max_pct=float(100 * (abs(Om[mov] - Om0) / Om0).max()), joint_speed_max_rad_s=float(qd[mov].max()),
               pad_err_rms_mm=float(np.sqrt(np.mean(np.sum((pad[mov] - ref[mov]) ** 2, 1))) * 1e3), pad_err_max_mm=float(np.linalg.norm(pad[mov] - ref[mov], axis=1).max() * 1e3)))
    d = os.path.join(HERE, "..", "results", "am3_arm_motion", tag); os.makedirs(d, exist_ok=True); np.savez_compressed(os.path.join(d, f"{c}_{kind}_s{s}_T{T}" + (f"_seed{seed}" if seed else "") + ".npz"), t=t, base_mm=e, tilt=tilt, qd=X[:, 16:19], q=X[:, 13:16], Om=Om, pad_err_mm=np.linalg.norm(pad - ref, axis=1) * 1e3, t_start=t_start); return out
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--kind", default="loop"); ap.add_argument("--seeds", default="0"); ap.add_argument("--cases", help="A:T,A:T overrides the osc grid"); ap.add_argument("--workers", type=int, default=12); a = ap.parse_args()
    from am3_run import make_ctrl; make_ctrl("N")
    with mp.get_context("spawn").Pool(a.workers) as pool: rows = pool.map(one, [(a.tag, c, s, T, a.kind, int(sd)) for sd in a.seeds.split(",") for s, T in ([tuple(map(float, x.split(":"))) for x in a.cases.split(",")] if a.cases else CASES if a.kind == "loop" else OSC if a.kind == "osc" else MOTG) for c in "BN"], chunksize=1)
    json.dump(rows, open(os.path.join(HERE, "..", "results", "am3_arm_motion", a.tag, "summary.json"), "w"), indent=1)
    for r in rows: print(r["ctrl"], r["seed"], r["scale"], r["seg_s"], "abort" if r["abort"] != "None" else "", "pre base %.2f mm" % r["pre"]["base_max_mm"], "| motion base %.1f mm tilt %.2f deg rotor dev %.0f rad/s (%.1f%%) joint %.2f rad/s pad err rms %.1f mm" % (r["motion"]["base_max_mm"], r["motion"]["tilt_max_deg"], r["motion"]["rotor_dev_max_rad_s"], r["motion"]["rotor_dev_max_pct"], r["motion"]["joint_speed_max_rad_s"], r["motion"]["pad_err_rms_mm"]))
