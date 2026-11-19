"""Score a controller on writing text (am3_write).  Metrics per episode:
completed (all strokes, no abort), trace_rms_mm / trace_max_mm (pad yz vs reference during trace), coverage (share of reference points within 2 mm of ink),
contact (pressing share of trace time), F_peak_N (largest pad force, impact included), drag (pressing share of pen-up time, should be 0), base_max_mm (vs the moving base reference), tilt_max_deg, time_s.
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
                F_peak_N=round(float(max(ep.pl.log["Fn"])), 3) if ep.pl.log["Fn"] else 0.0, base_max_mm=round(float(1e3 * max(np.linalg.norm(r["x"][:3] - r["ref"]["pB"]) for r in R)), 2), tilt_max_deg=round(float(tilt), 2), time_s=round(float(R[-1]["t"]), 1), **arm, **smooth_metrics(ep))


def smooth_metrics(ep):
    """Smoothness over the writing part (first trace sample to the end), comparable across control rates (NMPC 50 Hz, RL 100 Hz).
    Commands are piecewise constant, so total variation per second (sum |du| / time) does not depend on the sampling rate.
    rotor_tv: rotor speed commands, krad/s per s (mean of 4); tau_tv: joint torque commands, N m per s (mean of 3);
    angacc_rms: body angular acceleration, rad/s^2; jointacc_rms: joint acceleration, rad/s^2; pen_vel_err_rms: pad yz velocity minus reference velocity in trace, mm/s."""
    R = ep.rows; i0 = next((i for i, r in enumerate(R) if r["phase"] == "trace"), None)
    if i0 is None or len(R) - i0 < 3: return dict(rotor_tv=None, tau_tv=None, angacc_rms=None, jointacc_rms=None, pen_vel_err_rms=None)
    R = R[i0:]; t = np.array([r["t"] for r in R]); T = t[-1] - t[0]; dt = np.diff(t)[:, None]
    u = np.array([r["u_app"] for r in R]); x = np.array([r["x"] for r in R])
    tv = np.abs(np.diff(u, axis=0)).sum(axis=0) / T
    g = np.searchsorted(t, t[0] + np.arange(0, T, 0.02) - 1e-9); g = g[g < len(R)]; R, x, dt = [R[i] for i in g], x[g], np.diff(t[g])[:, None]      # states on a common 50 Hz grid
    wdot = np.diff(x[:, 10:13], axis=0) / dt; qdd = np.diff(x[:, 16:19], axis=0) / dt
    tr = np.array([r["phase"] == "trace" for r in R]); pv = np.diff(np.array([r["pad"][1:] for r in R]), axis=0) / dt
    vref = np.array([r["ref"]["v"][1:] for r in R])[:-1]; m = tr[:-1] & tr[1:]
    return dict(rotor_tv=round(float(tv[:4].mean() / 1e3), 3), tau_tv=round(float(tv[4:7].mean()), 3),
                angacc_rms=round(float(np.sqrt(np.mean(np.sum(wdot ** 2, axis=1)))), 2), jointacc_rms=round(float(np.sqrt(np.mean(np.sum(qdd ** 2, axis=1)))), 2),
                pen_vel_err_rms=round(float(1e3 * np.sqrt(np.mean(np.sum((pv[m] - vref[m]) ** 2, axis=1)))), 2) if m.any() else None)


def make(ctrl):
    if ctrl.startswith("NDFL:"):                                                           # NMPC + learned model correction: "NDFL:<run dir>/model_final.zip" (config.json gives nch)
        from stable_baselines3 import PPO
        from ocrl.am3_hybrid_l import NMPCDFL
        z = ctrl[5:]; cfg = json.load(open(os.path.join(os.path.dirname(z), "config.json")))
        return NMPCDFL(cfg["nch"], PPO.load(z, device="cpu"))
    if ctrl.endswith(".zip"):
        from stable_baselines3 import PPO, SAC
        from ocrl.am3_rl_wall import RLCtrl, KIND_KW
        cfg = os.path.join(os.path.dirname(ctrl), "config.json"); cfg = json.load(open(cfg)) if os.path.exists(cfg) else {}
        algo = SAC if cfg.get("algo") == "sac" else PPO
        return RLCtrl(algo.load(ctrl, device="cpu"), vel_ff=True, **KIND_KW.get(cfg.get("env"), {}))           # same env options the policy was trained with
    if ctrl.startswith("NDF"):                                                             # ND + state filter; "NDF:alpha" (observer gains as ND:10,10)
        from ocrl.am3_nmpc_filt import NMPCDF
        from ocrl.am3_nmpc_dob import K_DOB
        k = K_DOB.copy(); k[6:9] = 10.0
        a = ctrl.split(":")[1].split(",") if ":" in ctrl else ["0.3"]                           # "NDF:alpha[,f]": f = observer on the filtered state
        return NMPCDF(float(a[0]), k, obs_on_filtered=len(a) > 1 and a[1] == "f")
    if ctrl.startswith("NF"):                                                              # NMPC with a state filter; "NF:alpha" (default 0.3)
        from ocrl.am3_nmpc_filt import NMPCF
        return NMPCF(float(ctrl.split(":")[1])) if ":" in ctrl else NMPCF()
    if ctrl.startswith("ND"):                                                              # offset-free NMPC; "ND:kb,kj" sets the base (6) and joint (3) observer gains (1/s)
        from ocrl.am3_nmpc_dob import NMPCD, K_DOB
        k = K_DOB.copy()
        if ":" in ctrl: g = [float(v) for v in ctrl.split(":")[1].split(",")]; k[0:6] = g[0]; k[6:9] = g[1] if len(g) > 1 else 0.0       # "ND:base[,joint]"
        return NMPCD(k)
    from am3_run import make_ctrl
    return make_ctrl(ctrl)


def main(a):
    rows = []
    for text in a.text:
        t0 = time.time(); ep = WriteEpisode(make(a.ctrl), text, height=a.height, v_write=a.speed); ep.run(); m = dict(text=text, height=a.height, **write_metrics(ep), wall_s=round(time.time() - t0)); rows.append(m); print(m, flush=True)
    if a.out: json.dump(rows, open(os.path.join(HERE, "..", "results", "am3_write", a.out), "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ctrl"); ap.add_argument("text", nargs="+"); ap.add_argument("--height", type=float, default=0.06); ap.add_argument("--speed", type=float, default=0.025); ap.add_argument("--out", default=None); main(ap.parse_args())
