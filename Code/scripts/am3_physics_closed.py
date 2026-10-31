"""Closed-loop physics isolation (steps 1b, 4, 5 of the 30 Oct investigation), using the MuJoCo momentum balance of ocrl/am3_probe.py:
  step1b  stable stationary hover with feedback (B and N): known force pulse (0.3 N, x) and torque pulse (0.02 N m, y) -> response vs the expected F/m and tau
  step4   sliding contact: slow line and circle (B and N): normal force, friction, moments about the COM, rotor moment/force that balance them, COM vs base motion
  step5   fixed-command diagnostic: N's joint-torque schedule replayed open loop with the rotors frozen at the initial trim, against B and N closed loop (same arm motion)
Usage: am3_physics_closed.py -> results/am3_physics/{step1b,step4_line,step4_circle,step5}.{json,png}"""
import sys, os, json, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from am3_run import make_ctrl, free_waypoints
from ocrl.am3_episode import Episode, initial_state, BASE
from ocrl.am3_plant import AMPlant3
from ocrl.am3_pred import AMPred3
from ocrl.am3_probe import Probe
OUT = os.path.join(HERE, "..", "results", "am3_physics"); COL = dict(B="#d9731a", N="#1f5fbf", open="#444444"); pr = AMPred3(); res = {}
def live(c, path, T, mismatch=None, wp=None, seg=None, seed=0):
    x0, u0 = initial_state(); p0 = np.array(pr.pad_f(x0)[0]).ravel(); ep = Episode(make_ctrl(c), path, seed=seed, T_max=T, free_waypoints=(wp if wp is not None else ([p0] * (int(T / 5) + 2) if path == "hover" else None)), segment_s=seg or (5.0 if path == "hover" else None), mismatch=mismatch)
    pb = Probe(ep.pl); orig = ep.pl.step; ep.pl.step = lambda u: (orig(u), pb.record())[0]; ep.run(); return ep, pb.arrays()
def base_err(ep): return np.array([r["t"] for r in ep.rows]), (np.array([r["x"][0:3] for r in ep.rows]) - BASE) * 1e3, np.array([np.degrees(2 * np.arcsin(min(1, np.hypot(r["x"][4], r["x"][5])))) for r in ep.rows])

def step1b():
    global res
    # ---------------- step 1b
    F0, TAU0 = 0.3, 0.02; mm = dict(disturbance=[dict(kind="step", axis=0, amp=F0, t0=3.0, dur=0.2), dict(kind="step", axis=4, amp=TAU0, t0=7.0, dur=0.2)]); s1 = {}; fig, ax = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    for c in "BN":
        ep, A = live(c, "hover", 12.0, mismatch=mm); t, e, tilt = base_err(ep); tp = A["t"]; w = (tp >= 3.0005) & (tp < 3.01); wt = (tp >= 7.0005) & (tp < 7.01)
        s1[c] = dict(abort=str(ep.abort), a_com_first10ms_force_pulse=float(A["a"][w, 0].mean()), expected_F_over_m=F0 / ep.pl.mass, dLdt_first10ms_torque_pulse=float(A["dL"][wt, 1].mean()), expected_tau=TAU0, peak_base_dev_force_pulse_mm=float(np.abs(e[(t > 3) & (t < 6)]).max()), peak_tilt_force_pulse_deg=float(tilt[(t > 3) & (t < 6)].max()),
                     peak_base_dev_torque_pulse_mm=float(np.abs(e[(t > 7) & (t < 10)]).max()), base_mm_last2s=float(np.linalg.norm(e[t > 10], axis=1).max()), max_balance_res_F=float(np.abs(A["res_F"][~A["switch"].astype(bool)]).max()), max_balance_res_M=float(np.abs(A["res_M"][~A["switch"].astype(bool)]).max()))
        ax[0].plot(t, e[:, 0], color=COL[c], label=f"{c} base x"); ax[0].plot(t, e[:, 1], color=COL[c], ls="--"); ax[0].plot(t, e[:, 2], color=COL[c], ls=":"); ax[1].plot(t, tilt, color=COL[c]); ax[2].plot(tp, A["a"][:, 0], color=COL[c], lw=.8); f = np.array([r["u_app"][:4] for r in ep.rows]); ax[3].plot(t, np.array([r["info"].get("T", 0) for r in ep.rows]) if c == "B" else np.zeros(len(t)), color=COL[c], lw=0)
        ax[3].plot(tp, A["res_F"][:, 0] + 1e-12, color=COL[c], lw=.8)
    ax[2].axhline(F0 / ep.pl.mass, color="k", ls=":", label="F/m = %.3f m/s^2 (pulse 3.0-3.2 s)" % (F0 / ep.pl.mass)); ax[2].legend()
    for a, y in zip(ax, ("base position error [mm]\n(solid x, dashed y, dotted z)", "tilt [deg]", "COM acceleration x [m/s^2]", "Newton residual x: m a - (F_rot + m g + F_ext) [N]")): a.set_ylabel(y, fontsize=9); a.grid(alpha=.3)
    ax[0].legend(); ax[3].set_xlabel("simulation time [s]"); fig.suptitle("Step 1b: stationary arm, stable hover feedback. Force pulse 0.3 N at 3 s, torque pulse 0.02 N m at 7 s"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "step1b.png"), dpi=120); plt.close(fig); res["step1b"] = s1; json.dump(s1, open(os.path.join(OUT, "step1b.json"), "w"), indent=1)


def step4():
    global res
    # ---------------- step 4: sliding contact
    for path, T in (("line", 40.0), ("circle", 40.0)):
        s4 = {}; fig, ax = plt.subplots(6, 1, figsize=(12, 20), sharex=True)
        for c in "BN":
            ep, A = live(c, path, T); t, e, tilt = base_err(ep); tp = A["t"]; ph = np.array([str(r["phase"]) for r in ep.rows]); tw = (t[ph == "trace"][0], t[ph == "trace"][-1]) if (ph == "trace").any() else (0, 0); m = (tp >= tw[0]) & (tp <= tw[1]); pt = A["pt"]; com = A["com"]; n = np.array([-1.0, 0, 0])
            Fw = A["F_w"]; Fn = Fw @ n; Ff = Fw - np.outer(Fn, n); rarm = pt - com; Mn = np.cross(rarm, np.outer(Fn, n)); Mf = np.cross(rarm, Ff); ok = ~np.isnan(pt[:, 0])
            s4[c] = dict(abort=str(ep.abort), trace_window_s=list(map(float, tw)), max_balance_res_F=float(np.abs(A["res_F"][m & ok]).max()), max_balance_res_M=float(np.abs(A["res_M"][m & ok]).max()), Fn_mean=float(Fn[m & ok].mean()), friction_to_normal_mean=float(np.mean(np.linalg.norm(Ff[m & ok], axis=1) / np.maximum(Fn[m & ok], 1e-6))),
                         moment_normal_about_com_peak=np.abs(Mn[m & ok]).max(0).tolist(), moment_friction_about_com_peak=np.abs(Mf[m & ok]).max(0).tolist(), rotor_moment_peak=np.abs(A["M_rot"][m]).max(0).tolist(), lateral_rotor_force_y_plus_friction_y_rms_N=float(np.sqrt(np.mean((A["F_rot"][m & ok, 1] + Ff[m & ok, 1]) ** 2))), friction_y_rms_N=float(np.sqrt(np.mean(Ff[m & ok, 1] ** 2))), com_accel_y_rms_x_mass_N=float(np.sqrt(np.mean((ep.pl.mass * A["a"][m & ok, 1]) ** 2))),
                         com_excursion_mm=np.ptp((com[m] - com[m][0]) * 1e3, axis=0).tolist(), base_excursion_mm=np.ptp(e[(t >= tw[0]) & (t <= tw[1])], axis=0).tolist(), tilt_range_deg=[float(tilt[(t >= tw[0]) & (t <= tw[1])].min()), float(tilt[(t >= tw[0]) & (t <= tw[1])].max())])
            ax[0].plot(tp, Fn, color=COL[c], label=f"{c} normal"); ax[0].plot(tp, np.linalg.norm(Ff, axis=1), color=COL[c], ls="--", label=f"{c} friction"); ax[1].plot(tp, Mn[:, 1] * 1e3, color=COL[c], label=f"{c} normal-force moment (y)"); ax[1].plot(tp, Mf[:, 1] * 1e3, color=COL[c], ls="--", label=f"{c} friction moment (y)")
            ax[2].plot(tp, Mf[:, 2] * 1e3, color=COL[c], label=f"{c} friction moment (z, yaw)"); ax[2].plot(tp, A["M_rot"][:, 2] * 1e3, color=COL[c], ls=":", label=f"{c} rotor yaw moment"); ax[3].plot(tp, A["M_rot"][:, 1] * 1e3, color=COL[c], label=f"{c} rotor moment (y)"); ax[3].plot(tp, -(Mn[:, 1] + Mf[:, 1]) * 1e3, color="k", lw=.7, ls=":")
            ax[4].plot(tp, A["F_rot"][:, 1] , color=COL[c], label=f"{c} rotor force y"); ax[4].plot(tp, -Ff[:, 1], color="k", lw=.7, ls=":"); ax[5].plot(t, tilt, color=COL[c], label=f"{c} tilt [deg]"); ax[5].plot(t, np.linalg.norm(e, axis=1), color=COL[c], ls="--", label=f"{c} |base error| [mm]")
        for a, y in zip(ax, ("wall force on pad [N]", "moment about COM, y [mN m]", "yaw moments about COM [mN m]", "rotor moment y [mN m]; dotted = -(wall moment): rotors balance the wall", "rotor force y [N]; dotted = - friction force y", "response")): a.set_ylabel(y, fontsize=9); a.grid(alpha=.3); a.legend(fontsize=7, ncol=3)
        for a in ax:
            for x in (s4["N"]["trace_window_s"]): a.axvline(x, color="g", lw=.6)
        ax[0].set_xlim(s4["N"]["trace_window_s"][0] - 1.0, s4["N"]["trace_window_s"][1] + 1.0); ax[-1].set_xlabel("simulation time [s] (green: trace window)"); fig.suptitle(f"Step 4: sliding contact, {path}: loads about the COM and the response"); fig.tight_layout(); fig.savefig(os.path.join(OUT, f"step4_{path}.png"), dpi=120); plt.close(fig); res[f"step4_{path}"] = s4; json.dump(s4, open(os.path.join(OUT, f"step4_{path}.json"), "w"), indent=1)


def step5():
    global res
    # ---------------- step 5: fixed-command diagnostic on the vigorous free arm motion
    wp, seg = free_waypoints("free_big"); epN, AN = live("N", "free", 9.0, wp=wp, seg=seg); epB, AB = live("B", "free", 9.0, wp=wp, seg=seg); tN, eN, tiltN = base_err(epN); tB, eB, tiltB = base_err(epB)
    x0, u0 = initial_state(); sched = [(r["t"], r["u_next"][4:7].copy()) for r in epN.rows]; ts = np.array([a for a, _ in sched]); tau_s = np.array([b for _, b in sched]); Mq = pr.M_f
    def replay(frozen_torque, T=2.0):
        pl = AMPlant3(); pl.set_state(x0); rows = []; tau_hold = tau_s[0]
        for k in range(int(T / pl.dt)):
            t = pl.d.time; i = max(0, np.searchsorted(ts, t, side="right") - 1); u = np.r_[u0[:4], tau_hold if frozen_torque else tau_s[i]]; pl.step(u); d = pl.d; rows.append((d.time, d.qpos[0:3] - BASE, np.degrees(2 * np.arcsin(min(1, np.hypot(d.qpos[4], d.qpos[5])))), d.qacc.copy(), pl.state()))
        return rows
    RO, RF = replay(False), replay(True); to = np.array([r[0] for r in RO]); eo = np.array([r[1] for r in RO]) * 1e3; tiltO = np.array([r[2] for r in RO]); ef = np.array([r[1] for r in RF]) * 1e3; tiltF = np.array([r[2] for r in RF])
    sel = (to > 1.0) & (to < 1.5); cpl = np.array([-np.linalg.solve(np.array(Mq(r[4]))[0:6, 0:6], np.array(Mq(r[4]))[0:6, 6:9] @ r[3][6:9]) for r, k in zip(RO, sel) if k][::20]); meas = np.array([r[3][0:6] for r, k in zip(RO, sel) if k][::20])
    at = lambda tt, arr: float(np.linalg.norm(arr[np.searchsorted(to, tt) - 1]))
    s5 = dict(note="open loop = rotors frozen at the initial hover trim; joint torques either replayed from N's run (arm moves from 1 s) or frozen at the initial trim (control); N's torque schedule includes N's own base-stabilising corrections, so this shows magnitude and direction, not a clean pure-arm-motion response",
              base_dev_mm_arm_replayed={f"{tt}s": at(tt, eo) for tt in (0.5, 1.0, 1.25, 1.5, 1.75)}, base_dev_mm_torques_frozen={f"{tt}s": at(tt, ef) for tt in (0.5, 1.0, 1.25, 1.5, 1.75)}, tilt_deg_arm_replayed={f"{tt}s": float(tiltO[np.searchsorted(to, tt) - 1]) for tt in (0.5, 1.0, 1.25, 1.5, 1.75)}, tilt_deg_torques_frozen={f"{tt}s": float(tiltF[np.searchsorted(to, tt) - 1]) for tt in (0.5, 1.0, 1.25, 1.5, 1.75)},
              B_closed_loop_base_max_mm_first2s=float(np.linalg.norm(eB[tB < 2], axis=1).max()), N_closed_loop_base_max_mm_first2s=float(np.linalg.norm(eN[tN < 2], axis=1).max()), arm_motion_starts_s=1.0, window_1_to_1p5s_coupling_term_vs_measured_base_angular_accel_corr=[float(np.corrcoef(cpl[:, j], meas[:, j])[0, 1]) for j in (3, 4, 5) if np.std(meas[:, j]) > 1e-6])
    fig, ax = plt.subplots(3, 1, figsize=(11, 11), sharex=True)
    ax[0].plot(to, np.linalg.norm(eo, axis=1), color=COL["open"], label="open loop, arm torques replayed (arm moves from 1 s)"); ax[0].plot(to, np.linalg.norm(ef, axis=1), color="#999999", ls="--", label="open loop, arm torques frozen (control: no arm motion)"); ax[0].plot(tB[tB < 2], np.linalg.norm(eB[tB < 2], axis=1), color=COL["B"], label="B closed loop"); ax[0].plot(tN[tN < 2], np.linalg.norm(eN[tN < 2], axis=1), color=COL["N"], label="N closed loop"); ax[0].set_yscale("log"); ax[0].set_ylim(1e-3, 1e3); ax[0].legend(fontsize=8); ax[0].set_ylabel("|base error| [mm]")
    ax[1].plot(to, tiltO, color=COL["open"]); ax[1].plot(to, tiltF, color="#999999", ls="--"); ax[1].plot(tB[tB < 2], tiltB[tB < 2], color=COL["B"]); ax[1].plot(tN[tN < 2], tiltN[tN < 2], color=COL["N"]); ax[1].set_ylabel("tilt [deg]"); ax[1].set_ylim(0, 30)
    tt = np.array([r[0] for r, k in zip(RO, sel) if k][::20]); ax[2].plot(tt, meas[:, 4], color=COL["open"], marker=".", label="base pitch accel, measured (open loop, 1.0-1.5 s)"); ax[2].plot(tt, cpl[:, 4], "k--", label="coupling term  -M_bb^-1 M_bq qdd"); ax[2].legend(); ax[2].set_ylabel("rad/s^2")
    for a in ax: a.grid(alpha=.3); a.axvline(1.0, color="g", lw=.6); a.set_xlim(0, 2.0)
    ax[2].set_xlabel("simulation time [s]  (arm motion starts at 1 s; later times omitted: the open-loop assembly leaves the valid range)"); fig.suptitle("Step 5: fixed-command diagnostic: uncorrected response to the arm motion, next to B and N feedback"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "step5.png"), dpi=120); plt.close(fig); res["step5"] = s5; json.dump(s5, open(os.path.join(OUT, "step5.json"), "w"), indent=1)

if __name__ == "__main__":
    which = sys.argv[1:] or ["1b", "4", "5"]
    for k, f in (("1b", step1b), ("4", step4), ("5", step5)):
        if k in which: f()
    json.dump(res, open(os.path.join(OUT, "steps_closed_" + "_".join(which) + ".json"), "w"), indent=1)
