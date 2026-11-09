"""Measurable-metric figures for one matched B/N trajectory comparison, drawn from the saved closed-loop logs (results/am3_runs/<tag>_B|N/log.npz + summary.json).  Writes <out>_timeseries.png,
<out>_summary.png and <out>_metrics.json.  Usage: am3_figures.py RUN_B RUN_N OUT_PREFIX TITLE PATH"""
import sys, os, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from ocrl.am3_task import BASE, YZ0, make_path, XW
from ocrl.am3_plant import KF
COL = dict(B="#d9731a", N="#1f5fbf"); RUNS = os.path.join(HERE, "..", "results", "am3_runs"); PH_COL = dict(position="#eef", approach="#eef6ff", settle="#fff2d9", trace="#e6f5e6", endhold="#fff2d9", release="#f3e6f3", retreat="#eef", settle_resume="#fff2d9", recover="#fadada")

def load(run):
    L = np.load(os.path.join(RUNS, run, "log.npz"), allow_pickle=True); S = json.load(open(os.path.join(RUNS, run, "summary.json"))); return dict(L), S

def series(L):
    t = L["t"]; X = L["x"]; ph = np.array([str(p) for p in L["phase"]]); eb = X[:, 0:3] - BASE; tilt = np.degrees(2 * np.arcsin(np.minimum(1, np.hypot(X[:, 4], X[:, 5])))); pe = L["pad"] - L["ref"]
    vref = np.gradient(L["ref"][:, 1:3], t, axis=0); sp = np.linalg.norm(vref, axis=1); tang = np.where(sp[:, None] > 1e-4, vref / np.maximum(sp[:, None], 1e-9), 0.0); along = np.sum(pe[:, 1:3] * tang, axis=1); cross = pe[:, 1:3][:, 0] * (-tang[:, 1]) + pe[:, 1:3][:, 1] * tang[:, 0]
    tp = np.array(L["physics_t"]); Fn = np.array(L["physics_Fn"]); ft = np.linalg.norm(np.array(L["physics_ft"]), axis=1); f = KF * np.array(L["physics_Omega"]) ** 2; tau = np.abs(np.array(L["physics_tau"]))
    return dict(t=t, ph=ph, eb=eb, ebn=np.linalg.norm(eb, axis=1), tilt=tilt, pe=pe, along=along, cross=cross, moving=sp > 1e-4, tp=tp, Fn=Fn, ft=ft, fmin=f.min(1), fmax=f.max(1), tau=tau.max(1))

def shade(ax, t, ph):
    i = 0
    while i < len(t):
        j = i
        while j + 1 < len(t) and ph[j + 1] == ph[i]: j += 1
        if ph[i] in PH_COL: ax.axvspan(t[i], t[j], color=PH_COL[ph[i]], lw=0, zorder=0)
        i = j + 1

def make(runB, runN, out, title, path):
    LB, SB = load(runB); LN, SN = load(runN); D = dict(B=series(LB), N=series(LN)); S = dict(B=SB, N=SN)
    fig, ax = plt.subplots(8, 1, figsize=(12, 22), sharex=True); ref = D["N"]
    for a in ax: shade(a, ref["t"], ref["ph"]); a.grid(alpha=.3)
    for c in "BN":
        d = D[c]; ax[0].semilogy(d["t"], np.maximum(d["ebn"] * 1e3, 1e-2), color=COL[c], label=f"{c}"); ax[1].plot(d["t"], d["tilt"], color=COL[c]); ax[2].plot(d["t"], d["pe"][:, 0] * 1e3, color=COL[c]); ax[3].plot(d["t"], np.linalg.norm(d["pe"][:, 1:3], axis=1) * 1e3, color=COL[c])
        ax[4].plot(d["tp"], d["Fn"], color=COL[c], lw=1.2); ax[4].plot(d["tp"], d["ft"], color=COL[c], ls="--", lw=1.0); ax[5].plot(d["t"], (np.interp(d["t"], d["tp"], d["Fn"]) > 0.08).astype(float) + (0.06 if c == "N" else 0.0), color=COL[c], drawstyle="steps-post")
        ax[6].plot(d["tp"], d["fmin"], color=COL[c], lw=1.0); ax[6].plot(d["tp"], d["fmax"], color=COL[c], lw=1.0); ax[7].plot(d["tp"], d["tau"], color=COL[c], lw=1.0)
    ax[0].axhline(20, color="k", ls=":", lw=1); ax[0].text(0.01, 20, " declared tolerance 20 mm", va="bottom", fontsize=8, transform=ax[0].get_yaxis_transform()); ax[1].axhline(15, color="k", ls=":", lw=1)
    ax[6].axhline(0.1, color="k", ls=":"); ax[6].axhline(10, color="k", ls=":"); ax[7].axhline(1.0, color="k", ls=":")
    for a, y in zip(ax, ("|base position error| [mm] (log)", "base tilt [deg]", "pad - reference, wall-normal x [mm]\n(+ = away from wall)", "pad - reference, tangential [mm]", "Fn (solid), |ft| (dashed) [N]", "pad pressing (Fn > 0.08 N)\n(N offset by 0.06 for visibility)", "rotor thrust min / max [N]\n(limits 0.1 and 10)", "max joint torque [N m]\n(1 N m continuous)")): a.set_ylabel(y, fontsize=9)
    ax[0].legend(handles=[plt.Line2D([], [], color=COL["B"], label="B decoupled baseline (200 Hz)"), plt.Line2D([], [], color=COL["N"], label="N coupled NMPC (50 Hz)")], loc="upper left"); ax[-1].set_xlabel("simulation time [s]  (background colours: task phases)")
    fig.suptitle(f"{title}: measured time series, same plant, seed and reference", y=0.995); fig.tight_layout(); fig.savefig(out + "_timeseries.png", dpi=110); plt.close(fig)

    # ---- summary page: wall-plane projection, error distributions, metric table
    fig = plt.figure(figsize=(14, 13)); g = fig.add_gridspec(3, 2, height_ratios=[1.15, 0.8, 1.3])
    P = None
    if not path.startswith("free"):
        pth, T = make_path(path); P = np.array([YZ0 + pth.at(s)[0] for s in np.linspace(0, pth.length, 200)])
    for k, c in enumerate("BN"):
        a = fig.add_subplot(g[0, k]); L = (LB, LN)[k]; d = D[c]; pad = L["pad"]; press = np.interp(L["t"], d["tp"], d["Fn"]) > 0.08; sel = np.isin(d["ph"], ["settle", "settle_resume", "trace", "endhold", "recover"])
        if P is not None: a.plot(P[:, 0] * 1e3, P[:, 1] * 1e3, color="#22aa44", lw=3, alpha=.7, label="reference path")
        a.plot(pad[sel & ~press, 1] * 1e3, pad[sel & ~press, 2] * 1e3, ".", color="#2060ff", ms=4, label="pad, detached (projected)"); a.plot(pad[sel & press, 1] * 1e3, pad[sel & press, 2] * 1e3, ".", color="#cc1111", ms=3, label="pad, pressing"); a.set_aspect("equal"); a.set_title(f"{c}: pad position projected onto the wall plane [mm]", fontsize=10); a.legend(fontsize=8); a.grid(alpha=.3)
    # error distributions (trace window)
    a = fig.add_subplot(g[1, 0]); tw = {c: np.isin(D[c]["ph"], ["trace"]) for c in "BN"}
    for c in "BN":
        if tw[c].any(): e = np.linalg.norm(D[c]["pe"][tw[c]], axis=1) * 1e3; a.hist(e, bins=40, alpha=.6, color=COL[c], label=f"{c}: median {np.median(e):.2f}, p95 {np.percentile(e, 95):.2f}, max {e.max():.2f} mm")
    a.set_xlabel("|pad - reference| during trace [mm]"); a.set_ylabel("samples"); a.legend(fontsize=8); a.grid(alpha=.3)
    a = fig.add_subplot(g[1, 1])
    for c in "BN":
        if tw[c].any(): a.plot(D[c]["t"][tw[c]], D[c]["along"][tw[c]] * 1e3, color=COL[c], label=f"{c} along-track"); a.plot(D[c]["t"][tw[c]], D[c]["cross"][tw[c]] * 1e3, color=COL[c], ls="--", label=f"{c} cross-track")
    a.set_xlabel("simulation time [s]"); a.set_ylabel("tracking error components [mm]"); a.legend(fontsize=8, ncol=2); a.grid(alpha=.3)
    # table
    rows = []; M = {}
    for c in "BN":
        s = S[c]; d = D[c]; tr = s.get("trace", {}); k = s["contact"]; tw_ = tw[c]
        M[c] = dict(base_rms_mm=s["base"]["rms_mm"], base_max_mm=s["base"]["max_mm"], tilt_max_deg=s["attitude"]["tilt_max_deg"], tilt_rms_deg=s["attitude"]["tilt_rms_deg"], pad_ref_rms_mm=s["pad_vs_reference"]["rms_mm"], pad_ref_max_mm=s["pad_vs_reference"]["max_mm"],
                    trace_track_rms_mm=tr.get("track_rms_mm"), trace_track_max_mm=tr.get("track_max_mm"), along_rms_mm=float(np.sqrt(np.mean(d["along"][tw_] ** 2)) * 1e3) if tw_.any() else None, cross_rms_mm=float(np.sqrt(np.mean(d["cross"][tw_] ** 2)) * 1e3) if tw_.any() else None,
                    wall_normal_err_rms_mm=float(np.sqrt(np.mean(d["pe"][tw_, 0] ** 2)) * 1e3) if tw_.any() else None, endpoint_err_mm=tr.get("endpoint_err_mm"), coverage_5mm=tr.get("coverage_5mm"), contact_valid_fraction=k["valid_fraction"], detached_total_s=k.get("detached_total_s"), losses=k["losses"], recovery_s=k["recovery_s"],
                    peak_Fn_N=k["peak_Fn_N"], Fn_mean_trace_N=k["Fn_mean_trace_N"], Fn_std_trace_N=k["Fn_std_trace_N"], peak_ft_N=k["peak_ft_N"], rotor_force_min_N=s["actuation"]["rotor_force_min_N"], rotor_force_max_N=s["actuation"]["rotor_force_max_N"], tau_util_max=max(s["actuation"]["tau_util_max"]),
                    cycle_median_ms=s["timing"]["median_ms"], cycle_max_ms=s["timing"]["max_ms"], period_ms=s["timing"]["period_ms"], completed=s["completed"], t_end_s=s["t_end"], wall_clock_s=s["wall_s"])
    keys = [("base_rms_mm", "base error RMS [mm]"), ("base_max_mm", "base error max [mm]"), ("tilt_max_deg", "tilt max [deg]"), ("pad_ref_rms_mm", "pad-ref error RMS, whole run [mm]"), ("trace_track_rms_mm", "trace tracking RMS [mm]"), ("trace_track_max_mm", "trace tracking max [mm]"), ("along_rms_mm", "  along-track RMS [mm]"), ("cross_rms_mm", "  cross-track RMS [mm]"),
            ("wall_normal_err_rms_mm", "  wall-normal RMS [mm]"), ("endpoint_err_mm", "endpoint error [mm]"), ("coverage_5mm", "path coverage within 5 mm"), ("contact_valid_fraction", "contact-valid fraction (intended window)"), ("detached_total_s", "detached time in window [s]"), ("losses", "contact losses (recoveries)"), ("peak_Fn_N", "peak normal force [N]"),
            ("Fn_mean_trace_N", "mean Fn in trace [N]"), ("Fn_std_trace_N", "Fn std in trace [N]"), ("peak_ft_N", "peak friction force [N]"), ("rotor_force_min_N", "rotor thrust min [N]"), ("rotor_force_max_N", "rotor thrust max [N]"), ("tau_util_max", "joint torque / 1 N m max"), ("cycle_median_ms", "controller cycle median [ms]"), ("cycle_max_ms", "controller cycle max [ms]"), ("period_ms", "control period [ms]"), ("t_end_s", "simulated duration [s]")]
    fmt = lambda v: "-" if v is None else (f"{v:.3g}" if isinstance(v, float) else str(v)); cell = [[lab, fmt(M["B"][k]), fmt(M["N"][k])] for k, lab in keys]
    a = fig.add_subplot(g[2, :]); a.axis("off"); tb = a.table(cellText=cell, colLabels=["metric", "B (200 Hz)", "N (50 Hz)"], loc="center", cellLoc="left", colWidths=[0.5, 0.2, 0.2]); tb.auto_set_font_size(False); tb.set_fontsize(8.5); tb.scale(1, 1.18)
    for (i, j), cl in tb.get_celld().items():
        if i == 0: cl.set_facecolor("#dde6f5")
    fig.suptitle(f"{title}: path projection, error distributions and metrics", y=0.995); fig.tight_layout(); fig.savefig(out + "_summary.png", dpi=110); plt.close(fig); json.dump(M, open(out + "_metrics.json", "w"), indent=1, default=str); return M

if __name__ == "__main__": make(*sys.argv[1:6])
