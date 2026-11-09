"""Figures for results/am3_arm_motion/<TAG>: (1) time series of selected cases (motion start, joints, base displacement, tilt, rotor speeds, B vs N), (2) peak base displacement and tilt over the whole grid.  Usage: am3_arm_motion_figs.py TAG OUT_DIR [osc]   (osc: oscillation runs, time series only)"""
import sys, os, json, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
tag, out = sys.argv[1:3]; D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results", "am3_arm_motion", tag); os.makedirs(out, exist_ok=True); CB, CN = "#d55e00", "#0072b2"
osc = len(sys.argv) > 3 and sys.argv[3] == "osc"; kind = "osc" if osc else "loop"; cases = [(0.06, 0.5), (0.06, 0.25)] if osc else [(1.0, 1.5), (1.5, 0.8)]; fig, ax = plt.subplots(5, 2, figsize=(13, 13), sharex="col")
for j, (s, T) in enumerate(cases):
    B, N = (np.load(os.path.join(D, f"{c}_{kind}_s{s}_T{T}.npz")) for c in "BN"); t, t0 = B["t"], float(B["t_start"]); q0 = N["q"][np.argmin(abs(N["t"] - t0))]
    ax[0, j].set_title(f"pad oscillation +-{s} m, {T} s per half cycle (free flight)" if osc else f"pad-loop scale {s}, {T} s per leg (free flight, no wall)")
    for k, nm in enumerate(("q1 yaw", "q2 shoulder", "q3 elbow")): ax[0, j].plot(N["t"], np.degrees(N["q"][:, k] - q0[k]), label=nm + " (N)", lw=1.2); ax[0, j].plot(B["t"], np.degrees(B["q"][:, k] - B["q"][np.argmin(abs(B["t"] - t0)), k]), "--", lw=1, color=f"C{k}")
    ax[1, j].plot(N["t"], np.abs(N["qd"]).max(1), color=CN, label="N"); ax[1, j].plot(B["t"], np.abs(B["qd"]).max(1), color=CB, label="B")
    ax[2, j].plot(B["t"], B["base_mm"], color=CB, label="B"); ax[2, j].plot(N["t"], N["base_mm"], color=CN, label="N")
    ax[3, j].plot(B["t"], B["tilt"], color=CB); ax[3, j].plot(N["t"], N["tilt"], color=CN)
    pre = (N["t"] > t0 - 1) & (N["t"] < t0); prb = (B["t"] > t0 - 1) & (B["t"] < t0)
    for k in range(4): ax[4, j].plot(B["t"], B["Om"][:, k] - B["Om"][prb, k].mean(), color=CB, alpha=.35 + .15 * k, lw=1); ax[4, j].plot(N["t"], N["Om"][:, k] - N["Om"][pre, k].mean(), color=CN, alpha=.35 + .15 * k, lw=1)
    for a in ax[:, j]: a.axvline(t0, color="k", ls=":", lw=1); a.grid(alpha=.3)
    ax[0, j].legend(fontsize=7, ncol=3); ax[1, j].legend(fontsize=8); ax[2, j].legend(fontsize=8); ax[4, j].set_xlabel("simulation time (s); dotted line = arm motion commanded")
for a, l in zip(ax[:, 0], ("joint angle change (deg), B dashed", "max joint speed (rad/s)", "base displacement from hover ref (mm)", "tilt (deg)", "rotor speed change from pre-motion (rad/s), 4 rotors")): a.set_ylabel(l, fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(out, "arm_oscillation_timeseries.png" if osc else "arm_motion_timeseries.png"), dpi=130); plt.close(fig)
if osc: sys.exit()
R = json.load(open(os.path.join(D, "summary.json"))); fig, ax = plt.subplots(1, 3, figsize=(14, 4)); keys = [(s, T) for s in (0.5, 1.0, 1.5) for T in (3.0, 1.5, 0.8)]; x = np.arange(len(keys))
for a, (k, lab) in zip(ax, (("base_max_mm", "peak base displacement (mm)"), ("tilt_max_deg", "peak tilt (deg)"), ("rotor_dev_max_pct", "peak rotor-speed change (%)"))):
    for i, (c, col) in enumerate((("B", CB), ("N", CN))): a.bar(x + (i - .5) * .38, [next(r for r in R if r["ctrl"] == c and r["scale"] == s and r["seg_s"] == T)["motion"][k] for s, T in keys], .38, color=col, label=c)
    a.set_xticks(x); a.set_xticklabels([f"{s}\n{T}s" for s, T in keys], fontsize=7); a.set_title(lab, fontsize=10); a.grid(alpha=.3, axis="y")
ax[0].legend(); ax[0].set_xlabel("pad-loop scale / seconds per leg"); fig.tight_layout(); fig.savefig(os.path.join(out, "arm_motion_grid.png"), dpi=130)
