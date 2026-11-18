import json, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from ocrl.am3_kin_poe import ArmPoE, exp6
D = "/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck/"
RED, DK, GR, LG = "#AB162B", "#272626", "#6D6D6D", "#D9D9D9"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 15, "axes.edgecolor": GR, "axes.labelcolor": DK, "xtick.color": DK, "ytick.color": DK, "axes.spines.top": False, "axes.spines.right": False})

# F1: PoE schematic, side view (x-z) at the writing posture
A = ArmPoE(); q = np.radians([0, -37, -78]); T, pad = A.fk(q)
T1 = exp6(A.S[0], q[0]); T12 = T1 @ exp6(A.S[1], q[1]); T123 = T12 @ exp6(A.S[2], q[2])
pts = [np.zeros(3), A.o1, A.o2, (T12 @ np.r_[A.o3, 1])[:3], T[:3, 3], pad]
fig, ax = plt.subplots(figsize=(7.2, 6.2))
ax.add_patch(plt.Rectangle((-0.17, -0.012), 0.34, 0.024, color=DK)); ax.text(-0.165, 0.045, "drone body", color=DK, fontsize=14)
for s in (-0.15, 0.15): ax.add_patch(plt.Rectangle((s - 0.06, 0.012), 0.12, 0.008, color=GR))
P = np.array(pts); ax.plot(P[:4, 0], P[:4, 2], "-", color="#3B4252", lw=9, solid_capstyle="round"); ax.plot(P[3:5, 0], P[3:5, 2], "-", color="#3B4252", lw=9, solid_capstyle="round")
ax.plot(P[4:, 0], P[4:, 2], "-", color="#E07B39", lw=9, solid_capstyle="round")
for p, lab, off in [(A.o1, "joint 1 (yaw)\n$S_1$: axis $\\hat z$", (-0.19, -0.01)), (A.o2, "joint 2 (pitch)\n$S_2$: axis $\\hat y$", (0.045, -0.035)), (P[3], "joint 3 (pitch)\n$S_3$: axis $\\hat y$", (-0.21, -0.02))]:
    ax.plot(p[0], p[2], "o", ms=16, mfc="white", mec=RED, mew=3); ax.text(p[0] + off[0], p[2] + off[1], lab, color=RED, fontsize=13, va="center")
ax.plot(pad[0], pad[2], "o", ms=12, color=RED); ax.text(pad[0] + 0.02, pad[2] - 0.01, "pad (end effector)", color=DK, fontsize=13)
ax.annotate("", (0.07, 0), (0, 0), arrowprops=dict(arrowstyle="-|>", color=RED, lw=2.5)); ax.annotate("", (0, 0.07), (0, 0), arrowprops=dict(arrowstyle="-|>", color=RED, lw=2.5))
ax.text(0.072, -0.004, "$x_s$", color=RED, fontsize=15); ax.text(-0.012, 0.075, "$z_s$", color=RED, fontsize=15); ax.text(0.085, 0.045, "{s} = body frame", color=RED, fontsize=13)
ax.set_aspect("equal"); ax.set_xlim(-0.2, 0.33); ax.set_ylim(-0.25, 0.1); ax.axis("off"); fig.tight_layout(); fig.savefig(D + "f_poe.png", dpi=200, transparent=True, bbox_inches="tight")

# F2: mass estimate
m = json.load(open("results/capstone_kin/mass_check.json")); keys = ["nominal", "payload100g", "payload200g", "kf90"]; lab = ["no load", "100 g load", "200 g load", "no load,\nthrust -10 %"]
x = np.arange(4); w = 0.27; fig, ax = plt.subplots(figsize=(8.6, 4.6))
for i, (k, c, n) in enumerate([("true_kg", LG, "true"), ("joint_estimate_kg", RED, "ours: joint torques + PoE"), ("base_force_estimate_kg", GR, "vertical force only")]):
    v = [1000 * m[kk][k] for kk in keys]; ax.bar(x + (i - 1) * w, v, w, color=c, label=n)
    for xi, vi in zip(x, v): ax.text(xi + (i - 1) * w, max(vi, 0) + 4, f"{int(round(vi))}", ha="center", fontsize=12, color=DK)
ax.set_xticks(x, lab); ax.set_ylabel("payload mass (g)"); ax.set_ylim(0, 245); ax.legend(frameon=False, fontsize=13, loc="upper left"); fig.tight_layout(); fig.savefig(D + "f_mass.png", dpi=200, transparent=True, bbox_inches="tight")

# F3: shoulder torque vs payload
t = json.load(open(D + "torque.json")); g2, a2 = t["g_arm"][1], t["a"][1]; ms = np.linspace(0, 0.3, 61); tau = np.abs(g2) + a2 * ms
fig, ax = plt.subplots(figsize=(8.6, 4.6)); ax.plot(1000 * ms, tau, color=DK, lw=3, label="static shoulder torque (PoE)")
ax.axhline(1.0, color=RED, lw=2.5, ls="--"); ax.text(5, 1.025, "servo limit 1.0 N m", color=RED, fontsize=13)
mb = (1 - abs(g2)) / a2; ax.axvline(1000 * mb, color=RED, lw=1, ls=":"); ax.text(1000 * mb + 3, 0.5, f"{1000 * mb:.0f} g", color=RED, fontsize=13)
ax.axvspan(120, 180, color="#9BC59D", alpha=0.35, lw=0); ax.text(150, 0.53, "RL writes\n(sim)", ha="center", fontsize=12, color=DK)
ax.axvspan(195, 225, color=RED, alpha=0.18, lw=0); ax.text(210, 0.53, "RL flips\n(sim)", ha="center", fontsize=12, color=RED)
ax.set_xlabel("payload at the hand (g)"); ax.set_ylabel("torque (N m)"); ax.set_ylim(0.4, 1.12); ax.set_xlim(0, 300); fig.tight_layout(); fig.savefig(D + "f_torque.png", dpi=200, transparent=True, bbox_inches="tight")
print("figs ok", mb)
