import json, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from ocrl.am3_kin_poe import ArmPoE, exp6
D = "/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck/"
RED, DK, GR = "#AB162B", "#272626", "#6D6D6D"
plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif"], "mathtext.fontset": "stix", "font.size": 20, "axes.edgecolor": GR,
                     "axes.labelcolor": DK, "xtick.color": DK, "ytick.color": DK, "axes.spines.top": False, "axes.spines.right": False})
# lever picture: the arm as a scale
A = ArmPoE(); q = np.radians([0, -37, -78]); T, pad = A.fk(q); T12 = exp6(A.S[0], q[0]) @ exp6(A.S[1], q[1])
P = np.array([[0, 0, 0], A.o1, A.o2, (T12 @ np.r_[A.o3, 1])[:3], T[:3, 3]])
fig, ax = plt.subplots(figsize=(8, 6))
ax.add_patch(plt.Rectangle((-0.17, -0.012), 0.34, 0.024, color=DK)); ax.text(-0.17, 0.035, "drone body", color=DK)
ax.plot(P[:, 0], P[:, 2], "-", color="#3B4252", lw=10, solid_capstyle="round")
sh = A.o2; ax.plot(sh[0], sh[2], "o", ms=20, mfc="white", mec=RED, mew=3.5); ax.text(sh[0] - 0.2, sh[2] - 0.005, "shoulder\nmotor", color=RED, va="center")
h = P[-1]; ax.add_patch(plt.Rectangle((h[0] - 0.02, h[2] - 0.045), 0.04, 0.035, color="#D4A017")); ax.text(h[0] + 0.03, h[2] - 0.03, "load  m", color=DK)
ax.annotate("", (h[0], h[2] - 0.13), (h[0], h[2] - 0.045), arrowprops=dict(arrowstyle="-|>", color=RED, lw=3)); ax.text(h[0] + 0.012, h[2] - 0.11, "weight = m g", color=RED)
ax.plot([sh[0], h[0]], [sh[2] - 0.2, sh[2] - 0.2], color=DK, lw=1.5); ax.plot([sh[0]] * 2, [sh[2] - 0.19, sh[2] - 0.21], color=DK, lw=1.5); ax.plot([h[0]] * 2, [sh[2] - 0.19, sh[2] - 0.21], color=DK, lw=1.5)
ax.plot([h[0]] * 2, [h[2] - 0.13, sh[2] - 0.2], ":", color=GR, lw=1.2); ax.plot([sh[0]] * 2, [sh[2], sh[2] - 0.2], ":", color=GR, lw=1.2)
ax.text((sh[0] + h[0]) / 2, sh[2] - 0.235, "lever arm  d  (known from angles + lengths)", ha="center", color=DK, fontsize=17)
ax.set_aspect("equal"); ax.set_xlim(-0.24, 0.36); ax.set_ylim(-0.37, 0.07); ax.axis("off"); fig.savefig(D + "f_lever.png", dpi=200, transparent=True, bbox_inches="tight")
# torque vs load
t = json.load(open(D + "torque.json")); g2, a2 = abs(t["g_arm"][1]), t["a"][1]; ms = np.linspace(0, 0.3, 61)
fig, ax = plt.subplots(figsize=(9, 5)); ax.plot(1000 * ms, g2 + a2 * ms, color=DK, lw=3.5)
ax.text(165, 0.58, "torque the shoulder\nmust give", color=DK, fontsize=18)
ax.axhline(1.0, color=RED, lw=3, ls="--"); ax.text(5, 1.03, "strongest the shoulder motor can push: 1.0 N m", color=RED, fontsize=18)
mb = (1 - g2) / a2; ax.plot(1000 * mb, 1.0, "o", ms=12, color=RED); ax.annotate(f"limit: {1000 * mb:.0f} g", (1000 * mb, 1.0), (1000 * mb - 15, 0.72), color=RED, fontsize=19, ha="center", arrowprops=dict(arrowstyle="->", color=RED))
ax.set_xlabel("load in the gripper (g)"); ax.set_ylabel("shoulder torque (N m)"); ax.set_ylim(0.4, 1.12); ax.set_xlim(0, 300)
fig.savefig(D + "f_torque2.png", dpi=200, transparent=True, bbox_inches="tight"); print("ok")
