"""Physics isolation (user request 30 Oct): is the drone's motion a believable response to arm motion and wall contact?  Uses MuJoCo's own momentum balance (ocrl/am3_probe.py), not the predictor.
  step1  stationary arm, no wall: known small force / torque pulse at the COM (open loop) -> a_COM = F/m, dL/dt = tau, effective inertia
  step2  arm motion, no wall: one joint at a time (torque pulse, rotors at trim) -> joint accel, base accel, COM accel, coupling explanation M_bb^-1 M_bq qdd
  step3  gentle wall contact, no sliding: approach at 1 cm/s -> contact force, moment about the COM, balance residuals, pitch acceleration vs moment / inertia
Closed-loop steps (stable hover response, sliding, fixed-command diagnostic) are in am3_physics_closed.py.  Usage: am3_physics_check.py -> results/am3_physics/step{1,2,3}.{json,png}"""
import sys, os, json, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from ocrl.am3_episode import initial_state
from ocrl.am3_plant import AMPlant3, R_PAD
from ocrl.am3_pred import AMPred3
from ocrl.am3_ref import Equilibrium
from ocrl.am3_task import BASE
from ocrl.am3_probe import Probe
OUT = os.path.join(HERE, "..", "results", "am3_physics"); os.makedirs(OUT, exist_ok=True); pr = AMPred3(); res = {}
def run(x0, u_of_t, T, dist=None, tau_extra=None):
    """dist: plant disturbance list (force/torque steps applied at the drone-body COM through the plant's own mechanism; the probe counts them as external loads)."""
    pl = AMPlant3(); pl.set_state(x0); pl.dist = dist or []; pb = Probe(pl); log = []
    for k in range(int(T / pl.dt)):
        t = pl.d.time; u = np.array(u_of_t(t), float)
        if tau_extra is not None: u = u + np.r_[0, 0, 0, 0, tau_extra(t)]
        pl.step(u); pb.record(); d = pl.d
        log.append(dict(t=d.time, qacc=d.qacc.copy(), qvel=d.qvel.copy(), qpos=d.qpos.copy(), Fn=pl.pad_force()[0], Om=pl.Omega.copy()))
    return pl, pb.arrays(), log
x0, u0 = initial_state(); hold = lambda t: u0

# ---- step 1: known force / torque pulses at the drone-body COM, open loop; compared over the first 8 ms of the pulse (before the pulse tilts the thrust vector)
F0, TAU0 = 0.3, 0.02; pl, A, lg = run(x0, hold, 0.4, dist=[dict(kind="step", axis=0, amp=F0, t0=0.1, dur=0.1)]); m_tot = pl.mass; w = (A["t"] >= 0.1005) & (A["t"] < 0.108)
aF = A["a"][w, 0].mean(); step1 = dict(force_pulse=dict(F_N=F0, a_com_measured=float(aF), a_com_expected=F0 / m_tot, ratio=float(aF / (F0 / m_tot)), max_res_F=float(np.abs(A["res_F"][~A["switch"].astype(bool)]).max()), max_res_M=float(np.abs(A["res_M"][~A["switch"].astype(bool)]).max())))
pl2, A2, lg2 = run(x0, hold, 0.4, dist=[dict(kind="step", axis=4, amp=TAU0, t0=0.1, dur=0.1)]); w2 = (A2["t"] >= 0.1005) & (A2["t"] < 0.108); dLy = A2["dL"][w2, 1].mean(); alpha = np.mean([l["qacc"][4] for l, ok in zip(lg2[1:], w2) if ok])
step1["torque_pulse"] = dict(tau_Nm=TAU0, dL_dt_measured=float(dLy), ratio=float(dLy / TAU0), pitch_accel_measured=float(alpha), effective_pitch_inertia_about_com_kgm2=float(TAU0 / alpha), max_res_F=float(np.abs(A2["res_F"][~A2["switch"].astype(bool)]).max()), max_res_M=float(np.abs(A2["res_M"][~A2["switch"].astype(bool)]).max()), a_com_max=float(np.abs(A2["a"]).max()))
res["step1"] = step1
fig, ax = plt.subplots(2, 2, figsize=(11, 7)); ax[0, 0].plot(A["t"], A["a"][:, 0] * 1e3, label="a_COM,x measured"); ax[0, 0].axhline(F0 / m_tot * 1e3, color="k", ls="--", label="F / m (expected during pulse)"); ax[0, 0].set(title="Force pulse 0.3 N (0.1-0.2 s) at the COM", ylabel="mm/s^2", xlabel="t [s]"); ax[0, 0].legend()
ax[0, 1].semilogy(A["t"], np.abs(A["res_F"]).max(1) + 1e-16); ax[0, 1].set(title="Newton residual |m a - (F_rot + m g + F_ext)|", ylabel="N", xlabel="t [s]")
ax[1, 0].plot(A2["t"], A2["dL"][:, 1] * 1e3, label="dL_COM,y/dt measured"); ax[1, 0].axhline(TAU0 * 1e3, color="k", ls="--", label="applied torque"); ax[1, 0].set(title="Torque pulse 0.02 N m about y at the COM", ylabel="mN m", xlabel="t [s]"); ax[1, 0].legend()
ax[1, 1].plot(A2["t"], np.array([l["qacc"][4] for l in lg2[1:]]), label="base pitch accel"); ax[1, 1].plot(A2["t"], np.array([l["qacc"][8] for l in lg2[1:]]), label="q3 accel"); ax[1, 1].set(title="Response to the torque pulse (open loop)", ylabel="rad/s^2", xlabel="t [s]"); ax[1, 1].legend()
fig.suptitle("Step 1: stationary arm, no wall, open loop at hover trim"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "step1.png"), dpi=130); plt.close(fig)

# ---- step 2: one joint at a time (rotors fixed at trim), torque pulse 0.05 N m for 0.1 s; compared over the first 8 ms of the pulse (posture still unchanged)
step2 = {}; fig, ax = plt.subplots(3, 3, figsize=(13, 9))
for j, name in enumerate(("q1 (yaw)", "q2 (shoulder)", "q3 (elbow)")):
    tp = np.zeros(3); tp[j] = 0.05; pl, A, lg = run(x0, hold, 0.3, tau_extra=lambda t: tp * (0.1 <= t < 0.2)); t = A["t"]; qacc = np.array([l["qacc"] for l in lg[1:]]); w = (t >= 0.1005) & (t < 0.108)
    xs = np.zeros(23); xs[0:3] = x0[0:3]; xs[3:7] = x0[3:7]; xs[13:16] = x0[13:16]; M = np.array(pr.M_f(x0)); Mbb, Mbq = M[0:6, 0:6], M[0:6, 6:9]; qdd = qacc[w, 6:9].mean(0); coupling = -np.linalg.solve(Mbb, Mbq @ qdd); meas = qacc[w, 0:6].mean(0)
    step2[name] = dict(joint_accel=qdd.tolist(), base_accel_measured_lin_ang=meas.tolist(), base_accel_from_coupling_term=coupling.tolist(), com_accel_first8ms=A["a"][w].mean(0).tolist(), base_lin_accel_first8ms=meas[0:3].tolist(), max_res_F=float(np.abs(A["res_F"]).max()), max_res_M=float(np.abs(A["res_M"]).max()), rotor_speed_change=float(np.abs(np.array([l["Om"] for l in lg]) - x0[19:23]).max()))
    ax[0, j].plot(t, qacc[:, 6 + j]); ax[0, j].set(title=f"{name}: joint accel, 0.05 N m pulse", ylabel="rad/s^2"); ax[1, j].plot(t, qacc[:, 0] * 1e3, label="base x"); ax[1, j].plot(t, qacc[:, 2] * 1e3, label="base z"); ax[1, j].plot(t, A["a"][:, 0] * 1e3, "k--", label="COM x"); ax[1, j].plot(t, A["a"][:, 2] * 1e3, "k:", label="COM z"); ax[1, j].plot(t, (A["F_rot"][:, 0]) / pl.mass * 1e3, "r.", ms=1, label="(F_rotors+mg)/m, x"); ax[1, j].plot(t, (A["F_rot"][:, 2] - pl.mass * 9.81) / pl.mass * 1e3, "m.", ms=1, label="(F_rotors+mg)/m, z")
    ax[1, j].set(title="linear accel [mm/s^2]"); ax[1, j].legend(fontsize=7); ax[2, j].plot(t, qacc[:, 3:6]); ax[2, j].set(title="base angular accel [rad/s^2]", xlabel="t [s]"); ax[2, j].legend(["x", "y", "z"], fontsize=7)
res["step2"] = step2; fig.suptitle("Step 2: arm motion, no wall, rotors fixed at trim. At pulse onset the base accelerates, the COM does not; afterwards the COM follows the tilted rotor thrust (dots)"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "step2.png"), dpi=130); plt.close(fig)

# ---- step 3: gentle wall contact, no sliding (approach at 2 cm/s from 2 mm, rotors and joint torques fixed at the free-flight trim of the contact posture)
eq = Equilibrium(pr, BASE); e = eq.solve([0.8 - R_PAD, 0.0, 1.04], [0, 0, 0]); xc = e["x"].copy(); xc[0] -= 0.002; xc[7] = 0.02; pl, A, lg = run(xc, lambda t: e["u"], 0.45); t = A["t"]; Fn = np.array([l["Fn"] for l in lg[1:]]); on = Fn > 1e-3
Mw = A["M_w"][:, 1]; alpha_meas = np.array([l["qacc"][4] for l in lg[1:]]); qacc3 = np.array([l["qacc"] for l in lg[1:]])
step3 = dict(peak_Fn_N=float(Fn.max()), contact_start_s=float(t[on][0]) if on.any() else None, max_res_F=float(np.abs(A["res_F"]).max()), max_res_M=float(np.abs(A["res_M"]).max()), wall_moment_about_com_peak_Nm=float(np.abs(Mw).max()),
             com_x_accel_vs_balance_max_err=float(np.abs(A["a"][:, 0] - (A["F_rot"][:, 0] + A["F_w"][:, 0]) / pl.mass).max()))
res["step3"] = step3; fig, ax = plt.subplots(2, 2, figsize=(11, 7)); ax[0, 0].plot(t, Fn); ax[0, 0].set(title="Normal force on the pad", ylabel="N", xlabel="t [s]"); ax[0, 1].plot(t, A["a"][:, 0], label="a_COM,x measured"); ax[0, 1].plot(t, (A["F_rot"][:, 0] + A["F_w"][:, 0]) / pl.mass, "k--", label="(F_rotors + F_wall)/m"); ax[0, 1].legend(); ax[0, 1].set(title="COM acceleration vs external-force balance", ylabel="m/s^2", xlabel="t [s]")
ax[1, 0].plot(t, Mw * 1e3, label="wall moment about COM (y)"); ax[1, 0].plot(t, A["dL"][:, 1] * 1e3, "k--", label="dL_COM,y/dt"); ax[1, 0].legend(); ax[1, 0].set(title="Moment balance about the COM", ylabel="mN m", xlabel="t [s]"); ax[1, 1].plot(t, alpha_meas, label="base pitch accel"); ax[1, 1].plot(t, qacc3[:, 7], label="q2 accel"); ax[1, 1].plot(t, qacc3[:, 8], label="q3 accel"); ax[1, 1].legend(); ax[1, 1].set(title="The arm swings too (joints are torque-held, not locked)", ylabel="rad/s^2", xlabel="t [s]")
fig.suptitle("Step 3: gentle wall contact, open loop, no sliding"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "step3.png"), dpi=130); plt.close(fig)
json.dump(res, open(os.path.join(OUT, "steps_1_2_3.json"), "w"), indent=1); print(json.dumps(res, indent=1))
