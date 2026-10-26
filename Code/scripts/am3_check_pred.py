"""Independent verification of the v3 predictor against MuJoCo (no contact): pad forward kinematics, pad Jacobian (all three columns, also by finite differences), the full 9x9 mass matrix including base-joint
cross blocks, gravity+velocity bias, symmetry/positive definiteness, material-point velocity of the contact point, and joint/base accelerations under rotor and joint inputs at random base attitude, joint
state and velocities.  Writes results/am3_predictor/static_checks.json."""
import os, sys, json, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_pred import AMPred3
from ocrl.am3_plant import AMPlant3, KF
rng = np.random.default_rng(1); pr = AMPred3(); pl = AMPlant3(); m, d = pl.m, pl.d; n = pr.nx
LO, HI = np.radians([-35, -80, -125]), np.radians([35, -5, -30])
def rand_state():
    x = np.zeros(23); x[0:3] = [0.3, 0.0, 1.2] + 0.05 * rng.standard_normal(3); ang = 0.15 * rng.standard_normal(3); th = np.linalg.norm(ang); ax = ang / max(th, 1e-12); x[3:7] = [np.cos(th / 2), *(np.sin(th / 2) * ax)]
    x[7:10] = 0.3 * rng.standard_normal(3); x[10:13] = 0.5 * rng.standard_normal(3); x[13:16] = rng.uniform(LO, HI); x[16:19] = rng.uniform(-1.5, 1.5, 3); x[19:23] = rng.uniform(900, 1500, 4); return x
res = dict(fk=0.0, jac_cols=0.0, jac_fd=0.0, M=0.0, M_cross_norm=1e9, M_min_eig=1e9, bias=0.0, contact_point_vel=0.0, acc=0.0, M_scale=0.0, bias_scale=0.0, acc_scale=0.0)
skipped = 0
for _ in range(50):
    x = rand_state(); pl.set_state(x); R = d.xmat[1].reshape(3, 3)
    p_pred = np.array(pr.pad_f(x)[0]).ravel(); res["fk"] = max(res["fk"], np.abs(p_pred - d.site_xpos[pl.sid]).max())
    Jp = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, d, Jp, None, pl.sid); Jq = Jp[:, pl.vadr]; Jpred = R @ np.array(pr.Jpad_f(x[13:16])); res["jac_cols"] = max(res["jac_cols"], np.abs(Jq - Jpred).max())
    fd = np.stack([(np.array(pr.fk_f(x[13:16] + e)).ravel() - np.array(pr.fk_f(x[13:16] - e)).ravel()) / 2e-6 for e in 1e-6 * np.eye(3)], 1); res["jac_fd"] = max(res["jac_fd"], np.abs(fd - np.array(pr.Jpad_f(x[13:16]))).max())
    Mmj = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, Mmj); Mp = np.array(pr.M_f(x)); res["M"] = max(res["M"], np.abs(Mmj - Mp).max()); res["M_scale"] = max(res["M_scale"], np.abs(Mmj).max())
    res["M_cross_norm"] = min(res["M_cross_norm"], np.abs(Mp[0:6, 6:9]).max()); res["M_min_eig"] = min(res["M_min_eig"], np.linalg.eigvalsh(0.5 * (Mp + Mp.T)).min()); assert np.abs(Mp - Mp.T).max() < 1e-12
    bmj = d.qfrc_bias.copy(); bp = np.array(pr.bias_f(x)).ravel() + np.array(pr.grav_f(x)).ravel(); res["bias"] = max(res["bias"], np.abs(bmj - bp).max()); res["bias_scale"] = max(res["bias_scale"], np.abs(bmj).max())
    n_w = np.array([-1.0, 0, 0]); JC = np.array(pr.JC_f(x)[0]); nu = np.r_[x[7:10], x[10:13], x[16:19]]; pc = d.site_xpos[pl.sid] - pl.m.geom_size[pl.gpad][0] * n_w          # material point on the pad surface facing the wall
    Jc = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, Jc, None, pc, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand")); res["contact_point_vel"] = max(res["contact_point_vel"], np.abs(Jc @ d.qvel - JC @ nu).max())
    u = np.r_[rng.uniform(900, 1500, 4), rng.uniform(-0.8, 0.8, 3)]; pl.set_state(x); pl.Omega = x[19:23].copy()
    d.ctrl[:4] = KF * x[19:23] ** 2; d.ctrl[4:7] = u[4:7]; mujoco.mj_forward(m, d); acc_mj = d.qacc.copy(); acc_pr = np.array(pr.xdot_f(x, u, np.zeros(2))).ravel(); acc_pr = np.r_[acc_pr[7:10], acc_pr[10:13], acc_pr[16:19]]
    if d.ncon: skipped += 1; continue                       # random joint draws can touch the airframe/wall: contact accelerations are checked in the contact tests
    res["acc"] = max(res["acc"], np.abs(acc_mj - acc_pr).max()); res["acc_scale"] = max(res["acc_scale"], np.abs(acc_mj).max())

res["acc_samples_skipped_for_contact"] = skipped; res["note"] = "max abs error over 50 random states; M_cross_norm is the smallest max-abs base-joint cross block (non-zero coupling), contact point at the pad surface facing the wall; acc uses d.ctrl thrust kf*Omega^2 and joint torques, plant joint damping"
os.makedirs(os.path.join(HERE, "..", "results", "am3_predictor"), exist_ok=True); json.dump(res, open(os.path.join(HERE, "..", "results", "am3_predictor", "static_checks.json"), "w"), indent=1)
for k, v in res.items(): print(k, v)
