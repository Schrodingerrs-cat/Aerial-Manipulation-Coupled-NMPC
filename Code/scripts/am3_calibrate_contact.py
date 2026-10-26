"""Contact calibration of the v3 plant against the predictor's spring-damper law.
(a) static stiffness: fixture (gravity off, base held by a stiff spring, joints held by PD) presses the pad into the wall at set indentations; fitted slope = effective stiffness (compare k = 600 N/m).
(b) impact: plant vs predictor at 1-3 cm/s closing speed from the same equilibrium (peak force and impulse of the first contact at physics resolution) for several predictor damping values.
Usage: am3_calibrate_contact.py  -> results/am3_predictor/contact_calibration.json"""
import sys, os, json, numpy as np, mujoco
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ocrl import am3_plant
from ocrl.am3_plant import AMPlant3, R_PAD
from ocrl.am3_pred import AMPred3, C
from ocrl.am3_ref import Equilibrium
BASE = [0.8 - R_PAD - C["task"]["standoff_pad_from_base_m"], 0.0, 1.2]; ZP = 1.04; out = {}
def first_contact(F, dt):
    F = np.asarray(F); i = np.argmax(F > 1e-3) if (F > 1e-3).any() else None
    if i is None: return 0.0, 0.0
    j = i
    while j < len(F) and (F[j] > 1e-3 or (F[j:j + int(0.02 / dt)] > 1e-3).any()): j += 1
    return float(F[i:j].max()), float(F[i:j].sum() * dt)
eq = Equilibrium(AMPred3(), BASE)

pl = AMPlant3(); m, d = pl.m, pl.d; m.opt.gravity[:] = 0; pts = []; e = eq.solve([0.8 - R_PAD, 0, ZP], [0, 0, 0]); jv = pl.vadr
for delta in (0.0005, 0.001, 0.002, 0.003):
    pl.set_state(e["x"]); tgt = pl.d.qpos[0] + delta; q_hold = d.qpos[pl.qadr].copy(); base = d.qpos[:7].copy(); base[0] = tgt + (e["x"][0] - e["x"][0]); Fs = []
    base[0] = e["x"][0] + delta - (pl.pad_pos()[0] - (0.8 - R_PAD)) * 0
    for k in range(int(1.5 / pl.dt)):
        d.qfrc_applied[:3] = -20000 * (d.qpos[:3] - base[:3]) - 400 * d.qvel[:3]; qerr = np.zeros(3); mujoco.mju_subQuat(qerr, d.qpos[3:7], base[3:7]); d.qfrc_applied[3:6] = -400 * qerr - 8 * d.qvel[3:6]
        d.ctrl[:4] = 0; d.ctrl[4:7] = np.clip(-30 * (d.qpos[pl.qadr] - q_hold) - 3 * d.qvel[jv], -3, 3); mujoco.mj_step(m, d)
        if d.time > 1.0: Fs.append(pl.pad_force()[0])
    ind = pl.pad_pos()[0] + R_PAD - 0.8; pts.append((ind, float(np.mean(Fs)))); print(f"indentation {ind*1e3:6.3f} mm  F {np.mean(Fs):.3f} N")
ind, F = np.array(pts).T; k_eff = float((ind @ F) / (ind @ ind)); out["static"] = dict(points=pts, k_eff_N_m=k_eff, k_requested=C["contact"]["k_N_m"]); print("k_eff", k_eff, "-> K_CAL for 600:", am3_plant.K_CAL * 600 / k_eff)

res = {}; pl = AMPlant3(); e1 = eq.solve([0.8 - R_PAD + 0.003, 0, ZP], [0, 0, 0])
for v in (0.01, 0.02, 0.03):
    x0 = e1["x"].copy(); x0[0] -= 0.006; x0[7] = v; pl.set_state(x0); pl.reset_log()
    for _ in range(1600): pl.step(e1["u"])
    res[v] = first_contact(pl.log["Fn"], pl.dt)
out["plant_impact"] = {str(v): dict(peak_N=a, impulse_Ns=b) for v, (a, b) in res.items()}; print(out["plant_impact"])
for cp in (4.0, 6.0, 8.0, 10.0, 14.0):
    pr = AMPred3(c=cp); st = pr.step_fun(0.001, 1); row = []
    for v in (0.01, 0.02, 0.03):
        xp = e1["x"].copy(); xp[0] -= 0.006; xp[7] = v; Fp = []
        for _ in range(800): xp = np.array(st(xp, e1["u"], np.zeros(2))).ravel(); Fp.append(float(pr.contact_f(xp, e1["u"], np.zeros(2))[2]))
        pk, im = first_contact(Fp, 0.001); row.append((pk / max(res[v][0], 1e-9), im / max(res[v][1], 1e-12)))
    out[f"pred_c{cp}"] = dict(peak_ratio=[round(r[0], 3) for r in row], impulse_ratio=[round(r[1], 3) for r in row]); print(cp, out[f"pred_c{cp}"], flush=True)
json.dump(out, open(os.path.join(os.path.dirname(__file__), "..", "results", "am3_predictor", "contact_calibration.json"), "w"), indent=1)
