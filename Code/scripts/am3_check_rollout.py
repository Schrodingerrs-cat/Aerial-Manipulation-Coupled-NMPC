"""Open-loop rollout of the plant (MuJoCo, 0.25 ms) against the predictor (RK4, 1 ms steps) with identical held inputs from the predictor's equilibrium:
A free flight with a joint-torque step and a rotor-command step; B pad in contact (0.6 N equilibrium) with the same perturbations.  Reports state-error maxima at 0.5 s.  -> results/am3_predictor/rollout_checks.json"""
import sys, os, json, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ocrl.am3_plant import AMPlant3, R_PAD
from ocrl.am3_pred import AMPred3, C
from ocrl.am3_ref import Equilibrium
BASE = [0.8 - R_PAD - C["task"]["standoff_pad_from_base_m"], 0.0, 1.2]; ZP = 1.04; pr = AMPred3(); eq = Equilibrium(pr, BASE); st = pr.step_fun(0.001, 2); out = {}
for name, F, dx in (("free_flight", 0.0, -0.05), ("contact", 0.6, 0.0)):
    e = eq.solve([0.8 - R_PAD + F / 600, 0, ZP], [-F, 0, 0]); x0 = e["x"].copy(); x0[0] += dx; u = e["u"] + np.r_[20, -20, 20, -20, 0.01, -0.01, 0.005]; pl = AMPlant3(); pl.set_state(x0); xp = x0.copy(); err = []; T = 0.5; marks = {}
    for k in range(int(T / 0.001)):
        for _ in range(4): pl.step(u)
        xp = np.array(st(xp, u, np.zeros(2))).ravel(); xm = pl.state(); err.append(np.r_[np.abs(xm[0:3] - xp[0:3]), np.abs(xm[7:10] - xp[7:10]), np.abs(xm[10:13] - xp[10:13]), np.abs(xm[13:16] - xp[13:16]), np.abs(xm[19:23] - xp[19:23])])
    for tm in (0.05, 0.1, 0.2, 0.5): marks[str(tm)] = dict(p_m=float(np.max(np.array(err)[:int(tm / 0.001), 0:3])), q_rad=float(np.max(np.array(err)[:int(tm / 0.001), 9:12])), omega_rad_s=float(np.max(np.array(err)[:int(tm / 0.001), 6:9])))
    err = np.array(err).max(0); out[name] = dict(p_m=err[0:3].tolist(), v_m_s=err[3:6].tolist(), omega_rad_s=err[6:9].tolist(), q_rad=err[9:12].tolist(), Omega_rad_s=err[12:16].tolist(), growth=marks, final_pad_x_mm=float(pl.pad_pos()[0] * 1e3), Fn_plant_final=float(pl.pad_force()[0]))
    print(name, json.dumps(out[name]["growth"]))
json.dump(out, open(os.path.join(os.path.dirname(__file__), "..", "results", "am3_predictor", "rollout_checks.json"), "w"), indent=1)
