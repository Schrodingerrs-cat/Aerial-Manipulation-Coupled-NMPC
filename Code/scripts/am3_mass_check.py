"""Capstone: closed-form payload mass from the ND observer's joint-torque estimate (ocrl/am3_kin_poe.ArmPoE.payload_mass), compared with the
naive base-force reading -d_z/g.  Runs ND on HELLO WPI (real noise) for T s per case.  Usage: am3_mass_check.py [T] -> results/capstone_kin/mass_check.json"""
import sys, os, json, numpy as np
from stable_baselines3 import PPO          # torch before acados
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_write_eval import make
from am3_write_bench import RealSensors, HEIGHT, SPEED
from am3_write_mismatch import CASES
from ocrl.am3_write import WriteEpisode
from ocrl.am3_kin_poe import ArmPoE, G0
from ocrl.am3_pred import quat2R

T = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0; A = ArmPoE(); out = {}
for case, true_kg in (("nominal", 0.0), ("payload100g", 0.10), ("payload200g", 0.20), ("kf90", 0.0)):
    ep = WriteEpisode(make("ND:10,10"), "HELLO WPI", height=HEIGHT, v_write=SPEED, seed=0, mismatch=CASES[case], T_max=T); ep.sens = RealSensors(0); ep.run()
    m_j, m_z, t = [], [], []
    for r in ep.rows:
        d = r["info"].get("d") if r["info"] else None
        if d is None: continue
        xm = r["xm"]; R = np.array(quat2R(xm[3:7])); m_j.append(A.payload_mass(xm[13:16], R, d[6:9])); m_z.append(-d[2] / G0); t.append(r["t"])
    t, m_j, m_z = map(np.array, (t, m_j, m_z)); late = t > 5.0; i90 = np.argmax(np.abs(m_j - m_j[late].mean()) < 0.1 * max(true_kg, 0.05)) if late.any() else -1
    out[case] = dict(true_kg=true_kg, joint_estimate_kg=round(float(m_j[late].mean()), 4), joint_std_kg=round(float(m_j[late].std()), 4),
                     base_force_estimate_kg=round(float(m_z[late].mean()), 4), settle_s=round(float(t[i90]), 2), abort=ep.abort)
    print(case, out[case], flush=True)
json.dump(out, open(os.path.join(HERE, "..", "results", "capstone_kin", "mass_check.json"), "w"), indent=1)
