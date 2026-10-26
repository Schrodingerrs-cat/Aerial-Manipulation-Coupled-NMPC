"""Position-only feasibility of the wall paths with the base fixed at its hover reference (Plan s3.3): IK for the pad centre, Jacobian conditioning, joint limits/speeds, inverse dynamics through the
predictor (joint torques vs the servo envelope, rotor speeds, tilt, with sliding friction opposing the pad velocity), and MuJoCo clearances (non-pad geoms vs wall, arm vs airframe) along the timed path.
Usage: am3_feasibility.py [path ...] -> results/am3_task/feasibility_<path>.json"""
import sys, os, json, numpy as np
from scipy.optimize import least_squares
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_pred import AMPred3, C
from ocrl.am3_ref import Equilibrium
from ocrl.am3_static import Rig
from ocrl.am3_plant import servo_limit, OM_MIN, OM_MAX
from ocrl.am3_path import line_path, circle_path, rounded_polygon, reference
TASK = json.load(open(os.path.join(HERE, "..", "config", "am3_task_v1.json"))); R_PAD, XW = C["pad"]["radius_m"], C["wall_x_m"]; BASE = np.array(TASK["base_ref_world"]); YZ0 = np.array(TASK["patch_centre_world_yz_m"])
OP = C["joint_limits_deg"]["operational"]; QLO, QHI = np.radians([OP[k][0] for k in ("q1", "q2", "q3")]), np.radians([OP[k][1] for k in ("q1", "q2", "q3")]); KF = C["propulsion"]["f_max_N"] / C["propulsion"]["omega_max_rad_s"] ** 2

def make_path(name):
    p = TASK["paths"][name]
    if p["kind"] == "line": return line_path(p["a"], p["b"]), p["T_s"]
    if p["kind"] == "circle": return circle_path(p["centre"], p["radius"]), p["T_s"]
    return rounded_polygon(p["vertices"], p["fillet_m"]), p["T_s"]

def analyse(name, n=200, mu=C["contact"]["mu"], dt_fd=1e-3):
    path, T = make_path(name); pr = AMPred3(); eq = Equilibrium(pr, BASE); rig = Rig(); Fn = TASK["contact_policy"]["indentation_ref_m"] * C["contact"]["k_N_m"]; rows = [];
    for t in np.linspace(0, T, n):
        p, v, a = reference(path, T, t); tgt = np.r_[XW - R_PAD + Fn / C["contact"]["k_N_m"], YZ0 + p]
        v3, a3 = np.r_[0, v], np.r_[0, a]; vt = np.linalg.norm(v3); tdir = v3 / vt if vt > 1e-6 else np.zeros(3); Fc = np.r_[-Fn, -mu * Fn * tdir[1:]]                  # wall force on the robot: normal + friction opposing the pad velocity
        guess = np.r_[np.radians([0, -37, -78]), 0, 0.04, [1200] * 4, 0, 0, 0] if eq.last is None else eq.last
        for _ in range(3):                                          # attitude, joints, rotors, torques jointly; joint rates/accelerations from the pad reference at the current joints
            q = guess[0:3]; J = np.array(pr.Jpad_f(q)); qd = np.linalg.solve(J, v3); eps = 1e-5; dJ = (np.array(pr.Jpad_f(q + eps * qd)) - np.array(pr.Jpad_f(q - eps * qd))) / (2 * eps); qdd = np.linalg.solve(J, a3 - dJ @ qd)
            def res(z):
                xs, u = eq.unpack(z); xs[16:19] = qd; r = np.array(eq.res_f(xs, u, Fc)).ravel(); r[6:9] -= qdd; r[9:] = 1e3 * (r[9:] - tgt); return r
            s = least_squares(res, guess, x_scale=np.r_[0.05, 0.05, 0.05, 0.01, 0.01, [100] * 4, 0.1, 0.1, 0.1], xtol=1e-14, ftol=1e-14, gtol=1e-14); guess = s.x
        eq.last = guess; q = s.x[0:3]
        om, tau = s.x[5:9], s.x[9:12]; lim = np.abs(servo_limit(1e3 * np.sign(tau), qd)); rig.pose(BASE, s.x[3], s.x[4], q); cl = rig.clearances(); sv = np.linalg.svd(J, compute_uv=False)
        rows.append(dict(t=t, ik_err=float(np.abs(s.fun[9:]).max() / 1e3), q=q, qd=qd, qdd=qdd, sv_min=sv.min(), tilt_deg=np.degrees(np.hypot(*s.x[3:5])), f_rotor=KF * om ** 2, om=om, tau=tau, tau_util=np.abs(tau) / lim, res=float(np.linalg.norm(s.fun)),
                         axis_deg=np.degrees(np.arccos(rig.hand_axis()[0])), gap_wall=cl["wall_nonpad"], gap_air=cl["arm_airframe"], gap_arm=cl["arm_arm"]))
    A = lambda k: np.array([r[k] for r in rows]); out = dict(path=name, T_s=T, samples=n, ik_err_max_m=float(A("ik_err").max()), solve_res_max=float(A("res").max()),
        q_min_deg=np.degrees(A("q").min(0)).tolist(), q_max_deg=np.degrees(A("q").max(0)).tolist(), op_lo_deg=np.degrees(QLO).tolist(), op_hi_deg=np.degrees(QHI).tolist(), qd_max_rad_s=np.abs(A("qd")).max(0).tolist(), qdd_max_rad_s2=np.abs(A("qdd")).max(0).tolist(),
        sv_min_m_per_rad=float(A("sv_min").min()), tilt_max_deg=float(A("tilt_deg").max()), rotor_force_min_N=float(A("f_rotor").min()), rotor_force_max_N=float(A("f_rotor").max()), omega_min=float(A("om").min()), omega_max=float(A("om").max()),
        tau_max_Nm=np.abs(A("tau")).max(0).tolist(), tau_util_max=A("tau_util").max(0).tolist(), hand_axis_deg_max=float(A("axis_deg").max()), gap_wall_min_mm=float(A("gap_wall").min() * 1e3), gap_airframe_min_mm=float(A("gap_air").min() * 1e3), gap_arm_min_mm=float(A("gap_arm").min() * 1e3))
    out["pass"] = dict(ik=out["ik_err_max_m"] < 1e-4, joint_limits=bool((np.array(out["q_min_deg"]) > np.degrees(QLO) + 1).all() and (np.array(out["q_max_deg"]) < np.degrees(QHI) - 1).all()),
                       torque=max(out["tau_util_max"]) < 0.7, rotors=bool(out["omega_min"] > OM_MIN * 1.05 and out["omega_max"] < OM_MAX * 0.9), clearance=out["gap_wall_min_mm"] > 2.0 and out["gap_airframe_min_mm"] > 0.0 and out["gap_arm_min_mm"] > 0.0, solve=out["solve_res_max"] < 1e-6)
    return out

if __name__ == "__main__":
    os.makedirs(os.path.join(HERE, "..", "results", "am3_task"), exist_ok=True)
    for name in (sys.argv[1:] or list(TASK["paths"])):
        o = analyse(name); json.dump(o, open(os.path.join(HERE, "..", "results", "am3_task", f"feasibility_{name}.json"), "w"), indent=1); print(name, json.dumps({k: (np.round(v, 3).tolist() if not isinstance(v, (str, int, bool, dict)) else v) for k, v in o.items()}))
