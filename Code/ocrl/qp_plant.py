"""Payload study plant: quadrotor (v3 airframe, no arm) + 0.2 kg load on a 0.6 m rigid string (model/qp_quad_payload.xml).
Rotors have the same first-order motor lag and thrust law as the v3 plant (am3_plant).  Input: rotor speed commands Omega_cmd (4, rad/s).
Also the shared low-level controller: thrust-vector command -> SO(3) attitude PD -> rotor allocation."""
import os, json, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__)); XML = os.path.join(HERE, "..", "model", "qp_quad_payload.xml")
P = json.load(open(os.path.join(HERE, "..", "config", "am3_design_v3.json")))["propulsion"]
KF = P["f_max_N"] / P["omega_max_rad_s"] ** 2; OM_MAX = P["omega_max_rad_s"]; OM_MIN = np.sqrt(P["f_min_N"] / KF)
MQ, ML, L, G = 1.05, 0.2, 0.6, 9.81
J = np.diag([0.0017969, 0.0017969, 0.003454])
XY = np.array(P["rotor_xy_m"]); S = np.array(P["spin_reaction_sign"]); KM = P["km_over_kf_m"]
ALLOC = np.vstack([np.ones(4), XY[:, 1], -XY[:, 0], S * KM])          # [T, tx, ty, tz] = ALLOC @ rotor thrusts


def quat2R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class Plant:
    def __init__(self, p0=(0.0, 0.0, 1.5)):
        self.m = mujoco.MjModel.from_xml_path(XML); self.d = mujoco.MjData(self.m); self.dt = self.m.opt.timestep; self.bl = self.m.body("load").id; self.Jc = np.zeros((3, self.m.nv))
        self.d.qpos[:3] = p0; mujoco.mj_forward(self.m, self.d); self.Omega = np.full(4, np.sqrt((MQ + ML) * G / 4 / KF))

    def step(self, om_cmd):
        oc = np.clip(om_cmd, OM_MIN, OM_MAX); self.d.ctrl[:] = KF * self.Omega ** 2
        tc = np.where(oc > self.Omega, P["tau_up_s"], P["tau_down_s"]); self.Omega = oc + (self.Omega - oc) * np.exp(-self.dt / tc); mujoco.mj_step(self.m, self.d)

    def state(self):
        """p, R, v (world), w (body), load position and velocity (world)."""
        d = self.d; mujoco.mj_jacBodyCom(self.m, d, self.Jc, None, self.bl)
        return dict(p=d.qpos[:3].copy(), R=quat2R(d.qpos[3:7]), v=d.qvel[:3].copy(), w=d.qvel[3:6].copy(), r=d.xipos[self.bl].copy(), rv=self.Jc @ d.qvel)

    @staticmethod
    def swing_deg(s):
        c = (s["r"] - s["p"]) / L; return float(np.degrees(np.arccos(np.clip(-c[2], -1, 1))))


class Attitude:
    """Thrust vector f (world, N) and yaw 0 -> rotor speed commands.  SO(3) PD (Lee et al.), gains set for about 15 rad/s with the 40 ms motor lag."""
    def __init__(self, wn=15.0, zeta=0.9):
        self.kR, self.kw = J * wn ** 2, J * 2 * zeta * wn

    def __call__(self, s, f):
        R = s["R"]; b3 = f / max(np.linalg.norm(f), 1e-6); b2 = np.cross(b3, [1.0, 0.0, 0.0]); b2 /= np.linalg.norm(b2); Rd = np.column_stack([np.cross(b2, b3), b2, b3])
        E = 0.5 * (Rd.T @ R - R.T @ Rd); eR = np.array([E[2, 1], E[0, 2], E[1, 0]]); tau = -self.kR @ eR - self.kw @ s["w"] + np.cross(s["w"], J @ s["w"])
        fi = np.clip(np.linalg.solve(ALLOC, np.r_[f @ R[:, 2], tau]), P["f_min_N"], P["f_max_N"]); return np.sqrt(fi / KF)
