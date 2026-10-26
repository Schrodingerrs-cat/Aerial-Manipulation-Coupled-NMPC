"""Static analysis of the v3 assembly with MuJoCo kinematics: pose setting, pad position/Jacobian, collision clearances, position-only IK and the quasi-static rotor trim for a pad contact force.
Used for the design scan and path feasibility; it is independent of the CasADi predictor (it uses MuJoCo's own COM and kinematics)."""
import os, json, numpy as np, mujoco
from scipy.optimize import least_squares
HERE = os.path.dirname(os.path.abspath(__file__)); XML = os.path.join(HERE, "..", "model", "aerial_manipulator_v3.xml"); CFG = json.load(open(os.path.join(HERE, "..", "config", "am3_design_v3.json")))
G0 = 9.81
AIRFRAME = ["drone_core", "a00", "a10", "a20", "a30", "prop0", "prop1", "prop2", "prop3", "yaw_motor", "bracket"]
ARM = ["l1_box", "m2", "l2", "m3", "l3", "m4", "hand_body", "finger_l", "finger_r", "pad"]
ARM_PAIRS_SKIP = {("l1_box", "m2"), ("m2", "l2"), ("l2", "m3"), ("m3", "l3"), ("l3", "m4"), ("m4", "hand_body"), ("hand_body", "finger_l"), ("hand_body", "finger_r"), ("hand_body", "pad"), ("finger_l", "pad"), ("finger_r", "pad"), ("finger_l", "finger_r"),
                  ("l1_box", "l2"), ("l2", "l3"), ("m2", "m3"), ("m3", "m4"), ("l3", "hand_body"), ("m4", "finger_l"), ("m4", "finger_r"), ("m4", "pad")}      # permanently touching neighbours

def rpy_quat(roll, pitch, yaw=0.0):
    cr, sr, cp, sp, cy, sy = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2), np.cos(yaw / 2), np.sin(yaw / 2)       # R = Rz Ry Rx
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])

class Rig:
    def __init__(self, xml=XML):
        self.m = mujoco.MjModel.from_xml_path(xml); self.d = mujoco.MjData(self.m); m = self.m; g = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        self.gid = {n: g(n) for n in AIRFRAME + ARM + ["wall", "floor"] if g(n) >= 0}; self.sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "ee")
        self.jadr = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"q{i}")] for i in (1, 2, 3)]; self.mass = m.body_mass.sum()
        self.rotor = np.array([[x, y, CFG["propulsion"]["rotor_z_m"]] for x, y in CFG["propulsion"]["rotor_xy_m"]]); self.wall_x = CFG["wall_x_m"]; self.r_pad = CFG["pad"]["radius_m"]
    def pose(self, pB, roll, pitch, q):
        d = self.d; d.qpos[0:3] = pB; d.qpos[3:7] = rpy_quat(roll, pitch); d.qpos[7:10] = q; mujoco.mj_forward(self.m, d)
    def pad(self): return self.d.site_xpos[self.sid].copy()
    def com(self): return self.d.subtree_com[1].copy()
    def pad_jac(self):
        Jp = np.zeros((3, self.m.nv)); mujoco.mj_jacSite(self.m, self.d, Jp, None, self.sid); return Jp[:, self.jadr]           # world position velocity of the pad centre per joint rate
    def hand_axis(self): return -self.d.site_xmat[self.sid].reshape(3, 3)[:, 2]                                                 # fingers point along hand -z
    def clearances(self, margin=0.02):
        """Smallest signed gap [m] (capped at `margin`) in four categories from MuJoCo contact detection with every geom margin set to `margin`: non-pad geoms vs wall, arm vs airframe, arm vs arm (engine skips
        parent-child pairs, which join by design) and pad vs wall.  mj_geomDistance is not used: it returns 0 for rotated box pairs."""
        m, d = self.m, self.d; m.geom_margin[:] = margin; mujoco.mj_forward(m, d); out = dict(wall_nonpad=margin, arm_airframe=margin, arm_arm=margin, floor_nonpad=margin, pad_wall=margin); names = {g: n for n, g in self.gid.items()}
        for c in d.contact[:d.ncon]:
            a, b = sorted((names.get(c.geom1, "?"), names.get(c.geom2, "?")))
            if "wall" in (a, b): k = "pad_wall" if "pad" in (a, b) else "wall_nonpad"
            elif "floor" in (a, b): k = "floor_nonpad"
            elif (a in ARM) and (b in ARM): k = "arm_arm"
            elif (a in ARM) != (b in ARM): k = "arm_airframe"
            else: continue
            out[k] = min(out[k], float(c.dist))
        m.geom_margin[:] = 0.0; return out

    def ik(self, target, pB, roll=0.0, pitch=0.0, q0=None, lo=None, hi=None, q_ref=None, w_post=1e-3):
        """Position-only IK of the pad centre (world target) for a fixed base pose, with weak posture regularisation toward q_ref; returns (q, position error)."""
        lo = np.radians([-90, -120, -150]) if lo is None else lo; hi = -lo if hi is None else hi; q_ref = np.array([0.0, -1.0, -0.5]) if q_ref is None else q_ref
        def f(q): self.pose(pB, roll, pitch, q); return np.r_[self.pad() - target, w_post * (q - q_ref)]
        s = least_squares(f, np.clip(q0 if q0 is not None else q_ref, lo + 1e-6, hi - 1e-6), bounds=(lo, hi), xtol=1e-13, ftol=1e-13, gtol=1e-13); return s.x, float(np.linalg.norm(s.fun[:3]))

    def trim(self, target, Fc, pB, q0, base_xy_fixed=True):
        """Quasi-static balance: unknowns (q, roll, pitch, f1..f4); equations: pad at target, force and torque balance about the base origin with the wall force Fc (on the robot, world) at the contact point."""
        def f(z):
            q, roll, pitch, fr = z[0:3], z[3], z[4], z[5:9]; self.pose(pB, roll, pitch, q); R = self.d.xmat[1].reshape(3, 3); pc = self.pad() + np.array([self.r_pad, 0, 0]); pG = self.com(); P = CFG["propulsion"]
            Frot = R @ np.array([0, 0, fr.sum()]); Mrot = sum(np.cross(R @ r, R @ np.array([0, 0, fi])) for r, fi in zip(self.rotor, fr)) + R @ np.array([0, 0, sum(s * P["km_over_kf_m"] * fi for s, fi in zip(P["spin_reaction_sign"], fr))])
            Fg = np.array([0, 0, -self.mass * G0]); pB_ = self.d.qpos[0:3]
            return np.r_[(self.pad() - target) * 1e3, Frot + Fg + Fc, Mrot + np.cross(pG - pB_, Fg) + np.cross(pc - pB_, Fc)]
        z0 = np.r_[q0, 0.0, 0.0, [self.mass * G0 / 4] * 4]; s = least_squares(f, z0, xtol=1e-13, ftol=1e-13, gtol=1e-13)
        self.pose(pB, s.x[3], s.x[4], s.x[0:3]); return dict(q=s.x[0:3], roll=s.x[3], pitch=s.x[4], f=s.x[5:9], res=float(np.linalg.norm(s.fun)))
