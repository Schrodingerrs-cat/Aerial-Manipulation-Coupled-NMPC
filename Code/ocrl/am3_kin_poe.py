"""Analytical kinematics of the v3 arm (capstone, 2026-11-17): product of exponentials with the space frame {s} = drone body frame.
Joints: q1 yaw about body z, q2 and q3 pitch about the rotated y; the wrist is fixed at beta about y; the pad centre is c_hand in the hand frame.
FK:  T_sh(q) = e^[S1]q1 e^[S2]q2 e^[S3]q3 M.   IK (pad position): closed form, yaw from atan2, then a planar 2-link solve.
Payload: with q and the link lengths known, a point mass m at the hand origin loads the joints by tau = Jr(q)^T R^T (-m g e_z), so m is one
least-squares scalar from the observer's joint-torque estimate (no Bayesian estimator needed).  Geometry from config/am3_design_v3.json, as AMPred3."""
import json, os, numpy as np
from scipy.linalg import expm

C = json.load(open(os.path.join(os.path.dirname(__file__), "..", "config", "am3_design_v3.json")))
G0 = 9.81


def skew(a): return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
def Ry(a): return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


def screw(w, p):
    """Revolute screw axis (w, -w x p) for a unit axis w through point p."""
    return np.r_[w, -np.cross(w, p)]


def exp6(S, th):
    """e^[S] th as a 4x4 transform."""
    X = np.zeros((4, 4)); X[:3, :3] = skew(S[:3]); X[:3, 3] = S[3:]; return expm(X * th)


def adjoint(T):
    R, p = T[:3, :3], T[:3, 3]; A = np.zeros((6, 6)); A[:3, :3] = R; A[3:, 3:] = R; A[3:, :3] = skew(p) @ R; return A


class ArmPoE:
    def __init__(self):
        ch = C["chain_m"]; self.l0, self.l1, self.l2 = ch["mount_to_shoulder"], ch["upper_arm"], ch["forearm"]; self.beta = np.radians(C["wrist_beta_deg"])
        self.o1 = np.array([C["mount_x_m"], 0.0, -ch["root_below_base"]]); self.o2 = self.o1 + [0, 0, -self.l0]; self.o3 = self.o2 + [0, 0, -self.l1]; o4 = self.o3 + [0, 0, -self.l2]
        ez, ey = np.array([0.0, 0, 1]), np.array([0.0, 1, 0]); self.S = [screw(ez, self.o1), screw(ey, self.o2), screw(ey, self.o3)]
        self.c = np.array(C["pad"]["centre_hand_m"]); self.M = np.eye(4); self.M[:3, :3] = Ry(self.beta); self.M[:3, 3] = o4          # hand frame at q = 0
        self.w = np.array([0, 0, -self.l2]) + Ry(self.beta) @ self.c                                                                      # elbow -> pad in the forearm frame

    def fk(self, q):
        """Hand frame T_sh(q) (4x4) and pad centre (body frame)."""
        T = exp6(self.S[0], q[0]) @ exp6(self.S[1], q[1]) @ exp6(self.S[2], q[2]) @ self.M; return T, T[:3, :3] @ self.c + T[:3, 3]

    def jac_space(self, q):
        """Space Jacobian (6x3, rows [omega; v]) of the hand."""
        T = np.eye(4); J = np.zeros((6, 3))
        for i in range(3): J[:, i] = adjoint(T) @ self.S[i]; T = T @ exp6(self.S[i], q[i])
        return J

    def jac_point(self, q, p):
        """Linear-velocity Jacobian (3x3, body frame) of a point p fixed in the hand: v = v_s + omega_s x p."""
        Js = self.jac_space(q); return Js[3:] - skew(p) @ Js[:3]

    def ik_pad(self, p, q_ref=np.radians([0.0, -37.0, -78.0])):
        """Closed-form joint angles putting the pad centre at body-frame point p.  With u = z + i x, Ry(a) multiplies u by e^{ia}, so in the
        yawed plane  u_pad = e^{i q2} (-l1 + e^{i q3} w_c).  Up to four solutions (yaw or yaw + pi, two elbows); returns the one closest to
        q_ref, and whether p is reachable."""
        d = p - self.o2; wc = self.w[2] + 1j * self.w[0]; L = abs(wc); sols = []
        for q1 in (np.arctan2(d[1], d[0]), np.arctan2(d[1], d[0]) + np.pi):            # the chain below the shoulder stays in the yawed plane
            up = d[2] + 1j * (np.cos(q1) * d[0] + np.sin(q1) * d[1]); k = (self.l1 ** 2 + L ** 2 - abs(up) ** 2) / (2 * self.l1 * L)       # cos(q3 + arg wc)
            for s in (-1, 1):
                q3 = s * np.arccos(np.clip(k, -1, 1)) - np.angle(wc); q2 = np.angle(up) - np.angle(-self.l1 + np.exp(1j * q3) * wc)
                sols.append(np.arctan2(np.sin([q1, q2, q3]), np.cos([q1, q2, q3])))
        dist = [np.abs(np.arctan2(np.sin(s - q_ref), np.cos(s - q_ref))).sum() for s in sols]
        return sols[int(np.argmin(dist))], abs(k) <= 1.0

    def payload_mass(self, q, R, tau_d):
        """m from the observer's joint-torque estimate tau_d (3): tau = a m,  a = Jr^T R^T (-g e_z), Jr of the hand origin; least squares."""
        T, _ = self.fk(q); Jr = self.jac_point(q, T[:3, 3]); a = Jr.T @ R.T @ np.array([0, 0, -G0]); return float(a @ tau_d / (a @ a))
