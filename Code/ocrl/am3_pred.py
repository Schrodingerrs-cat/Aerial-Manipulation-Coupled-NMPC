"""CasADi prediction model of the aerial manipulator, design v3 (supplied centred mount, fixed wrist beta, spherical contact pad on the hand).  Built only from config/am3_design_v3.json; it never queries MuJoCo.
x (23) = [p(3) base origin world, Q(4) wxyz body->world, v(3) world, omega(3) body, q(3), qd(3), Omega(4)];  u (7) = [Omega_cmd(4), tau(3)];  correction d (2) = [F_hat along the wall normal, pitch moment about body y].
M(R,q) nu_dot + b_vel + G = tau_rot + S^T(tau - D qd) + J_C^T f_C + J_C^T n F_hat + E_pitch tau_hat,   nu = (v, omega, qd)     (Plan s5, s6, s8)
M = sum_i m_i Jv_i^T Jv_i + Jw_i^T I_i^W Jw_i;  b_vel from the derivative of the Jacobians along the kinematic flow with nu held constant.
`coupling` selects the model used for ablations: "full" (default) or "decoupled" (Plan s7.1): M reduced to its base block and joint block, b_vel of each block computed with the other block's velocities set to zero, gravity kept.  The plant is never changed."""
import os, sys, json, numpy as np, casadi as ca
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
from am3_build_model import airframe_inertial
C = json.load(open(os.path.join(HERE, "..", "config", "am3_design_v3.json"))); G0 = 9.81

def quat2R(Q):
    w, x, y, z = Q[0], Q[1], Q[2], Q[3]
    return ca.vertcat(ca.horzcat(1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), ca.horzcat(2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                      ca.horzcat(2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))
def quat_rate(Q, w): return 0.5 * ca.vertcat(-Q[1] * w[0] - Q[2] * w[1] - Q[3] * w[2], Q[0] * w[0] + Q[2] * w[2] - Q[3] * w[1], Q[0] * w[1] - Q[1] * w[2] + Q[3] * w[0], Q[0] * w[2] + Q[1] * w[1] - Q[2] * w[0])
def hat(a): return ca.vertcat(ca.horzcat(0, -a[2], a[1]), ca.horzcat(a[2], 0, -a[0]), ca.horzcat(-a[1], a[0], 0))
def Rz(a): return ca.vertcat(ca.horzcat(ca.cos(a), -ca.sin(a), 0), ca.horzcat(ca.sin(a), ca.cos(a), 0), ca.horzcat(0, 0, 1))
def Ry(a): return ca.vertcat(ca.horzcat(ca.cos(a), 0, ca.sin(a)), ca.horzcat(0, 1, 0), ca.horzcat(-ca.sin(a), 0, ca.cos(a)))
def softplus(z): return ca.fmax(z, 0) + ca.log1p(ca.exp(-ca.fabs(z)))
def sigmoid(z): return ca.if_else(z >= 0, 1 / (1 + ca.exp(-z)), ca.exp(z) / (1 + ca.exp(z)))

class AMPred3:
    def __init__(self, k=C["contact"]["k_N_m"], c=C["contact"]["c_pred_Ns_m"], mu=C["contact"]["mu"], eps_g=1e-5, eps_F=1e-3, v_eps=2e-3, coupling="full"):
        self.k, self.c, self.mu, self.eps_g, self.eps_F, self.v_eps, self.coupling = k, c, mu, eps_g, eps_F, v_eps, coupling
        S, ch, P, Sv = C["supplied"], C["chain_m"], C["propulsion"], C["servo"]; self.nx, self.nu, self.nd = 23, 7, 2; self.P = P
        self.kf = P["f_max_N"] / P["omega_max_rad_s"] ** 2; self.om_max = P["omega_max_rad_s"]; self.om_min = np.sqrt(P["f_min_N"] / self.kf); self.tau_c = Sv["continuous_torque_Nm"]; self.D = C["joint_damping_Nms_per_rad"]
        self.r_pad = C["pad"]["radius_m"]; self.x_wall = C["wall_x_m"]; beta = np.radians(C["wrist_beta_deg"]); mx = C["mount_x_m"]; mA, cA, IA = airframe_inertial(C, mx)
        self.m_tot = mA + sum(S[n]["mass_kg"] for n in ("link1", "link2", "link3", "hand"))
        x = ca.SX.sym("x", 23); u = ca.SX.sym("u", 7); d = ca.SX.sym("d", 2); self.x_s, self.u_s, self.d_s = x, u, d
        p, Q, v, w, q, qd, Om = x[0:3], x[3:7], x[7:10], x[10:13], x[13:16], x[16:19], x[19:23]; R = quat2R(Q); nu = ca.vertcat(v, w, qd); ez = ca.DM([0, 0, 1]); ey = ca.DM([0, 1, 0])
        o1 = ca.DM([mx, 0, -ch["root_below_base"]]); R1 = Rz(q[0]); o2 = o1 + R1 @ ca.DM([0, 0, -ch["mount_to_shoulder"]]); R2 = R1 @ Ry(q[1]); o3 = o2 + R2 @ ca.DM([0, 0, -ch["upper_arm"]])
        R3 = R2 @ Ry(q[2]); o4 = o3 + R3 @ ca.DM([0, 0, -ch["forearm"]]); Rh = R3 @ Ry(beta); axes = [ez, R1 @ ey, R2 @ ey]; origins = [o1, o2, o3]; r_pad = o4 + Rh @ ca.DM(C["pad"]["centre_hand_m"])
        bodies = [(mA, ca.DM(cA), ca.DM.eye(3), IA, 0), (S["link1"]["mass_kg"], o1 + R1 @ ca.DM(S["link1"]["com_m"]), R1, np.diag(S["link1"]["I_diag"]), 1), (S["link2"]["mass_kg"], o2 + R2 @ ca.DM(S["link2"]["com_m"]), R2, np.diag(S["link2"]["I_diag"]), 2),
                  (S["link3"]["mass_kg"], o3 + R3 @ ca.DM(S["link3"]["com_m"]), R3, np.diag(S["link3"]["I_diag"]), 3), (S["hand"]["mass_kg"], o4 + Rh @ ca.DM(S["hand"]["com_m"]), Rh, np.diag(S["hand"]["I_diag"]), 3)]
        def jac(cpos, nj):                        # base-frame joint columns for the velocity of a point fixed in a body whose ancestors are joints 0..nj-1
            Jr = ca.horzcat(*[ca.cross(axes[j], cpos - origins[j]) if j < nj else ca.DM.zeros(3) for j in range(3)]); Jo = ca.horzcat(*[axes[j] if j < nj else ca.DM.zeros(3) for j in range(3)]); return Jr, Jo
        kin, kind = ca.vertcat(Q, q), ca.vertcat(quat_rate(Q, w), qd); M = ca.SX.zeros(9, 9); bvel = ca.SX.zeros(9); Gv = ca.SX.zeros(9)
        for mi, cpos, Rb, Ib, nj in bodies:
            Jr, Jo = jac(cpos, nj); Jv = ca.horzcat(ca.DM.eye(3), -R @ hat(cpos), R @ Jr); Jw = ca.horzcat(ca.DM.zeros(3, 3), R, R @ Jo); RW = R @ Rb; IW = RW @ ca.DM(Ib) @ RW.T
            a0 = ca.jacobian(Jv @ nu, kin) @ kind; b0 = ca.jacobian(Jw @ nu, kin) @ kind; wi = Jw @ nu
            M += mi * Jv.T @ Jv + Jw.T @ IW @ Jw; bvel += Jv.T @ (mi * a0) + Jw.T @ (IW @ b0 + ca.cross(wi, IW @ wi)); Gv += Jv.T @ (mi * G0 * ez)
        if coupling == "decoupled":      # ablation: base block and joint block only; each sees only its own velocities in the bias; gravity unchanged
            bf = ca.Function("b", [x], [bvel]); xq0 = ca.vertcat(x[0:7], ca.SX.zeros(6), x[13:16], ca.SX.zeros(3), x[19:23]); xb0 = ca.vertcat(x[0:7], x[7:13], x[13:16], ca.SX.zeros(3), x[19:23]); xj0 = ca.vertcat(x[0:7], ca.SX.zeros(6), x[13:16], x[16:19], x[19:23])
            M = ca.vertcat(ca.horzcat(M[0:6, 0:6], ca.SX.zeros(6, 3)), ca.horzcat(ca.SX.zeros(3, 6), M[6:9, 6:9])); bvel = ca.vertcat(bf(xb0)[0:6], bf(xj0)[6:9])
        rot = np.array([[xy[0], xy[1], P["rotor_z_m"]] for xy in P["rotor_xy_m"]]); s = P["spin_reaction_sign"]; kap = P["km_over_kf_m"]; f = self.kf * Om ** 2; Tb = ca.SX.zeros(3)
        for i in range(4): Tb += ca.cross(ca.DM(rot[i]), ca.DM([0, 0, 1])) * f[i] + ca.DM([0, 0, s[i] * kap]) * f[i]
        tau_rot = ca.vertcat(R @ ca.vertcat(0, 0, ca.sum1(f)), Tb, ca.DM.zeros(3))
        n = ca.DM([-1.0, 0, 0]); Jr_t, Jo_t = jac(r_pad, 3); pT = p + R @ r_pad; JT = ca.horzcat(ca.DM.eye(3), -R @ hat(r_pad), R @ Jr_t); JwT = ca.horzcat(ca.DM.zeros(3, 3), R, R @ Jo_t); JC = JT + self.r_pad * hat(n) @ JwT
        g = self.x_wall - pT[0] - self.r_pad; vc = -(n.T @ JT @ nu); vC = JC @ nu; z = -g / self.eps_g
        delta = self.eps_g * softplus(z); chi = sigmoid(z); a = self.k * delta + self.c * vc
        Fn = chi * ca.if_else(a <= 0, 0.0, ca.if_else(a < self.eps_F, a * a / (2 * self.eps_F), a - self.eps_F / 2))
        vt = vC - n * (n.T @ vC); ft = -self.mu * Fn * vt / ca.sqrt(vt.T @ vt + self.v_eps ** 2); fC = n * Fn + ft
        tau_c_gen = JC.T @ fC + JC.T @ n * d[0] + ca.vertcat(0, 0, 0, 0, d[1], 0, 0, 0, 0); tau_j = ca.vertcat(ca.DM.zeros(6), u[4:7] - self.D * qd)
        rhs = tau_rot + tau_j + tau_c_gen - bvel - Gv; nud = ca.solve(M, rhs)
        tm = P["tau_down_s"] + (P["tau_up_s"] - P["tau_down_s"]) * sigmoid((u[0:4] - Om) / 10.0); Omd = (u[0:4] - Om) / tm
        xdot = ca.vertcat(v, quat_rate(Q, w), nud[0:3], nud[3:6], qd, nud[6:9], Omd); fCs = ca.SX.sym("fC", 3)
        self.nud_force_f = ca.Function("nud_force", [x, u, fCs], [ca.solve(M, tau_rot + tau_j + JC.T @ fCs - bvel - Gv)])        # explicit contact force instead of the contact law (statics, checks)
        self.rhs_f = ca.Function("rhs", [x, u, d], [rhs]); self.Om_dot_f = ca.Function("Omd", [x, u], [Omd]); self.xdot_f = ca.Function("xdot", [x, u, d], [xdot]); self.M_f = ca.Function("M", [x], [M])
        self.bias_f = ca.Function("bias", [x], [bvel]); self.grav_f = ca.Function("grav", [x], [Gv]); self.tau_rot_f = ca.Function("tau_rot", [x], [tau_rot])
        self.pad_f = ca.Function("pad", [x], [pT, R @ (Rh @ ca.DM([0, 0, -1]))]); self.contact_f = ca.Function("contact", [x, u, d], [g, vc, Fn, ft, vC]); self.JC_f = ca.Function("JC", [x], [JC, JT, JwT])
        self.com_f = ca.Function("com", [x], [sum(b[0] * (p + R @ b[1]) for b in bodies) / self.m_tot]); self.energy_f = ca.Function("energy", [x], [0.5 * nu.T @ M @ nu + sum(b[0] * G0 * (p + R @ b[1])[2] for b in bodies)])
        self.pts_f = ca.Function("pts", [x], [o3, o4, r_pad]); self.fk_f = ca.Function("fk", [q], [r_pad]); self.Jpad_f = ca.Function("Jpad", [q], [Jr_t])                                      # pad centre in the base frame and its joint Jacobian

    def xdot_mx(self):
        """Same vector field with MX calls and one linear solve: far cheaper derivatives than the expanded SX solve."""
        x = ca.MX.sym("x", 23); u = ca.MX.sym("u", 7); d = ca.MX.sym("d", 2); nud = ca.solve(self.M_f(x), self.rhs_f(x, u, d), "qr")
        return ca.Function("xdot_mx", [x, u, d], [ca.vertcat(x[7:10], quat_rate(x[3:7], x[10:13]), nud[0:3], nud[3:6], x[16:19], nud[6:9], self.Om_dot_f(x, u))])

    def step_fun(self, h, nsub=8):
        """RK4 map over h with held u, d; quaternion renormalised after each substep."""
        x = ca.SX.sym("x", 23); u = ca.SX.sym("u", 7); d = ca.SX.sym("d", 2); xs = x; hh = h / nsub
        for _ in range(nsub):
            k1 = self.xdot_f(xs, u, d); k2 = self.xdot_f(xs + hh / 2 * k1, u, d); k3 = self.xdot_f(xs + hh / 2 * k2, u, d); k4 = self.xdot_f(xs + hh * k3, u, d)
            xs = xs + hh / 6 * (k1 + 2 * k2 + 2 * k3 + k4); xs = ca.vertcat(xs[0:3], xs[3:7] / ca.norm_2(xs[3:7]), xs[7:23])
        return ca.Function("step", [x, u, d], [xs])
