"""Production plant of the v3 aerial manipulator: MuJoCo full floating-base dynamics, four rotors with first-order motor lag (state Omega, thrust kf Omega^2 through the rotor actuators), three joint
torque sources with a torque-speed envelope, native pad-wall contact.  Convention (same as am3_pred): x (23) = [p(3), Q(4) wxyz, v(3) world, omega(3) body, q(3), qd(3), Omega(4)];
u (7) = [Omega_cmd(4) rad/s, tau_cmd(3) N m].  The same plant serves every controller; coupling is never removed here."""
import os, json, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__)); CFG = json.load(open(os.path.join(HERE, "..", "config", "am3_design_v3.json"))); XML = os.path.join(HERE, "..", "model", "aerial_manipulator_v3.xml")
P = CFG["propulsion"]; KF = P["f_max_N"] / P["omega_max_rad_s"] ** 2; OM_MAX = P["omega_max_rad_s"]; OM_MIN = np.sqrt(P["f_min_N"] / KF)
SV = CFG["servo"]; TAU_C, TAU_S, W_NL = SV["continuous_torque_Nm"], SV["stall_torque_Nm"], SV["no_load_speed_rad_s"]
WALL_X, R_PAD, D_IMP = CFG["wall_x_m"], CFG["pad"]["radius_m"], 0.05
K_CAL = 0.95           # solref stiffness scale: static regression of results/am3_predictor/contact_calibration.json gave 632 N/m for a requested 600 at K_CAL = 1

def servo_limit(tau_cmd, qd):
    """Braking up to tau_c, driving min(tau_c, tau_stall (1 - |qd| / w_nl))."""
    lim = np.where(np.sign(tau_cmd) * qd <= 0, TAU_C, np.maximum(0.0, np.minimum(TAU_C, TAU_S * (1 - np.abs(qd) / W_NL)))); return np.clip(tau_cmd, -lim, lim)

def solref_for(k, c, A, Ap): den = D_IMP * A + (1 - D_IMP) * Ap; return [-k * den, -c * den]

class AMPlant3:
    def __init__(self, k=CFG["contact"]["k_N_m"], c=CFG["contact"]["c_Ns_m"], mu=CFG["contact"]["mu"], dt=0.00025, xml=XML):
        self.m = mujoco.MjModel.from_xml_path(xml); self.d = mujoco.MjData(self.m); m = self.m; m.opt.timestep = dt; self.dt = dt; nm = lambda t, n: mujoco.mj_name2id(m, t, n); O = mujoco.mjtObj
        self.gpad, self.gwall = nm(O.mjOBJ_GEOM, "pad"), nm(O.mjOBJ_GEOM, "wall"); self.sid = nm(O.mjOBJ_SITE, "ee"); self.jid = [nm(O.mjOBJ_JOINT, f"q{i}") for i in (1, 2, 3)]
        self.qadr = m.jnt_qposadr[self.jid]; self.vadr = m.jnt_dofadr[self.jid]; self.mass = m.body_mass.sum(); self.Omega = np.zeros(4); self.kf_scale = self.tau_scale = 1.0; self.dist = []
        self.set_contact(k, c, mu); self.reset_log()

    def set_contact(self, k, c, mu):
        m = self.m; self.k, self.c, self.mu = k, c, mu; A, Ap = self.contact_terms(); sr = solref_for(k * K_CAL, c, A, Ap)
        for g in (self.gpad, self.gwall): m.geom_solref[g] = sr; m.geom_solimp[g] = [D_IMP, D_IMP, 0.001, 0.5, 2]; m.geom_friction[g, 0] = mu

    def contact_terms(self):
        """A = J M^-1 J^T and A' = efc_diagA of the pad-wall contact at the nominal contact posture (1 mm penetration); sets the soft-contact reference acceleration scale."""
        m = self.m; d2 = mujoco.MjData(m); d2.qpos[:] = self.nominal_qpos(); mujoco.mj_forward(m, d2); d2.qpos[0] += WALL_X - R_PAD - d2.site_xpos[self.sid][0] + 1e-3; mujoco.mj_forward(m, d2)
        row = next(cc.efc_address for cc in d2.contact[:d2.ncon] if {cc.geom1, cc.geom2} == {self.gpad, self.gwall}); J = d2.efc_J.reshape(d2.nefc, m.nv)[row:row + 1]; Mf = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d2, Mf)
        return float((J @ np.linalg.inv(Mf) @ J.T)[0, 0]), float(d2.efc_diagA[row])

    def nominal_qpos(self):
        return np.r_[WALL_X - R_PAD - CFG["task"]["standoff_pad_from_base_m"], 0.0, 1.2, 1, 0, 0, 0, 0.0, np.radians(-37.0), np.radians(-78.0)]

    def apply_mismatch(self, mm):
        """Named model mismatch unknown to controllers: payload {kg, pos_hand_m}, kf_scale, motor_tau_scale, joint_damping_scale, disturbance [{kind step|sine, axis 0-5 world force/torque on the base, amp, t0, dur|freq}]."""
        m = self.m; self.kf_scale = mm.get("kf_scale", 1.0); self.tau_scale = mm.get("motor_tau_scale", 1.0); self.dist = mm.get("disturbance", [])
        if "joint_damping_scale" in mm: m.dof_damping[self.vadr] *= mm["joint_damping_scale"]
        if "payload" in mm:
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "payload"); kg = mm["payload"]["kg"]; m.body_pos[b] = mm["payload"]["pos_hand_m"]; m.body_mass[b] = kg; m.body_inertia[b] = 0.4 * kg * 0.02 ** 2
            mujoco.mj_setConst(m, self.d); self.mass = m.body_mass.sum(); self.set_contact(self.k, self.c, self.mu)

    def _disturb(self):
        t = self.d.time; self.d.xfrc_applied[1, :] = 0.0
        for e in self.dist:
            if e["kind"] == "step" and e["t0"] <= t < e["t0"] + e["dur"]: self.d.xfrc_applied[1, e["axis"]] += e["amp"]
            if e["kind"] == "sine" and t >= e["t0"]: self.d.xfrc_applied[1, e["axis"]] += e["amp"] * np.sin(2 * np.pi * e["freq"] * (t - e["t0"]))

    def state(self):
        d = self.d; return np.concatenate([d.qpos[:7], d.qvel[:3], d.qvel[3:6], d.qpos[self.qadr], d.qvel[self.vadr], self.Omega])

    def set_state(self, x):
        d = self.d; mujoco.mj_resetData(self.m, d); d.qpos[:7], d.qvel[:3], d.qvel[3:6] = x[:7], x[7:10], x[10:13]; d.qpos[self.qadr] = x[13:16]; d.qvel[self.vadr] = x[16:19]; self.Omega = np.array(x[19:23], float)
        d.xfrc_applied[:] = 0; d.ctrl[:] = 0; mujoco.mj_forward(self.m, d)

    def step(self, u):
        """One physics step with held command u; motor lag integrated exactly over dt, thrust from the lagged speed at the step start; requested and applied commands are logged."""
        d = self.d; oc = np.clip(u[:4], OM_MIN, OM_MAX); tau = servo_limit(np.asarray(u[4:7], float), d.qvel[self.vadr]); self._disturb(); d.ctrl[:4] = KF * self.kf_scale * self.Omega ** 2; d.ctrl[4:7] = tau
        tc = np.where(oc > self.Omega, P["tau_up_s"], P["tau_down_s"]) * self.tau_scale; self.Omega = oc + (self.Omega - oc) * np.exp(-self.dt / tc)
        mujoco.mj_step(self.m, d); self.tau_applied = tau; self.tau_req = np.asarray(u[4:7], float); self._log()

    # ---- measurements and ground truth
    def pad_pos(self): return self.d.site_xpos[self.sid].copy()
    def gap(self): return WALL_X - self.pad_pos()[0] - R_PAD

    def pad_force(self):
        """Contact force on the pad from the wall in world axes: normal force >= 0 (along -x) and tangential (y, z)."""
        F, ft, f6 = 0.0, np.zeros(2), np.zeros(6)
        for i in range(self.d.ncon):
            cc = self.d.contact[i]
            if {cc.geom1, cc.geom2} == {self.gpad, self.gwall}:
                mujoco.mj_contactForce(self.m, self.d, i, f6); fw = cc.frame.reshape(3, 3).T @ f6[:3]               # contact frame -> world, force on geom2 from geom1
                if cc.geom1 == self.gpad: fw = -fw
                F += -fw[0]; ft += fw[1:3]
        return F, ft

    def other_contacts(self):
        """Geom-name pairs in contact other than pad-wall (unintended wall/floor/self/airframe contact)."""
        return [(self.m.geom(c.geom1).name, self.m.geom(c.geom2).name) for c in self.d.contact[:self.d.ncon] if {c.geom1, c.geom2} != {self.gpad, self.gwall}]

    def com(self): return self.d.subtree_com[1].copy()

    # ---- physics-resolution log (peaks and impulses must not depend on the control rate)
    def reset_log(self): self.log = dict(t=[], Fn=[], ft=[], tau=[], Omega=[]); self.impulse = 0.0; self.other_names = set(); self.n_other_steps = 0; self.other_first = None

    def _log(self):
        F, ft = self.pad_force(); L = self.log; L["t"].append(self.d.time); L["Fn"].append(F); L["ft"].append(ft); L["tau"].append(self.tau_applied.copy()); L["Omega"].append(self.Omega.copy()); self.impulse += F * self.dt
        o = self.other_contacts()
        if o:
            self.n_other_steps += 1; self.other_names.update(tuple(sorted(p)) for p in o)
            if self.other_first is None: self.other_first = (self.d.time, sorted(tuple(sorted(p)) for p in o))
