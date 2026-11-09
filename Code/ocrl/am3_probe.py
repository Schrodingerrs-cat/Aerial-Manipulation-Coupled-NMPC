"""Physics-rate diagnostics of the whole assembly from MuJoCo's own quantities (not the CasADi predictor): total mass, COM position/velocity, angular momentum about the COM, external loads (rotor thrusts and reaction
torques, gravity, wall contact force at its contact point) and the Newton-Euler residuals
    m a_COM - (F_rotors + m g + F_wall)           (internal arm reactions cannot appear here; F_rotors includes any test disturbance applied through xfrc)
    dL_COM/dt - (M_rotors + r_wall x F_wall)       (moments about the COM)
a_COM and dL/dt are finite differences of the subtree linear velocity and subtree angular momentum between consecutive physics steps."""
import numpy as np, mujoco

class Probe:
    def __init__(self, pl): self.pl = pl; self.m = pl.m; self.mass = pl.m.body_mass.sum(); self.reset()
    def reset(self): self.prev = None; self.prev_loads = None; self.rows = []; self.prev_ext = None
    def _state(self):
        m, d = self.pl.m, self.pl.d; mujoco.mj_subtreeVel(m, d); return d.time, d.subtree_linvel[1].copy(), d.subtree_angmom[1].copy(), d.subtree_com[1].copy()
    def loads(self):
        """External force and moment about the COM at the current instant: rotors, gravity, wall contact (pad)."""
        m, d, pl = self.pl.m, self.pl.d, self.pl; com = d.subtree_com[1]; F_rot = np.zeros(3); M_rot = np.zeros(3)
        for i in range(4):
            a = m.actuator(i); sid = m.actuator_trnid[i, 0]; z = d.site_xmat[sid].reshape(3, 3)[:, 2]; f = d.actuator_force[i]; F = f * z; F_rot += F; M_rot += np.cross(d.site_xpos[sid] - com, F) + m.actuator_gear[i, 5] * f * z
        F_x = d.xfrc_applied[1, :3].copy(); M_x = np.cross(d.xipos[1] - com, F_x) + d.xfrc_applied[1, 3:6]          # test disturbances (force at the drone-body COM, torque)
        F_w = np.zeros(3); M_w = np.zeros(3); f6 = np.zeros(6); pt = None
        for i in range(d.ncon):
            c = d.contact[i]
            if {c.geom1, c.geom2} == {pl.gpad, pl.gwall}:
                mujoco.mj_contactForce(m, d, i, f6); fw = c.frame.reshape(3, 3).T @ f6[:3]; fw = -fw if c.geom1 == pl.gpad else fw; F_w += fw; M_w += np.cross(c.pos - com, fw); pt = c.pos.copy()
        return F_rot + F_x, M_rot + M_x, np.array([0, 0, -self.mass * 9.81]), F_w, M_w, pt
    def record(self):
        """Call once after every physics step."""
        t, v, L, com = self._state()
        cur = self.loads(); com_l = com.copy()
        if self.prev is not None and self.prev_loads is not None:
            # after mj_step the derived quantities (subtree velocity, forces, contacts) belong to the state at the START of the step, so the finite difference between records k-1 and k is the effect of the loads of record k-1
            dt = t - self.prev[0]; a = (v - self.prev[1]) / dt; dL = (L - self.prev[2]) / dt; F_rot, M_rot, F_g, F_w, M_w, pt = self.prev_loads
            ext = self.pl.d.xfrc_applied[1].copy(); changed = self.prev_ext is not None and bool(np.any(ext != self.prev_ext)); self.prev_ext = ext          # the step in which a test disturbance switches is excluded from the balance statistics
            self.rows.append(dict(t=t, switch=changed, com=com.copy(), v=v.copy(), a=a, dL=dL, L=L.copy(), F_rot=F_rot, M_rot=M_rot, F_w=F_w, M_w=M_w, res_F=self.mass * a - (F_rot + F_g + F_w), res_M=dL - (M_rot + M_w), pt=pt))
        self.prev = (t, v, L, com); self.prev_loads = cur
    def arrays(self):
        R = self.rows; out = {k: np.array([r[k] for r in R]) for k in R[0] if k != "pt"}; out["pt"] = np.array([r["pt"] if r["pt"] is not None else [np.nan] * 3 for r in R]); return out
