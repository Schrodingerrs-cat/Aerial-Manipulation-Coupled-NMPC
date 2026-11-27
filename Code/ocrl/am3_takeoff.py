"""Take-off from a launch stand before writing (2026-11-26, user request; outside the frozen plan, which starts in hover).
The drone rests on a 4-post stand (the rotor arms sit on the posts), arm folded (upper arm 45 deg down-forward, forearm folded back up) so it clears the stand and floor.
Phases before WriteTask's own: ground (rotors spin up, references at rest) -> takeoff (vertical climb to hover height, arm folded) ->
unfold (base flies forward to the hover point while the pen moves to the writing start) -> hover, position, approach, ... as in am3_write.
The stand is added to a copy of the frozen model at run time; the frozen files are not touched.  The stand top is at 0.33 m so the base starts above the
frozen Episode's 0.3 m divergence floor."""
import os, tempfile, numpy as np, mujoco
from ocrl.am3_plant import AMPlant3, XML, CFG, OM_MIN, KF
from ocrl.am3_episode import initial_state
from ocrl.am3_task import BASE
from ocrl.am3_path import min_jerk
from ocrl.am3_write import WriteTask, WriteEpisode, text_strokes, V_WRITE
from ocrl.am3_nmpc import PH_W
from ocrl.am3_pred import AMPred3

PH_W.setdefault("ground", "free"); PH_W.setdefault("takeoff", "free"); PH_W.setdefault("unfold", "free")
STAND_TOP = 0.33                                     # m, post tops (rotor-arm undersides rest here)
START = np.array([BASE[0] - 0.45, 0.0, STAND_TOP + 0.0025])    # base on the stand, 0.45 m behind the hover point (folded arm stays clear of the wall)
Q_FOLD = np.radians([0.0, -35.0, -110.0])            # yaw, shoulder, elbow: fold inside the operational range (q2 -80..-5, q3 -125..-30), same elbow branch as writing; 6 cm from the propeller discs
                                                     # under +-8 deg joint and +-10 deg yaw wiggle, 8.5 cm above the floor, shoulder 0.72 N m with 200 g. (-45, -120) left 0.5 cm: R25 hit prop1 with a finger in
                                                     # every take-off episode. The straight L (q2 -90) is outside the range and needs 1.2 N m with 200 g, over the 1 N m servo.
T_GROUND, T_TAKEOFF, T_UNFOLD = 1.5, 3.0, 3.0        # s

POSTS = "".join(f'<geom name="stand_post{i}" type="cylinder" fromto="{START[0] + sx * 0.071:.4f} {sy * 0.071:.4f} 0 {START[0] + sx * 0.071:.4f} {sy * 0.071:.4f} {STAND_TOP - 0.004:.4f}" size="0.008" rgba="0.3 0.3 0.32 1"/>'
                f'<geom name="stand_pad{i}" type="box" pos="{START[0] + sx * 0.071:.4f} {sy * 0.071:.4f} {STAND_TOP - 0.002:.4f}" size="0.015 0.015 0.002" rgba="0.15 0.15 0.15 1"/>'
                for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, -1), (-1, 1)]))
BASEPLATE = f'<geom name="stand_base" type="box" pos="{START[0]:.4f} 0 0.004" size="0.11 0.11 0.004" rgba="0.3 0.3 0.32 1"/>'


_XML = []
def stand_xml():
    """Frozen model + stand, written once per process to a temp file (AMPlant3 loads from a path)."""
    if _XML: return _XML[0]
    s = open(XML).read(); k = s.index('<body name="drone"'); f = tempfile.NamedTemporaryFile("w", suffix="_am3_stand.xml", delete=False); f.write(s[:k] + POSTS + BASEPLATE + "\n    " + s[k:]); f.close(); _XML.append(f.name); return f.name


def start_state(xml):
    """Base resting on the stand, arm folded, rotors at idle, joint torques = gravity hold of the folded arm."""
    pl = AMPlant3(xml=xml); x = np.zeros(23); x[0:3] = START; x[3] = 1.0; x[13:16] = Q_FOLD; x[19:23] = OM_MIN
    pl.set_state(x); mujoco.mj_forward(pl.m, pl.d); tau = pl.d.qfrc_bias[6:9].copy()
    return x, np.r_[x[19:23], tau]


def rotor_share(xml, x0):
    """Rotor speeds whose thrusts carry the weight with no net pitch/roll moment about the centre of mass (rotors at (+-0.1, +-0.1), order as prop0..3)."""
    pl = AMPlant3(xml=xml); pl.set_state(x0); mujoco.mj_forward(pl.m, pl.d); W = pl.m.body_subtreemass[1] * 9.81; dx, dy = pl.com()[0:2] - x0[0:2]
    sx, sy = np.array([1, 1, -1, -1]), np.array([1, -1, -1, 1]); F = W / 4 + W * dx / 0.4 * sx + W * dy / 0.4 * sy; return np.sqrt(F / KF)


KP_J, KD_J = 1.5, 0.06                               # N m/rad, N m s/rad: servo position hold of the fold on the stand (the folded-up forearm drifts under a constant torque)


def spinup_cmd(u0, om_share, t, xm, spinup=0.85):
    """Command on the stand at time t: rotors idle -> sqrt(spinup) x their weight-share speed (thrust = spinup x share, min-jerk over T_GROUND);
    joints held at Q_FOLD by gravity feed-forward (nominal arm) + PD on the measured joint angles and rates."""
    f = min_jerk(min(t / T_GROUND, 1.0))[0]; tau = u0[4:] + KP_J * (Q_FOLD - xm[13:16]) - KD_J * xm[16:19]
    return np.r_[OM_MIN + (np.sqrt(spinup) * om_share - OM_MIN) * f, tau]


class GroundStart:
    """Wraps any controller.  On the stand ('ground' phase) the rotors spin up open-loop from idle to 85 % of their static share of the weight (min-jerk; the folded
    arm puts the centre of mass forward, so the front rotors carry more and equal speeds would tip the drone onto its nose), with the folded arm held by
    its servos (gravity feed-forward + PD); at the first 'takeoff' cycle the controller is reset from the measured state and the last command, then flies from rest.
    Same for every controller (the controllers' models have no stand, so closing the loop while it carries the weight makes them push off)."""
    def __init__(self, ctrl, u0, om_hover): self.inner, self.u0, self.om_hover, self.handed = ctrl, np.asarray(u0, float), np.asarray(om_hover, float), False; self.u = self.u0.copy()
    def __getattr__(self, k): return getattr(self.inner, k)
    def reset(self, x0, u0, task): self.handed = False; self.u = np.asarray(u0, float).copy(); self.inner.reset(x0, u0, task)
    def compute(self, t, xm, Fm, task):
        if task.phase == "ground":
            self.u = spinup_cmd(self.u0, self.om_hover, task.tph + 0.02, xm); return self.u.copy(), {}
        if not self.handed: self.handed = True; self.inner.reset(xm, self.u, task)
        return self.inner.compute(t, xm, Fm, task)


class TakeoffWriteTask(WriteTask):
    """WriteTask preceded by ground, takeoff and unfold.  p0 is the usual hover pen point; pen_fold the folded pen position relative to the base."""
    def __init__(self, p0, strokes, pen_fold, height=0.06, z0=0.0, v_write=V_WRITE, f_ref=None):
        super().__init__(p0, strokes, height, z0, v_write, f_ref); self.pen_fold = np.asarray(pen_fold, float); self.phase = "ground"
        self.top = np.r_[START[0], 0.0, BASE[2]]

    def advance(self, dt, F_meas, gap_meas, nominal=False):
        ph = self.phase
        if ph in ("ground", "takeoff", "unfold"):
            self.t += dt; self.tph += dt; T = dict(ground=T_GROUND, takeoff=T_TAKEOFF, unfold=T_UNFOLD)[ph]
            if self.tph >= T: self._enter(dict(ground="takeoff", takeoff="unfold", unfold="hover")[ph])
            return
        super().advance(dt, F_meas, gap_meas, nominal)

    def base_ahead(self, h):
        """Base reference h seconds ahead along take-off -> unfold -> hover point (look-ahead input of the learned controllers)."""
        s = self.tph + (T_TAKEOFF if self.phase == "unfold" else 0.0) + h
        if s < T_TAKEOFF: return START + (self.top - START) * min_jerk(s / T_TAKEOFF)[0]
        return self.top + (BASE - self.top) * min_jerk(min((s - T_TAKEOFF) / T_UNFOLD, 1.0))[0]

    def ref(self):
        ph = self.phase
        if ph not in ("ground", "takeoff", "unfold"): return super().ref()
        if ph == "ground": pB, vB = START.copy(), np.zeros(3); p, v = pB + self.pen_fold, np.zeros(3)
        elif ph == "takeoff":
            f, f1, _ = min_jerk(min(self.tph / T_TAKEOFF, 1.0)); pB = START + (self.top - START) * f; vB = (self.top - START) * f1 / T_TAKEOFF; p, v = pB + self.pen_fold, vB
        else:
            f, f1, _ = min_jerk(min(self.tph / T_UNFOLD, 1.0)); pB = self.top + (BASE - self.top) * f; vB = (BASE - self.top) * f1 / T_UNFOLD
            a = self.top + self.pen_fold; p = a + (self.p0 - a) * f; v = (self.p0 - a) * f1 / T_UNFOLD
        return dict(phase=ph, p=p, v=v, gap=self.gap_ref, contact_expected=False, s=0.0, pB=pB, vB=vB, stroke=self.i)


class TakeoffWriteEpisode(WriteEpisode):
    """WriteEpisode that starts on the stand: the plant is rebuilt with the stand scene and the task is a TakeoffWriteTask."""
    XML = None; takeoff = True                       # takeoff: video cameras follow the drone back to the stand

    def __init__(self, controller, text, height=0.06, z0=0.0, v_write=V_WRITE, f_ref=None, mismatch=None, **kw):
        if TakeoffWriteEpisode.XML is None: TakeoffWriteEpisode.XML = stand_xml()
        x0, u0 = start_state(TakeoffWriteEpisode.XML)
        controller = GroundStart(controller, u0, rotor_share(TakeoffWriteEpisode.XML, x0))
        super().__init__(controller, text, height, z0, v_write, f_ref, mismatch=mismatch, x0=(x0, u0), **kw)
        mm = mismatch or {}; self.pl = AMPlant3(k=CFG["contact"]["k_N_m"] * mm.get("k_scale", 1.0), c=CFG["contact"]["c_Ns_m"] * mm.get("c_scale", 1.0), mu=CFG["contact"]["mu"] * mm.get("mu_scale", 1.0), xml=TakeoffWriteEpisode.XML)
        if mm: self.pl.apply_mismatch(mm)
        self.pl.set_state(self.x0)
        p_hover = np.array(AMPred3().pad_f(initial_state()[0])[0]).ravel(); pen_fold = self.p0 - START
        self.task = TakeoffWriteTask(p_hover, text_strokes(text, height), pen_fold, height, z0, v_write, f_ref)
        if hasattr(controller, "attach"): controller.attach(self)
        if hasattr(controller, "reset"): controller.reset(self.x0, self.u0, self.task)
