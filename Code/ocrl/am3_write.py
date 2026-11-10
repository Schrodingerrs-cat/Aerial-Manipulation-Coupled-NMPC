"""Writing on the wall (2026-11-10, user request; outside the frozen plan, which holds the base at one hover point).
Text -> pen strokes (single-stroke capital font) -> WriteTask: for every stroke  position/travel (pen up) -> approach -> settle -> trace -> release -> lift,
with the same contact detection, settle, recovery and limits as am3_task.Task.  The base reference follows the pen across the wall with the hover offset
(pB = BASE + pen displacement in y, z), so letters can be any size; the arm keeps the contact and the fine detail.
Wall frame: the drone faces +x, so reading direction (left to right for a viewer behind the drone) is -y.  Glyph units: width 0.6, height 1."""
import copy, numpy as np
from ocrl.am3_task import Task, CP, TASK, BASE, YZ0, DELTA, SMOOTH, wall_x
from ocrl.am3_path import rounded_polygon, reference, min_jerk
from ocrl.am3_episode import Episode
from ocrl.am3_nmpc import PH_W
PH_W.setdefault("lift", "free"); PH_W.setdefault("travel", "free")                 # pen-up phases use the NMPC's free-flight weights (as retreat does); added at run time, frozen file untouched

G_UP = 0.012            # pen-up gap (m)
V_WRITE = 0.025         # mean pen speed along a stroke (m/s); a min-jerk stroke peaks at 1.9x this
V_TRAVEL = 0.05         # mean pen-up travel speed between strokes (m/s)
LIFT_S = 0.4            # release gap -> pen-up gap


def _ell(c, r, a0, a1, n=14):
    a = np.radians(np.linspace(a0, a1, n)); return [(c[0] + r[0] * np.cos(t), c[1] + r[1] * np.sin(t)) for t in a]


FONT = {
    "A": [[(0, 0), (0.3, 1), (0.6, 0)], [(0.13, 0.42), (0.47, 0.42)]],
    "B": [[(0, 0), (0, 1), (0.4, 1)] + _ell((0.4, 0.76), (0.18, 0.24), 90, -90, 8) + [(0, 0.52)], [(0, 0.52), (0.4, 0.52)] + _ell((0.4, 0.26), (0.2, 0.26), 90, -90, 8) + [(0, 0)]],
    "C": [_ell((0.3, 0.5), (0.3, 0.5), 50, 310)],
    "D": [[(0, 0), (0, 1), (0.25, 1)] + _ell((0.25, 0.5), (0.35, 0.5), 90, -90, 12) + [(0, 0)]],
    "E": [[(0.6, 1), (0, 1), (0, 0), (0.6, 0)], [(0, 0.5), (0.45, 0.5)]],
    "F": [[(0.6, 1), (0, 1), (0, 0)], [(0, 0.5), (0.45, 0.5)]],
    "G": [_ell((0.3, 0.5), (0.3, 0.5), 50, 360) + [(0.35, 0.5)]],
    "H": [[(0, 1), (0, 0)], [(0.6, 1), (0.6, 0)], [(0, 0.5), (0.6, 0.5)]],
    "I": [[(0.3, 1), (0.3, 0)]],
    "J": [[(0.6, 1), (0.6, 0.3)] + _ell((0.3, 0.3), (0.3, 0.3), 0, -180, 10)],
    "K": [[(0, 1), (0, 0)], [(0.6, 1), (0, 0.42), (0.6, 0)]],
    "L": [[(0, 1), (0, 0), (0.6, 0)]],
    "M": [[(0, 0), (0, 1), (0.3, 0.4), (0.6, 1), (0.6, 0)]],
    "N": [[(0, 0), (0, 1), (0.6, 0), (0.6, 1)]],
    "O": [_ell((0.3, 0.5), (0.3, 0.5), 90, 450, 24)],
    "P": [[(0, 0), (0, 1), (0.4, 1)] + _ell((0.4, 0.75), (0.2, 0.25), 90, -90, 8) + [(0, 0.5)]],
    "Q": [_ell((0.3, 0.5), (0.3, 0.5), 90, 450, 24), [(0.38, 0.25), (0.65, -0.05)]],
    "R": [[(0, 0), (0, 1), (0.4, 1)] + _ell((0.4, 0.75), (0.2, 0.25), 90, -90, 8) + [(0, 0.5), (0.6, 0)]],
    "S": [_ell((0.3, 0.75), (0.28, 0.25), 20, 270, 10) + _ell((0.3, 0.25), (0.3, 0.25), 90, -160, 10)],
    "T": [[(0, 1), (0.6, 1)], [(0.3, 1), (0.3, 0)]],
    "U": [[(0, 1), (0, 0.3)] + _ell((0.3, 0.3), (0.3, 0.3), 180, 360, 10) + [(0.6, 1)]],
    "V": [[(0, 1), (0.3, 0), (0.6, 1)]],
    "W": [[(0, 1), (0.15, 0), (0.3, 0.6), (0.45, 0), (0.6, 1)]],
    "X": [[(0, 1), (0.6, 0)], [(0.6, 1), (0, 0)]],
    "Y": [[(0, 1), (0.3, 0.5), (0.6, 1)], [(0.3, 0.5), (0.3, 0)]],
    "Z": [[(0, 1), (0.6, 1), (0, 0), (0.6, 0)]],
}


def text_strokes(text, height=0.06, pitch=1.0, space=0.7):
    """Strokes (lists of wall yz points, m) for `text`, first stroke starting at (0, 0).  pitch/space in letter heights; letter width = 0.6 height."""
    out, u = [], 0.0
    for ch in text.upper():
        if ch == " ": u += space * height; continue
        for s in FONT[ch]:
            P = np.array([[-(u + height * a), height * b] for a, b in s]); keep = np.r_[True, np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-9]
            out.append(P[keep])                                                       # reading direction is -y; repeated points (arc joins) would give zero-length fillet segments
        u += pitch * height
    o = out[0][0].copy(); return [s - o for s in out]


class WriteTask(Task):
    """Multi-stroke contact writing with a moving base.  Phases: hover, position, approach, settle, trace, release, lift, travel, (recover, settle_resume), done."""
    def __init__(self, p0, strokes, height=0.06, z0=0.0):
        super().__init__("free", p0, free_waypoints=[p0], segment_s=1.0)            # parent only supplies the contact-detection state; everything else is set here
        self.free = False; fil = 0.08 * height; self.strokes = strokes; self.paths = [rounded_polygon(s, fil) for s in strokes]
        self.Ts = [max(1.0, p.length / V_WRITE) for p in self.paths]; self.i = 0; self.yz0 = YZ0 + np.array([0.0, z0])
        self.set_stroke(0); self.pre = np.r_[wall_x(G_UP), self.yz0 + self.path.at(0)[0]]; self.gap_ref = G_UP; self.tr_from = self.tr_to = None; self.T_tr = 1.0; self.written = []

    def set_stroke(self, i): self.i = i; self.path, self.T = self.paths[i], self.Ts[i]; self.tau_p = 0.0

    def advance(self, dt, F_meas, gap_meas, nominal=False):
        ph = self.phase
        if ph in ("approach", "settle", "settle_resume", "trace", "recover", "hover", "position") or ph == "endhold":
            if ph == "position":                                                    # pen-up flight to the first stroke at G_UP (parent would go to approach at 4 cm)
                self.t += dt; self.tph += dt
                if self.tph >= TASK["position_s"]: self.gap_ref = G_UP; self._enter("approach")
                return
            super().advance(dt, F_meas, gap_meas, nominal)
            if self.phase == "endhold": self._enter("release")                       # no dwell at a stroke end
            return
        self.t += dt; self.tph += dt; g0 = self.gap_ref
        if not nominal:                                                             # keep the contact detector running while the pen is up (drag check uses it)
            self.n_on = self.n_on + 1 if F_meas > CP["F_on_N"] else 0; self.n_off = self.n_off + 1 if F_meas < CP["F_off_N"] else 0
            if not self.contact and self.n_on >= max(1, round(CP["T_on_s"] / dt)): self.contact = True
            if self.contact and self.n_off >= max(1, round(CP["T_off_s"] / dt)): self.contact = False
        if ph == "release":
            self.gap_ref = -DELTA + (CP["release_gap_m"] + DELTA) * SMOOTH(self.tph / CP["release_s"])
            if self.tph >= CP["release_s"]: self._enter("lift")
        elif ph == "lift":
            self.gap_ref = CP["release_gap_m"] + (G_UP - CP["release_gap_m"]) * SMOOTH(self.tph / LIFT_S)
            if self.tph >= LIFT_S:
                self.written.append(self.i)
                if self.i + 1 >= len(self.paths): self._enter("done"); self.t_done = self.t
                else:
                    self.tr_from = self.path.at(self.path.length)[0]; self.set_stroke(self.i + 1); self.tr_to = self.path.at(0)[0]
                    self.T_tr = max(0.8, np.linalg.norm(self.tr_to - self.tr_from) / V_TRAVEL); self._enter("travel")
        elif ph == "travel":
            if self.tph >= self.T_tr: self._enter("approach")
        self.gap_rate = (self.gap_ref - g0) / dt if dt > 0 else 0.0

    def pen(self):
        """Pen position and velocity on the wall (yz, relative to the text origin)."""
        ph = self.phase
        if ph in ("hover", "position"): return self.path.at(0)[0], np.zeros(2)
        if ph == "travel":
            f, f1, _ = min_jerk(min(self.tph / self.T_tr, 1.0)); return self.tr_from + (self.tr_to - self.tr_from) * f, (self.tr_to - self.tr_from) * f1 / self.T_tr
        if ph == "done": return self.path.at(self.path.length)[0], np.zeros(2)
        yz, vyz, _ = reference(self.path, self.T, min(self.tau_p, self.T)); return yz, vyz * (self.rho if ph == "trace" else 0.0)

    def ref(self):
        ph = self.phase; yz, vyz = self.pen(); d = yz - self.paths[0].at(0)[0]
        pB = BASE + np.r_[0.0, d] + np.r_[0.0, self.yz0 - YZ0]; vB = np.r_[0.0, vyz]
        if ph == "hover": p, v = self.p0, np.zeros(3); pB, vB = BASE.copy(), np.zeros(3)
        elif ph == "position":
            f, f1, _ = min_jerk(min(self.tph / TASK["position_s"], 1.0)); p = self.p0 + (self.pre - self.p0) * f; v = (self.pre - self.p0) * f1 / TASK["position_s"]
            vB = (pB - BASE) * f1 / TASK["position_s"]; pB = BASE + (pB - BASE) * f
        else: p = np.r_[wall_x(self.gap_ref), self.yz0 + yz]; v = np.r_[-self.gap_rate, vyz]
        contact = ph in ("settle", "settle_resume", "trace")
        return dict(phase=ph, p=p, v=v, gap=self.gap_ref, contact_expected=contact, s=(len(self.written) + min(1.0, self.tau_p / self.T)) / len(self.paths), pB=pB, vB=vB, stroke=self.i)


class WriteEpisode(Episode):
    """Episode whose task is a WriteTask (the parent's placeholder task is replaced before the run, as in am3_traj.TrajEpisode)."""
    def __init__(self, controller, text, height=0.06, z0=0.0, **kw):
        kw.setdefault("T_max", 600.0); super().__init__(controller, "write", free_waypoints=[[0.0, 0.0, 0.0]], segment_s=1.0, **kw)
        self.text = text; self.task = WriteTask(self.p0, text_strokes(text, height), height, z0)
        if hasattr(controller, "attach"): controller.attach(self)                       # RL controller reads the episode plant (as in am3_eval_wall.run)
        if hasattr(controller, "reset"): controller.reset(self.x0, self.u0, self.task)
