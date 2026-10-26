"""Task logic shared by every controller (Plan s3, s7.3): reference generation for the pad centre and the contact state machine
hover -> position (arm moves to the pre-approach point) -> approach -> settle (persistent acquisition and brief contact settling) -> trace -> endhold -> release -> retreat -> done, with bounded recovery.
Nothing here reads the plant: it consumes the measured normal force and a measured pad gap.  The only contact variable is the gap reference (gap < 0 is indentation of the pad); there is no force setpoint,
no orientation reference and no dwell.  The base-position reference is fixed for the whole run.  A `free` task moves the pad through a smooth 3D loop without a wall (arm motion in free flight)."""
import os, json, copy, numpy as np
from ocrl.am3_pred import C
from ocrl.am3_path import line_path, circle_path, rounded_polygon, min_jerk, reference

HERE = os.path.dirname(os.path.abspath(__file__)); TASK = json.load(open(os.path.join(HERE, "..", "config", "am3_task_v1.json"))); CP = TASK["contact_policy"]
XW, R_PAD = C["wall_x_m"], C["pad"]["radius_m"]; BASE = np.array(TASK["base_ref_world"]); YZ0 = np.array(TASK["patch_centre_world_yz_m"]); DELTA = CP["indentation_ref_m"]
SMOOTH = lambda x: 3 * np.clip(x, 0, 1) ** 2 - 2 * np.clip(x, 0, 1) ** 3

def make_path(name):
    p = TASK["paths"][name]
    if p["kind"] == "line": return line_path(p["a"], p["b"]), p["T_s"]
    if p["kind"] == "circle": return circle_path(p["centre"], p["radius"]), p["T_s"]
    return rounded_polygon(p["vertices"], p["fillet_m"]), p["T_s"]

def wall_x(gap): return XW - R_PAD - gap            # pad-centre x for a given gap

class Task:
    """advance(dt, F_meas, gap_meas) is the only entry that changes phase; horizon(N, h) previews the nominal timeline (no events).  ref() -> dict(phase, p, v, gap, contact_expected, s)."""
    def __init__(self, path_name, p0, free_waypoints=None, segment_s=None):
        self.name = path_name; self.p0 = np.asarray(p0, float); self.free = free_waypoints is not None; self.t = 0.0; self.tph = 0.0; self.phase = "hover"; self.prev = None
        if self.free: self.wp = [self.p0] + [np.asarray(w, float) for w in free_waypoints]; self.T_seg = segment_s or TASK["free_segment_s"]; self.T = self.T_seg * (len(self.wp) - 1); self.path = None
        else: self.path, self.T = make_path(path_name); self.pre = np.r_[wall_x(CP["approach_gap0_m"]), YZ0 + self.path.at(0)[0]]
        self.tau_p = 0.0; self.rho = 0.0; self.gap_ref = CP["approach_gap0_m"]; self.gap_rate = 0.0; self.n_on = self.n_off = 0; self.contact = False; self.recoveries = []; self.t_recover = 0.0; self.failed = None
        self.retreat_v = 0.0; self.t_done = None; self.gap_settle0 = 0.0
    def _speed(self, g): return CP["v_final_m_s"] + (CP["v_approach_m_s"] - CP["v_final_m_s"]) * SMOOTH((g - 0.01) / 0.01)
    def _enter(self, ph): self.phase = ph; self.tph = 0.0
    def advance(self, dt, F_meas, gap_meas, nominal=False):
        self.t += dt; self.tph += dt; ph = self.phase; g0 = self.gap_ref
        if not nominal:                                       # persistent contact detection with hysteresis on the measured force
            self.n_on = self.n_on + 1 if F_meas > CP["F_on_N"] else 0; self.n_off = self.n_off + 1 if F_meas < CP["F_off_N"] else 0
            if not self.contact and self.n_on >= CP["T_on_samples"]: self.contact = True
            if self.contact and self.n_off >= CP["T_off_samples"]: self.contact = False
        self.rho += np.clip((1.0 if ph == "trace" else 0.0) - self.rho, -dt / CP["progress_slew_s"], dt / CP["progress_slew_s"])
        if self.free:
            if ph == "hover" and self.tph >= TASK["hover_s"]: self._enter("trace")
            elif ph == "trace":
                self.tau_p = min(self.tau_p + dt, self.T)
                if self.tau_p >= self.T: self._enter("endhold")
            elif ph == "endhold" and self.tph >= TASK["endhold_s"]: self._enter("done"); self.t_done = self.t
            return
        if ph == "hover":
            if self.tph >= TASK["hover_s"]: self._enter("position")
        elif ph == "position":
            if self.tph >= TASK["position_s"]: self._enter("approach")
        elif ph == "approach":
            self.gap_ref = max(self.gap_ref - (self._speed(self.gap_ref) if self.gap_ref > 0.0 else CP["v_seek_m_s"]) * dt, CP["seek_floor_m"])      # creeps in past zero until contact is measured
            if self.contact and not nominal: self.gap_settle0 = self.gap_ref; self._enter("settle")
            elif not nominal and self.gap_ref <= CP["seek_floor_m"] and self.tph > CP["acquire_max_s"]: self.failed = "no contact acquired"
        elif ph in ("settle", "settle_resume"):
            self.gap_ref = self.gap_settle0 + (-DELTA - self.gap_settle0) * SMOOTH(self.tph / CP["settle_ramp_s"])
            if self.tph >= CP["settle_s"]: self._enter(self.prev if ph == "settle_resume" else "trace")
        elif ph == "trace":
            self.tau_p = min(self.tau_p + dt * self.rho, self.T)
            if self.tau_p >= self.T: self._enter("endhold")
        elif ph == "endhold":
            if self.tph >= TASK["endhold_s"]: self._enter("release")
        elif ph == "release":
            self.gap_ref = -DELTA + (CP["release_gap_m"] + DELTA) * SMOOTH(self.tph / CP["release_s"])
            if self.tph >= CP["release_s"]: self.retreat_v = 0.0; self._enter("retreat")
        elif ph == "retreat":
            self.retreat_v = min(self.retreat_v + CP["retreat_accel_m_s2"] * dt, CP["retreat_speed_m_s"]); self.gap_ref = min(self.gap_ref + self.retreat_v * dt, CP["release_gap_m"] + CP["retreat_m"])
            if self.gap_ref >= CP["release_gap_m"] + CP["retreat_m"]: self._enter("done"); self.t_done = self.t
        elif ph == "recover":                                 # bounded recovery: progress paused, re-approach at the final speed until contact is measured again
            self.t_recover += dt; self.gap_ref = max(self.gap_ref - (CP["v_final_m_s"] if self.gap_ref > 0.0 else CP["v_seek_m_s"]) * dt, CP["seek_floor_m"])
            if self.contact and not nominal: self.recoveries[-1]["duration_s"] = self.t_recover; self.gap_settle0 = self.gap_ref; self._enter("settle_resume")
            elif self.t_recover > CP["recover_max_s"]: self.failed = "recovery timeout"
        if not nominal and ph in ("settle", "trace", "endhold", "settle_resume") and not self.contact and self.n_off >= CP["T_off_samples"]:        # confirmed loss
            self.prev = "endhold" if ph == "endhold" else "trace"; self.recoveries.append(dict(t=self.t, phase=ph, tau_p=self.tau_p)); self.t_recover = 0.0
            self.gap_ref = float(np.clip(gap_meas, -DELTA, CP["recover_gap_max_m"])); self._enter("recover")
        self.gap_rate = (self.gap_ref - g0) / dt if dt > 0 else 0.0
    def ref(self):
        ph = self.phase
        if self.free:
            seg = min(int(self.tau_p // self.T_seg), len(self.wp) - 2); f, f1, _ = min_jerk((self.tau_p - seg * self.T_seg) / self.T_seg); a, b = self.wp[seg], self.wp[seg + 1]
            p = a + (b - a) * f; v = (b - a) * f1 / self.T_seg * (1.0 if ph == "trace" else 0.0); return dict(phase=ph, p=p, v=v, gap=None, contact_expected=False, s=self.tau_p / self.T)
        if ph == "hover": p, v = self.p0, np.zeros(3)
        elif ph == "position":
            f, f1, _ = min_jerk(self.tph / TASK["position_s"]); p = self.p0 + (self.pre - self.p0) * f; v = (self.pre - self.p0) * f1 / TASK["position_s"]
        else:
            yz, vyz, _ = reference(self.path, self.T, min(self.tau_p, self.T)); vyz = vyz * (1.0 if ph == "trace" else 0.0); p = np.r_[wall_x(self.gap_ref), YZ0 + yz]; v = np.r_[-self.gap_rate, vyz]
        contact = ph in ("settle", "settle_resume", "trace", "endhold")
        return dict(phase=ph, p=p, v=v, gap=self.gap_ref, contact_expected=contact, s=min(1.0, self.tau_p / self.T))
    def horizon(self, N, h, dt=0.02):
        sc = copy.deepcopy(self); out = []
        for _ in range(N + 1):
            out.append(sc.ref())
            for _ in range(int(round(h / dt))): sc.advance(dt, 0.0, 0.0, nominal=True)
        return out
