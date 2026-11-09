"""Continuous free-flight pad motions for the arm-motion tests (no wall): the task is the standard free task (hover, then `trace`), but the pad reference is a smooth periodic motion about the start point p0, with its amplitude ramped in over RAMP_S.
Name format `free_<kind>_A<amplitude_m>_f<frequency_Hz>`, kind: circ (circle in the y-z plane), yosc (horizontal), zosc (vertical), reach (0 -> A toward the wall and back, the arm extending and retracting).  Duration MOTION_S."""
import numpy as np
from ocrl.am3_task import Task
from ocrl.am3_path import min_jerk
KINDS = ("circ", "yosc", "zosc", "reach"); RAMP_S = 1.0; MOTION_S = 6.0
def shape(kind, A, w, t):
    s, c = np.sin(w * t), np.cos(w * t)
    if kind == "circ": return A * np.array([0.0, s, 1 - c]), A * w * np.array([0.0, c, s])
    if kind == "yosc": return A * np.array([0.0, s, 0.0]), A * w * np.array([0.0, c, 0.0])
    if kind == "zosc": return A * np.array([0.0, 0.0, s]), A * w * np.array([0.0, 0.0, c])
    return A / 2 * np.array([1 - c, 0.0, 0.0]), A / 2 * w * np.array([s, 0.0, 0.0])
class FreeMotion(Task):
    def __init__(self, path_name, p0, free_waypoints=None, segment_s=None):
        _, kind, a, f = path_name.split("_"); self.kind, self.A, self.w = kind, float(a[1:]), 2 * np.pi * float(f[1:]); super().__init__(path_name, p0, [p0], MOTION_S)
    def ref(self):
        t = self.tau_p; r, r1, _ = min_jerk(t / RAMP_S) if t < RAMP_S else (1.0, 0.0, 0.0); d, dv = shape(self.kind, self.A, self.w, t); on = self.phase == "trace"
        return dict(phase=self.phase, p=self.p0 + r * d, v=(r * dv + r1 / RAMP_S * d) * (1.0 if on else 0.0), gap=None, contact_expected=False, s=self.tau_p / self.T)
