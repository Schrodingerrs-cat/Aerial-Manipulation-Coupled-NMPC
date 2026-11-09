"""Moving-base demonstration: the drone tracks a smooth world trajectory while the arm simultaneously moves the pad relative to the moving base (free flight, no wall contact).
Plan s3 fixes the base-position reference for the contact task; this module does NOT change that task - it adds a separate demonstration that the coupled controller stays stable
while the base itself moves.  The pad reference rides along with the base, so the arm motion is the same relative motion as the `free` task.
Base and arm share one clock and one segment count, so both move at once.  Frozen controller/task/record files are not edited; the only change behind this is the additive
base-velocity reference parameter in am3_nmpc/am3_ctrl_n (zero for every existing task, see freeze v2)."""
import numpy as np
from PIL import Image, ImageDraw
from ocrl.am3_task import Task, BASE
from ocrl.am3_path import min_jerk
from ocrl.am3_episode import Episode
from ocrl.am3_record import Recorder, CAMS
import mujoco


class TrajTask(Task):
    """`free` task whose base reference follows its own waypoint trajectory; ref() gains pB/vB and the pad reference is shifted by the base displacement."""
    def __init__(self, p0, base_offsets, pad_offsets, segment_s=2.0):
        assert len(base_offsets) == len(pad_offsets), "base and arm trajectories share one clock"
        p0 = np.asarray(p0, float)
        super().__init__("traj", p0, free_waypoints=[p0 + np.asarray(o, float) for o in pad_offsets], segment_s=segment_s)
        self.base_wp = [BASE.copy()] + [BASE + np.asarray(o, float) for o in base_offsets]

    def _at(self, wps, tau, moving):
        seg = min(int(tau // self.T_seg), len(wps) - 2); f, f1, _ = min_jerk((tau - seg * self.T_seg) / self.T_seg); a, b = wps[seg], wps[seg + 1]
        return a + (b - a) * f, (b - a) * f1 / self.T_seg * (1.0 if moving else 0.0)

    def ref(self):
        r = super().ref(); pB, vB = self._at(self.base_wp, self.tau_p, self.phase == "trace")
        r["pB"], r["vB"] = pB, vB; r["p"] = r["p"] + (pB - self.base_wp[0]); r["v"] = r["v"] + vB          # the pad rides with the base; the arm adds its own relative motion
        return r


class LoopTask(Task):
    """Continuous periodic demonstration: the base flies a lemniscate (figure eight, the infinity symbol) for `base_laps` laps while the pad circles the base `arm_laps` times, both over `T` seconds.
    Angles are integrated through a smooth rate ramp of length `ramp_s` at each end, so the motion starts and stops from rest and still completes exactly the requested number of laps."""
    SHAPES = dict(lemniscate=(1, 2, 0.0), lissajous=(2, 3, np.pi / 2), sine=(1, 3, 0.0))     # (ny, nz, phase): y = A sin(ny th), z = C sin(nz th + phase)

    def __init__(self, p0, T=30.0, base_laps=3, arm_laps=10, amp_y=0.15, amp_z=0.05, r_arm=0.05, ramp_s=2.5, shape="lemniscate"):
        p0 = np.asarray(p0, float)
        super().__init__("loop", p0, free_waypoints=[p0], segment_s=T)          # one placeholder segment: fixes self.T = T and the hover/trace/endhold phase timeline
        self.Tl, self.base_laps, self.arm_laps = T, base_laps, arm_laps
        self.amp_y, self.amp_z, self.r_arm, self.ramp_s = amp_y, amp_z, r_arm, ramp_s
        self.shape = shape; self.ny, self.nz, self.ph0 = self.SHAPES[shape]

    def _ramped(self, u, laps):
        """Cumulative angle, angular rate, and the unit rate profile with its derivative.  The profile ramps 0->1->0 by smoothstep and is scaled so u = T gives exactly `laps` revolutions;
        it doubles as the arm-circle radius envelope, so the pad leaves and returns to the base-relative origin at rest."""
        T, Tr = self.Tl, min(self.ramp_s, self.Tl / 2)
        if u <= Tr: x = u / Tr; te, rt, drt = Tr * (x ** 3 - x ** 4 / 2), 3 * x ** 2 - 2 * x ** 3, (6 * x - 6 * x ** 2) / Tr
        elif u <= T - Tr: te, rt, drt = 0.5 * Tr + (u - Tr), 1.0, 0.0
        else: y = max(0.0, (T - u) / Tr); te, rt, drt = (T - 1.5 * Tr) + (0.5 * Tr - Tr * (y ** 3 - y ** 4 / 2)), 3 * y ** 2 - 2 * y ** 3, -(6 * y - 6 * y ** 2) / Tr
        w = 2 * np.pi * laps / max(T - Tr, 1e-9); return w * te, w * rt, rt, drt

    def ref(self):
        r = super().ref(); moving = self.phase == "trace"; u = self.tau_p
        th, thd, _, _ = self._ramped(u, self.base_laps); ph, phd, amp, damp = self._ramped(u, self.arm_laps)
        if not moving: thd = phd = damp = 0.0
        ny, nz, p0h = self.ny, self.nz, self.ph0                                                              # Lissajous figure in the y-z plane (1:2 with zero phase is the Gerono lemniscate)
        z0 = self.amp_z * np.sin(p0h)                                                                         # translate so the figure starts at the hover reference: no step at t = 0
        pB = BASE + np.array([0.0, self.amp_y * np.sin(ny * th), self.amp_z * np.sin(nz * th + p0h) - z0])
        vB = np.array([0.0, self.amp_y * ny * np.cos(ny * th) * thd, self.amp_z * nz * np.cos(nz * th + p0h) * thd])
        c, s = np.cos(ph), np.sin(ph); rr = self.r_arm                                                         # circle centred on the moving base, radius grown and shrunk by the same envelope
        arm = amp * rr * np.array([0.0, c, s])
        varm = damp * rr * np.array([0.0, c, s]) + amp * rr * phd * np.array([0.0, -s, c])
        r["pB"], r["vB"] = pB, vB; r["p"] = self.p0 + (pB - BASE) + arm; r["v"] = vB + varm
        return r


class TrajEpisode(Episode):
    """Episode whose task is a TrajTask.  The parent builds a plain `free` task first (its waypoints are placeholders) and it is replaced before the run starts."""
    def __init__(self, controller, base_offsets, pad_offsets, segment_s=2.0, **kw):
        super().__init__(controller, "traj", free_waypoints=[[0.0, 0.0, 0.0]] * len(pad_offsets), segment_s=segment_s, **kw)
        self.task = TrajTask(self.p0, base_offsets, pad_offsets, segment_s)
        if hasattr(controller, "reset"): controller.reset(self.x0, self.u0, self.task)


class LoopEpisode(Episode):
    """Episode running a LoopTask (figure-eight base, circling arm); the parent's placeholder task is replaced before the run starts."""
    def __init__(self, controller, T=30.0, **task_kw):
        kw = {k: task_kw.pop(k) for k in list(task_kw) if k in ("seed", "mismatch", "T_max", "record", "x0", "miss", "sensors")}
        no_wall = task_kw.pop("no_wall", False)
        super().__init__(controller, "loop", free_waypoints=[[0.0, 0.0, 0.0]], segment_s=T, **kw)
        if no_wall: remove_wall(self)
        self.task = LoopTask(self.p0, T=T, **task_kw)
        if hasattr(controller, "reset"): controller.reset(self.x0, self.u0, self.task)


class TrajRecorder(Recorder):
    """Recorder whose overlay reports base error against the MOVING reference and draws that reference.  Body copied from Recorder.__call__ (frozen) with those two changes."""
    def __call__(self, ep, t):
        r = ep.rows[-1]; pad = r["pad"]; pB = np.asarray(r["ref"]["pB"], float); self.trace.append((pad.copy(), r["F"] > 0.08))
        self.btrace = getattr(self, "btrace", []); self.btrace.append((r["x"][0:3].copy(), pB.copy()))
        if t < self.next_t: return
        self.next_t += self.dt
        if self.renderer is None:
            ep.pl.m.vis.global_.offwidth = max(self.size[0], ep.pl.m.vis.global_.offwidth); ep.pl.m.vis.global_.offheight = max(self.size[1], ep.pl.m.vis.global_.offheight)
            self.renderer = mujoco.Renderer(ep.pl.m, self.size[1], self.size[0])
        e_base = np.linalg.norm(r["x"][0:3] - pB) * 1e3; e_pad = np.linalg.norm(pad - r["ref"]["p"]) * 1e3
        txt = [f"{self.label}   run {self.tag}", f"t = {r['t']:6.2f} s   phase: {r['phase']}", f"base vs MOVING ref {e_base:5.2f} mm   pad-reference error {e_pad:5.2f} mm",
               f"base ref y {pB[1]:+.3f}  z {pB[2]:.3f} m   |v_ref| {np.linalg.norm(r['ref']['vB']):.3f} m/s"]
        for cam in self.cams:
            c = CAMS[cam]; mc = mujoco.MjvCamera(); mc.lookat[:] = c["lookat"]; mc.distance = c["distance"]; mc.azimuth = c["azimuth"]; mc.elevation = c["elevation"]
            self.renderer.update_scene(ep.pl.d, mc); scn = self.renderer.scene
            B = self.btrace[:: max(1, len(self.btrace) // 400)]
            for (a, ar), (b, br) in zip(B[:-1], B[1:]): self._seg(scn, ar, br, (0.1, 0.8, 0.2, 1), 0.0015)          # green: commanded base trajectory
            for (a, ar), (b, br) in zip(B[:-1], B[1:]): self._seg(scn, a, b, (1.0, 0.5, 0.0, 1), 0.0015)            # orange: actual base path
            T = self.trace[:: max(1, len(self.trace) // 400)]
            for (a, ca), (b, cb) in zip(T[:-1], T[1:]): self._seg(scn, a, b, (0.2, 0.4, 1.0, 1), 0.0015)            # blue: actual pad path
            img = Image.fromarray(self.renderer.render()); d = ImageDraw.Draw(img); d.rectangle([0, 0, self.size[0], 4 + 17 * len(txt)], fill=(0, 0, 0))
            for i, line in enumerate(txt): d.text((6, 3 + 17 * i), line, fill=(255, 255, 255), font=self.font)
            img.save(f"{self.dir}/{cam}/{self.n:05d}.png")
        self.n += 1


def down_initial_state(q_down_deg=(0.0, -10.0, -35.0)):
    """Hover equilibrium with the arm hanging down instead of reaching forward at the wall.  q_down_deg stays inside the configured operational joint ranges
    (q1 +-35, q2 [-80,-5], q3 [-125,-30] deg), so the NMPC's soft joint-range constraints are satisfied at the start pose."""
    from ocrl.am3_pred import AMPred3
    from ocrl.am3_ref import Equilibrium
    pr = AMPred3(); r = np.array(pr.fk_f(np.radians(q_down_deg))).ravel()
    e = Equilibrium(pr, BASE).solve(BASE + r, [0.0, 0.0, 0.0])
    return (e["x"], e["u"]), BASE + r


def remove_wall(ep):
    """Free-flight demonstrations: take the wall out of the scene entirely - no collision and not drawn.  The plant is otherwise untouched (contact calibration already happened in __init__)."""
    m, g = ep.pl.m, ep.pl.gwall
    m.geom_contype[g] = 0; m.geom_conaffinity[g] = 0; m.geom_rgba[g] = [0.0, 0.0, 0.0, 0.0]
    return ep


def base_metrics(ep):
    """Tracking of the moving base reference and of the pad reference, plus attitude and actuation headroom."""
    R = ep.rows; t = np.array([r["t"] for r in R]); X = np.array([r["x"] for r in R])
    pB = np.array([r["ref"]["pB"] for r in R]); vB = np.array([r["ref"]["vB"] for r in R])
    pad = np.array([r["pad"] for r in R]); ref = np.array([r["ref"]["p"] for r in R]); ph = np.array([r["phase"] for r in R])
    eB = np.linalg.norm(X[:, 0:3] - pB, axis=1); eP = np.linalg.norm(pad - ref, axis=1); mv = ph == "trace"
    tilt = np.degrees(2 * np.arcsin(np.minimum(1, np.hypot(X[:, 4], X[:, 5]))))
    rms = lambda a: float(np.sqrt(np.mean(a ** 2))) if len(a) else float("nan")
    f = (np.array(ep.pl.log["Omega"]) ** 2) * (10.0 / 1800.0 ** 2)
    return dict(controller=getattr(ep.ctrl, "name", "?"), abort=ep.abort, completed=bool(ep.abort is None and ep.task.phase == "done"), t_end=float(t[-1]),
                base_track_rms_mm=rms(eB) * 1e3, base_track_max_mm=float(eB.max() * 1e3), base_track_rms_moving_mm=rms(eB[mv]) * 1e3,
                pad_track_rms_mm=rms(eP) * 1e3, pad_track_max_mm=float(eP.max() * 1e3),
                base_ref_speed_max=float(np.linalg.norm(vB, axis=1).max()), base_travel_m=float(np.linalg.norm(pB.max(0) - pB.min(0))),
                tilt_max_deg=float(tilt.max()), qd_max=np.abs(X[:, 16:19]).max(0).tolist(),
                rotor_force_min_N=float(f.min()), rotor_force_max_N=float(f.max()),
                solver_failures=sum(1 for r in R if r["info"].get("ok") is False), solver_fallback=sum(1 for r in R if r["info"].get("fallback")),
                cycle_median_ms=float(np.median([r["cycle_s"] for r in R]) * 1e3), cycle_max_ms=float(max(r["cycle_s"] for r in R) * 1e3))
