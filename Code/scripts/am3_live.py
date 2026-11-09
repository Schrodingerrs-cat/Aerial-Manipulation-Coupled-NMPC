"""Live interactive MuJoCo viewer for any closed-loop run: the real controller drives the real plant while you watch and can orbit/zoom the camera.
Nothing here changes the simulation - it reuses the same Episode record hook the offscreen recorder uses, so what you see is the same run the videos and metrics come from.

  am3_live.py N line                      wall-contact task (paths: line, circle, rounded_triangle, line_fast, free, free_big)
  am3_live.py B free_big                  arm motion in free flight
  am3_live.py N loop                      figure-eight base + circling arm
  am3_live.py N loop --shape lissajous --no-wall --arm-down --base-laps 2 --arm-laps 0 --r-arm 0
  am3_live.py B loop --no-wall --arm-down --arm-laps 0 --r-arm 0 --speed 0.5     watch the baseline diverge in slow motion

--speed scales playback (0.25 = quarter speed).  Close the window or press Ctrl-C to stop early."""
import sys, os, time, argparse, numpy as np, mujoco, mujoco.viewer
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_episode import Episode
from ocrl.am3_task import BASE, make_path, XW, YZ0
from ocrl.am3_traj import LoopEpisode, down_initial_state, remove_wall, base_metrics
from ocrl.am3_eval import metrics
from am3_run import make_ctrl, free_waypoints

WALL_PATHS = ("line", "line_fast", "circle", "rounded_triangle", "line_T1", "circle_T4", "rounded_triangle_T5")


class Closed(Exception):
    """The viewer window was closed: stop the episode cleanly."""


class LiveSync:
    """Episode record hook: paces the run against the wall clock, redraws trails, refreshes the viewer and prints a status line."""
    def __init__(self, speed=1.0, trails=True, ref_path=None, status_every=0.25):
        self.viewer = None; self.speed = speed; self.trails = trails; self.ref_path = ref_path
        self.pad, self.base = [], []; self.t0 = None; self.sim0 = None; self.next_status = 0.0; self.every = status_every

    def _seg(self, scn, a, b, rgba, w=0.0015):
        if scn.ngeom >= scn.maxgeom: return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.asarray(rgba, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, np.asarray(a, float), np.asarray(b, float)); scn.ngeom += 1

    def __call__(self, ep, t):
        v = self.viewer
        if v is None: return
        if not v.is_running(): raise Closed()
        r = ep.rows[-1]; pB = np.asarray(r["ref"].get("pB", BASE), float)
        self.pad.append((r["pad"].copy(), r["F"] > 0.08)); self.base.append((r["x"][0:3].copy(), pB.copy()))
        if self.t0 is None: self.t0, self.sim0 = time.time(), t
        if self.trails:
            scn = v.user_scn; scn.ngeom = 0
            if self.ref_path is not None:
                pts = [np.r_[XW - 0.0015, YZ0 + self.ref_path.at(s)[0]] for s in np.linspace(0, self.ref_path.length, 60)]
                for a, b in zip(pts[:-1], pts[1:]): self._seg(scn, a, b, (0.1, 0.8, 0.2, 1), 0.0012)
            B = self.base[:: max(1, len(self.base) // 250)]
            for (a, ar), (b, br) in zip(B[:-1], B[1:]):
                self._seg(scn, ar, br, (0.1, 0.8, 0.2, 1)); self._seg(scn, a, b, (1.0, 0.5, 0.0, 1))
            P = self.pad[:: max(1, len(self.pad) // 250)]
            for (a, ca), (b, cb) in zip(P[:-1], P[1:]): self._seg(scn, a, b, (0.9, 0.1, 0.1, 1) if (ca and cb) else (0.2, 0.4, 1.0, 1))
        v.sync()
        if t >= self.next_status:
            self.next_status = t + self.every
            eb = np.linalg.norm(r["x"][0:3] - pB) * 1e3; epd = np.linalg.norm(r["pad"] - r["ref"]["p"]) * 1e3
            print(f"\rt {t:6.2f}s  {r['phase']:<13s} base {eb:7.2f} mm  pad {epd:7.2f} mm  Fn {r['Fm']:5.2f} N  "
                  f"tilt {np.degrees(2 * np.arcsin(min(1.0, np.hypot(r['x'][4], r['x'][5])))):5.2f} deg   ", end="", flush=True)
        target = self.t0 + (t - self.sim0) / max(self.speed, 1e-6); dt = target - time.time()
        if dt > 0: time.sleep(dt)


def main(a):
    live = LiveSync(speed=a.speed, trails=not a.no_trails, ref_path=make_path(a.path)[0] if a.path in WALL_PATHS else None)
    ctrl = make_ctrl(a.controller)
    if a.path == "loop":
        ep = LoopEpisode(ctrl, T=a.duration, base_laps=a.base_laps, arm_laps=a.arm_laps, amp_y=0.15 * a.scale, amp_z=0.05 * a.scale,
                         r_arm=a.r_arm, shape=a.shape, no_wall=a.no_wall, seed=a.seed, T_max=a.duration + 15.0, record=live,
                         x0=down_initial_state()[0] if a.arm_down else None)
    else:
        wp, seg = free_waypoints(a.path) if a.path.startswith("free") else (None, None)
        ep = Episode(ctrl, a.path, seed=a.seed, free_waypoints=wp, segment_s=seg, record=live, T_max=a.tmax)
        if a.no_wall: remove_wall(ep)
    print(f"controller {a.controller}  task {a.path}  playback {a.speed}x -- close the window to stop\n")
    with mujoco.viewer.launch_passive(ep.pl.m, ep.pl.d, show_left_ui=False, show_right_ui=False) as v:
        live.viewer = v
        v.cam.lookat[:] = (0.56, 0.0, 1.20); v.cam.distance = 1.6; v.cam.azimuth = -52; v.cam.elevation = -14
        try: ep.run()
        except (Closed, KeyboardInterrupt): print("\nstopped by user")
    m = base_metrics(ep) if a.path == "loop" else metrics(ep)
    print("\n" + ("-" * 70))
    print(f"completed : {m.get('completed')}")
    print(f"abort     : {m.get('abort')}")
    if a.path == "loop":
        print(f"base vs moving reference : RMS {m['base_track_rms_mm']:.3f} mm   max {m['base_track_max_mm']:.3f} mm")
        print(f"pad vs reference         : RMS {m['pad_track_rms_mm']:.3f} mm   max {m['pad_track_max_mm']:.3f} mm")
        print(f"tilt max {m['tilt_max_deg']:.2f} deg   rotor force {m['rotor_force_min_N']:.2f}-{m['rotor_force_max_N']:.2f} N (limits 0.1-10)")
        print(f"solver failures {m['solver_failures']}, fallbacks {m['solver_fallback']}   cycle median {m['cycle_median_ms']:.1f} ms")
    else:
        print(f"base error  : RMS {m['base']['rms_mm']:.3f} mm   max {m['base']['max_mm']:.3f} mm")
        print(f"pad vs ref  : RMS {m['pad_vs_reference']['rms_mm']:.3f} mm   max {m['pad_vs_reference']['max_mm']:.3f} mm")
        if "trace" in m: print(f"trace       : RMS {m['trace']['track_rms_mm']:.3f} mm   coverage {m['trace']['coverage_5mm']:.2f}   endpoint {m['trace']['endpoint_err_mm']:.3f} mm")
        print(f"contact     : valid {m['contact']['valid_fraction']}   losses {m['contact']['losses']}   peak Fn {m['contact']['peak_Fn_N']:.3f} N")
        print(f"tilt max {m['attitude']['tilt_max_deg']:.2f} deg   cycle median {m['timing']['median_ms']:.1f} ms, max {m['timing']['max_ms']:.1f} ms")
    print("-" * 70)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="live MuJoCo viewer for a closed-loop run")
    ap.add_argument("controller", help="B, N, Nd (D and L need their extra dependencies)")
    ap.add_argument("path", help="line, circle, rounded_triangle, free, free_big, or loop")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed, 1.0 = real time")
    ap.add_argument("--seed", type=int, default=0); ap.add_argument("--tmax", type=float, default=40.0)
    ap.add_argument("--no-trails", action="store_true"); ap.add_argument("--no-wall", action="store_true"); ap.add_argument("--arm-down", action="store_true")
    ap.add_argument("--duration", type=float, default=30.0); ap.add_argument("--base-laps", type=int, default=3)
    ap.add_argument("--arm-laps", type=int, default=10); ap.add_argument("--r-arm", type=float, default=0.05)
    ap.add_argument("--scale", type=float, default=1.0); ap.add_argument("--shape", choices=("lemniscate", "lissajous", "sine"), default="lemniscate")
    main(ap.parse_args())
