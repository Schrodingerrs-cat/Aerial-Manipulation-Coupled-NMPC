"""Live MuJoCo viewer: an RL policy (or the scripted PD) flies the aerial manipulator in real time and YOU push it with the mouse.

  am3_rl_live.py results/am3_rl/R1_cont_B_fine/model_final.zip                    hover policy (R1)
  am3_rl_live.py results/am3_rl/R2_track_s0/model_final.zip --env track --ref fig8_slow    trajectory policy (R2)
  am3_rl_live.py results/am3_rl/R3_big_s0/model_final.zip --env track3 --ref big_fig8   R3 policy; refs: big_fig8 circle_xy fwd_up lissa3d
  am3_rl_live.py results/am3_rl/R5_turing_s0/model_final.zip --env track5 --ref lissa3d   R5 (trajectory + moving arm)
  am3_rl_live.py pd --env hover                                                   scripted PD for comparison (pd, zero also work for track)

Push: the drone is pre-selected.  Hold Ctrl and drag with the RIGHT mouse button = force, LEFT button = torque.  Orbit/zoom as usual.
If the drone is lost (tilt > 60 deg, > 1.5 m off, or too low) the run restarts from a new random start.  Close the window to stop."""
import sys, os, time, argparse
import numpy as np
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
import mujoco, mujoco.viewer
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
from ocrl.am3_rl_direct import TrackEnv, Track3Env, Track4Env, Track5Env, Track6Env, HoverEnv
from am3_eval_track import REFS
from am3_eval_direct import pd_action


def seg(scn, a, b, rgba, w=0.0015):
    if scn.ngeom >= scn.maxgeom: return
    g = scn.geoms[scn.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, dtype=np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, np.asarray(a, float), np.asarray(b, float)); scn.ngeom += 1


def main(a):
    env = {"track": TrackEnv, "track3": Track3Env, "track4": Track4Env, "track5": Track5Env, "track6": Track6Env, "hover": HoverEnv}[a.env](seed=0, **({} if a.env == "hover" else dict(ref=REFS[a.ref])), max_steps=10 ** 6)
    pol = a.policy if a.policy in ("pd", "zero") else PPO.load(a.policy, device="cpu")
    env.pl._disturb = lambda: None          # the plant zeroes the base force every step; leave it to the viewer so mouse pushes act
    seed = a.seed; obs, _ = env.reset(seed=seed); n_ep = 1; trail = []; t_next = time.time()
    with mujoco.viewer.launch_passive(env.pl.m, env.pl.d, show_left_ui=False, show_right_ui=False) as v:
        v.cam.lookat[:] = (0.52, 0.0, 1.20); v.cam.distance = 3.2 if a.env in ("track3", "track4", "track5", "track6") else 1.9; v.cam.azimuth = 0; v.cam.elevation = -6; v.perturb.select = 1
        t_end = time.time() + a.max_seconds if a.max_seconds else None; k = 0
        while v.is_running() and (t_end is None or time.time() < t_end):
            x = env.pl.state()
            act = np.zeros(10) if pol == "zero" else (env.expert_action(x) if a.env != "hover" else pd_action(x)) if pol == "pd" else pol.predict(obs, deterministic=True)[0]
            with v.lock(): obs, r, term, trunc, info = env.step(act)
            x = env.pl.state(); trail.append((x[0:3].copy(), env.pref.copy())); trail = trail[-600:]; k += 1
            if k % 4 == 0:
                with v.lock():
                    scn = v.user_scn; scn.ngeom = 0; T = trail[::3]
                    for (p, rp), (q, rq) in zip(T[:-1], T[1:]): seg(scn, rp, rq, (0.1, 0.8, 0.2, 1)); seg(scn, p, q, (1.0, 0.5, 0.0, 1))
                v.sync(); print(f"\rrun {n_ep}  t {env.n * env.dt_c:6.1f}s  error {info['e'] * 1e3:7.1f} mm  tilt {info['tilt']:5.1f} deg   ", end="", flush=True)
            if term:
                print(f"\nlost the drone at t={env.n * env.dt_c:.1f}s (error {info['e'] * 1e3:.0f} mm, tilt {info['tilt']:.0f} deg) -> restarting")
                seed += 1; n_ep += 1; trail = []; obs, _ = env.reset(seed=seed)
            t_next += env.dt_c; d = t_next - time.time()
            if d > 0: time.sleep(d)
            else: t_next = time.time()
    print()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("policy", help="model .zip, or pd / zero"); ap.add_argument("--env", default="hover", choices=("hover", "track", "track3", "track4", "track5", "track6"))
    ap.add_argument("--ref", default="fig8_slow"); ap.add_argument("--seed", type=int, default=100); ap.add_argument("--max-seconds", type=float, default=0); main(ap.parse_args())
