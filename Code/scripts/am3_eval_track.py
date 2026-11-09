"""Held-out evaluation of base-trajectory tracking (R2) on FIXED named reference trajectories, against two references on identical episodes.

  zero   zero action: the floor
  PD     scripted outer-loop PD with reference-acceleration feedforward, through the same action interface: the bar a learned policy has to clear

Trajectories are the Lissajous family the NMPC demos used (ocrl/am3_traj.py LoopTask), with the arm held still.  Error is the position error to the MOVING reference, RMS over the run after a
2 s settling window.  Starts differ per episode seed (offset, velocity, attitude), identically across policies.  Training draws random trajectories; these named ones are fixed, and
`fig8_slow` is the figure-eight of the NMPC video (3 laps in 30 s, +-15 cm x +-5 cm).

Usage: am3_eval_track.py results/am3_rl/R2_x/model_final.zip [more.zip] [--n 8] [--refs fig8_slow,fig8_fast,liss_mid]
"""
import sys, os, json, argparse
import numpy as np
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_rl_direct import TrackEnv, Track3Env, Track4Env, Track5Env, Track6Env, peak_accel, peak_accel3

REFS = dict(
    fig8_slow=dict(Ay=0.15, Az=0.05, T=30.0, Tr=2.5, laps=3, shape="lemniscate"),     # the NMPC video figure-eight
    fig8_fast=dict(Ay=0.15, Az=0.05, T=15.0, Tr=2.5, laps=3, shape="lemniscate"),     # twice the speed
    liss_mid=dict(Ay=0.10, Az=0.05, T=20.0, Tr=2.5, laps=2, shape="lissajous"),
)

PI = np.pi
def _r3(T, laps, **k):
    d = dict(T=T, Tr=2.5, laps=laps, Ax=0.0, Ay=0.0, Az=0.0, nx=1, ny=1, nz=1, px=0.0, py=0.0, pz=0.0); d.update(k); return d
REFS3 = dict(                                                                   # large 3-D figures, all six directions; start and end at the hover point
    big_fig8=_r3(30, 2, Ay=0.5, Az=0.2, nz=2),                                  # wide figure-eight in y-z, +-0.5 m x +-0.2 m
    circle_xy=_r3(30, 2, Ax=0.3, Ay=0.4, px=PI / 2),                            # circle in the horizontal plane (forward/back + left/right), +-0.3 x +-0.4 m
    fwd_up=_r3(24, 2, Ax=0.35, Az=0.25, pz=PI / 2),                             # forward/back with up/down, +-0.35 x +-0.25 m
    lissa3d=_r3(35, 2, Ax=0.25, Ay=0.4, Az=0.15, nx=3, ny=2, nz=3, px=PI / 2, pz=PI / 4),   # full 3-D Lissajous
)
REFS.update(REFS3)


def rollout(env, policy, seed):
    obs, _ = env.reset(seed=seed); err, tilt, dA, ev, ej = [], [], [], [], []; a_prev = np.zeros(10); terminated = False
    while True:
        if policy == "zero": a = np.zeros(10)
        elif policy == "pd": a = env.expert_action(env.pl.state())
        else: a = policy.predict(obs, deterministic=True)[0]
        obs, r, term, trunc, info = env.step(a); err.append(info["e"]); ev.append(env.pl.state()[0:3] - env.pref); tilt.append(info["tilt"]); ej.append(info.get("ej", 0.0)); dA.append(np.linalg.norm(np.asarray(a) - a_prev)); a_prev = np.asarray(a)
        if term: terminated = True; break
        if trunc: break
    e = np.array(err); k0 = int(2.0 / env.dt_c); body = e[k0:] if len(e) > k0 else e
    return dict(seed=seed, steps=len(e), survived=not terminated, rms_mm=float(np.sqrt(np.mean(body ** 2)) * 1e3), max_mm=float(body.max() * 1e3),
                tilt_max=float(np.max(tilt)), joint_rms_deg=float(np.degrees(np.sqrt(np.mean(np.array(ej[k0:]) ** 2)))) if len(ej) > k0 else 0.0, rough=float(np.mean(dA)), bias_mm=(np.mean(np.array(ev)[k0:], axis=0) * 1e3).tolist() if len(ev) > k0 else [0, 0, 0])


def main(a):
    a.refs = a.refs or (",".join(REFS3) if a.env in ("track3", "track4", "track5", "track6") else "fig8_slow,fig8_fast,liss_mid")
    Env, pa = {"track3": (Track3Env, peak_accel3), "track4": (Track4Env, peak_accel3), "track5": (Track5Env, peak_accel3), "track6": (Track6Env, peak_accel3)}.get(a.env, (TrackEnv, peak_accel))
    pols = {"zero": "zero", "PD+ff expert": "pd"}
    for p in a.models: pols[os.path.basename(os.path.dirname(p)) + "/" + os.path.basename(p)] = PPO.load(p, device="cpu")
    out = {}
    for rname in a.refs.split(","):
        env = Env(seed=0, ref=REFS[rname]); res = {}
        for name, pol in pols.items(): res[name] = [rollout(env, pol, a.start + k) for k in range(a.n)]
        out[rname] = res
        print(f"\n=== {rname}: {REFS[rname].get('shape', '3D')}, {REFS[rname]['laps']} laps in {REFS[rname]['T']:.0f} s, peak ref accel {pa(REFS[rname]):.2f} m/s^2   ({a.n} starts) ===")
        print(f"{'policy':36s} {'survived':>9s} {'RMS err mm':>11s} {'median':>8s} {'max err mm':>11s} {'tilt max':>9s} {'roughness':>10s} {'bias x,y,z mm':>20s}")
        for name, rows in res.items():
            ok = [r for r in rows if r["survived"]]
            print(f"{name:36s} {len(ok):>6d}/{len(rows)} {(np.mean([r['rms_mm'] for r in ok]) if ok else float('nan')):>11.1f} {(np.median([r['rms_mm'] for r in ok]) if ok else float('nan')):>8.1f} "
                  f"{(np.mean([r['max_mm'] for r in ok]) if ok else float('nan')):>11.1f} {np.mean([r['tilt_max'] for r in rows]):>8.1f}° {np.mean([r['rough'] for r in rows]):>10.3f} {str(np.round(np.mean([r['bias_mm'] for r in ok], axis=0), 1)) if ok else '':>20s}")
    od = os.path.join(HERE, "..", "results", "am3_rl", "eval"); os.makedirs(od, exist_ok=True)
    json.dump(out, open(os.path.join(od, a.out), "w"), indent=1); print(f"\nwritten results/am3_rl/eval/{a.out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("models", nargs="*"); ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--refs", default=None); ap.add_argument("--start", type=int, default=100); ap.add_argument("--env", default="track", choices=("track", "track3", "track4", "track5", "track6")); ap.add_argument("--out", default="r2_eval.json"); main(ap.parse_args())
