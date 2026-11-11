"""A->B through a zigzag of two posts with the slung load (professor's obstacle task, 2026-11-10).  Same plant, limits, trackers and settle test as qp_run.
Posts are vertical cylinders across the straight line, offset to opposite sides, so the flight must weave.  Drone (rotor tips) and load must both clear every post.
  U  normal drone: lumped-mass plan through the posts (load assumed straight below the drone), drone-only PD; duration swept, best collision-free run reported
  S  load-aware plan, swing held under 3 deg, posts on drone and load, LQR
  E  load-aware plan, swing free, posts on drone and load, LQR
Clearance = distance to the post surface minus the body radius (rotor tips 0.19 m, load sphere 0.035 m); below 0 = hit (posts have no contact in the sim, hits are measured).
Usage: qp_obstacle.py [--video TAG] [--tsim 10]"""
import sys, os, json, time, argparse, shutil, subprocess
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from qp_run import fly, metrics, pd
from ocrl.qp_ocp import plan, R_DRONE, R_LOAD
from ocrl.qp_lqr import from_plan

A = np.array([0.0, 0.0, 1.5]); B = A + [3.0, 0.0, 0.0]
POSTS = [(1.0, -0.1, 0.2), (2.0, 0.1, 0.2)]; VIA = [(1.0, 0.45), (2.0, -0.45)]                 # VIA only seeds the optimizer (pass left of post 1, right of post 2)
N = 60                                                                                  # N = 100 lands E in a worse local optimum (3.85 s detour)


def clearance(rows):
    c = lambda key, rad: min(np.hypot(r[key][0] - x, r[key][1] - y) - R - rad for r in rows for x, y, R in POSTS)
    return dict(drone_clear_cm=100 * c("p", R_DRONE), load_clear_cm=100 * c("r", R_LOAD))


def score(rows, T):
    m = metrics(rows, B, T); m.update(clearance(rows)); m["hit"] = bool(min(m["drone_clear_cm"], m["load_clear_cm"]) < 0); return m


def main(a):
    import qp_video; real = dict(seed=0, wind=a.wind) if a.noise or a.wind else None; qp_video.REAL = real
    out = {}; plans = {}
    for k in ("S", "E"):
        t0 = time.time(); plans[k] = plan(A, B, k, N=N, posts=POSTS, via=VIA); print(k, f"plan T {plans[k]['T']:.2f} s, solve {time.time() - t0:.0f} s", flush=True)
    sweep = {}
    for T in np.arange(2.0, 6.01, 0.5):
        try: pl = plan(A, B, "U", N=N, posts=POSTS, via=VIA, T_fix=float(T))
        except RuntimeError: print(f"U T {T:.1f}: plan failed", flush=True); continue
        m = score(fly(pd(pl), A, a.tsim, posts=POSTS, real=real), T); sweep[round(float(T), 2)] = m; plans[f"U{T:.1f}"] = pl
        print(f"U T {T:.1f}: settle {m['settle_s']:.2f}  hit {m['hit']}  drone {m['drone_clear_cm']:.1f} cm  load {m['load_clear_cm']:.1f} cm", flush=True)
    ok = [T for T, m in sweep.items() if not m["hit"]]; Tb = min(ok, key=lambda T: sweep[T]["settle_s"]) if ok else None
    Tf = min(sweep, key=lambda T: abs(T - plans["E"]["T"]))
    cases = dict(U_fast=(f"Normal drone (load ignored), {Tf:.1f} s move", pd(plans[f"U{Tf:.1f}"]), Tf),
                 S=(f"Load-aware, swing suppressed, {plans['S']['T']:.1f} s plan", from_plan(plans["S"]), plans["S"]["T"]),
                 E=(f"Load-aware, swing exploited, {plans['E']['T']:.1f} s plan", from_plan(plans["E"]), plans["E"]["T"]))
    if Tb is not None: cases = dict(U_fast=cases["U_fast"], U_best=(f"Normal drone (load ignored), best: {Tb:.1f} s move", pd(plans[f"U{Tb:.1f}"]), Tb), S=cases["S"], E=cases["E"])
    lines = []
    for c, (label, ctrl, T) in cases.items():
        if a.video:
            from qp_video import record, MEDIA
            d = os.path.join(MEDIA, a.video); os.makedirs(d, exist_ok=True)
            rows, _ = record(ctrl, A, B, a.tsim, label, os.path.join(d, "frames", c), posts=POSTS, cam=([1.5, 0.0, 1.2], 3.4, 90, -55))
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "30", "-i", os.path.join(d, "frames", c, "%05d.png"), "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", os.path.join(d, f"{c}.mp4")], check=True)
        else: rows = fly(ctrl, A, a.tsim, posts=POSTS, real=real)
        m = score(rows, T); out[c] = dict(label=label, metrics=m, rows=[{k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in r.items()} for r in rows])
        lines.append(f"{c}: {label}: settled {m['settle_s']:.2f} s, hit {m['hit']}, min clearance drone {m['drone_clear_cm']:.1f} cm load {m['load_clear_cm']:.1f} cm, max swing {m['swing_max_deg']:.0f} deg, thrust {m['thrust_integral_Ns']}"); print(lines[-1], flush=True)
    if a.video:
        ins = sum([["-i", os.path.join(d, f"{c}.mp4")] for c in cases], []); n = len(cases)
        flt = "[0:v][1:v]hstack[t];[2:v][3:v]hstack[b];[t][b]vstack[v]" if n == 4 else "[0:v][1:v][2:v]hstack=inputs=3[v]"
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + ins + ["-filter_complex", flt, "-map", "[v]", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", os.path.join(d, "compare.mp4")], check=True)
        shutil.rmtree(os.path.join(d, "frames")); open(os.path.join(d, "README.txt"), "w").write(f"A {A.tolist()} -> B {B.tolist()}, posts (x, y, radius) {POSTS}, 0.2 kg load on 0.6 m string. Red cylinders = posts, green = A, red ball = B, blue dots = load trail. Video 1.0x. Sensing: {real or 'exact state'} (qp_run.SIG).\n" + "\n".join(lines) + "\n")
    json.dump(dict(A=A.tolist(), B=B.tolist(), posts=POSTS, sweep_U={str(k): v for k, v in sweep.items()}, **out), open(os.path.join(HERE, "..", "results", "payload", "obstacle_zigzag" + ("" if real is None else "_wind" if real["wind"] else "_noise") + ".json"), "w"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--video", default=None); ap.add_argument("--tsim", type=float, default=10.0); ap.add_argument("--noise", action="store_true"); ap.add_argument("--wind", action="store_true"); main(ap.parse_args())
