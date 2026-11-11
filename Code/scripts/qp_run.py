"""A->B with a slung load, fly each case in MuJoCo and report the time to settle.  All controllers run at 100 Hz on the same plant and attitude loop.
  U      unaware: minimum-jerk move, drone-only PD (f = f_plan + m (Kp e_p + Kd e_v)); duration swept, best one reported
  U+LQR  same move, load-aware LQR feedback (does feedback alone fix it?)
  S      minimum-time plan with swing held under 3 deg, load-aware LQR
  E      minimum-time plan with the swing free (exploited), load-aware LQR
  E-NMPC E plan, load-aware tracking NMPC (acados) instead of the LQR
  E-PD   E plan with the drone-only PD (shows the plan needs load feedback)
Settled = from that time to the end, drone within 5 cm of B AND swing under 2 deg.
Usage: qp_run.py [--dx 2.0] [--tsim 10] [--out results/payload/ab_2m.json]"""
import sys, os, json, time, argparse
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.qp_plant import Plant, Attitude, MQ, ML, L
from ocrl.qp_ocp import plan, minjerk, sample
from ocrl.qp_lqr import from_plan, DT
from ocrl.qp_nmpc import from_plan as nmpc_from_plan

KP, KD = 2.5 ** 2, 2 * 2.5          # drone-only position loop, 2.5 rad/s, critically damped


def pd(pl):
    def ctrl(t, s):
        p_r, v_r, f_r = sample(pl, t); return f_r + (MQ + ML) * (KP * (p_r - s["p"]) + KD * (v_r - s["v"]))
    return ctrl


# Realistic sensing (2026-11-11, added after review: the first runs gave every controller the exact simulator state).
# Outer loop (100 Hz): drone position / velocity / attitude / gyro noise, load direction measured to 1 deg (string-angle sensor), load velocity noise,
# command applied one period late.  Attitude loop (every plant step): its own IMU noise.  Optional wind on the drone: steady + one gust.
SIG = dict(p=0.01, v=0.05, att=np.radians(1.0), w=0.05, swing=np.radians(1.0), rv=0.05, att_imu=np.radians(0.5), w_imu=0.03)


def _rot(R, rng, sig):
    w = rng.normal(0, sig, 3); K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]]); U, _, Vt = np.linalg.svd(R @ (np.eye(3) + K)); return U @ Vt


def noisy(s, rng):
    """Measured state for the outer controller; truth stays in the plant and in the logged rows."""
    p = s["p"] + rng.normal(0, SIG["p"], 3); d = (s["r"] - s["p"]) / L; d = _rot(np.eye(3), rng, SIG["swing"]) @ d
    return dict(p=p, R=_rot(s["R"], rng, SIG["att"]), v=s["v"] + rng.normal(0, SIG["v"], 3), w=s["w"] + rng.normal(0, SIG["w"], 3), r=p + L * d, rv=s["rv"] + rng.normal(0, SIG["rv"], 3))


def fly(ctrl, A, tsim, on_step=None, posts=(), real=None):
    """real = None: exact state, no delay (the original runs).  real = dict(seed=k, wind=bool): noise and one-period delay as above, wind if asked."""
    P = Plant(A, posts); att = Attitude(); n = int(round(DT / P.dt)); rows = []; f = None; f_next = None; rng = np.random.default_rng(real["seed"]) if real else None
    if real and real.get("wind"):
        ang = rng.uniform(0, 2 * np.pi); steady = 0.3 * np.array([np.cos(ang), np.sin(ang), 0.0]); g_t0 = rng.uniform(0.3, 2.0); gust = rng.normal(0, 0.5, 3) * [1, 1, 0.3]   # N, on the drone (weight ~12 N)
    bd = P.m.body("drone").id
    for k in range(int(tsim / P.dt)):
        t = k * P.dt; s = P.state()
        if k % n == 0:
            sm = noisy(s, rng) if real else s
            u = np.asarray(ctrl(t, sm), float).copy(); u[2] = max(u[2], 1.0)          # never command downward thrust
            if real: f, f_next = (u if f_next is None else f_next), u                  # one-period computation delay
            else: f = u
            rows.append(dict(t=t, p=s["p"], r=s["r"], swing=Plant.swing_deg(s), f=f, thrust=float(np.linalg.norm(f))))
        if real and real.get("wind"): P.d.xfrc_applied[bd, :3] = steady + (gust if g_t0 <= t < g_t0 + 0.5 else 0.0)
        P.step(att(dict(s, R=_rot(s["R"], rng, SIG["att_imu"]), w=s["w"] + rng.normal(0, SIG["w_imu"], 3)) if real else s, f))
        if on_step: on_step(P, (k + 1) * P.dt)
    return rows


def metrics(rows, B, T):
    t = np.array([r["t"] for r in rows]); e = np.array([np.linalg.norm(r["p"] - B) for r in rows]); sw = np.array([r["swing"] for r in rows])
    bad = np.where((e > 0.05) | (sw > 2.0))[0]; ts = 0.0 if not len(bad) else (float(t[bad[-1] + 1]) if bad[-1] + 1 < len(t) else float("inf"))
    px = np.array([r["p"][0] for r in rows])
    return dict(plan_T=T, settle_s=ts, overshoot_cm=float((px.max() - B[0]) * 100), swing_max_deg=float(sw.max()), swing_after_plan_deg=float(sw[t >= T].max()),
                thrust_integral_Ns=float(np.sum([r["thrust"] for r in rows if r["t"] <= ts]) * DT) if np.isfinite(ts) else None)


def main(a):
    A = np.array([0.0, 0.0, 1.5]); B = A + [a.dx, 0.0, 0.0]; out = {}; plans = {}
    for kind in ("S", "E"):
        t0 = time.time(); plans[kind] = plan(A, B, kind); print(kind, f"plan T {plans[kind]['T']:.3f} s, solve {time.time() - t0:.1f} s, plan swing max {np.degrees(np.abs(plans[kind]['swing']).max()):.1f} deg", flush=True)
    sweep = {}
    for T in np.arange(1.5, 5.01, 0.25):                                     # the unaware user picks the best duration
        mj = minjerk(A, B, T); sweep[round(T, 2)] = dict(U=metrics(fly(pd(mj), A, a.tsim), B, T), ULQR=metrics(fly(from_plan(mj), A, a.tsim), B, T))
        print(f"min-jerk T {T:.2f}: U settle {sweep[round(T, 2)]['U']['settle_s']:.2f}  U+LQR settle {sweep[round(T, 2)]['ULQR']['settle_s']:.2f}", flush=True)
    bestU = min(sweep, key=lambda T: sweep[T]["U"]["settle_s"]); bestL = min(sweep, key=lambda T: sweep[T]["ULQR"]["settle_s"])
    cases = dict(U=minjerk(A, B, bestU), ULQR=minjerk(A, B, bestL), S=plans["S"], E=plans["E"], ENMPC=plans["E"], EPD=plans["E"])
    for c, pl in cases.items():
        rows = fly(pd(pl) if c in ("U", "EPD") else nmpc_from_plan(pl) if c == "ENMPC" else from_plan(pl), A, a.tsim); m = metrics(rows, B, pl["T"])
        out[c] = dict(metrics=m, plan={k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in pl.items()}, rows=[{k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in r.items()} for r in rows])
        print(c, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()}, flush=True)
    os.makedirs(os.path.dirname(a.out), exist_ok=True); json.dump(dict(A=A.tolist(), B=B.tolist(), sweep=sweep, **out), open(a.out, "w"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--dx", type=float, default=2.0); ap.add_argument("--tsim", type=float, default=10.0); ap.add_argument("--out", default=os.path.join(HERE, "..", "results", "payload", "ab_2m.json")); main(ap.parse_args())
