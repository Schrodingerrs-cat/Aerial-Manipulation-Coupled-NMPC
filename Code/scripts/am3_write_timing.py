"""Real-time and prediction-at-contact check for the writing task (2026-11-21; promised in the progress report).
One process at a time on a quiet machine (timing is only meaningful that way), "HELLO WPI", one seed, the benchmark's settings (am3_write_bench).
Timing: per-cycle compute time of ctrl.compute (filter + observer + policy + solve, as the Episode measures it) against the controller's own
period (NMPC 20 ms, RL 10 ms); NMPC solver status, retries, fallbacks, SQP iterations.
Prediction (NMPC-based controllers only): every solve's predicted trajectory X (X[:, j] is the state at t + (j + 1) * 20 ms, the first stage
being the latency-compensated state) is compared with what the plant did at that time, in the 'trace' phase (pen writing):
  pad normal position  (wall-normal x of the pad centre)           predicted vs true
  normal force         (model contact law + d[0], the force the model believes acts; d[0] = observer/learned extra normal force)   vs true plant force
Usage: am3_write_timing.py --name NAME [--ctrls N,NDF:0.5,f,...] [--cases nominal,wall_soft] [--noise lab]"""
import sys, os, json, argparse, time
import numpy as np
from stable_baselines3 import PPO          # torch before acados
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_write import WriteEpisode
from am3_write_eval import make, write_metrics
from am3_write_mismatch import CASES
from am3_write_bench import HEIGHT, SPEED, WALL_LIMIT_S, RealSensors, LabSensors

L2W = "NDFL:" + os.path.join(HERE, "..", "results", "am3_rl", "2026-11-20_2217_L2W_lab_hybrid_s0", "model_final.zip")
R21 = os.path.join(HERE, "..", "results", "am3_rl", "R21_write15drjt_fromR19_s0", "model_final.zip")
LABEL = {"N": "N", "NDF:0.5,f": "NDF", L2W: "L2W", R21: "R21"}
J = (0, 4, 9)                                   # horizon stages reported: 20, 100, 200 ms ahead


def pct(a, q): return round(float(np.percentile(a, q)) * 1e3, 2) if len(a) else None


def run(ctrl, case, noise, seed=0, text="HELLO WPI"):
    c = make(ctrl); mm = dict(CASES[case]) if case != "nominal" else None; log = []
    if hasattr(c, "solver"):                    # record every solve (prediction + params) without changing it
        solve0 = c.solver.solve
        def solve(x0, u_prev, P, Xg, Ug, **kw):
            r = solve0(x0, u_prev, P, Xg, Ug, **kw); log.append(dict(X=r["X"].copy(), d=np.array(P[11:13, :]).copy(), ok=r["ok"])); return r
        c.solver.solve = solve
    ep = WriteEpisode(c, text, height=HEIGHT, v_write=SPEED, seed=seed, mismatch=mm); ep.wall_limit = WALL_LIMIT_S
    ep.sens = dict(real=RealSensors, lab=LabSensors).get(noise, type(ep.sens))(seed)
    t0 = time.time(); ep.run(); m = write_metrics(ep); rows = ep.rows; period = getattr(c, "period", 0.02)
    cyc = np.array([r["cycle_s"] for r in rows if not r["missed"]]); info = [r["info"] for r in rows]
    out = dict(ctrl=LABEL.get(ctrl, ctrl), case=case, noise=noise, completed=m["completed"], abort=str(m["abort"]), trace_rms_mm=m["trace_rms_mm"], wall_s=round(time.time() - t0),
               period_ms=period * 1e3, cycles=len(cyc), cycle_ms=dict(median=pct(cyc, 50), p95=pct(cyc, 95), p99=pct(cyc, 99), max=round(float(cyc.max()) * 1e3, 1), first=round(float(cyc[0]) * 1e3, 1)),
               over_period=round(float(np.mean(cyc[1:] > period)), 4), over_20ms=round(float(np.mean(cyc[1:] > 0.02)), 4))
    sv = np.array([i["solve_s"] for i in info if "solve_s" in i])
    if len(sv):
        out.update(solve_ms=dict(median=pct(sv, 50), p99=pct(sv, 99), max=round(float(sv.max()) * 1e3, 1)), status_nonzero=int(sum(i.get("status", 0) != 0 for i in info)),
                   retries=int(sum(bool(i.get("retry")) for i in info)), fallbacks=int(sum(bool(i.get("fallback")) for i in info)),
                   iters_max=int(max(i.get("iters", -1) for i in info)))
    if log and len(log) == len(rows):           # one solve per row (a retry would add one: then skip the prediction part rather than misalign)
        pr = c.pr if hasattr(c, "pr") else c.form.pr; T = len(rows); res = {j: dict(dx=[], dF=[], F=[]) for j in J}
        for k, (r, L) in enumerate(zip(rows, log)):
            if r["phase"] != "trace" or not L["ok"]: continue
            for j in J:
                if k + 1 + j >= T or rows[k + 1 + j]["phase"] != "trace": continue
                x = L["X"][:, j]; d = L["d"][:, min(j, L["d"].shape[1] - 1)]
                g, vc, Fn, ft, vC = pr.contact_f(x, np.zeros(7), d); Fp = float(Fn) + float(d[0])
                fut = rows[k + 1 + j]; res[j]["dx"].append(1e3 * (float(np.array(pr.pad_f(x)[0]).ravel()[0]) - fut["pad"][0])); res[j]["dF"].append(Fp - fut["F"]); res[j]["F"].append(fut["F"])
        out["prediction"] = {f"{20 * (j + 1)}ms": dict(n=len(v["dx"]), pad_x_bias_mm=round(float(np.mean(v["dx"])), 3), pad_x_rms_mm=round(float(np.sqrt(np.mean(np.square(v["dx"])))), 3),
                                                     F_bias_N=round(float(np.mean(v["dF"])), 3), F_rms_N=round(float(np.sqrt(np.mean(np.square(v["dF"])))), 3), F_true_mean_N=round(float(np.mean(v["F"])), 3))
                             for j, v in res.items() if v["dx"]}
    elif log: out["prediction"] = f"skipped: {len(log)} solves for {len(rows)} cycles (retries)"
    return out


def main(a):
    ctrls = [dict(L2W=L2W, R21=R21).get(s, s) for s in a.ctrls.split(";")]; res = []
    for case in a.cases.split(","):
        for ctrl in ctrls:
            o = run(ctrl, case, a.noise); res.append(o); print(json.dumps(o), flush=True)
            od = os.path.join(HERE, "..", "results", "am3_timing"); os.makedirs(od, exist_ok=True)
            json.dump(dict(cpu=open("/proc/cpuinfo").read().split("model name")[1].split("\n")[0].strip(": "), threads=os.environ.get("OMP_NUM_THREADS"), text="HELLO WPI", seed=0, rows=res),
                      open(os.path.join(od, f"{a.name}.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--name", required=True); ap.add_argument("--ctrls", default="N;NDF:0.5,f;L2W;R21"); ap.add_argument("--cases", default="nominal,wall_soft")
    ap.add_argument("--noise", default="lab"); main(ap.parse_args())
