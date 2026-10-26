"""Run one closed-loop episode and store everything needed to audit it: results/am3_runs/<tag>/{summary.json, log.npz, config.json}.
Usage: am3_run.py TAG CONTROLLER PATH [--seed S] [--free] [--tmax T] [--miss P]   CONTROLLER in {B, N, Nd} (Nd = NMPC with decoupled model)   PATH: a path name from config/am3_task_v1.json or `free`"""
import sys, os, json, time, hashlib, argparse, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_episode import Episode, initial_state
from ocrl.am3_eval import metrics
from ocrl.am3_pred import AMPred3
FREE_WPS = None
def make_ctrl(name):
    if name == "D":
        from ocrl.am3_ctrl_corr import CorrectedNMPC, ZeroCorrector; from ocrl.am3_ekf import ClassicalCorrector
        n = CorrectedNMPC(ZeroCorrector(), "D"); r = os.path.join(HERE, "..", "results", "am3_predictor", "ekf_residuals.json")
        qs = None
        if os.path.exists(r): e = json.load(open(r)); qs = np.r_[[e["p"]["sigma_mad"]] * 3, [e["att"]["sigma_mad"]] * 3, [e["v"]["sigma_mad"]] * 3, [e["w"]["sigma_mad"]] * 3, [e["q"]["sigma_mad"]] * 3, [e["qd"]["sigma_mad"]] * 3, [e["Om"]["sigma_mad"]] * 4]
        n.corrector = ClassicalCorrector(n.form.pr, n.form.step_d, q_state=qs); return n
    if name.startswith("L:"):
        from stable_baselines3 import SAC; from ocrl.am3_ctrl_corr import CorrectedNMPC; from ocrl.am3_rl import PolicyCorrector
        n = CorrectedNMPC(None, "L"); n.corrector = PolicyCorrector(SAC.load(os.path.join(HERE, "..", name[2:]), device="cpu"), n.form.pr); return n
    if name == "Z":
        from ocrl.am3_ctrl_corr import CorrectedNMPC, ZeroCorrector; return CorrectedNMPC(ZeroCorrector(), "Z")
    if name == "B": from ocrl.am3_ctrl_b import BaselineB; return BaselineB()
    from ocrl.am3_ctrl_n import NMPC; return NMPC("full" if name == "N" else "decoupled")
def free_waypoints(kind="free", gap0=0.06):
    x0, _ = initial_state(gap0); p0 = np.array(AMPred3().pad_f(x0)[0]).ravel()
    if kind == "free_big": return [p0 + d for d in ([0.02, 0.08, 0.04], [0.0, -0.08, 0.04], [0.03, -0.08, -0.04], [0.0, 0.08, -0.04], [0, 0, 0])], 1.2
    return [p0 + [0, 0.06, 0.03], p0 + [0.02, -0.06, 0.03], p0 + [0, 0, -0.03], p0], None
def run(tag, ctrl_name, path, seed=0, tmax=40.0, miss_p=0.0, outdir=None, record=None, gap0=0.06, dyz=(0.0, 0.0), enforce_deadline=False, mismatch=None):
    rng = np.random.default_rng(1000 + seed); miss = (lambda k: bool(rng.random() < miss_p)) if miss_p > 0 else None
    wp, seg = free_waypoints(path) if path.startswith("free") else (None, None); ep = Episode(make_ctrl(ctrl_name), path, seed=seed, T_max=tmax, free_waypoints=wp, segment_s=seg, miss=miss, record=record, x0=initial_state(gap0, (0.0 + dyz[0], 1.04 + dyz[1])), enforce_deadline=enforce_deadline, mismatch=mismatch).run(); m = metrics(ep)
    out = outdir or os.path.join(HERE, "..", "results", "am3_runs", tag); os.makedirs(out, exist_ok=True); R = ep.rows
    np.savez_compressed(os.path.join(out, "log.npz"), t=[r["t"] for r in R], x=[r["x"] for r in R], xm=[r["xm"] for r in R], F=[r["F"] for r in R], Fm=[r["Fm"] for r in R], ft=[r["ft"] for r in R], pad=[r["pad"] for r in R], ref=[r["ref"]["p"] for r in R], phase=[r["phase"] for r in R],
                        u_cmd=[r["u_cmd"] for r in R], u_app=[r["u_app"] for r in R], com=[r["com"] for r in R], cycle_s=[r["cycle_s"] for r in R], physics_t=ep.pl.log["t"], physics_Fn=ep.pl.log["Fn"], physics_tau=ep.pl.log["tau"], physics_Omega=ep.pl.log["Omega"])
    src = hashlib.sha1(b"".join(open(os.path.join(HERE, "..", "ocrl", f), "rb").read() for f in sorted(os.listdir(os.path.join(HERE, "..", "ocrl"))) if f.startswith("am3_") and f.endswith(".py"))).hexdigest()[:12]
    json.dump(dict(m, command=" ".join(sys.argv), source_hash=src, miss_p=miss_p), open(os.path.join(out, "summary.json"), "w"), indent=1, default=str); return ep, m
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("controller"); ap.add_argument("path"); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--tmax", type=float, default=40.0); ap.add_argument("--miss", type=float, default=0.0); a = ap.parse_args()
    ep, m = run(a.tag, a.controller, a.path, a.seed, a.tmax, a.miss); print(json.dumps({k: m[k] for k in ("controller", "path", "abort", "completed", "base", "pad_vs_reference", "trace", "contact", "timing") if k in m}, default=str, indent=1))
