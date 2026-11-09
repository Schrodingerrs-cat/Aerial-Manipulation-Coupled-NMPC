"""Oracle projection error of the two-channel correction interface (Plan s8).

The NMPC predicts with the nominal model; the true plant carries a mismatch.  For the prediction to match reality the correction would have to supply the generalised force

    dtau = M_nom (nudot_true - nudot_nom)          (9-vector: base force 3, base moment 3, joint torque 3)

but the interface can only supply forces in the 2-dimensional span

    D(x) = [ J_C^T n , E_pitch ]                   (contact-normal force, body pitch moment)

so the question is how much of dtau lies inside that span.  Whatever lies outside it cannot be produced by ANY corrector - classical or learned - so this is an upper bound on what D or L could ever achieve.

Metric: generalised forces of mixed units are compared in the kinetic (M^-1) metric, <a,b> = a^T M_nom^-1 b, which is unit-consistent and is the natural inner product for generalised forces.  Two bounds are reported:
unconstrained least squares (pure span question) and least squares subject to |d| <= D_MAX (what the interface is actually allowed to command).  The block breakdown of the residual says WHERE the uncorrectable part lives.

Usage: am3_projection.py [--run verify1027_N_line] [--samples 40] [--draws 6]   ->  results/am3_projection/projection.json
"""
import sys, os, json, argparse, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_plant import AMPlant3, KF, servo_limit, CFG
from ocrl.am3_pred import AMPred3
from ocrl.am3_nmpc import D_MAX
from ocrl.am3_mismatch import FAM, sample_mismatch

PRED = AMPred3()


def qacc_of(plant, x, u, t=None):
    """Acceleration the plant produces at state x under command u, without stepping: the same ctrl mapping am3_plant.step applies."""
    d = plant.d; plant.set_state(x)
    d.ctrl[:4] = KF * plant.kf_scale * np.asarray(x[19:23], float) ** 2
    d.ctrl[4:7] = servo_limit(np.asarray(u[4:7], float), d.qvel[plant.vadr])
    d.xfrc_applied[:] = 0.0
    if t is not None:                       # external base disturbance, evaluated at the time it is active
        plant.d.time = t; plant._disturb()
    mujoco.mj_forward(plant.m, d)
    return d.qacc.copy()


def channels(x):
    """D(x): columns are the contact-normal force channel and the body pitch-moment channel."""
    JC = np.array(PRED.JC_f(x)[0]); n = np.array([-1.0, 0.0, 0.0])
    c0 = JC.T @ n; c1 = np.zeros(9); c1[4] = 1.0
    return np.column_stack([c0, c1])


def project(dtau, D, Minv):
    """Least squares in the M^-1 metric, unconstrained and with |d| <= D_MAX.  Returns (d_free, d_bounded, residual vectors)."""
    A = D.T @ Minv @ D; b = D.T @ Minv @ dtau
    d_free = np.linalg.solve(A, b) if np.linalg.cond(A) < 1e12 else np.linalg.lstsq(A, b, rcond=None)[0]
    # 2-D box-constrained QP: the unconstrained point if feasible, else the best point on the box boundary
    if np.all(np.abs(d_free) <= D_MAX): d_box = d_free
    else:
        best, bval = None, np.inf
        for i in (0, 1):                                      # clamp one channel, optimise the other
            for s in (-1.0, 1.0):
                d = np.empty(2); d[i] = s * D_MAX[i]; j = 1 - i
                num = b[j] - A[j, i] * d[i]
                d[j] = np.clip(num / A[j, j], -D_MAX[j], D_MAX[j])
                r = dtau - D @ d; v = float(r @ Minv @ r)
                if v < bval: bval, best = v, d.copy()
        d_box = best
    return d_free, d_box, dtau - D @ d_free, dtau - D @ d_box


def norm_M(v, Minv): return float(np.sqrt(max(v @ Minv @ v, 0.0)))


def main(a):
    log = np.load(os.path.join(HERE, "..", "results", "am3_runs", a.run, "log.npz"), allow_pickle=True)
    X, U, PH = log["x"], log["u_cmd"], log["phase"]
    idx = [i for i, p in enumerate(PH) if p in ("trace", "endhold", "settle")]          # states where the task is actually being performed, in contact
    if len(idx) < a.samples: idx = list(range(len(X)))
    idx = np.array(idx)[np.linspace(0, len(idx) - 1, a.samples).astype(int)]
    print(f"{len(idx)} sampled states from run '{a.run}' (phases: {sorted(set(PH[i] for i in idx))})\n")

    nom = AMPlant3(); rng = np.random.default_rng(7); out = {}
    for fam in FAM:
        free_r, box_r, blocks, mags, used = [], [], [], [], []
        for k in range(a.draws):
            mm = sample_mismatch(rng, fam)
            pl = AMPlant3(k=CFG["contact"]["k_N_m"] * mm.get("k_scale", 1.0), c=CFG["contact"]["c_Ns_m"] * mm.get("c_scale", 1.0),
                          mu=CFG["contact"]["mu"] * mm.get("mu_scale", 1.0))
            pl.apply_mismatch(mm)
            tdist = None
            if "disturbance" in mm:                                                       # evaluate while the disturbance is actually on
                e = mm["disturbance"][0]; tdist = e["t0"] + (0.25 * e.get("dur", 1.0) if e["kind"] == "step" else 0.25 / e["freq"])
            for i in idx:
                x, u = np.asarray(X[i], float), np.asarray(U[i], float)
                dtau = np.array(PRED.M_f(x)) @ (qacc_of(pl, x, u, tdist) - qacc_of(nom, x, u))
                M = np.array(PRED.M_f(x)); Minv = np.linalg.inv(M); D = channels(x)
                _, d_box, r_free, r_box = project(dtau, D, Minv)
                tot = norm_M(dtau, Minv)
                if tot < 1e-9: continue
                free_r.append(norm_M(r_free, Minv) / tot); box_r.append(norm_M(r_box, Minv) / tot)
                blocks.append([np.linalg.norm(dtau[0:3]), np.linalg.norm(dtau[3:6]), np.linalg.norm(dtau[6:9])])
                mags.append(tot); used.append(np.abs(d_box) / D_MAX)
        bl = np.array(blocks).mean(0); bl = bl / max(bl.sum(), 1e-12)
        out[fam] = dict(n=len(free_r), resid_unbounded_med=float(np.median(free_r)), resid_bounded_med=float(np.median(box_r)),
                        resid_bounded_p90=float(np.percentile(box_r, 90)), dtau_norm_med=float(np.median(mags)),
                        block_share=dict(base_force=float(bl[0]), base_moment=float(bl[1]), joint_torque=float(bl[2])),
                        channel_saturation_med=[float(v) for v in np.median(np.array(used), 0)])
        print(f"{fam:20s} unrepresentable (span only) {out[fam]['resid_unbounded_med']*100:5.1f} %   "
              f"after |d|<=D_MAX {out[fam]['resid_bounded_med']*100:5.1f} %   "
              f"error lives in: force {bl[0]*100:4.1f} / moment {bl[1]*100:4.1f} / joint {bl[2]*100:4.1f} %")

    od = os.path.join(HERE, "..", "results", "am3_projection"); os.makedirs(od, exist_ok=True)
    json.dump(dict(run=a.run, n_states=len(idx), draws=a.draws, D_MAX=D_MAX.tolist(),
                   metric="kinetic (M^-1) norm of the generalised-force residual", families=out),
              open(os.path.join(od, "projection.json"), "w"), indent=1)
    print(f"\nwritten: Code/results/am3_projection/projection.json")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--run", default="verify1027_N_line")
    ap.add_argument("--samples", type=int, default=40); ap.add_argument("--draws", type=int, default=6)
    main(ap.parse_args())
