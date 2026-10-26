"""Empirical process-noise levels for the EKF (D): one-step prediction residual (measured next state minus nominal-model prediction from the measured state and the applied command) per tangent channel, over logged
closed-loop runs of N on the mismatch DEVELOPMENT list, contact phases and free flight.  Usage: am3_ekf_tune.py RUN_TAG  (e.g. mmdev_N) -> results/am3_predictor/ekf_residuals.json"""
import sys, os, json, glob, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_pred import AMPred3
from ocrl.am3_nmpc import Formulation
step = Formulation(AMPred3()).step; res = []
for f in glob.glob(os.path.join(HERE, "..", "results", "am3_runs", sys.argv[1], "*", "log.npz")):
    L = np.load(f, allow_pickle=True); xm, ua = L["xm"], L["u_app"]
    for k in range(len(xm) - 1):
        xp = np.array(step(xm[k], ua[k])).ravel(); xb = xm[k + 1]; Q, Qr = xp[3:7], xb[3:7]
        qe = 2 * np.array([Q[0] * Qr[1] - Q[1] * Qr[0] - Q[2] * Qr[3] + Q[3] * Qr[2], Q[0] * Qr[2] + Q[1] * Qr[3] - Q[2] * Qr[0] - Q[3] * Qr[1], Q[0] * Qr[3] - Q[1] * Qr[2] + Q[2] * Qr[1] - Q[3] * Qr[0]]) * np.sign(Q @ Qr)
        res.append(np.r_[xb[0:3] - xp[0:3], qe, xb[7:23] - xp[7:23]])
res = np.array(res); names = ["p"] * 3 + ["att"] * 3 + ["v"] * 3 + ["w"] * 3 + ["q"] * 3 + ["qd"] * 3 + ["Om"] * 4; out = {"n": int(len(res))}
for g in ("p", "att", "v", "w", "q", "qd", "Om"):
    idx = [i for i, n in enumerate(names) if n == g]; r = res[:, idx]; out[g] = dict(rms=float(np.sqrt((r ** 2).mean())), p95=float(np.percentile(np.abs(r), 95)), sigma_mad=float(1.4826 * np.median(np.abs(r)))); print(f"{g:4s} rms {out[g]['rms']:.3e}  p95 {out[g]['p95']:.3e}  robust sigma {out[g]['sigma_mad']:.3e}")
json.dump(out, open(os.path.join(HERE, "..", "results", "am3_predictor", "ekf_residuals.json"), "w"), indent=1)
