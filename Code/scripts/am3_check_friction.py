"""Sliding-contact friction check from closed-loop logs: tangential/normal force ratio of the pad against the pad's tangential speed during the trace (plant truth), including direction reversals (circle,
rounded triangle).  Compares with mu (0.15).  Usage: am3_check_friction.py RUNDIR [RUNDIR ...]  -> results/am3_predictor/friction_check.json"""
import sys, os, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); out = {}
for d in sys.argv[1:]:
    L = np.load(os.path.join(d, "log.npz"), allow_pickle=True); t = L["t"]; ph = np.array([str(p) for p in L["phase"]]); pad = L["pad"]; F = L["F"]; ft = L["ft"]
    v = np.gradient(pad[:, 1:3], t, axis=0); sp = np.linalg.norm(v, axis=1); tr = (ph == "trace") & (F > 0.1); ratio = np.linalg.norm(ft, axis=1) / np.maximum(F, 1e-6); bins = [0, 0.005, 0.01, 0.02, 0.04, 1.0]; rows = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        k = tr & (sp >= lo) & (sp < hi)
        if k.sum() > 10: rows.append(dict(speed_mm_s=[lo * 1e3, hi * 1e3], n=int(k.sum()), ratio_median=float(np.median(ratio[k])), ratio_p10=float(np.percentile(ratio[k], 10)), ratio_p90=float(np.percentile(ratio[k], 90))))
    # direction: friction force should oppose the velocity
    k = tr & (sp > 0.01); cosang = np.sum(ft[k] * v[k], axis=1) / (np.linalg.norm(ft[k], axis=1) * sp[k] + 1e-12); out[os.path.basename(d)] = dict(bins=rows, opposes_velocity_median_cos=float(np.median(cosang)) if k.any() else None, mu=0.15)
    print(os.path.basename(d), json.dumps(out[os.path.basename(d)], indent=1))
json.dump(out, open(os.path.join(HERE, "..", "results", "am3_predictor", "friction_check.json"), "w"), indent=1)
