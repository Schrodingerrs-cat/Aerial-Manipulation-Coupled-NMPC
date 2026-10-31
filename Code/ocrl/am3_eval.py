"""Episode metrics and pass criteria (Plan s9.2-9.3).  Metrics come from the closed-loop log (control rate) and the plant's physics-rate log (forces, torques, rotor speeds).  Criteria are read from a config file
(draft until frozen).  No orientation, alignment or force-target metric exists: forces are reported and bounded, tilt is reported as response."""
import json, numpy as np
from ocrl.am3_task import BASE, TASK, make_path, YZ0
from ocrl.am3_path import reference
from ocrl.am3_plant import OM_MIN, OM_MAX, KF, TAU_C

def _rms(a): return float(np.sqrt(np.mean(np.square(a)))) if len(a) else float("nan")

def metrics(ep, F_off=0.08):
    R, pl, task = ep.rows, ep.pl, ep.task; t = np.array([r["t"] for r in R]); X = np.array([r["x"] for r in R]); ph = np.array([r["phase"] for r in R]); pad = np.array([r["pad"] for r in R]); ref = np.array([r["ref"]["p"] for r in R])
    F = np.array([r["F"] for r in R]); ft = np.array([r["ft"] for r in R]); com = np.array([r["com"] for r in R]); cyc = np.array([r["cycle_s"] for r in R]); contact_phase = np.isin(ph, ["settle", "settle_resume", "trace", "endhold"]) & (not ep.task.free)
    e_base = np.linalg.norm(X[:, 0:3] - BASE, axis=1); tilt = np.degrees(2 * np.arcsin(np.minimum(1, np.hypot(X[:, 4], X[:, 5])))); m = dict(controller=getattr(ep.ctrl, "name", "?"), path=ep.name, seed=ep.seed, abort=ep.abort, t_end=float(t[-1]), wall_s=ep.wall_s)
    m["base"] = dict(rms_mm=_rms(e_base) * 1e3, max_mm=float(e_base.max() * 1e3), final_mm=float(e_base[-1] * 1e3), vel_max=float(np.linalg.norm(X[:, 7:10], axis=1).max()))
    m["base"]["settle_s"] = float(t[np.where(e_base > 5e-3)[0].max()] if (e_base > 5e-3).any() else 0.0)
    m["com"] = dict(range_mm=((com.max(0) - com.min(0)) * 1e3).tolist(), rms_dev_mm=_rms(np.linalg.norm(com - com.mean(0), axis=1)) * 1e3)
    m["attitude"] = dict(tilt_max_deg=float(tilt.max()), tilt_rms_deg=_rms(tilt), omega_max=float(np.linalg.norm(X[:, 10:13], axis=1).max()))
    e_pad = np.linalg.norm(pad - ref, axis=1); m["pad_vs_reference"] = dict(rms_mm=_rms(e_pad) * 1e3, max_mm=float(e_pad.max() * 1e3))
    tr = ph == "trace"
    if not ep.task.free and tr.any():
        path, T = make_path(ep.name); t0 = t[tr][0]; tau = np.clip(t[tr] - t0, 0, T); nom = np.array([np.r_[0.0, YZ0 + reference(path, T, a)[0]] for a in tau]); d = pad[tr][:, 1:3] - nom[:, 1:3]
        dT = np.linalg.norm(pad[tr] - ref[tr], axis=1); m["trace"] = dict(track_rms_mm=_rms(dT) * 1e3, track_max_mm=float(dT.max() * 1e3), timed_nominal_rms_mm=_rms(np.linalg.norm(d, axis=1)) * 1e3, timed_nominal_max_mm=float(np.linalg.norm(d, axis=1).max() * 1e3), duration_s=float(t[tr][-1] - t0), nominal_T=T, delay_s=float(t[tr][-1] - t0 - T), x_dev_rms_mm=_rms(pad[tr][:, 0] - ref[tr][:, 0]) * 1e3)
        touch = (np.isin(ph, ["trace", "endhold"])) & (F > F_off)                                       # samples where the pad is physically pressing the wall during the trace
        pts = np.array([YZ0 + path.at(s)[0] for s in np.linspace(0, path.length, 60)]); dist = np.linalg.norm(pts[:, None, :] - pad[touch][None, :, 1:3], axis=2).min(1) if touch.any() else np.full(60, 1.0); m["trace"]["coverage_5mm"] = float((dist < 5e-3).mean())
        end = np.where(ph == "endhold")[0]; m["trace"]["endpoint_err_mm"] = float(np.linalg.norm(pad[end[-1]][1:3] - (YZ0 + path.at(path.length)[0])) * 1e3) if len(end) else float("nan"); m["trace"]["reached_end"] = bool(len(end))
    L = pl.log; Fn = np.array(L["Fn"]); tp = np.array(L["t"]); tt = np.array(L["tau"]); Om = np.array(L["Omega"]); ftl = np.array(L["ft"])
    # contact-intended window: from the first settle sample to the end of the last endhold sample, recoveries included; losses are counted at physics rate over the whole contiguous window
    win = np.flatnonzero(np.isin(ph, ["settle", "settle_resume", "trace", "endhold", "recover"]) & (not ep.task.free)); w0, w1 = (t[win[0]], t[win[-1]]) if len(win) else (0.0, -1.0); cp = (tp >= w0) & (tp <= w1); cf = Fn[cp] > F_off
    edges = np.flatnonzero(np.diff(np.r_[0, cf.astype(int), 0])); runs = (edges[1::2] - edges[0::2]).tolist() if cf.any() else []; gaps = (edges[2::2] - edges[1:-1:2]).tolist() if len(edges) > 2 else []
    m["contact"] = dict(valid_fraction=float(cf.mean()) if cp.any() else float("nan"), longest_s=float(max(runs) * pl.dt) if runs else 0.0, detached_intervals_s=[float(g * pl.dt) for g in gaps], detached_total_s=float(sum(gaps) * pl.dt), losses=len(task.recoveries), recovery_s=float(sum(r.get("duration_s", 0.0) for r in task.recoveries)), recoveries=task.recoveries,
                        peak_Fn_N=float(Fn.max()), impulse_Ns=float(Fn.sum() * pl.dt), peak_ft_N=float(np.linalg.norm(ftl, axis=1).max()), Fn_mean_trace_N=float(F[tr].mean()) if tr.any() else float("nan"), Fn_std_trace_N=float(F[tr].std()) if tr.any() else float("nan"),
                        unexpected=sorted(map(list, pl.other_names)), unexpected_first=pl.other_first)
    f = KF * Om ** 2; m["actuation"] = dict(tau_util_max=(np.abs(tt).max(0) / TAU_C).tolist(), rotor_force_min_N=float(f.min()), rotor_force_max_N=float(f.max()), rotor_reserve_up_N=float(10.0 - f.max()), rotor_reserve_down_N=float(f.min() - 0.1),
                         qd_max=np.abs(X[:, 16:19]).max(0).tolist(), cmd_clip_fraction=float(np.mean([(np.abs(r["u_cmd"][4:] - r["u_app"][4:]) > 0).any() or (r["u_cmd"][:4] >= OM_MAX - 1e-6).any() or (r["u_cmd"][:4] <= OM_MIN + 1e-6).any() for r in R])))
    inf = [r["info"] for r in R]; m["solver"] = dict(failures=sum(1 for i in inf if i.get("ok") is False), retries=sum(1 for i in inf if i.get("retry")), fallback=sum(1 for i in inf if i.get("fallback")), slack_max=float(max([i.get("slack", 0.0) for i in inf] + [0.0])))
    m["timing"] = dict(median_ms=float(np.median(cyc) * 1e3), p95_ms=float(np.percentile(cyc, 95) * 1e3), p99_ms=float(np.percentile(cyc, 99) * 1e3), max_ms=float(cyc.max() * 1e3), period_ms=float(getattr(ep.ctrl, "period", 0.02) * 1e3), over_period_fraction=float(np.mean(cyc > getattr(ep.ctrl, "period", 0.02))), injected_misses=int(sum(r["missed"] for r in R)))
    m["final_gap_mm"] = float(R[-1]["gap"] * 1e3) if not ep.task.free else None
    m["completed"] = bool(ep.abort is None and task.phase == "done"); return m

def passes(m, crit):
    """Pass flags against criteria dict (see config/am3_acceptance_*.json).  Functional pass only; timing is reported separately."""
    c, out = crit, {}
    out["no_abort"] = m["abort"] is None and m["completed"]; out["base_max"] = m["base"]["max_mm"] <= c["base_max_mm"]; out["tilt"] = m["attitude"]["tilt_max_deg"] <= c["tilt_max_deg"]
    free = m["path"].startswith("free")
    if free: out["free_pad_tracking_rms"] = m["pad_vs_reference"]["rms_mm"] <= c["pad_track_rms_mm"]
    else: out["trace_metrics_present"] = "trace" in m and not np.isnan(m["contact"]["valid_fraction"]); out["retreat_clear"] = (m["final_gap_mm"] or 0.0) >= c["final_gap_mm_min"]
    if "trace" in m: out["path_tracking_rms"] = m["trace"]["track_rms_mm"] <= c["pad_track_rms_mm"]; out["coverage"] = m["trace"]["coverage_5mm"] >= c["coverage"]; out["endpoint"] = m["trace"]["endpoint_err_mm"] <= c["endpoint_mm"]
    k = m["contact"]; out["contact_fraction"] = (k["valid_fraction"] >= c["contact_valid_fraction"]) if not np.isnan(k["valid_fraction"]) else free; out["force_peak"] = k["peak_Fn_N"] <= c["force_peak_N"]; out["no_unexpected_contact"] = len(k["unexpected"]) == 0
    out["recovery"] = k["losses"] <= c["max_losses"]; out["rotor_reserve"] = m["actuation"]["rotor_reserve_down_N"] >= c["rotor_reserve_N"] and m["actuation"]["rotor_reserve_up_N"] >= c["rotor_reserve_N"]; out["solver"] = m["solver"]["failures"] == 0 or m["solver"]["fallback"] == 0
    out["all"] = bool(all(out.values())); return out
