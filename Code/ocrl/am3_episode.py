"""Closed-loop episode shared by every controller: the same plant, sensors, latency, task logic and logging.  Each control period (20 ms): noisy measurement of the true state and normal force ->
task.advance (contact state machine on measured force and measured pad gap) -> controller.compute -> command applied one period later (simulated computation latency) -> 80 plant steps of 0.25 ms.
A controller has compute(t, xm, Fm, task) -> (u (7), info dict) and optional reset(x0).  Deadline misses can be injected: on a missed period the previous command stays pending."""
import time, numpy as np
from ocrl.am3_plant import AMPlant3, WALL_X, R_PAD
from ocrl.am3_pred import AMPred3, C
from ocrl.am3_task import Task, BASE, TASK, CP
from ocrl.am3_ref import Equilibrium, quat_from_rp

DT_C = TASK["control_period_s"]          # default period (NMPC); a controller can set its own `period`

class Sensors:
    """Noisy full state (output of a high-grade estimator: mocap + IMU fusion; rotor speeds as telemetry) and a scalar normal-force sensor.  Identical for every controller."""
    SIG = dict(p=1e-4, att=np.radians(0.01), v=1e-3, w=5e-4, q=5e-5, qd=1e-3, Om=0.5, F=0.01)
    def __init__(self, seed=0, enabled=True): self.rng = np.random.default_rng(seed); self.on = enabled
    def measure(self, x, F):
        if not self.on: return x.copy(), F
        s, r = self.SIG, self.rng.standard_normal; xm = x.copy(); xm[0:3] += s["p"] * r(3); dth = s["att"] * r(3); Q = x[3:7]
        dq = 0.5 * np.array([-Q[1] * dth[0] - Q[2] * dth[1] - Q[3] * dth[2], Q[0] * dth[0] + Q[2] * dth[2] - Q[3] * dth[1], Q[0] * dth[1] - Q[1] * dth[2] + Q[3] * dth[0], Q[0] * dth[2] + Q[1] * dth[1] - Q[2] * dth[0]])
        xm[3:7] = (Q + dq) / np.linalg.norm(Q + dq); xm[7:10] += s["v"] * r(3); xm[10:13] += s["w"] * r(3); xm[13:16] += s["q"] * r(3); xm[16:19] += s["qd"] * r(3); xm[19:23] += s["Om"] * r(4); return xm, F + s["F"] * r()

def initial_state(gap0=0.06, yz=(0.0, 1.04)):
    """Hover in free flight, base at its fixed reference, pad at gap0 from the wall in front of the patch centre; rotor speeds, joints and torques at the predictor's equilibrium for that pose (no wall force)."""
    pr = AMPred3(); e = Equilibrium(pr, BASE).solve([WALL_X - R_PAD - gap0, *yz], [0, 0, 0]); return e["x"], e["u"]

class Episode:
    def __init__(self, controller, path_name, seed=0, mismatch=None, T_max=40.0, free_waypoints=None, miss=None, sensors=True, record=None, x0=None, enforce_deadline=False, segment_s=None, wall_limit_s=400.0):
        self.ctrl, self.name, self.seed, self.T_max, self.miss, self.enforce = controller, path_name, seed, T_max, miss or (lambda k: False), enforce_deadline; self.wall_limit = wall_limit_s; mm = mismatch or {}; from ocrl.am3_plant import CFG; self.pl = AMPlant3(k=CFG["contact"]["k_N_m"] * mm.get("k_scale", 1.0), c=CFG["contact"]["c_Ns_m"] * mm.get("c_scale", 1.0), mu=CFG["contact"]["mu"] * mm.get("mu_scale", 1.0))
        if mm: self.pl.apply_mismatch(mm)
        self.x0, self.u0 = initial_state() if x0 is None else x0; self.pl.set_state(self.x0); self.kin = AMPred3(); self.sens = Sensors(seed, sensors); self.record = record
        self.p0 = np.array(self.kin.pad_f(self.x0)[0]).ravel(); self.task = Task(path_name, self.p0, free_waypoints, segment_s); self.rows = []; self.abort = None
        if hasattr(controller, "reset"): controller.reset(self.x0, self.u0, self.task)
    def gap_meas(self, xm): return WALL_X - R_PAD - float(self.kin.pad_f(xm)[0][0])
    def run(self):
        pl, task = self.pl, self.task; u_pend = self.u0.copy(); u_app = self.u0.copy(); k = 0; dt_c = getattr(self.ctrl, "period", DT_C); n_sub = int(round(dt_c / pl.dt)); t_wall0 = time.time()
        while pl.d.time < self.T_max:
            if time.time() - t_wall0 > self.wall_limit: self.abort = (pl.d.time, "wall-clock limit (stuck solver or plant)"); break
            x = pl.state()
            if not np.isfinite(x).all() or (self.rows and pl.d.time < self.rows[-1]["t"]): self.abort = (self.rows[-1]["t"] if self.rows else 0.0, "numerical instability in the plant (MuJoCo reset)"); break          # MuJoCo resets its data (and time) on divergence
            F, ft = pl.pad_force(); xm, Fm = self.sens.measure(x, F); gm = self.gap_meas(xm); task.advance(dt_c, Fm, gm) if k else None
            if task.failed: self.abort = (pl.d.time, task.failed); break
            tilt_m = np.degrees(2 * np.arcsin(min(1.0, np.hypot(xm[4], xm[5]))))
            pref = np.asarray(task.ref().get("pB", BASE), float)          # failsafe is relative to the CURRENT base reference (equals BASE for the fixed-hover task)
            if tilt_m > 40 or np.linalg.norm(xm[0:3] - pref) > 0.25: self.abort = (pl.d.time, f"loss of control (failsafe): tilt {tilt_m:.0f} deg, base error {np.linalg.norm(xm[0:3] - pref) * 1e3:.0f} mm"); break          # same monitor for every controller
            t0 = time.perf_counter(); missed = bool(self.miss(k)); info = {}
            if not missed: u_new, info = self.ctrl.compute(pl.d.time, xm, Fm, task); u_new = np.asarray(u_new, float)
            else: u_new = u_pend.copy()
            cyc = time.perf_counter() - t0
            if self.enforce and not missed and cyc > dt_c:                                  # result not ready at the next boundary: the previous command stays
                u_new = u_pend.copy(); missed = True; info = dict(info, deadline_miss=True)
                if hasattr(self.ctrl, "held"): self.ctrl.held(u_new)
            ref = task.ref(); pad = np.array(self.kin.pad_f(x)[0]).ravel()
            self.rows.append(dict(t=pl.d.time, x=x, xm=xm, F=F, Fm=Fm, ft=ft.copy(), phase=task.phase, ref=ref, pad=pad, gap=pl.gap(), u_cmd=u_new.copy(), u_app=u_app.copy(), com=pl.com(), cycle_s=cyc, missed=missed, contact=task.contact, other=len(pl.other_contacts()), info=info))
            if self.record is not None: self.record(self, pl.d.time)
            u_app, u_pend = u_pend, u_new                                              # one-period computation latency
            for _ in range(n_sub): pl.step(u_app)
            k += 1; tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x[4] ** 2 + x[5] ** 2), -1, 1)))
            if task.phase == "done" and pl.d.time > task.t_done + 1.0: break
            if tilt > 60 or np.linalg.norm(x[0:3] - pref) > 0.4 or x[2] < 0.3: self.abort = (pl.d.time, f"divergence: tilt {tilt:.0f} deg, base error {np.linalg.norm(x[0:3] - pref):.2f} m"); break
        self.wall_s = time.time() - t_wall0; return self
