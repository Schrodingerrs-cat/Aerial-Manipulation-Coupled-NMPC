"""Verifies the information boundary of baseline B (ocrl/am3_ctrl_b.py): drone and arm controller exchange nothing.
1 static   : DroneController.compute takes only (t, y); no arm/task/contact identifier appears in its source; ArmController has no rotor/drone-controller attribute.
2 dynamic  : same drone measurements and history, different arm states / task reference / contact force -> identical rotor commands (bit-exact);
             same arm measurements and reference, different drone states / rotor speeds -> identical joint torques (bit-exact).
3 episode  : a short closed-loop run in which B's task is advanced with its own gap_ref, never a drone-derived gap.
Usage: am3_check_b_separation.py  (exit code 1 on any failure)"""
import sys, os, ast, inspect, copy, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import ocrl.am3_ctrl_b as B
from ocrl.am3_episode import Episode, initial_state
from ocrl.am3_task import Task
FORBIDDEN = {"task", "ref", "gap", "q", "qd", "Fm", "joint", "arm", "contact", "wall", "xm", "x0"}; fails = []
def check(name, ok): print(("PASS " if ok else "FAIL ") + name); fails.append(name) if not ok else None
# 1 static
src = ast.parse(inspect.getsource(B.DroneController)); names = {n.id for n in ast.walk(src) if isinstance(n, ast.Name)} | {n.arg for n in ast.walk(src) if isinstance(n, ast.arg)} | {n.attr for n in ast.walk(src) if isinstance(n, ast.Attribute)}
check("DroneController.compute signature == (self, t, y)", list(inspect.signature(B.DroneController.compute).parameters) == ["self", "t", "y"])
check("DroneController source has no arm/task/contact identifiers: " + str(sorted(names & FORBIDDEN)), not (names & FORBIDDEN))
check("DroneController imports no arm model (only AMPred3 for its declared nominal constants, read in __init__)", "Jpad_f" not in names and "JC_f" not in names and "fk_f" in names)
a_names = {n.attr for n in ast.walk(ast.parse(inspect.getsource(B.ArmController))) if isinstance(n, ast.Attribute)} | {n.arg for n in ast.walk(ast.parse(inspect.getsource(B.ArmController))) if isinstance(n, ast.arg)}
check("ArmController.compute signature == (self, t, q, qd, ref, a_ref)", list(inspect.signature(B.ArmController.compute).parameters) == ["self", "t", "q", "qd", "ref", "a_ref"])
check("ArmController has no rotor/drone-controller identifiers: " + str(sorted(a_names & {"om", "Om", "drone", "Ainv", "rc", "Mbias"})), not (a_names & {"om", "Om", "drone", "Ainv", "rc", "Mbias"}))
# 2 dynamic
x0, u0 = initial_state(); rng = np.random.default_rng(0); task = Task("line", np.array([0.7, 0.0, 1.04]), None, None)
class Stub:                                              # arbitrary task/reference content for perturbation
    def __init__(s, gap, p, v): s.g, s.p, s.v = gap, p, v
    def ref(s): return dict(phase="trace", p=s.p, v=s.v, gap=s.g, contact_expected=True, s=0.5)
    def horizon(s, N, h, dt): return [dict(v=s.v), dict(v=s.v * 1.3)]
def drive(ctrl, xms, tasks, Fms):
    ctrl.reset(x0, u0, tasks[0]); out = []
    for k, (xm, tk, Fm) in enumerate(zip(xms, tasks, Fms)): out.append(ctrl.compute(0.005 * k, xm, Fm, tk)[0])
    return np.array(out)
xms = [x0 + 1e-3 * rng.standard_normal(23) * (np.arange(23) < 13) for _ in range(300)]; xms = [np.r_[x[:13], x0[13:]] for x in xms]; xms = [np.r_[x[:19], x0[19:]] for x in xms]
t1 = [Stub(0.03, np.array([0.7, 0.0, 1.04]), np.zeros(3))] * 300; Fm1 = [0.0] * 300
t2 = [Stub(-0.001, np.array([0.7, 0.04, 1.07]), np.array([0, 0.03, 0.0]))] * 300; Fm2 = [0.8] * 300
xms2 = [np.r_[x[:13], x[13:16] + [0.3, -0.2, 0.25], x[16:19] + [1.0, -2.0, 1.5], x[19:]] for x in xms]
ua = drive(B.BaselineB(), xms, t1, Fm1); ub = drive(B.BaselineB(), xms2, t2, Fm2)
check("rotor commands bit-identical for different arm states/references/contact force (300 steps)", np.array_equal(ua[:, :4], ub[:, :4]))
check("sanity: joint torques DO differ in that case", not np.allclose(ua[:, 4:], ub[:, 4:]))
xms3 = [np.r_[x[:13] + [0.02, -0.01, 0.015, 0.01, 0.02, 0, 0, 0.03, 0.0, 0.0, 0, 0.1, 0.0], x[13:19], x[19:] * 1.05] for x in xms]
uc = drive(B.BaselineB(), xms3, t1, Fm1)
check("joint torques bit-identical for different drone states/rotor speeds (300 steps)", np.array_equal(ua[:, 4:], uc[:, 4:]))
check("sanity: rotor commands DO differ in that case", not np.allclose(ua[:, :4], uc[:, :4]))
# 3 episode: task is advanced with its own gap_ref
ep = Episode(B.BaselineB(), "line", T_max=1.0); seen = []; adv = ep.task.advance
def spy(dt, F, gap, nominal=False): seen.append((gap, ep.task.gap_ref)) if not nominal else None; return adv(dt, F, gap, nominal)
ep.task.advance = spy; ep.run()
check(f"Episode passes task.gap_ref (not a drone-derived gap) to B's task ({len(seen)} calls)", len(seen) > 0 and all(g == r for g, r in seen))
print("ALL PASS" if not fails else "FAILED: " + "; ".join(fails)); sys.exit(1 if fails else 0)
