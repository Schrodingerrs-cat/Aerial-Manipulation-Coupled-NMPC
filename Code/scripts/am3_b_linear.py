"""Linearised closed loop of baseline B around the hover trim, using the predictor as plant (RK4, 20 ms, one-period command latency): discrete map (x, u_pending, controller state) -> next, finite-difference
Jacobian in tangent coordinates, eigenvalues.  Development tool for B's gains.  Usage: am3_b_linear.py"""
import sys, os, itertools, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ocrl.am3_episode import initial_state
from ocrl.am3_pred import AMPred3
from ocrl.am3_task import Task, BASE
from ocrl.am3_ctrl_b import BaselineB
pr = AMPred3(); T0 = BaselineB.period; step = pr.step_fun(T0, 4); x0, u0 = initial_state(); p0 = np.array(pr.pad_f(x0)[0]).ravel()

def qmul(a, b): return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3], a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2], a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1], a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])
def retract(x, d):
    y = x.copy(); y[0:3] += d[0:3]; th = d[3:6]; n = np.linalg.norm(th); dq = np.r_[np.cos(n / 2), (np.sin(n / 2) / n if n > 0 else 0.5) * th]; y[3:7] = qmul(x[3:7], dq); y[7:23] += d[6:22]; return y
def local(x, y):
    dq = qmul(np.r_[x[3], -x[4:7]], y[3:7]); d = np.zeros(22); d[0:3] = y[0:3] - x[0:3]; d[3:6] = 2 * dq[1:4] * np.sign(dq[0]); d[6:22] = y[7:23] - x[7:23]; return d
def cstate(b): return np.r_[b.ip, b.ia, b.corr, b.q_ref, b.om_prev]
def set_c(b, c): b.ip, b.ia, b.corr, b.q_ref, b.om_prev = c[0:3].copy(), c[3:6].copy(), c[6:9].copy(), c[9:12].copy(), c[12:16].copy()

def eigs(g):
    b = BaselineB(g); task = Task("h", p0, [p0]); b.reset(x0, u0, task); c0 = cstate(b); b.t_prev = 0.0; u1, _ = b.compute(T0, x0, 0.0, task); c1 = cstate(b); set_c(b, c0)
    def Fn(z):
        xk = retract(x0, z[0:22]); upk = u0 + z[22:29]; set_c(b, c0 + z[29:45]); b.t_prev = 0.0; u_new, _ = b.compute(T0, xk, 0.0, task); xn = np.array(step(xk, upk, np.zeros(2))).ravel()
        return np.r_[local(x0, xn), u_new - u0, cstate(b) - c0]
    z0 = np.zeros(45); f0 = Fn(z0); J = np.zeros((45, 45)); h = 1e-6
    for i in range(45): e = np.zeros(45); e[i] = h; J[:, i] = (Fn(e) - Fn(-e)) / (2 * h)
    # latency: x_{k+1} uses the pending command (z[22:29]); new command is stored as pending: map u_new -> next pending
    ev = np.linalg.eigvals(J); return np.abs(ev).max(), np.sort(np.abs(ev))[::-1][:4], np.abs(f0).max()
if __name__ == "__main__":
    print("default", eigs({}))
    for tl, lg, w, kp in itertools.product((0.03,), (0.0, 0.5, 1.0), (4, 6, 8), (1.0, 2.5)):
        print(tl, lg, w, kp, eigs(dict(t_lat=tl, lag_gain=lg, w_att=w, kp_pos=kp, kd_pos=2 * kp ** 0.5)))

def search(out="results/am3_b_tuning/linear_search.json"):
    import json
    os.makedirs(os.path.dirname(out), exist_ok=True); res = []
    for wj, zj, w, lg, tl in itertools.product((1.0, 1.5, 2.0, 3.0, 4.0), (1.0, 2.0, 4.0, 8.0), (4, 6, 8, 10), (0.0, 0.5), (0.0, 0.03)):
        g = dict(w_joint=wj, zeta_joint=zj, w_att=w, lag_gain=lg, t_lat=tl, kp_pos=1.0, kd_pos=2.0, ki_pos=0.1); rho = float(eigs(g)[0]); res.append(dict(g=g, rho=rho))
    res.sort(key=lambda r: r["rho"]); json.dump(res, open(out, "w"), indent=1); return res
