"""Rest-to-rest minimum-time A->B plans for the quadrotor with a slung load, one optimizer, three assumptions about the load:
  U  load ignored: one point mass MQ+ML (what a payload-unaware planner does)
  S  load modelled, swing held under SWING_MAX_S the whole way (suppress)
  E  load modelled, swing free, but drone AND load must arrive at rest (exploit)
Model: drone point mass at p, load at p + L d(a, b), d = [sin a cos b, sin b, -cos a cos b], Euler-Lagrange equations built with CasADi.
The input is the commanded thrust vector f_cmd (world); the applied thrust follows it with a first-order lag TAU_F (measured step response of the
attitude loop + motor lag, 2026-11-10: 132 ms to 63 %).  The command change per node is bounded so it stays smooth.  Same thrust, tilt and altitude limits for all three."""
import numpy as np, casadi as ca
from ocrl.qp_plant import MQ, ML, L, G

F_MAX, TILT_MAX, FDOT_MAX, SWING_MAX_S = 30.0, np.radians(35.0), 60.0, np.radians(3.0)
F_MIN, DZ_MAX, TAU_F = 0.5 * (MQ + ML) * G, 0.25, 0.13      # without these the optimizer hops up and coasts in free fall, where the load does not swing


def dynamics(lumped):
    q = ca.SX.sym("q", 5); qd = ca.SX.sym("qd", 5); f = ca.SX.sym("f", 3)
    if lumped:
        return ca.Function("dyn", [q, qd, f], [ca.vertcat(f / (MQ + ML) - ca.DM([0, 0, G]), 0, 0)])
    a, b = q[3], q[4]; d = ca.vertcat(ca.sin(a) * ca.cos(b), ca.sin(b), -ca.cos(a) * ca.cos(b)); r = q[:3] + L * d
    rd = ca.jtimes(r, q, qd); T = 0.5 * MQ * ca.dot(qd[:3], qd[:3]) + 0.5 * ML * ca.dot(rd, rd); V = G * (MQ * q[2] + ML * r[2])
    M = ca.hessian(T, qd)[0]; dTdqd = ca.gradient(T, qd)
    rhs = ca.vertcat(f, 0, 0) - ca.gradient(V, q) + ca.gradient(T, q) - ca.jtimes(dTdqd, q, qd)
    return ca.Function("dyn", [q, qd, f], [ca.solve(M, rhs)])


def plan(A, B, kind, N=80, T0=2.5):
    lumped = kind == "U"; dyn = dynamics(lumped); opti = ca.Opti()
    X = opti.variable(13, N + 1); U = opti.variable(3, N); T = opti.variable(); h = T / N; fh = np.array([0, 0, (MQ + ML) * G])
    def ode(x, u): return ca.vertcat(x[5:10], dyn(x[:5], x[5:10], x[10:13]), (u - x[10:13]) / TAU_F)
    for k in range(N):
        x, u = X[:, k], U[:, k]; k1 = ode(x, u); k2 = ode(x + h / 2 * k1, u); k3 = ode(x + h / 2 * k2, u); k4 = ode(x + h * k3, u)
        opti.subject_to(X[:, k + 1] == x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
    rest = lambda p: np.r_[p, 0, 0, np.zeros(5), fh]
    opti.subject_to(X[:, 0] == rest(A)); opti.subject_to(X[:, N] == rest(B)); opti.subject_to(U[:, N - 1] == fh)
    n2 = ca.sum1(U ** 2); opti.subject_to(opti.bounded(F_MIN ** 2, n2, F_MAX ** 2)); opti.subject_to(U[2, :] >= np.cos(TILT_MAX) * ca.sqrt(n2 + 1e-9))
    opti.subject_to(opti.bounded(-FDOT_MAX * h, ca.vec(ca.diff(ca.horzcat(fh, U), 1, 1)), FDOT_MAX * h))
    opti.subject_to(opti.bounded(min(A[2], B[2]) - DZ_MAX, X[2, :], max(A[2], B[2]) + DZ_MAX)); opti.subject_to(opti.bounded(0.3, T, 10.0))
    if kind == "S": opti.subject_to(opti.bounded(-SWING_MAX_S, ca.vec(X[3:5, :]), SWING_MAX_S))
    opti.minimize(T + 1e-4 * ca.sumsqr(ca.diff(U, 1, 1)))
    s = np.linspace(0, 1, N + 1); opti.set_initial(X[:3, :], np.outer(A, 1 - s) + np.outer(B, s)); opti.set_initial(X[10:13, :], np.tile(fh[:, None], N + 1)); opti.set_initial(U, np.tile(fh[:, None], N)); opti.set_initial(T, T0)
    opti.solver("ipopt", dict(print_time=False), dict(print_level=0, max_iter=3000, tol=1e-8)); sol = opti.solve()
    x, u, Tf = sol.value(X), sol.value(U), float(sol.value(T))
    return dict(kind=kind, T=Tf, t=np.linspace(0, Tf, N + 1), p=x[:3].T, v=x[5:8].T, swing=x[3:5].T, swing_d=x[8:10].T, f=np.vstack([u.T, fh]), f_act=x[10:13].T)


def sample(pl, t):
    """Plan reference (position, velocity, thrust command) at time t, linear interpolation; holds the end state after T."""
    if t >= pl["T"]: return pl["p"][-1], np.zeros(3), pl["f"][-1]
    i = lambda Y: np.array([np.interp(t, pl["t"], Y[:, j]) for j in range(Y.shape[1])])
    return i(pl["p"]), i(pl["v"]), i(pl["f"])


def minjerk(A, B, T, N=200):
    """Plain smooth move a payload-unaware user would fly: minimum-jerk drone path, thrust for the lumped mass, zero swing assumed."""
    t = np.linspace(0, T, N + 1); s = t / T; D = np.asarray(B) - np.asarray(A)
    pos = 10 * s**3 - 15 * s**4 + 6 * s**5; vel = (30 * s**2 - 60 * s**3 + 30 * s**4) / T; acc = (60 * s - 180 * s**2 + 120 * s**3) / T**2
    f = (MQ + ML) * (np.outer(acc, D) + [0, 0, G])
    return dict(kind="minjerk", T=T, t=t, p=np.asarray(A) + np.outer(pos, D), v=np.outer(vel, D), swing=np.zeros((N + 1, 2)), swing_d=np.zeros((N + 1, 2)), f=f, f_act=f)

