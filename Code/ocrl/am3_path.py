"""Smooth planar wall paths in the wall plane (u = horizontal y, w = vertical z, metres relative to the patch centre) with analytic position, tangent and curvature vector,
and a rest-to-rest minimum-jerk progress law.  p_ref = p(s(t)), v_ref = p'(s) sd, a_ref = p''(s) sd^2 + p'(s) sdd  (Plan s4.1)."""
import numpy as np

class Path:
    """Piecewise lines and circular arcs, arclength parametrised.  at(s) -> (point, unit tangent, p'' = curvature * normal)."""
    def __init__(self, pieces): self.pieces = pieces; self.cum = np.cumsum([0.0] + [p["len"] for p in pieces]); self.length = float(self.cum[-1])

    def at(self, s):
        s = float(np.clip(s, 0.0, self.length)); i = min(int(np.searchsorted(self.cum, s, side="right")) - 1, len(self.pieces) - 1); p = self.pieces[i]; l = s - self.cum[i]
        if p["kind"] == "line": return p["a"] + p["t"] * l, p["t"], np.zeros(2)
        a = p["a0"] + p["sgn"] * l / p["r"]; c = p["c"]; pos = c + p["r"] * np.array([np.cos(a), np.sin(a)]); tan = p["sgn"] * np.array([-np.sin(a), np.cos(a)]); return pos, tan, -np.array([np.cos(a), np.sin(a)]) / p["r"]

def line(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float); L = np.linalg.norm(b - a); return dict(kind="line", a=a, t=(b - a) / L, len=L)

def arc(c, r, a0, sweep): return dict(kind="arc", c=np.asarray(c, float), r=r, a0=a0, sgn=float(np.sign(sweep)), len=abs(sweep) * r)

class Reversed:
    """Same curve traversed backwards (tangent and arclength parameter reversed; curvature vector unchanged)."""
    def __init__(self, path): self.path, self.length = path, path.length
    def at(self, s): p, t, kn = self.path.at(self.length - s); return p, -t, kn

def line_path(a, b): return Path([line(a, b)])

def circle_path(c, r, a0=-np.pi / 2): return Path([arc(c, r, a0, 2 * np.pi)])

def rounded_polygon(vertices, r):
    """Open polyline through the vertices with circular fillets of radius r at the interior vertices (tangent-continuous; curvature steps at fillet ends)."""
    V = [np.asarray(v, float) for v in vertices]; pieces, cur = [], V[0]
    for i in range(1, len(V) - 1):
        d1, d2 = (V[i] - V[i - 1]) / np.linalg.norm(V[i] - V[i - 1]), (V[i + 1] - V[i]) / np.linalg.norm(V[i + 1] - V[i]); th = np.arccos(np.clip(d1 @ d2, -1, 1)); off = r * np.tan(th / 2)
        p_in, p_out = V[i] - d1 * off, V[i] + d2 * off; pieces.append(line(cur, p_in)); turn = np.sign(d1[0] * d2[1] - d1[1] * d2[0]); n = turn * np.array([-d1[1], d1[0]]); c = p_in + n * r
        a0 = np.arctan2(*(p_in - c)[::-1]); pieces.append(arc(c, r, a0, turn * th)); cur = p_out
    pieces.append(line(cur, V[-1])); return Path(pieces)

def min_jerk(xi):
    """Rest-to-rest progress fraction and its first two derivatives w.r.t. xi in [0,1]."""
    xi = np.clip(xi, 0.0, 1.0); return 10 * xi ** 3 - 15 * xi ** 4 + 6 * xi ** 5, 30 * xi ** 2 - 60 * xi ** 3 + 30 * xi ** 4, 60 * xi - 180 * xi ** 2 + 120 * xi ** 3

def reference(path, T, tau):
    """Position, velocity, acceleration at progress time tau for a path traversed in T seconds."""
    f, f1, f2 = min_jerk(tau / T); s, sd, sdd = f * path.length, f1 * path.length / T, f2 * path.length / T ** 2; p, t, kn = path.at(s); return p, t * sd, kn * sd ** 2 + t * sdd

def curvature_speed_check(path, T, n=400):
    """Peak speed, peak tool acceleration magnitude along the timed path."""
    taus = np.linspace(0, T, n); out = [reference(path, T, t) for t in taus]; return max(np.linalg.norm(o[1]) for o in out), max(np.linalg.norm(o[2]) for o in out)
