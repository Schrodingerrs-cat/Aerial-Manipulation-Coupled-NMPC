"""Evidence for the v3 design choices (wrist angle, pad radius, mount, patch, friction/preload).  Reads config/am3_design_v3.json as the baseline and prints/writes results/am3_design/design_scan.txt:
(1) wrist angle x pad radius: hand-axis angle to the wall normal and smallest gap of any non-pad geom to the wall over a wide patch,
(2) gap map over the chosen patch for the chosen design, (3) quasi-static rotor force range over the patch for preload x friction (friction in 8 directions)."""
import os, sys, copy, tempfile, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..")); sys.path.insert(0, HERE)
import am3_build_model as bm
from ocrl.am3_static import Rig, CFG
OUT = os.path.join(HERE, "..", "results", "am3_design", "design_scan.txt"); lines = []
def say(s): print(s); lines.append(s)

def rig(**over):
    C = copy.deepcopy(CFG)
    for k, v in over.items(): C[k] = {**C[k], **v} if isinstance(v, dict) else v
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f: path = f.name
    bm.build(C, path); r = Rig(path); os.unlink(path); return r, C

def standoff_base(C, d): return np.array([C["wall_x_m"] - C["pad"]["radius_m"] - d, 0.0, 1.2])
def target(C, y, zr, Fn): return np.array([C["wall_x_m"] - C["pad"]["radius_m"] + Fn / C["contact"]["k_N_m"], y, 1.2 + zr])
Q0 = np.array([0, -0.7, -1.0]); D = CFG["task"]["standoff_pad_from_base_m"]

say("(1) wrist angle x pad radius; patch y in +-0.10, z below base -0.10..-0.26, standoff 0.28, 2 mm indentation")
for rp in (0.010, 0.012, 0.015):
    for beta in (0, 10, 20, 30):
        r, C = rig(wrist_beta_deg=float(beta), pad={"radius_m": rp}); pB = standoff_base(C, D); ang, gap = 0.0, 1.0
        for zr in (-0.10, -0.14, -0.18, -0.22, -0.26):
            for y in (-0.10, -0.05, 0, 0.05, 0.10):
                q, e = r.ik(target(C, y, zr, 1.2), pB, q0=Q0); r.pose(pB, 0, 0, q); ang = max(ang, np.degrees(np.arccos(r.hand_axis()[0]))); gap = min(gap, r.clearances()["wall_nonpad"])
        say(f"  pad r {rp:.3f} beta {beta:2d}: max hand-axis angle {ang:4.1f} deg, min non-pad gap to wall {gap * 1e3:5.1f} mm")

r, C = rig(); pB = standoff_base(C, D); hz = CFG["task"]["patch_half_extent_m"]; zc = -CFG["task"]["pad_below_base_m"]
say(f"(2) gap map of non-pad geoms to the wall [mm] (hand-axis angle) for the chosen design, rows z below base, columns y = -0.10 .. 0.10")
for zr in (zc + 0.04, zc, zc - 0.04):
    row = []
    for y in (-0.10, -0.05, 0, 0.05, 0.10):
        q, e = r.ik(target(C, y, zr, 0.6), pB, q0=Q0); r.pose(pB, 0, 0, q); row.append(f"{r.clearances()['wall_nonpad'] * 1e3:5.1f}({np.degrees(np.arccos(r.hand_axis()[0])):2.0f})")
    say(f"  {zr:5.2f}  " + "  ".join(row))

say("(3) quasi-static rotor force range [N] over y in +-0.05, z in zc +-0.04 (limits 0.1 .. 10) for preload Fn and friction mu")
for yh in (0.05, 0.10):
    for Fn in (0.6, 0.9):
        for mu in (0.15, 0.3):
            lo, hi = 9.0, 0.0
            for zr in (zc + 0.04, zc, zc - 0.04):
                for y in (-yh, 0, yh):
                    tgt = target(C, y, zr, Fn); q, e = r.ik(tgt, pB, q0=Q0)
                    for a in [None] + list(np.arange(8) * np.pi / 4):
                        Fc = np.array([-Fn, 0, 0]) if a is None else np.array([-Fn, mu * Fn * np.cos(a), mu * Fn * np.sin(a)]); t = r.trim(tgt, Fc, pB, q); lo, hi = min(lo, t["f"].min()), max(hi, t["f"].max())
            say(f"  |y| <= {yh:.2f}  Fn {Fn:.1f} N  mu {mu:.2f}: [{lo:5.2f}, {hi:5.2f}]")
open(OUT, "w").write("\n".join(lines) + "\n")
