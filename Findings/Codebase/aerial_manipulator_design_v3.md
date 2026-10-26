# Design v3 (25 Oct 2026): direct end-effector wall contact

Files: `Code/config/am3_design_v3.json` (single source), `Code/scripts/am3_build_model.py` -> `Code/model/aerial_manipulator_v3.xml`, analysis `Code/ocrl/am3_static.py`, `Code/scripts/am3_design_scan.py` -> `Code/results/am3_design/design_scan.txt`, renders `Code/results/am3_design/assembly_*.png`. `scan.json` is the first coarse exploration (pad r = 10 mm, before the final choice). v2 files are untouched (jobs on the v2 probe design were still running when v3 was started).

## What was wrong in v2 and what v3 does
| v2 | v3 |
|---|---|
| arm root moved to (-0.08,-0.06,-0.035), yaw housing left behind (45 mm gap), no bracket | supplied centred mount: yaw housing (overlaps airframe core) sits directly on link1; continuous, no added mass; the builder supports `mount_x_m > 0` with a bracket plate and moved housing (mass/COM/inertia folded into the airframe inertial, checked: exact supplied inertial at mount_x = 0) |
| 33 cm lateral rod, tracked tip | no rod. Contact = 12 mm-radius rubber pad between the finger tips, centred on the supplied `ee` site (hand z = -0.064); it protrudes 12 mm beyond the finger faces and overlaps their inner faces by 2 mm (same body). Collision geom `pad`, viewer shows it; tool point = pad centre. Its mass is part of the supplied 0.10 kg hand (assumption, hand inertial unchanged) |
| wrist beta = 0, arm yawed -90 deg to point the rod | beta = 20 deg (chosen below), arm in its supplied orientation, q4 fixed, no orientation objective |

## Choices and evidence (design_scan.txt)
* **Mount: supplied centred.** Moving it forward by 0.04 m added thrust spread (first scan) and a bracket mass; moving it back loses reach (max pad reach from the shoulder 0.364 m).
* **Wrist beta = 20 deg, pad radius 12 mm.** Maximises the smallest gap between any non-pad hand/finger/airframe geom and the wall over the patch (>= 3.9 mm, 2 mm indentation). The finger corners limit the hand-axis tilt to about 25-30 deg; over the chosen patch the hand axis stays within 22 deg of the wall normal (a consequence, not an objective).
* **Geometry: pad standoff 0.28 m from the base origin, patch centre 0.16 m below the base, half extents y 0.05 m, z 0.04 m** (rotor disc edge to wall about 0.09 m). Rows z = -0.12..-0.20, columns y = +-0.10 are all collision-free with >= 3.9 mm clearance.
* **Yaw authority limits the patch width and preload.** The wall normal force acts at the lateral offset y and the friction force at the 0.28 m lever; the rotors cancel the resulting yaw moment only through their drag arm (0.016 m). At F_n = 0.9 N and |y| = 0.10 m a rotor would need negative thrust (static range [-0.03, 6.7] N); at |y| <= 0.05 m and F_n = 0.6 N the range is [1.4, 5.8] N (limits 0.1..10). Hence preload 0.6 N (1 mm indentation at 600 N/m), |y| <= 0.05 m, and friction mu = 0.15 (PTFE-capped pad) with mu = 0.3 kept as a robustness case ([1.0, 6.2] N). These are declared assumptions of this vehicle, not hardware data.
* **Operational joint ranges** (mechanical stay +-90/120/150 deg): q1 [-35, 35], q2 [-80, -5], q3 [-125, -30] deg, from the IK solutions over the patch (q1 +-20, q2 -16..-45, q3 -62..-102 deg) plus margin; q2 within +-90 deg also keeps link2 clear of the link1 box (adjacent pair, skipped by the engine).

## Limitations
Propulsion (10 N / rotor, 0.016 m drag arm, motor lags) and servo continuous torque are declared assumptions (sensitivity +-20 % belongs to the mismatch suite). No rotor downwash on the arm or wall effect is modelled. The pad's 1 mm indentation is a soft-contact regularisation that stands for pad compression. Clearances come from MuJoCo contact detection with margin (mj_geomDistance returns 0 for rotated boxes); sampled poses are not a continuous certificate, the path check (stage 2) samples densely.
