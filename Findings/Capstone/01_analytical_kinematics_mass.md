# [Capstone] Analytical arm kinematics and closed-form payload mass (2026-11-17 19:00 EST)

Capstone track (advisor Ashis), kept apart from the RBE595 optimal-control track. Advisor's points from the latest meeting:
- Analytical solution for the manipulator.
- Product of exponentials with the space frame on the drone body frame.
- Angles and link lengths are known, so no Bayesian mass/length estimator is needed.

## What was built
`Code/ocrl/am3_kin_poe.py` (`ArmPoE`), geometry from `config/am3_design_v3.json`, the same source as the CasADi predictor:
- **Screw axes, space frame {s} = drone body:**
  - S1 = (e_z, -e_z x o1): yaw at the arm root o1.
  - S2 and S3 = (e_y, -e_y x o_i): pitch at the shoulder o2 and elbow o3 (home pose q = 0).
- **Home pose:** M = (Ry(beta = 20 deg), o4). The pad centre is c = (0, 0, -64 mm) in the hand frame.
- **FK:** T_sh(q) = e^[S1]q1 e^[S2]q2 e^[S3]q3 M.
- **Space Jacobian:** J_s = [S1, Ad(e^[S1]q1) S2, Ad(e^[S1]q1 e^[S2]q2) S3]. A point p on the hand moves with v = v_s + omega_s x p.
- **IK of the pad, closed form:** the chain below the shoulder stays in the yawed plane, so q1 = atan2(dy, dx) (or + pi). In that plane, with u = z + i x, Ry(a) multiplies u by e^{ia}:
  - u_pad = e^{i q2}(-l1 + e^{i q3} w), where w = elbow -> pad in the forearm frame.
  - Then cos(q3 + arg w) = (l1^2 + |w|^2 - |u_pad|^2) / (2 l1 |w|), and q2 = arg u_pad - arg(-l1 + e^{i q3} w).
  - Of the four solutions, the one closest to the nominal posture is returned.
- **Payload mass:** a point mass m at the hand origin loads the joints by tau = Jr(q)^T R^T (-m g e_z), linear in m. So m = a.tau_d / a.a, with a = Jr^T R^T (-g e_z) and tau_d the observer's joint-torque estimate.

## Verification (`Code/scripts/am3_kin_check.py`, 1000 random poses in the operational joint ranges)

| check | result |
|---|---|
| FK vs CasADi predictor `AMPred3.pad_f` | max 4.3e-16 m |
| IK round trip (angles / pad) | 3.6e-15 rad / 4.5e-16 m; all 1000 reachable |
| Jacobian vs finite differences | max 1.3e-10 |
| IK time, analytical vs numerical (`am3_static.Rig.ik`, MuJoCo + least squares) | 0.066 ms vs 1.17 ms (18x) |
| numerical IK vs analytical | up to 0.7 mrad off (its posture regularisation biases it) |

Output: `Code/results/capstone_kin/kin_check.json`.

## Closed-form payload mass (`Code/scripts/am3_mass_check.py`)
Setup: ND (offset-free NMPC with the momentum observer), HELLO WPI, real sensor noise, 25 s; mean after 5 s.

| case | true | joint-torque formula | base vertical force (-d_z/g) |
|---|---|---|---|
| nominal | 0 | 0.0000 +- 0.0057 kg | 0.0021 |
| payload 100 g | 0.100 | 0.0994 +- 0.0051 | 0.1015 |
| payload 200 g | 0.200 | 0.1996 +- 0.0059 | 0.2013 |
| thrust -10% (kf90) | 0 | -0.0002 | **0.1705 (wrong)** |

- The estimate settles in about 0.2 s, with about 5 g noise.
- The joint-torque formula only sees loads carried on the arm. A thrust error or wind cannot enter it, while the base-force reading mistakes a 10% thrust loss for a 170 g payload.
- This confirms the advisor's point: with known kinematics, one scalar is unknown and it has a closed-form estimate. No Bayesian estimator is needed for a point payload at the hand.

## Limits (to test next)
- **Assumes a point mass at a known place (hand origin).** A payload hung elsewhere, or with unknown position, needs the 4-parameter linear form (m, m*r).
- **Swing is not modelled:** a slung or swinging payload breaks the rigid-attachment assumption.
- **The noise floor (about 5 g) comes from the observer and sensor noise.** A sweep of payload mass and placement is pending.

## Next
- Feed m-hat into the model explicitly: payload term in ND's predictor and in the RL inner loop's gravity compensation, instead of the generic disturbance channel. Compare with the generic observer.
- Swap the analytical IK in for the numerical one where the code uses IK (`am3_static`, `am3_ref`, baseline B).
