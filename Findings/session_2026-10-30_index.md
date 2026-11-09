# 30 Oct 2026: B vs N work, index

Principle throughout: report what was measured, no manufactured failure or success; N, plant, rates and acceptance criteria were not changed for any B result.

## 1. Baseline B made strictly separate (B v5)
Two controllers, nothing exchanged: `DroneController` (drone measurements, fixed hover reference, no force feedforward) and `ArmController` (joint state, own reference, J^T F of its own intended contact force). Shared task advanced with the force measurement and its own gap reference. Verified by `Code/scripts/am3_check_b_separation.py` (static + bit-exact invariance both directions): all pass.
Docs: `Codebase/b_separation_2026-10-30.md`, `Codebase/controllers_v3.md` (v4 vs v5 table). Gains `Code/config/am3_b_gains_v5.json` (v4 numbers minus k_corr, not retuned). Hover stable; held-out gate 0/30 (base max median 29.8 mm). B v1-v3 deleted; v4 archived (`am3_ctrl_b_v4.py`, `make_ctrl("B4")`).
Media: `Media_results/AM3_BvsN_{line,circle,rounded_triangle}_2026-10-30_sepB/` (v4 sets kept).

## 2. B presses more weakly than N
Less wall force/tilt for B accompanies worse tracking, so tilt is not a like-for-like comparison. Numbers in `Codebase/controllers_v3.md` (line: trace RMS 17.7 mm, mean Fn 0.17 N, contact fraction 0.78 for v5).

## 3. Search for B failures
`Codebase/b_failure_search_2026-10-30.md`. The mm_027 failure (thrust-deficit sag from a clipped integrator) was **retracted**: a pre-existing hover offset, not caused by arm motion or contact; its video was discarded, record kept. No severe nominal-plant task failure (no aborts) for B with the wall.

## 4. Failures actually caused by arm motion
`Physics/arm_motion_vs_hover_2026-10-30.md`, raw `Code/results/am3_arm_motion/`.
* Free-flight arm loops displace B 30-129 mm (N 0.3 mm); mostly amplitude/posture driven (hypothesis: COM shift versus fixed trim, not tested).
* B loses control (failsafe tilt > 40 deg or base error > 0.25 m) under a 6 cm circle at 2 Hz in every seed tried (clean: no wall contact in the seed-0 logs); the +-6 cm oscillation also aborts B, but that run includes a wall collision (8.13 N at 3.784 s) and is not a clean free-flight failure; N completes (circle: rotor-speed change 80 %, close to its limit).
* Not clean wins: circle 4 cm / 3 Hz (N tracks poorly), 6 cm at 3 Hz (both abort), side-to-side 4 cm / 3 Hz (B abort not reproducible).
Media: `AM3_BvsN_arm_oscillation_B_loses_control_2026-10-30/`, `AM3_BvsN_arm_circle_2Hz_B_loses_control_2026-10-30/` (rendered, not watched).

## 5. Open / not done
* Mechanism of B's loss of control not isolated; the B/N gap bundles rate (200 vs 50 Hz), latency, preview, architecture and model. An equal-rate B and a B with explicit arm/contact compensation would attribute it; not run.
* N's own failure boundary untested; mismatch failures in the stress runs (`Code/results/am3_stress`) were not arm-caused.
* Scripts changed since freeze v2; no freeze v3; nothing committed.
