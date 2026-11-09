# Baseline B v5: strict drone/arm separation (30 Oct 2026)

Change requested by the user: B's drone and arm controllers must share no information. N, controller rates (B 200 Hz / 5 ms latency, N 50 Hz / 20 ms), plant parameters and acceptance criteria (`config/am3_acceptance_v2.json`) are unchanged. Plant stays fully coupled. Design and interfaces: `controllers_v3.md`, section "Baseline B v5". Previous B (v4, information-sharing) is kept as `am3_ctrl_b_v4.py` / `make_ctrl("B4")`; all earlier results (GATE_v2_B, `Media_results/AM3_BvsN_*_2026-10-30/`) are preserved unchanged.

## Information paths removed (v4 -> v5)
1. Wall-force (indentation-derived) feedforward into the drone's thrust.
2. Task/contact phase and pad reference into the drone controller (it now sees only the fixed hover reference).
3. Measured pad position (hence drone pose via FK) into the arm: the slow integral correction `k_corr` and pad-error feedback are gone.
4. Drone-derived measured gap into the shared task for B (`Episode` passes `task.gap_ref`, `task_uses_state = False`). The task still receives the pad-force measurement (arm-side sensing).
5. Drone state in the arm's IK base pose: the arm assumes a fixed mounting (drone at BASE, pitch atan(F_ref/mg) from the arm's own contact reference, 0 in free flight). Reference generation does **not** use the measured base pose.
Kept: drone nominal mass, locked-posture COM/inertia, hover-trim from rotor telemetry at t0; arm's own decoupled model and contact-force torque for its own intended indentation force. Gains: v4 numbers, not retuned (`config/am3_b_gains_v5.json`).

## Verification
* `Code/scripts/am3_check_b_separation.py`: ALL PASS (static signatures/identifiers; rotor commands bit-identical when arm states, task reference and contact force change; joint torques bit-identical when drone states and rotor speeds change, 300 steps each; `Episode` passes `task.gap_ref`). Sanity checks confirm the perturbations do change the other controller's output.
* Stationary hover, 30 s, four postures, noise on, with and without kick: stable everywhere, base 0.2-0.4 mm in the last 10 s (`results/am3_physics/hover_check_B.json`; overwrites the v4 file of the same name, v4 numbers are in `config/am3_b_gains_v4.json`).
* N unchanged: N metrics in the three new recordings are identical to the 30 Oct recordings (e.g. line trace RMS 0.345 mm, circle 0.426, polyline 0.411).

## Results (held-out gate, 30 scenarios, `results/am3_runs/GATE_v3_B`, logs `results/am3_runs_GATE_v3_B.log`)
B v5: 0/30 joint successes (v4: 0/30); base max median 29.8 mm (v4 33.3), max 75.5; trace RMS median 18.3 mm (v4 18.3); every scenario fails base_max; 27/27 contact scenarios also fail path tracking, coverage, endpoint, contact fraction and recovery. N (unchanged, GATE_v2_N): 29/30.

## Recordings (v5 B vs N, `Media_results/AM3_BvsN_<path>_2026-10-30_sepB/`)
| path | B v4 trace RMS / contact frac / losses / mean Fn | B v5 | N |
|---|---|---|---|
| line | 13.2 mm / 1.00 / 0 / 0.54 N | 17.7 mm / 0.78 / 11 / 0.17 N | 0.35 mm |
| circle | 13.0 / 1.00 / 0 / 0.53 | 14.9 / 0.87 / 9 / 0.21 | 0.43 |
| rounded triangle | 8.8 / 1.00 / 0 / 0.53 | 13.0 / 0.87 / 9 / 0.21 | 0.41 |

## What changed in behaviour (observed) 
* Base error is about the same (RMS 11-14 mm). Peak tilt halves (about 3.2 deg -> 1.4-1.6 deg).
* The pad presses much more weakly (mean trace force 0.17-0.21 N vs 0.53 N) and loses contact repeatedly (9-11 losses, 2.1-2.5 s detached, contact-valid fraction 0.78-0.87), so path coverage falls (0.10-0.53 on line/circle/triangle vs 0.60-0.97) and cross-track error grows.
* No sustained oscillation appeared; B was not tuned to fail or oscillate.
* Likely cause (hypothesis, not isolated by an experiment): without pad-position feedback and without the force feedforward in thrust, the pad's normal position is set only by the fixed-mounting IK, so the 1 mm indentation reference is not held while the base moves tens of mm; the contact policy then sees losses. The normal-force and tilt differences were not individually attributed to each removed path.
* Both B variants remain far from the acceptance criteria; rate/latency/architecture differences between B and N remain part of the comparison (see `controllers_v3.md`). Not yet done (suggested): equal-rate B vs N, and B with explicit arm/contact compensation for attribution.
