# Simulation Results & Experimental Evidence (for claude to read)

This directory is dedicated exclusively to **simulation evidence, experimental results, and controller validation**.

Only the following artifacts belong here:
- Simulation screenshots and screen recordings.
- Numerical results in JSON or CSV format.
- Relevant logs and configurations required to reproduce results.
- Documentation linking each experiment to its corresponding evidence.

## 1. Document Every Milestone

Every meaningful milestone, experiment, and proof of concept (POC) must be documented here, especially when a control algorithm demonstrates working behavior in simulation.

For each experiment, record:
- **Objective:** What was being tested or validated.
- **Implementation:** Which controller, configuration, and parameters were used.
- **Results:** Numerical metrics and observations from the simulation.
- **Evidence:** Exact paths to the corresponding screenshots, recordings, JSON, and CSV files.
- **Status:** Successful, partially successful, or failed.
- **Conclusion:** What the experiment demonstrated and what remains unresolved.

Maintain clear traceability between every experiment, its configuration, numerical results, and visual evidence. Never mix results from different runs or present unrelated artifacts as evidence of the same experiment.

## 2. Document Failures With Equal Priority

**Failed experiments are just as important as successful ones. Never hide, discard, or overwrite failure evidence.**

For every meaningful failure:
- Preserve simulation screenshots or recordings showing the failure.
- Record the corresponding numerical results and relevant logs.
- Explain what went wrong and the observed behavior.
- Investigate and document the suspected or confirmed root cause.
- Describe the corrective changes attempted.
- Link to subsequent experiments demonstrating whether the issue was resolved.

Clearly distinguish between a suspected cause and a verified root cause. Never claim that a failure has been fixed without validating the correction in simulation.

## 3. Simulation Is the Primary Proof of Success

**Numerical metrics alone are NOT sufficient to declare a controller successful.**

The controller must visibly demonstrate its intended behavior in the actual simulation.

A successful result must include:
1. A reproducible simulation run using the implemented controller.
2. Screenshots or recordings clearly showing the intended behavior.
3. Numerical metrics supporting the observed simulation performance.
4. Clear references linking the visual evidence and numerical data to the same experiment.

A controller producing favorable numbers while failing to perform the intended task in simulation is **NOT a successful implementation**.

Never substitute hardcoded trajectories, scripted behavior, modified success criteria, or artificial demonstrations for actual controller performance.

## 4. Maintain a Clear Experiment History

Keep this README updated as experiments progress.

Maintain a chronological record of milestones, successful runs, partial successes, failures, fixes, and subsequent validations.

Each entry must identify the experiment, its outcome, and the location of its supporting artifacts.

Do not silently replace previous results. Preserve the experimental history so that progress, regressions, and implementation decisions can be audited.

## Design v3: matched baseline B (stable v4) vs coupled NMPC N on three wall trajectories (2026-10-30)

Replaces the 25 Oct recordings (deleted; lower quality, made with B v3 and the task defects listed in `Findings/Audits/AM3_B_N_AUDIT_RESPONSE_2026-10-30.md`). Method: `Code/scripts/am3_compare.py TAG PATH` runs B and N on the same plant, initial state, reference and sensor seed. Frames are rendered from the **live plant instance during the closed-loop run** (MuJoCo offscreen, 8x MSAA, shadows) at exact simulation instants, 30 fps = 1.0x simulation time, 1280x720 per panel; the composites are 2560x1440 (`overview_wall`: overview over wall-front close-up) and 2560x720 (`contact_side`); B on the left, N on the right. These are renders of the live simulation, not viewer screen captures. Overlay: controller, trajectory, simulation time, phase, base-position error, pad-reference error, tilt, pressing/detached, true normal force, phase timeline. Green = reference path of the contact point on the wall plane; red = actual pad contact point while pressing (true Fn > 0.08 N, drawn 0.7 mm in front of the surface because the pad indents the soft wall by about 1 mm); blue = the same point while detached (never drawn on the wall). Rotor blades are drawn from the physical rotor speeds, slowed 100x. Raw per-controller videos (`raw_*`) are uninterrupted originals; the composite is built by the commands in `compose_commands.txt` (frames were deleted afterwards; rerun the script to regenerate, the simulation is deterministic given the seed). Run logs `Code/results/am3_runs/` hold the same data as the figures. Each folder also has `<path>_timeseries.png` (8 measured traces: base error, tilt, wall-normal and tangential pad error, Fn and friction, pressing state, rotor thrust, joint torque) and `<path>_summary.png` (wall-plane projection, error distributions, 25-row metric table).

B runs at 200 Hz and N at 50 Hz; B uses the stable-hover gains v4 (`Findings/Codebase/controllers_v3.md`). `rounded_triangle` is an open polyline with one rounded corner, not a closed triangle. Rendering makes each run take several minutes of wall-clock time; the simulation does not depend on wall time.

| Folder | Trajectory | Result (B vs N) | Evidence | Status |
|---|---|---|---|---|
| `AM3_BvsN_line_2026-10-30` | horizontal line, 8 cm in 4 s | base RMS/max 15.3/28.9 vs 0.17/0.42 mm; trace tracking RMS 13.2 vs 0.34 mm (along 13.0 vs 0.30, cross 0.8 vs 0.05); endpoint 10.6 vs 0.22 mm; coverage 0.97 vs 1.00; tilt max 3.2 vs 2.5 deg; peak Fn 0.97 vs 0.67 N; contact losses 0 / 0 | `compare_line_overview_wall.mp4`, `compare_line_contact_side.mp4`, `raw_*.mp4`, `line_timeseries.png`, `line_summary.png`, `line_metrics.json`, `compose_commands.txt` | both completed the full sequence |
| `AM3_BvsN_circle_2026-10-30` | circle, radius 2.5 cm, 10 s | base RMS/max 10.8/25.6 vs 0.19/0.50 mm; trace tracking RMS 13.0 vs 0.43 mm (along 11.1 vs 0.39, cross 6.8 vs 0.06); endpoint 8.5 vs 0.21 mm; coverage 0.60 vs 1.00; tilt max 3.3 vs 2.6 deg; peak Fn 0.99 vs 0.69 N; contact losses 0 / 0 | `compare_circle_overview_wall.mp4`, `compare_circle_contact_side.mp4`, `raw_*.mp4`, `circle_timeseries.png`, `circle_summary.png`, `circle_metrics.json`, `compose_commands.txt` | both completed the full sequence |
| `AM3_BvsN_rounded_triangle_2026-10-30` | open polyline with rounded corner, 10 s | base RMS/max 11.6/26.6 vs 0.18/0.47 mm; trace tracking RMS 8.8 vs 0.41 mm (along 6.6 vs 0.38, cross 5.6 vs 0.04); endpoint 9.4 vs 0.16 mm; coverage 0.77 vs 1.00; tilt max 3.3 vs 2.6 deg; peak Fn 1.00 vs 0.69 N; contact losses 0 / 0 | `compare_rounded_triangle_overview_wall.mp4`, `compare_rounded_triangle_contact_side.mp4`, `raw_*.mp4`, `rounded_triangle_timeseries.png`, `rounded_triangle_summary.png`, `rounded_triangle_metrics.json`, `compose_commands.txt` | both completed the full sequence |

Conclusion: on these three seeds, N tracks the commanded path with 0.35-0.43 mm RMS while keeping the base within 0.5 mm; the stable baseline B completes the sequence with bounded forces but its trace deviates by 9-13 mm RMS (mostly along the path on the line, also cross-track on the curves) and its base error is 11-15 mm RMS (26-29 mm maximum). The physical cause of B's motion is analysed in `Findings/Physics/physics_isolation_2026-10-30.md`; these recordings do not by themselves show that coupling is the cause. Not established here: robustness to model mismatch, real-time compliance of N (about 1.5 % of its cycles exceed 20 ms), behaviour on other seeds beyond the 30-scenario gate in `Findings/Codebase/nominal_gate_v2.md`.

## Final Rule

**If there is no reproducible simulation evidence demonstrating the intended controller behavior, there is no verified success.**

Every success claim must be supported by actual simulation behavior, numerical results, and clearly traceable evidence. Every significant failure must be documented with the same level of integrity.

**No undocumented milestones. No unexplained failures. No unsupported success claims.**


`AM3_BvsN_<path>_2026-10-30_sepB/`: B v5 (separate drone and arm controllers, no information sharing) vs N, same settings as the `_2026-10-30` set (which used the information-sharing B v4). Details: `Findings/Codebase/b_separation_2026-10-30.md`.

## Arm-motion failure recordings (30 Oct 2026, wall-free task, nominal plant; wall geometry still present)
Both: B v5 vs N, same recorder/layout as above; the pad follows a prescribed periodic motion after 3 s of stationary hover. Study and caveats: `Findings/Physics/arm_motion_vs_hover_2026-10-30.md`. The videos were rendered but not watched; claims rest on the metrics JSON and `recording_info.json`.

| Folder | Motion | Result | Files |
|---|---|---|---|
| `AM3_BvsN_arm_oscillation_B_loses_control_2026-10-30` | pad oscillation +-6 cm, half-cycle 0.25 s (about 2 Hz) | B loses control (failsafe abort) but **B's pad hit the wall at 3.784 s (8.13 N) before the abort, so this is not a clean free-flight failure**; N completes | `compare_free_osc0.06_T0.25_{overview_wall,contact_side}.mp4`, `raw_*`, `*_metrics.json`, `*_summary.png`, `*_timeseries.png`, `compose_commands.txt` |
| `AM3_BvsN_arm_circle_2Hz_B_loses_control_2026-10-30` | pad circle radius 6 cm, 2 Hz (pad speed about 0.75 m/s) | B aborts at 4.845 s (tilt 41 deg, base error 114 mm), also in seeds 1-3; no wall contact in the seed-0 logs (peak Fn 0 N); N completes (base 6 mm, tilt 9.9 deg, pad error 1.2 mm RMS, rotor-speed change 80 %, near its limit) | `compare_free_circ_A0.06_f2.0_{overview_wall,contact_side}.mp4`, `raw_*`, `*_metrics.json`, `*_summary.png`, `*_timeseries.png`, `recording_info.json`, `compose_commands.txt` |

Not established: the mechanism of B's loss of control, N's failure boundary, and whether a B with explicit arm/contact compensation or equal controller rate would survive (not run). The discarded mm_027 video (a pre-existing hover offset, not arm-induced) is documented only in `Findings/Codebase/b_failure_search_2026-10-30.md`.
