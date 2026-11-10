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

## Moving-base capability test: trajectory flight with simultaneous arm motion (2026-10-27)

**Outside the 25 Oct plan**, which fixes the base-position reference. This is a capability demonstration, not gate evidence: no frozen criterion covers it and it was not part of freeze v1.

Method: `Code/scripts/am3_traj_run.py TAG [--ctrls N,B] [--scale S] [--segment T]`, new module `Code/ocrl/am3_traj.py` (`TrajTask` puts the base on its own min-jerk waypoint trajectory and shifts the pad reference by the base displacement, so base and arm move on one shared clock). Rendered live from the plant as in the B/N comparisons; overlay reports base error against the **moving** reference. Green = commanded base path, orange = actual base path, blue = actual pad path.

Enabling change (freeze v2): a base-velocity reference `vB(3)` was appended to the NMPC parameter vector (`am3_nmpc.py` NP 21 -> 24, residual `x[7:10] - p[21:24]`), and `am3_ctrl_n.py` / `am3_ctrl_b.py` now read `ref.get("pB", BASE)` / `ref.get("vB", 0.0)`. Both default to the previous values, so existing tasks are unchanged. **Equivalence evidence: re-running N and B on `line` after the change reproduced the pre-change run bit-for-bit** (`results/am3_runs/parity_v2_{N,B}_line`; max abs difference 0.0 on the logged state, command, force and pad arrays; every summary metric identical to 12 significant figures).

| Entry | Objective | Result (N) | Evidence | Status |
|---|---|---|---|---|
| `AM3_TRAJ_moving_base_2026-10-27` | Base tracks a y-z diamond (+-0.15 m horizontal, 0.08 m vertical, 2 s segments, peak reference speed 0.146 m/s, 0.31 m travelled) while the arm simultaneously runs its own +-6 cm pad loop; free flight, no wall | completed; base vs moving reference RMS 0.175 mm (max 0.488); pad vs reference RMS 0.112 mm (max 0.219); tilt max 1.67 deg; rotor force 2.49-5.00 N of the 0.1-10 N range; 0 solver failures, 0 fallbacks; cycle median 9.2 ms, max 10.3 ms | `trajectory_demo.mp4` (front + isometric), `raw_N_traj_*.mp4`, `trajectory.png`, `summary.json` | successful |

Conclusion: under simultaneous base translation and arm motion the coupled NMPC holds the moving base reference to sub-millimetre error with ample actuator margin and no solver failure. Not established: the same under wall contact (the contact task still uses a fixed base reference), under model mismatch, or for the baseline B (B was given the same moving-reference interface but has not been run on this task).

### Continuous figure-eight: 3 lemniscate laps + 10 arm circles over 30 s (2026-10-27)

Same capability-test caveat as the entry above: **outside the 25 Oct plan, not gate evidence.** Command: `am3_traj_run.py TAG --mode loop --duration 30 --base-laps 3 --arm-laps 10`. `LoopTask` (`Code/ocrl/am3_traj.py`) drives both motions analytically off one clock: the base follows a Gerono lemniscate `y = A sin(th)`, `z = C sin(2 th)` while the pad circles the moving base. A single smoothstep rate envelope ramps 0 -> 1 -> 0 so the motion starts and ends at rest while completing exactly the requested laps; the same envelope scales the arm-circle radius, so the pad spirals out of and back into the base-relative origin with no velocity step at either end.

| Entry | Objective | Result (N) | Evidence | Status |
|---|---|---|---|---|
| `AM3_LOOP_figure8_2026-10-27` | 30 s of continuous periodic motion: base flies 3 figure-eight laps (+-0.15 m y, +-0.05 m z, peak reference speed 0.124 m/s, 0.32 m span) while the arm runs 10 circles of 5 cm radius about the moving base | completed; base vs moving reference RMS 0.128 mm (max 0.272); pad vs reference RMS 0.109 mm (max 0.249); tilt max 0.65 deg; joint rates <= 0.55 of the 3.0 rad/s limit; rotor force 2.63-4.76 N of the 0.1-10 N range; 0 solver failures, 0 fallbacks; cycle median 9.3 ms, max 10.7 ms, **no cycle over the 20 ms period in 1500 cycles** | `figure8_demo.mp4` (front + isometric), `raw_N_traj_*.mp4`, `paths.png`, `trajectory.png`, `summary.json` | successful |

Conclusion: sustained simultaneous base and arm motion is tracked to about 0.13 mm with every constraint far from binding. Tracking is *better* than the waypoint diamond above (0.128 vs 0.175 mm RMS, tilt 0.65 vs 1.67 deg) because the lemniscate is smooth and periodic whereas the diamond restarts from rest at each waypoint - the limit here is reference smoothness, not speed. Not established: behaviour near the actuator limits (this run uses roughly half the thrust envelope and a fifth of the joint-rate limit), under wall contact, or under model mismatch.

### Matched B vs N under base and/or arm motion, three isolations (2026-10-27)

Same capability-test caveat: **outside the 25 Oct plan, not gate evidence.** Each pair runs B and N on the identical plant, reference, initial state, seed and sensor noise; both panels are live closed-loop renders at 1.0x simulation time; an aborted run holds its last frame under a red banner giving the abort time and cause (never looped, never hidden). Columns are controllers (B left, N right), rows are cameras (isometric, front). Commands are in each `compose_commands.txt`.

**Failsafe correction made first.** The shared failsafe measured base error against the *fixed* `BASE` constant, so a drone legitimately flying a large trajectory tripped the 250 mm abort while perfectly on reference (observed: both controllers "losing control" at 3.7 s with tilt 0-1 deg and N tracking at 0.139 mm). `am3_episode.py` now measures against the current base reference, `ref.get("pB", BASE)`. Defaults to the old behaviour for every existing task; **re-running N and B on `line` reproduced the previous runs bit-for-bit** (`results/am3_runs/parity_v3_{N,B}_line`, max abs diff 0.0).

| Entry | What it isolates | N | B | Evidence |
|---|---|---|---|---|
| `AM3_LOOP_BvsN_hover_armonly_2026-10-27` | **Arm reaction alone**: base holds its fixed hover reference, only the arm moves (10 circles, 5 cm radius, 30 s) | completed; base RMS 0.127 mm, tilt max 0.32 deg | **loss of control at 13.55 s** (tilt 40 deg); base RMS 36.6 mm, pad RMS 53.3 mm | `compare_loop.mp4`, `paths.png`, `trajectory.png`, `summary.json` |
| `AM3_LOOP_BvsN_figure8_2026-10-27` | **Base and arm moving together**: 3 lemniscate laps (+-0.15 m) + 10 arm circles, 30 s | completed; base RMS 0.128 mm, tilt max 0.65 deg | **loss of control at 15.05 s** (tilt 41 deg); base RMS 58.0 mm, pad RMS 74.1 mm | as above |
| `AM3_LOOP_BvsN_bigfig8_armhold_2026-10-27` | **Standing COM offset while manoeuvring**: arm ramps out to a fixed 8 cm offset pose and HOLDS it (`--arm-laps 0`) while the base flies 2 large lemniscate laps (+-0.345 m y, +-0.115 m z), 30 s | completed; base RMS 0.129 mm, tilt max 0.66 deg | **loss of control at 11.94 s** (tilt 41 deg); base RMS 34.5 mm, pad RMS 54.0 mm | as above |

Conclusion: in free flight the coupling-neglecting baseline does not merely track worse than it does on the wall task - it **diverges** in all three isolations, while the coupled NMPC holds about 0.13 mm throughout. The hover-with-arm-only case is the sharpest: the drone is asked only to hold station, and B still tips to 40 deg purely from the arm's reaction, which is the term B omits by construction.

Limits of this claim: B runs at 200 Hz with gains searched for the **fixed-hover wall-contact** task (`Findings/Codebase/controllers_v3.md`); it was given the moving-reference interface but was NOT retuned for free-flight manoeuvring. The honest statement is "B as tuned for the original task diverges here", not "no decoupled controller can do this". N meanwhile uses roughly half the thrust envelope and a fifth of the joint-rate limit, so neither controller is near an actuation limit when B fails - the failure is a modelling failure, not saturation.

### Free-flight Lissajous, no wall, arm hanging: coupled vs decoupled (2026-10-27)

Same capability-test caveat: **outside the 25 Oct plan, not gate evidence.** Presentation note with the full explanation: `AM3_FREE_lissajous_BvsN_2026-10-27/NOTES.md`.

Command: `am3_traj_run.py TAG --mode loop --shape lissajous --duration 30 --base-laps 2 --arm-laps 0 --r-arm 0.0 --scale 1.5 --no-wall --arm-down --ctrls B,N`.

New options behind it, all in `Code/ocrl/am3_traj.py` / `Code/scripts/am3_traj_run.py` (no frozen file touched by these three):
* `--shape {lemniscate,lissajous,sine}`: the base follows `y = A sin(n_y th)`, `z = C sin(n_z th + phi)` with `(n_y, n_z, phi)` = (1,2,0) / (2,3,pi/2) / (1,3,0). The figure is translated so it starts at the hover reference, so there is no step at t = 0.
* `--arm-down`: start from a hover equilibrium with the arm hanging instead of reaching at the wall, `q = (0, -10, -35) deg` (inside the operational ranges, so the NMPC's soft joint-range constraints are satisfied at the start pose). The pad sits 400 mm below and 154 mm forward of the base origin.
* `--no-wall`: the wall geom is removed from collision (`contype`/`conaffinity` = 0) and from the render (alpha 0) after the plant is built, so contact calibration in `AMPlant3.__init__` is unaffected.
* `--arm-laps 0 --r-arm 0.0`: the arm is given no commanded motion at all - it holds its hanging pose while the base manoeuvres, so the only thing acting on the airframe is the arm's own reaction.

| Entry | Objective | N | B | Evidence |
|---|---|---|---|---|
| `AM3_FREE_lissajous_BvsN_2026-10-27` | Open-air flight, no wall, arm hanging and uncommanded: base flies 2 laps of a 2:3 Lissajous (+-0.225 m y, 0.15 m z span, peak reference speed 0.21 m/s) over 30 s | completed; base vs moving reference RMS 0.126 mm, tilt max 1.21 deg | **loss of control at 4.82 s** (tilt 40 deg); base RMS 35.6 mm, pad RMS 80.3 mm | `compare_loop.mp4`, `paths.png`, `trajectory.png`, `summary.json`, `NOTES.md` |

Conclusion: with the arm merely hanging - never commanded to move - the decoupled baseline diverges in under five seconds, faster than in any other test so far, because a 0.47 kg arm suspended 400 mm below the airframe is a much harsher reaction load than the tucked contact posture. The coupled NMPC holds 0.126 mm throughout. Same caveats as the entry above: B was tuned for the fixed-hover wall task and never retuned for free flight, and neither controller is near an actuator limit when B fails.

## Correction comparison under model mismatch: nominal NMPC vs classical EKF correction (2026-10-30)

First matched comparison of two **correction** controllers on the same mismatched plant. This is on-plan work (Plan s8), not a capability demo. `am3_compare.py` now takes `--scenario <id>`, which resolves the path, seed, initial offsets and plant mismatch from the `am3_domain_mm*` lists, so N/D/L run on an identical mismatched plant.

Command: `am3_compare.py AM3_NvsD_propulsion_mm007_2026-10-30 circle --ctrls N,D --scenario mm_dev_007`

Scenario `mm_dev_007`: **propulsion** family, circle path, `kf_scale` 1.0692 (thrust coefficient 6.9 % higher than the controller believes), `motor_tau_scale` 1.1115 (motor lag 11 % slower).

Interface validated first: **controller Z (NMPC + zero correction) reproduces N bit-for-bit** (`results/am3_runs/zparity_Z` vs `verify1027_N_line`, max abs diff 0.0 on state, command, force and pad arrays). The correction interface therefore contributes nothing by itself, and any N/D/L difference is attributable to the corrector alone. Solver parity re-confirmed the same day (`results/am3_predictor/solver_parity.json`): IPOPT and acados agree to ~1e-9 relative cost and 1.4e-4 on the first input across hover, position, approach and trace.

| | N (nominal) | D (EKF correction) |
|---|---|---|
| completed | **no** (ran to the 40 s cap) | **yes**, at 21.6 s |
| contact-valid fraction | 0.841 | **1.000** |
| contact losses | **24** | **0** |
| mean normal force in trace | **0.292 N** | 0.511 N |
| completion delay | 22.3 s | 0.24 s |
| base error max | 6.4 mm | 5.0 mm |
| trace tracking RMS | 1.03 mm | 1.82 mm |

What the run shows: with the thrust coefficient wrong, N presses at about **half** the intended force (0.29 N against the 0.6 N the 1 mm indentation reference implies). It keeps crossing the contact-loss threshold, enters the recovery loop 24 times, and never finishes the circle inside the episode cap - visible in the plot after t = 18 s as a regular oscillation in force, tilt and pad error. D's EKF estimates the deficit, presses at 0.51 N, holds contact for the whole trace and finishes on time.

Note the trade: D's *tracking* RMS is worse (1.82 vs 1.03 mm). N's tracking number is computed only over the samples where it was in contact, and it spends 16 % of the trace detached, so the two numbers are not measuring the same thing. Completion and contact fraction are the meaningful comparison here.

Evidence: `compare_circle.mp4` (N left, D right; overview and contact close-up), `compare_circle.png`, `raw_{N,D}_*.mp4`, `summary.json`.

**Scope of this result: one scenario, one family.** It demonstrates that the correction pipeline works end to end and that a classical correction can rescue a propulsion mismatch. It is not evidence about any other family - per-family statistics on the larger `mm2_*` lists are in progress, and `Findings/RL/01_where_the_model_fails.md` records that D changes nothing outside the propulsion family on the v1 lists.

## RL_R4_{big_fig8,lissa3d,circle_xy}_2026-10-31
Side-by-side (zero action | PD+feedforward | RL policy), R4_warm_s1 @1M steps, `--env track4`, start seed 101. Green = reference, orange = actual path. This is one start that survives; at n=20 the policy lost 1/20 starts (seed 111), see Findings/RL/03_endtoend_rl_design.md.

## RL_R5_{big_fig8,lissa3d,circle_xy}_2026-11-04  (rendered 2026-11-04 13:40 EST)
Trajectory + MOVING ARM, R5_turing_s0 final (15M steps), `--env track5`, start seed 101, free flight. Each folder: compare.mp4 (zero | PD+ff | RL side by side) plus zero.mp4, pd.mp4, rl.mp4 full-size. Green = reference, orange = actual base path; header shows base error, tilt and arm joint error. RMS error after 2 s: RL 2.4-2.6 mm vs PD+ff 394-470 mm (zero action crashes). One start per trajectory; n=20 statistics in Findings/RL/03_endtoend_rl_design.md.

## RL_R6_{big_fig8,lissa3d,circle_xy}_2026-11-09  (rendered 2026-11-09 08:25 EST)
Trajectory + moving arm (10-50% of range), posture-aware inner loop, R6_turing_s1 final (15M steps), `--env track6`, start seed 1000 (first held-out seed, chosen before looking at it), free flight. Each folder: compare.mp4 (zero | PD+ff | RL) plus zero.mp4, pd.mp4, rl.mp4. RMS after 2 s: RL 2.7 / 2.9 / 2.7 mm, PD+ff 17.3 / 17.4 / 17.3 mm (big_fig8 / lissa3d / circle_xy). This is a typical start, not the worst: seed 1008 gives 83 mm on big_fig8, and about 1% of starts crash early (Findings/RL/03).

`AM3_BvsN_<path>_2026-10-30_sepB/`: B v5 (separate drone and arm controllers, no information sharing) vs N, same settings as the `_2026-10-30` set (which used the information-sharing B v4). Details: `Findings/Codebase/b_separation_2026-10-30.md`.

## Arm-motion failure recordings (30 Oct 2026, wall-free task, nominal plant; wall geometry still present)
Both: B v5 vs N, same recorder/layout as above; the pad follows a prescribed periodic motion after 3 s of stationary hover. Study and caveats: `Findings/Physics/arm_motion_vs_hover_2026-10-30.md`. The videos were rendered but not watched; claims rest on the metrics JSON and `recording_info.json`.

| Folder | Motion | Result | Files |
|---|---|---|---|
| `AM3_BvsN_arm_oscillation_B_loses_control_2026-10-30` | pad oscillation +-6 cm, half-cycle 0.25 s (about 2 Hz) | B loses control (failsafe abort) but **B's pad hit the wall at 3.784 s (8.13 N) before the abort, so this is not a clean free-flight failure**; N completes | `compare_free_osc0.06_T0.25_{overview_wall,contact_side}.mp4`, `raw_*`, `*_metrics.json`, `*_summary.png`, `*_timeseries.png`, `compose_commands.txt` |
| `AM3_BvsN_arm_circle_2Hz_B_loses_control_2026-10-30` | pad circle radius 6 cm, 2 Hz (pad speed about 0.75 m/s) | B aborts at 4.845 s (tilt 41 deg, base error 114 mm), also in seeds 1-3; no wall contact in the seed-0 logs (peak Fn 0 N); N completes (base 6 mm, tilt 9.9 deg, pad error 1.2 mm RMS, rotor-speed change 80 %, near its limit) | `compare_free_circ_A0.06_f2.0_{overview_wall,contact_side}.mp4`, `raw_*`, `*_metrics.json`, `*_summary.png`, `*_timeseries.png`, `recording_info.json`, `compose_commands.txt` |

Not established: the mechanism of B's loss of control, N's failure boundary, and whether a B with explicit arm/contact compensation or equal controller rate would survive (not run). The discarded mm_027 video (a pre-existing hover offset, not arm-induced) is documented only in `Findings/Codebase/b_failure_search_2026-10-30.md`.

## RL_R8_{big_fig8,lissa3d,circle_xy}_2026-11-09  (rendered 2026-11-09 14:40 EST)
Trajectory + moving arm (10-50% of range), fixed yaw, acceleration limit +-1.75 m/s^2, R8_turing_s1 final (15M steps), `--env track8`, start seed 1000 (same seed as R6, chosen beforehand), free flight. Each folder: compare.mp4 (zero | PD+ff | RL) plus zero.mp4, pd.mp4, rl.mp4. RMS after 2 s: RL 4.8 / 5.5 / 4.9 mm, PD+ff 17.3 / 17.4 / 17.3 mm (big_fig8 / lissa3d / circle_xy). One typical start; statistics in Findings/RL/03.

## NMPC_vs_RL_{big_fig8,lissa3d,circle_xy}_seed{1000,1006}_2026-11-09  (rendered 2026-11-09 17:40 EST)
Coupled NMPC (N, unchanged, frame shifted 0.5 m in x so its internal wall model is clear of free flight) | PD+ff | RL R8_turing_s1, same plant, start, path and arm motion; free flight. Each folder: compare.mp4 plus nmpc.mp4, pd.mp4, rl.mp4; RMS in README.txt. Seed 1000: typical start (N 0.13-0.15 mm, RL 4.8-5.5 mm, PD 17 mm). Seed 1006: chosen as a start N loses on every path (crash at about 0.2 s); RL survives at 39-68 mm, PD 14-20 mm. These are two chosen starts, not statistics; see Findings/RL/03.

## NMPC_vs_RL9_{big_fig8,lissa3d,circle_xy}_seed{1000,1006}_2026-11-09  (rendered 2026-11-09 18:43 EST)
Same as NMPC_vs_RL above, with RL = R9_turing_s0 (selected, acceleration limit +-2.5 m/s^2). RMS after 2 s, seed 1000: N 0.13-0.15 mm, RL 2.67-2.92 mm, PD 17.3-17.4 mm. Seed 1006 (chosen as a start N loses on every path): N crashes, RL 2.87-3.04 mm, PD 13.6-19.9 mm. Two chosen starts, not statistics; see Findings/RL/03.
