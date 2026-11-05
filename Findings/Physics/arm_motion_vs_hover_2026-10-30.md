# Does commanding arm motion disturb the drone? B v5 vs N, free flight (30 Oct 2026)

Question (user, 30 Oct): establish, without the wall and without any failure already present before the arm moves, whether arm motion displaces/tilts the drone, and whether N keeps up on exactly the same case. Method: `Code/scripts/am3_arm_motion.py` (runs), `am3_arm_motion_figs.py` (figures); raw `Code/results/am3_arm_motion/A1/` (summary.json + one npz per run). Nominal plant, sensors on, seed 0, no wall contact. The arm holds still for 3 s (pre-motion baseline; the task's hover time is set to 3 s by the script, nothing else changes), then the pad follows the standard `free` loop (waypoints scaled about the start point by 0.5 / 1.0 / 1.5) at 3.0 / 1.5 / 0.8 s per leg: 9 cases, B and N each. Same model, plant, task logic, acceptance-independent metrics. Figures: `arm_motion_2026-10-30/arm_motion_timeseries.png` (motion start, joints, base, tilt, rotors for two cases) and `arm_motion_grid.png` (all cases).

## Result
Before the arm moves (the 1 s before the command at t = 3.0 s): B base 0.81 mm, tilt 0.04 deg; N 0.22 mm, 0.16 deg. Both hover. After the command (window: motion + 0.5 s):
| loop scale / s per leg | peak joint speed (rad/s) | B base disp (mm) | N base disp | B tilt (deg) | N tilt | B rotor change | N rotor change |
|---|---|---|---|---|---|---|---|
| 0.5 / 3.0 | 0.17 | 30 | 0.3 | 0.30 | 0.26 | 3.6 % | 4.6 % |
| 0.5 / 0.8 | 0.6 | 35 | 0.2 | 3.3 | 0.5 | 6.7 % | 6.3 % |
| 1.0 / 3.0 | 0.33 | 64 | 0.3 | 0.61 | 0.27 | 7.5 % | 8.3 % |
| 1.0 / 1.5 | 0.65 | 76 | 0.3 | 2.6 | 0.32 | 9.6 % | 9.2 % |
| 1.0 / 0.8 | 1.2 | 77 | 0.3 | 7.3 | 0.9 | 15.0 % | 12.5 % |
| 1.5 / 3.0 | 0.48 | 101 | 0.3 | 0.96 | 0.28 | 12.0 % | 12.8 % |
| 1.5 / 1.5 | 0.94 | 122 | 0.3 | 4.3 | 0.40 | 15.8 % | 13.9 % |
| 1.5 / 0.8 | 1.76 | 129 | 0.4 | 12.3 | 1.28 | 25.6 % | 19.8 % |
(The 0.5 / 1.5 s row is omitted here: B 35 mm, 1.18 deg, 4.4 %; N 0.2 mm, 0.25 deg, 5.0 %. Full table in `summary.json`.)

* **Arm motion does disturb B, and only N keeps up.** In every case B's base leaves the hover reference starting when the joints start moving (about 0.1-0.4 s after the command in the 3 s cases) and is displaced by 30-129 mm and up to 12.3 deg; N stays within 0.2-0.4 mm and 1.3 deg. B's pad tracking error is 14-67 mm RMS, N 0.1 mm.
* **No catastrophic failure in the tested feasible range.** No abort, no loss of control, no divergence; B's displacement decays after the motion ends (e.g. 1.0 / 1.5: 76 mm peak, about 3 mm at the end of the run; 1.5 / 0.8: 129 mm peak, about 7 mm). The degradation above is the measured effect; it is large, not a collapse.
* **Rotor response is similar in size for both** (N 4.6-19.8 %, B 3.6-25.6 % of the pre-motion speeds, time series in the figure), so N is not working visibly harder; its rotor changes are the ones that offset the arm reaction, B's differ in phase and size and the base moves.
* **What the displacement depends on:** amplitude much more than speed. Slowest legs (3 s, joint speed 0.2-0.5 rad/s, small accelerations) already displace B by 30-101 mm; speeding up mainly raises tilt (0.3 -> 12 deg) and rotor change. In the 1.0 / 3.0 case the base displacement correlates with the joint displacement from the start posture (r = 0.61) more than with joint speed (r = 0.16), and the base is still 21 mm off at t = 6 s with the joints momentarily at rest. This points to a posture-dependent (arm centre-of-mass shift) cause that B's fixed-posture model and trim do not follow, plus a slow recovery, rather than to a purely dynamic reaction to joint acceleration. **Hypothesis, not isolated by an experiment** (B's moment trim and COM are fixed at the start posture; N's model follows the posture).

## Wall contact, examined separately (phases of the existing recordings, nominal plant, `Media_results/AM3_BvsN_*_2026-10-30_sepB`)
Peak base displacement per phase (mm), B v5 / N:
| path | hover | position (arm moves, no wall) | approach | settle | trace | release+retreat |
|---|---|---|---|---|---|---|
| line | 0.6 / 0.3 | 23.0 / 0.3 | 22.3 / 0.2 | 21.1 / 0.3 | 28.1 / 0.4 | 15.1 / 0.4 |
| circle | 0.6 / 0.3 | 5.6 / 0.3 | 8.6 / 0.2 | 17.1 / 0.3 | 22.9 / 0.5 | 10.9 / 0.4 |
| rounded triangle | 0.6 / 0.3 | 22.2 / 0.3 | 21.6 / 0.2 | 22.6 / 0.3 | 17.8 / 0.5 | 12.2 / 0.4 |
* For the line and the triangle B is already displaced 22-23 mm in the position phase, before any wall force (peak Fn 0 N) - the arm motion alone does it; wall contact adds little (22 -> 28 mm line, 22 -> 18 mm triangle). For the circle the position phase is small (5.6 mm) and the displacement grows as contact starts and the pad traces (to 23 mm), so there contact/trace-phase arm motion and wall force are not separated by this data. N is at 0.2-0.5 mm in every phase.
* A free-flight run with the same pad path would be needed to separate the wall force from the arm motion for the circle; not done.

## Limits
One seed, nominal plant, one loop shape scaled in amplitude/time; B rotor trim fixed at the start posture by design. The earlier "mm_027" thrust-mismatch failure is not part of this evidence (see `Findings/Codebase/b_failure_search_2026-10-30.md`).

## Arm oscillation: B loses control, N does not (CONTAMINATED by wall contact: not a clean free-flight failure)
**Correction (4 Nov 2026):** the plant still contains the wall geometry. In the seed-0 oscillation run B's pad touched the wall at 3.784 s (peak Fn 8.13 N, finger_l/finger_r/prop1 contacts) before the 4.355 s abort, after B had already reached about 153 mm and 22 deg. The oscillation therefore shows arm-induced disturbance followed by an unintended collision, not an isolated free-flight failure. The clean case is the 6 cm, 2 Hz circle below (peak Fn 0 N, no unexpected contacts in the seed-0 logs); wall contact was not audited for the other grid runs.
Same script, `--kind osc`: after 3 s of stationary hover (B base 0.81 mm, tilt 0.04 deg before the command) the pad oscillates +-A along (y, z/2), 12 legs of T seconds (half cycle). Grid fixed before running (A = 3 or 6 cm; T = 0.8, 0.5, 0.35, 0.25 s), then the boundary was probed with 5 more noise seeds. Raw: `Code/results/am3_arm_motion/O1` (grid), `O2` (seeds 1-5 and neighbours), figure `arm_motion_2026-10-30/arm_oscillation_timeseries.png`.
| A (cm) / half cycle T (s) | B | N |
|---|---|---|
| 3 / 0.8, 0.5, 0.35, 0.25 | survives: base 33 / 46 / 36 / 27 mm, tilt 3.2 / 11.8 / 6.8 / 3.8 deg | base 0.3-2.8 mm, tilt 0.5-3.9 deg |
| 6 / 0.8, 0.5, 0.35 | survives: base 67 / 111 / 57 mm, tilt 6.7 / **23.9** / 15.4 deg | 0.4-1.9 mm, 0.9-4.1 deg |
| **6 / 0.25 (2 Hz)** | **loss of control (failsafe) 1.35 s after the arm starts: tilt 27 deg, base error 254 mm** | no abort: base 4.9 mm, tilt 8.6 deg, pad error 0.5 mm RMS |
* **Reproducible**: seeds 1-5 give the same outcome. B also aborts at A = 6 cm / T = 0.30 s, A = 5 cm / T = 0.25 s and A = 4 cm / T = 0.25 s (abort 1.4-3 s after motion start; seeds 1-5, `O2/summary.json`); N never aborts in any of these cases (base 3.1-4.9 mm, tilt 5-8.6 deg).
* **Not a pre-existing failure**: B hovers at 0.8 mm / 0.04 deg until the command, the joints start moving, within about 0.25 s the rotors are saturating (B rotor-speed change up to 81 % of the pre-motion speeds), tilt reaches 12 deg at t = 3.5 s and the failsafe (tilt > 40 deg or base error > 0.25 m) fires at t = 4.35 s. The cause is the arm motion alone: no wall, no model mismatch.
* **N is close to its limit too**: at A = 6 cm / T = 0.25 s N's rotor speeds change by 75 % and its tilt reaches 8.6 deg; it keeps the pad on the reference (0.5 mm RMS) and the base within 5 mm. Not tested beyond this amplitude/frequency, so N's own failure boundary is unknown.
* **Mechanism not isolated.** B ignores the arm in the drone loop (by design); here the arm reaction torque and COM shift at 2 Hz drive the 11.6 rad/s (1.8 Hz) attitude loop and the 5 ms-latency drone controller toward large tilt and saturation. Whether this is resonance, rotor saturation or the interplay with the arm controller's fixed-mounting assumption was not tested separately.
* Wall-contact-caused instability was not found: with the wall B degrades (23-59 mm) but never aborts on any tested trajectory or held-out mismatch family (nominal grids in `b_failure_search_2026-10-30.md`).

## Other practical arm motions (continuous periodic pad motion, free flight, nominal plant)
`Code/ocrl/am3_free_motion.py`, runner `am3_arm_motion.py --kind circ|yosc|zosc|reach`: after the 3 s hover the pad follows a smooth periodic motion for 6 s (amplitude ramped in over 1 s): circle of radius A in the y-z plane, side-to-side (yosc), up-down (zosc), reach toward the wall and back (reach, 0 to A). Grid fixed before running: A = 2, 4, 6 cm, f = 1, 1.5, 2, 3 Hz, seed 0 (raw `Code/results/am3_arm_motion/P_<kind>/`), then seeds 1-3 on the cases that aborted B.
* **Circle, radius 6 cm, 2 Hz: B loses control, N does not (reproducible).** B aborts (failsafe: tilt 40 deg) about 1.9 s after the motion starts at seed 0 (t = 4.85 s) and in all three further seeds (`P2_circ`); N completes: base 6 mm, tilt 9.9 deg, pad error 1.2 mm RMS, rotor-speed change 80 % (close to its limit, as above). This is the failure case recorded in `Media_results/AM3_BvsN_arm_circle_2Hz_B_loses_control_2026-10-30/`.
* **Not clean wins:** circle 4 cm at 3 Hz (B aborts at 4.99 s but N tracks poorly: pad error 8.2 mm, tilt 22 deg, no abort); circle and side-to-side 6 cm at 3 Hz (both abort; N at 6.2 s / 7.1 s). Side-to-side 4 cm at 3 Hz: B aborts at 8.5 s in seed 0 but not in seeds 1-3 (base 83-113 mm, tilt 13-21 deg), N completes (base 6-7 mm, pad 2.6 mm): a near-failure, not reproducible.
* **Severe degradation without abort for B** (N at 1-2 mm / 1-2 deg): circle 6 cm at 1 Hz (base 127 mm, tilt 25 deg), side-to-side 6 cm at 2 Hz (195 mm, 13 deg), 6 cm at 1 Hz (111 mm, 23 deg).
* **No failure found:** up-down (zosc, B base 9-36 mm, tilt up to 7 deg) and reach/retract (B base 7-37 mm, tilt up to 5.8 deg) at every tested amplitude and frequency; N also fine. In these directions the arm reaction does not excite B's attitude loop as strongly as sideways/circular motion (observation; reason not tested).
* Same caveats: one loop family per kind, nominal plant, mechanism not isolated, N's own limit untested beyond these ranges, and the circle at 2 Hz asks for a pad speed of about 0.75 m/s, which is aggressive for this arm.
