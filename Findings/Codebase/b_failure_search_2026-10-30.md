# Where does the separated B (v5) fail and N does better? (30 Oct 2026)

Task from the user: search headless for tasks/configurations/motions where B v5 really fails and N does much better; change no model, controller, gain or criterion; do not assist N or weaken B. Method: a grid fixed before running (`config/am3_stress_v1.json`, `am3_stress_v2.json`, runner `scripts/am3_stress.py`; extra paths are injected into the task at run time) plus the existing held-out plant-mismatch set (`am3_domain_mm_heldout.json`, families sampled for the plan, unknown to both controllers). Same plant, sensors, seed, task logic, acceptance criteria (`am3_acceptance_v2.json`) for both. Raw: `Code/results/am3_stress/S1,S2/table.json`, `results/am3_runs/MM_B,MM_N,MM_N2/`.

## Results
**S1, 22 nominal-plant variants (speed, size, start offset):** N passes 22/22, B 0/22. No B abort and no loss of control; B fails the acceptance criteria everywhere (base max 23-59 mm, trace RMS 11-38 mm, contact fraction 0.63-0.89). B gets worse with speed (line T 2.5->1.0 s: base max 39->55 mm) and size (line half-length 8 cm: trace RMS 37.8 mm); N stays at 0.4-1.3 mm.
**S2, speed x size combinations (7):** N passes 6/7 (fails only line 16 cm in 1.0 s, contact fraction 0.95). B 0/7; worst: line 16 cm in 2 s, base max 91 mm and trace RMS 52.9 mm (N 1.3 / 0.98 mm); in 1 s the base max is 98 mm.
**Plant mismatch, B on all 40 held-out scenarios:** 0/40 passes, 4 not completed (4 payload-family scenarios: no contact acquired / task not finished); propulsion family worst (base max 28-164 mm). N was run on the 14 B-worst and the other propulsion scenarios only (not all 40):
| scenario (family) | B | N |
|---|---|---|
| mm_027 propulsion kf x0.907, lag x0.90, rounded triangle | base max 164 mm, trace RMS 124.5 mm, coverage 0 | pass: 11 mm, 0.99 mm, coverage 1.0 |
| mm_022 propulsion kf x0.924, lag x1.14, line | 131 mm, 80.5 mm | pass: 8.5 mm, 0.73 mm |
| mm_007 propulsion kf x0.936, lag x0.98, line | 109 mm, 52.7 mm | pass: 7.0 mm, 0.56 mm |
| mm_002/012/032 propulsion (kf 0.97-1.03) | 29-51 mm, 14-17 mm | pass |
| mm_037 propulsion kf x0.950, lag x0.80, line | 85 mm | **N fails too** (base max 116 mm, trace 0.73 mm) |
| mm_017 propulsion kf x0.941, lag x0.71, rounded triangle | 101 mm, 45 mm | N fails too (base 94 mm, trace 5.7 mm) |
| payload (000, 005, 015, 030, 010, 035) | fail | **N fails too** (3 of the 6 abort for N, no contact acquired or loss of control; the other three finish with 42-56 mm trace error, for B as well) |

## Honest conclusions
* **Where B fails hard and N does much better:** a thrust coefficient lower than assumed (kf x0.91-0.94). N passes at kf 0.907, 0.924, 0.936 but fails at 0.941 (lag x0.71) and 0.950 (lag x0.80), so N is not immune; the failing N cases have the fastest motor lag, cause not investigated. B sags about 130 mm below the hover reference for the whole run, never reaches the wall patch (coverage 0), N holds 7-11 mm and traces to 1 mm. The video of mm_heldout_027 was recorded and then **discarded at the user's request** (metrics kept in `Code/results/am3_stress/mm027_metrics/`). It is the worst of the B results, selected after the search; N's own result there is not perfect (base error 11 mm, tilt up to 5.9 deg, peak Fn 1.6 N, still within the criteria).
* **Mechanism (hypothesis supported by arithmetic, not by an isolating experiment):** B's position integrator is clipped at i_pos_max = 0.3 (gain 1.23 -> 0.37 m/s^2, about 3.7 % of weight); a 9 % thrust deficit cannot be compensated, so B stays at a steady sag. N's model-based loop has no such saturation. Not a defect added for this test: it is a B v4/v5 parameter.
* **Not a clean N win:** the payload family and two propulsion cases (017, 037) defeat both controllers. N was not run on all 40 mismatch scenarios, so no overall mismatch score for N is claimed.
* **Nominal-plant tasks:** B degrades smoothly but does not "really fail" (no abort, bounded 23-98 mm error, contact fraction 0.63-0.89); N passes nearly everything. The largest nominal-plant gap is line 16 cm in 2 s.
* Caveats: one seed per scenario, grids not exhaustive, mismatch magnitudes as sampled for the plan, acceptance criteria unchanged. `am3_compare.py` was edited (scenario option); no freeze v3 has been made, so freeze v2 no longer matches that script.

## Retraction of the use of this failure (user review, 30 Oct)
The mm_027 sag (about 130 mm below the hover reference) is already present at 2 s, before the arm moves, and is caused by the thrust-coefficient mismatch plus the clipped integrator. It is **not** evidence that arm motion disturbs B and does not support the plan's problem statement. The arm-motion evidence is in `arm_motion_vs_hover_2026-10-30.md`.
