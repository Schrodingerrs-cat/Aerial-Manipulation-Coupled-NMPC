# Nominal held-out gate v2, B versus N (30 Oct 2026)

Supersedes `nominal_gate_v1.md` for B and N (v1 numbers stay as history of the earlier code; they were obtained with a task that contained the defects below and with B v3, which was unstable in stationary hover).

## What changed since v1 (response to the audit, `Findings/Audits/AM3_B_N_AUDIT_RESPONSE_2026-10-30.md`)
* B retuned for stationary stability (B v4); B v3 loses control after 8.5-16.9 s at three arm postures (B01).
* Contact acquisition/loss persistence defined in seconds (0.06 s / 0.10 s) instead of samples: B and N now confirm contact and loss at the same elapsed time (B02).
* Path velocity scaled by the progress rate during progress ramps and recovery (B03); finite-difference check `Code/scripts/am3_check_task.py` (max velocity error 1.2e-6 m/s).
* Evaluator: free-arm runs must meet the pad-tracking RMS criterion, wall runs fail closed without trace/contact metrics, measured final gap after retreat >= 50 mm, contact fractions over the whole contact-intended window including recoveries (B05, B06); asserted counterexamples (`am3_check_eval.py`, exit 0).
* Sensor noise is a deterministic function of (seed, physical time), so B and N see the same noise values at coincident times (B11); command logs carry the next-interval command (`u_next`) (B13).
* New complete freeze `Code/config/am3_freeze_v2.json` (package, configs, model, runner/evaluator/figure scripts) and acceptance `am3_acceptance_v2.json` (v1 criteria plus the final-gap criterion); fresh held-out list `am3_domain_heldout2.json` (seeds disjoint from dev/val/heldout; none of its scenarios was seen during tuning). Gate commands: `am3_batch.py GATE_v2_<N|B> heldout2 <N|B> --workers 8`, `am3_gate.py GATE_v2_<N|B> config/am3_acceptance_v2.json`.

## Result (30 scenarios: 27 wall tasks over seven path/speed variants, 3 free-arm motions)
| | Completed | Meets all frozen criteria | Base error max (median / worst) | Trace tracking RMS (median / worst) | Endpoint | Path coverage (median / worst) | Peak Fn |
|---|---|---|---|---|---|---|---|
| N | 30/30 | **29/30** | 0.58 mm / 113.8 mm (one case) | 0.42 / 0.91 mm | 0.18-0.25 mm | 1.0 / 1.0 | 0.73 N |
| B (v4) | 30/30 | **0/30** | 33.3 / 74.4 mm | 18.3 / 25.7 mm | median 10.0 mm, worst 32.2 mm | 0.77 / 0.60 | 1.00 N |
Neither controller lost contact beyond the allowed recoveries in the wall tasks (0 loss events). B fails the base-error tolerance (20 mm) in all 30 and the tracking tolerance (3 mm) in all 27 wall tasks and 3/3 free-arm motions (pad error about 28-29 mm RMS); in the recorded line run it is dominated by along-track lag (13 mm along, 0.8 mm cross-track), in the recorded circle and polyline runs also by cross-track error (6-7 mm). Tolerances were derived from the task before any result (`am3_acceptance_v1.json` rationale); B's raw error distributions are in `Code/results/am3_runs/GATE_v2_B/table.json` so other thresholds can be judged.

**The N failure (heldout2_07, free-arm).** Base error 113.8 mm, tilt 22.5 deg, rotor reserve exhausted, propeller contacts with the wall, pad error 7 mm RMS. Cause found by inspection, not proven by an intervention: the scenario's predeclared reference is not feasible within the declared operational joint limits. Its initial hover posture already sits at the q2 limit (-4.9 deg of [-80, -5]) and one waypoint cannot be reached by position-only IK within the limits (residual 7 mm at q2 = -5 deg); the base error starts to grow when the arm begins to move at 1 s. The scenario generator has no feasibility screen; the scenario was left unchanged and counts as a failure, 29/30. A reachability screen before freezing is a needed domain fix for any later gate.

## Caveats that apply to this result
* A functional pass only; **timing is not part of it**. N's per-cycle times in the batch (8 concurrent processes) are not valid; the quiet-machine study (`Findings/Codebase/nominal_gate_v1.md`, `Code/results/am3_timing/`) was run on the v1 code and not repeated: N median 15 ms, about 1.5 % of cycles above 20 ms, loss of control when late results are discarded (deadline enforced). N is not real-time compliant.
* B runs at 200 Hz with 5 ms latency and a short preview, N at 50 Hz with 20 ms latency and a 0.2 s preview; B's gains were found with a larger search budget than N's weights; B's stable gains trade moving-arm accuracy for stationary stability. The comparison is practical (architecture, model, rate, tuning), not an isolation of one coupling term.
* The nominal predictor equals the plant up to integration and contact calibration; N's accuracy is an upper bound for mismatched conditions (not evaluated here).
* Not covered here: production-step discretisation validation, IPOPT/acados parity at acquired contact, collision certification, actuator-envelope prediction (audit B07, B08, B12, B14); physical parameters are declared assumptions.

## Post-freeze note
After freeze v2, `Code/scripts/am3_check_eval.py` was edited once (its acceptance file path changed from the draft to `am3_acceptance_v2.json`; the two files have identical criteria). It is a test script, not part of the experiment path; every other file in the manifest still matches its hash.
