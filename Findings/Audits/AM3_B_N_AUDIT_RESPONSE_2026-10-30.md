# Response to the B/N audit of 30 Oct 2026

Audit: `AM3_B_N_AUDIT_2026-10-30.md` (20 findings). Status codes: FIXED (changed and verified), EXPLAINED (evidence added, no change needed to the claim), OPEN (not addressed; the claim is limited accordingly), PARTIAL. Frozen artefacts: `Code/config/am3_freeze_v2.json`; gate `Findings/Codebase/nominal_gate_v2.md`; physics `Findings/Physics/physics_isolation_2026-10-30.md`.

| # | Finding | Status | What was done / what remains |
|---|---|---|---|
| B01 | B oscillates with a stationary arm | **FIXED** (reproduced first) | Reproduced with B v3 (file since deleted with B v3: loss of control after 8.5-16.9 s at three postures; linearised radius 1.0015-1.0023). Retuned with stationary stability as a requirement (stages 5-6); B v4 holds four postures for 30 s with noise, recovers kicks (`hover_check_B.json`). Cost: lower moving-arm accuracy. All earlier B/N results with B v3 are superseded. |
| B02 | Contact detection 4x faster for B | **FIXED** | Persistence in seconds (0.06 / 0.10 s); `am3_check_task.py`: same detection time at 5 ms and 20 ms periods. |
| B03 | Path velocity vs derivative of position | **FIXED** | Velocity scaled by the progress rate; max velocity error 1.2e-6 m/s through ramp and recovery. Acceleration preview in B is the finite difference of the (now consistent) velocity; N uses position and velocity only. |
| B04 | N fails enforced deadlines | **OPEN, disclosed** | Unchanged: N median 15 ms, about 1.5 % of cycles above 20 ms, enforced deadlines lose control. No real-time claim is made anywhere. Remedies (bounded QP iterations, fewer substeps, scheduler) not tried. |
| B05 | Free-arm gate omits tracking | **FIXED** | Free-arm tracking criterion, fail-closed wall metrics, asserted counterexamples (`am3_check_eval.py` exits non-zero if any counterexample passes). |
| B06 | Contact/completion definitions weak | **PARTIAL** | Contact fraction/longest/detached intervals now over the whole contact-intended window including recoveries; measured final gap >= 50 mm required. Open: up to two recoveries are still accepted by the criteria (none occurred in gate v2); the runner still holds one second after `done` (the plan asks to stop commands at done). |
| B07 | Production-step discretisation not validated | **OPEN** | Rollout/contact validation used finer steps than production (5 ms RK4 substeps). No production-step claim is made. |
| B08 | IPOPT/acados parity incomplete | **OPEN** | `solver_parity.json` is not evidence for acquired-contact/trace states or slack equivalence; its `trace` entry was an approach state. Not used for any claim here. |
| B09 | Solver diagnostics incomplete | **OPEN** | Per-attempt diagnostics and native-hang watchdog not implemented (a hung acados call occurred in diverging runs of the ablation; external timeouts were used). |
| B10 | B tuning evidence / ineffective parameters | **PARTIAL** | The linear tool now evaluates the actual gain set (`am3_b_linear.py`; stale knobs in stages 1-4 acknowledged in `controllers_v3.md`); v4 sets the lag-inversion gain to 0. Open: unbounded Newton IK and integral-corrected targets are not checked against limits. |
| B11 | Noise/preview not matched | **PARTIAL** | Noise now a deterministic function of (seed, physical time). Rate, latency and preview differences are disclosed, not removed. |
| B12 | Actuator prediction/scoring | **OPEN** | Servo envelope not in the predictor; `cmd_clip_fraction` is unreliable and is not used in any claim; reserves use nominal coefficients. |
| B13 | Applied-input log labelling | **FIXED** | `u_app` documented as the previous-interval command; `u_next` logs the command that drives the following interval. |
| B14 | Clearance/feasibility partial | **OPEN** | Sampled quasi-static path feasibility only; NMPC clearance uses two point heights. A scenario-feasibility failure was then observed in the gate (heldout2_07: waypoint unreachable within the operational joint limits); the generator has no reachability screen. |
| B15 | Friction validation by pad-centre speed | **PARTIAL** | The step-4 physics check uses native contact forces about the COM; the older speed-binned friction check still uses pad-centre speed and is not claimed as validation of rotation effects. |
| B16 | Path naming | **FIXED (documentation)** | `rounded_triangle` is an open polyline with one rounded corner (curvature jumps at joins); labelled so in videos and docs. |
| B17 | Evaluator tests without assertions | **FIXED** | Asserted expected outcomes incl. metre-scale free-arm error and missing metrics. |
| B18 | Provenance incomplete | **PARTIAL** | Freeze v2 covers package, configs, model, runner/batch/gate/compare/figure/freeze scripts; the gate now requires an explicit acceptance file. Open: per-run `config.json`, generated-library fingerprints, snapshot verification before/after batches. |
| B19 | Media not screen captures | **EXPLAINED / OPEN** | Frames are rendered from the live closed-loop plant (not viewer screen captures); now 1280x720 per panel, 30 fps of simulation time, shadows, 8x MSAA, rotor blades from physical rotor speeds (slowed 100x, stated), frames rendered at exact simulation instants so B and N are time-aligned, trace drawn at the pad contact point with the true-force pressing/detached colouring (stated). No viewer screen recording was made. |
| B20 | Docs overstate/lag | **FIXED** | `controllers_v3.md`, STATUS, TODO, gate notes rewritten; v1 gate marked superseded; N tuning statement corrected. |

## Claims the evidence supports after this response
* Arm reactions and wall loads are transmitted as rigid-body physics requires (momentum balance to 1e-6 to 1e-4); contact alone gives a smooth tilt and a steady rotor-moment demand.
* In the fresh held-out gate N meets the frozen functional criteria in 29/30 and B (stable v4) in 0/30; N's one failure is a scenario whose reference is infeasible within the operational joint limits.
* N's nominal tracking (sub-millimetre) is credible for the nominal model; B is stable but 20-100x less accurate in this setup.

## Claims not supported
Real-time compliance of N; that B's error is *caused* by ignoring the coupling (a plausible contributor, the locked-posture inertia 6x the free-joint inertia, was not isolated); robustness to model mismatch; hardware fidelity of any assumed parameter; any statement about D, L or the decoupled-model ablation from this work.
