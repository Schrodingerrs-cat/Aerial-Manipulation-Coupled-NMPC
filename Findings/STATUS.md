# STATUS (updated 26 October 2026)

Historical v2 (33 cm probe) gates, code and results were deleted in the 26 Oct cleanup (backup outside the repo); they were never evidence for the current task.

## Revised task (25 Oct plan): design v3 track (updated as work proceeds; evidence = files named, nothing here is a final result)
| Stage | Status | Evidence | Caveat |
|---|---|---|---|
| 1 Assembly (continuous mount, direct pad contact, no rod) | DONE (static + renders) | `Findings/Codebase/aerial_manipulator_design_v3.md`, `Code/results/am3_design/` | propulsion/servo/contact values are declared assumptions |
| 2 Workspace / path feasibility | DONE for line, circle, rounded triangle (quasi-static attitude) | `Code/results/am3_task/feasibility_*.json`, `Code/scripts/am3_feasibility.py` | patch limited to |y| <= 5 cm by yaw authority; acquisition/release IK checked only at sample points |
| 3 Independent predictor vs MuJoCo | PASS (FK/Jacobian/M/bias/acc 1e-9..5e-6; contact k_eff 600.2 N/m, impact ratios 0.99-1.0; 0.2 s rollouts 0.2 mm) | `Code/results/am3_predictor/` | friction/sliding/reversal validation still open |
| 4 Shared task logic + baseline B | B runs the full sequence on all three paths; gains from a development search (budget logged) | `Code/results/am3_b_tuning/`, `Code/results/am3_runs/dev_B_*` | B runs at 200 Hz (its compute is microseconds); second-stage random search running |
| 5 NMPC N (+ decoupled ablation) | N completes all three paths in development runs; ablation Nd first runs in progress | `Code/results/am3_runs/dev_N_*` | single-thread timing: median 16 ms, max 40 ms (> 20 ms period in some cycles); weights not yet tuned beyond first guess |
| 6 B/N matched runs + recordings | DONE: free-flight arm motion, wall line, wall circle (B and N each) | `Media_results/AM3_BvsN_*`, README section | B at 200 Hz; N circle/line also covered by gate |
| 7 Freeze, gate, timing | Freeze v1 and nominal held-out gate DONE (N 30/30, B 1/30 with 30/30 completed, Nd 0/30); quiet-machine timing + injected/enforced deadline trials DONE (N median 15 ms but ~1.5 % of cycles > 20 ms, enforced deadlines make N lose control; random 2-10 % misses survived by N); delayed-actuation sweep OPEN | `Findings/Codebase/nominal_gate_v1.md`, `Code/config/am3_freeze_v1.json`, `Code/results/am3_runs/GATE_v1_*` | batch timing invalid (8 concurrent processes); N not real-time compliant so far |
| 8 D / L | OPEN | | |
