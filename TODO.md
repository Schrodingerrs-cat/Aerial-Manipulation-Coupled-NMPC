# TODO - checklist from Plan.md (25 Oct 2026: direct end-effector wall trajectory, hover under coupling)

Status key: [x] done with evidence, [~] partial, [ ] open. Evidence paths are in Findings/STATUS.md.

- [x] 0. Reconcile CLAUDE.md / TODO.md / STATUS.md with the 25 Oct plan
- [x] 1. Assembly: continuous mount, compact pad contact, no rod (design v3 doc + renders)
- [x] 2. Position-only feasibility of line / circle / rounded triangle (acquisition/release checked at samples)
- [~] 3. Predictor vs MuJoCo: kinematics, Jacobians, M, bias, accelerations, contact stiffness/impact PASS; sliding/reversal/loss contact validation open
- [~] 4. Shared task logic + baseline B (tuned on development scenarios; 16/16 development completions)
- [~] 5. NMPC N + decoupled ablation (N 16/16 development completions; weights untuned; timing not real-time on a loaded machine)
- [x] 6. Matched B/N recordings (free-flight arm motion, wall line, wall circle) with plots
- [~] 7. Freeze v1 + 30-case held-out gate DONE; quiet-machine timing, deadline-miss and delayed-actuation trials open
- [~] 8. Mismatch suite, correction interface (zero parity exact), EKF D (dev: N 11/20, D 13/20 joint successes) and SAC L (3 seeds, training PAUSED at about 100k of 200k steps) done; L evaluation, validation-based checkpoint choice, freeze v2 and paired held-out N/D/L open
- [ ] 9. Clean reproduction and final report
