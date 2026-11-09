# TODO - checklist from Plan.md (25 Oct 2026: direct end-effector wall trajectory, hover under coupling)

Status key: [x] done with evidence, [~] partial, [ ] open. Evidence paths are in Findings/STATUS.md.

- [x] 0. Reconcile CLAUDE.md / TODO.md / STATUS.md with the 25 Oct plan; update after the 30 Oct audit
- [x] 1. Assembly: continuous mount, compact pad contact, no rod
- [x] 2. Position-only feasibility of line / circle / rounded polyline (sampled)
- [x] 3. Physics isolation: stationary arm, one-joint motion, gentle contact, sliding, fixed-command diagnostic (`Findings/Physics/`)
- [~] 3b. Predictor vs MuJoCo: instantaneous PASS; production-step discretisation, parity at contact, sliding validation open
- [x] 4. Baseline B retuned for stationary stability (v4); audit B01 fixed
- [x] 5. NMPC N; fresh held-out gate 29/30 (one infeasible free-arm scenario)
- [x] 6. B/N recordings and metric figures on three trajectories (2026-10-30)
- [x] 7. Freeze v2 + fresh held-out gate for B and N
- [ ] 7b. N deadline compliance (bounded solves or scheduler), delayed-actuation trials on freeze v2, reachability screen for free-arm scenarios
- [ ] 8. Rerun the NMPC decoupled-model ablation, D (EKF) and L (SAC, training paused) on freeze v2
- [ ] 9. Remaining audit items (B06-B09, B12, B14, B18) and final report
