# STATUS (updated 30 October 2026)

Historical v2 (33 cm probe) gates, code and results were deleted in the 26 Oct cleanup (backup outside the repo). A B/N audit on 30 Oct found defects; the response and what changed are in `Findings/Audits/AM3_B_N_AUDIT_RESPONSE_2026-10-30.md`. Results obtained with B v3 or the pre-audit task (gate v1, 25 Oct recordings) are superseded.

## Design v3 track
| Stage | Status | Evidence | Caveat |
|---|---|---|---|
| 1 Assembly (continuous mount, direct pad contact, no rod) | DONE | `Findings/Codebase/aerial_manipulator_design_v3.md`, `Code/results/am3_design/` | propulsion/servo/contact values are declared assumptions |
| 2 Workspace / path feasibility | DONE for the three nominal paths (sampled, quasi-static) | `Code/results/am3_task/` | no reachability screen for free-arm scenarios (one held-out free-arm scenario was infeasible) |
| 3 Physics isolation (arm reaction, wall load, sliding, fixed-command) | DONE (explanatory, MuJoCo momentum balance) | `Findings/Physics/physics_isolation_2026-10-30.md`, `Code/results/am3_physics/` | does not prove why B drifts; parameters are assumptions |
| 3b Predictor vs MuJoCo | instantaneous dynamics PASS; production-step, parity at contact, sliding validation OPEN | `Code/results/am3_predictor/` | audit B07/B08/B15 |
| 4 Baseline B | B v4: stable stationary hover (4 postures, noise, kicks); completes all gate scenarios; 0/30 criteria | `Findings/Codebase/controllers_v3.md` | B v3 was unstable in hover (superseded); rate/latency/preview differ from N |
| 5 NMPC N | 29/30 in the fresh held-out gate; one failure on an infeasible free-arm scenario | `Findings/Codebase/nominal_gate_v2.md` | not real-time compliant; enforced deadlines lose control |
| 6 B/N recordings and metric figures | DONE (line, circle, rounded polyline), 2560x1440, with 8-panel time series, summary page and metric table | `Media_results/AM3_BvsN_*_2026-10-30/` | renders of the live run, not viewer screen captures |
| 7 Freeze v2 + fresh held-out gate | DONE for B and N | `Code/config/am3_freeze_v2.json`, `Code/results/am3_runs/GATE_v2_*` | quiet-machine timing from the v1 code; not repeated |
| 8 NMPC decoupled-model ablation, D (EKF), L (SAC) | NOT part of this round; Nd/D/L results predate the audit and must be rerun on freeze v2 before any claim; SAC training paused at about 100k steps | `Code/results/am3_runs/GATE_v1_Nd`, `mmdev_*` (old code) | |


**30 Oct 2026, B v5 (strict drone/arm separation):** see `Findings/Codebase/b_separation_2026-10-30.md`. Separation check passes; hover stable; held-out gate GATE_v3_B 0/30 (base max median 29.8 mm); N unchanged. Recordings: `Media_results/AM3_BvsN_*_2026-10-30_sepB/` (the earlier v4 recordings are kept).
- B-failure search, 30 Oct: `Findings/Codebase/b_failure_search_2026-10-30.md` (documented only; its mm_027 video was discarded because the failure is a pre-existing hover offset, not arm-induced).
- Arm-motion study, 30 Oct: `Findings/Physics/arm_motion_vs_hover_2026-10-30.md`. Free-flight arm loops displace B 30-129 mm (N 0.3 mm); B loses control under a 2 Hz +-6 cm pad oscillation (all seeds) while N completes it.
- Arm-motion failure recordings, 30 Oct: oscillation (+-6 cm, ~2 Hz) and circle (r 6 cm, 2 Hz) in `Media_results/AM3_BvsN_arm_{oscillation,circle_2Hz}_B_loses_control_2026-10-30/`; B aborts in all seeds, N completes (circle: rotor change 80 %, near its limit). Mechanism not isolated.
- Index of all 30 Oct work: `Findings/session_2026-10-30_index.md`. Scripts changed since freeze v2 (`am3_compare.py`, `am3_run.py`, `am3_arm_motion*.py`, `ocrl/am3_free_motion.py`, `am3_ctrl_b.py`, `am3_episode.py`); no freeze v3 yet; nothing committed.
