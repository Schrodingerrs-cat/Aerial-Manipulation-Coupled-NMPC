# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.


## Success Criteria — NON-NEGOTIABLE

**A controller is NOT considered successful merely because numerical metrics look good. It MUST demonstrably work in the actual simulation.**

The primary proof of success is a working, reproducible simulation showing the controller performing its intended task. Numerical metrics are required to support that demonstration, not replace it.

### Mandatory Requirements

1. **Simulation is the final authority.** Every controller implementation must be validated in the actual simulation. Show the controller operating, the system responding, and the intended behavior being achieved. Provide a reproducible simulation run and visual evidence whenever rendering is available. No rendering, actual screenrecord.

2. **No shortcuts. No cheating. No manufactured success.** Never manipulate simulation physics, initial conditions, contact parameters, evaluation metrics, reference trajectories, or success thresholds simply to make results look better. Never substitute scripted behavior, hardcoded actions, or artificial demonstrations for actual closed-loop controller performance.

3. **Failure is not proof of impossibility.** A failed experiment does NOT justify declaring an approach infeasible, fundamentally limited, or outside scope. Investigate the failure, identify its cause, form a hypothesis, implement a justified correction, and test again. Distinguish implementation bugs, tuning problems, numerical issues, and actual methodological limitations.

4. **Do not silently change the research plan.** Preserve the original objectives, methodology, and evaluation criteria. Implementation-level corrections and justified tuning are allowed. Any substantive deviation from the agreed approach MUST be explained and approved before proceeding. Never quietly simplify the problem to achieve success.

5. **Be persistent, but scientific.** Do not give up after one or several failed attempts. Explore reasonable solutions systematically, record meaningful findings, and use evidence to determine the next step. Do not repeat unsuccessful experiments without a clear reason.

6. **Report limitations honestly.** If an approach genuinely cannot achieve the intended behavior after meaningful investigation, report the evidence, attempted remedies, likely root causes, and remaining possibilities. Clearly distinguish confirmed limitations from unresolved failures. Never claim success without satisfying the original criteria.

7. **Make all results auditable.** Preserve the relevant configurations, commands, logs, metrics, and simulation artifacts needed to reproduce important results. Clearly distinguish successful runs from failed or incomplete experiments.

### Definition of Done

An implementation is complete ONLY when:

- The intended controller is actually implemented and running in closed loop.
- The simulation visibly demonstrates the intended behavior.
- Quantitative results support the observed behavior.
- The result is reproducible without hidden interventions.
- The original objectives and evaluation criteria are satisfied.
- There are no concealed failures, artificial demonstrations, or unapproved shortcuts.

**If the simulation does not demonstrate the intended behavior, the controller is NOT DONE, regardless of how promising the numerical results appear.**

---

## Code Quality — KEEP IT SIMPLE

Write clean, minimal, readable, and auditable code.

1. **Simplicity first.** Prefer straightforward implementations over clever abstractions or unnecessarily complex architectures.
2. **Minimal comments.** Add only short, meaningful comments explaining non-obvious logic, important mathematical reasoning, or critical implementation decisions. Do not narrate obvious code.
3. **No unnecessary complexity.** Avoid excessive helper functions, classes, abstraction layers, defensive wrappers, or dependencies when simpler solutions are sufficient.
4. **Readable structure.** Use descriptive variable names, small functions where appropriate, and clear execution flow.
5. **No premature engineering.** Do not introduce frameworks, extensibility mechanisms, or elaborate design patterns for hypothetical future requirements.
6. **Scientific transparency.** Keep the implementation close to the underlying mathematical formulation so that the controller logic is easy to inspect and verify.
7. **No unnecessary code.** Remove abandoned experiments, dead code, redundant logic, and debugging clutter from the final implementation, while preserving relevant experimental evidence separately.

**The objective is code that I can open, understand, verify, and audit without having to reverse-engineer unnecessary complexity.**

### Guiding Principle

**Correctness over convenience. Evidence over claims. Simplicity over cleverness. Scientific integrity over impressive-looking results.**

---

## Current state

- `Plan.md` (25 Oct 2026) is the authoritative plan: the supplied aerial manipulator (three active joints, fixed wrist) holds a fixed hover-position reference while its own end effector tracks wall trajectories in contact; coupled NMPC N versus coupling-neglecting baseline B first (plus an NMPC coupling ablation), then N vs classical (D) vs learned (L) correction. No probe, no pointing/alignment objective, no force-setpoint task. Follow it as written; do not change its objectives or criteria without approval. Design parameters (wrist angle, mount, pad, joint ranges, propulsion) are the implementer's to derive, document and version. `TODO.md` is the checklist, `Findings/STATUS.md` the gate table.
- Repository cleaned on 26 Oct 2026: the historical R2 and v2 (33 cm probe) code, configs, results, recordings, audits and plan snapshots were deleted at the user's request (backup outside the repo: `~/project_OCRL_pre_cleanup_2026-10-26.tar.gz`, without raw logs). `Model_candidate_2026-10-21/` (supplied originals), `proposal.pdf` and `SOURCE_TECHNICAL_REFERENCE.*` stay.
- Working files: `Code/model/aerial_manipulator_v3.xml`, `Code/ocrl/am3_*.py`, `Code/scripts/am3_*.py`, `Code/config/am3_*.json`, `Findings/Codebase/{aerial_manipulator_design_v3,controllers_v3,nominal_gate_v2}.md`, `Findings/Physics/`, `Findings/Audits/`. Frozen files (listed in `Code/config/am3_freeze_v2.json`; v1 is historical) are not edited without a new freeze version. New results go in `Code/results/<experiment>/<tag>/`, recordings in `Media_results/` following its README.
- Third party (unmodified): `Code/third_party/{MuJoCo-drones-gym,acados}` (acados built). MuJoCo 3.10, CasADi 3.8.1, stable-baselines3 2.7.0, Python 3.10.
- Report what changed, the command run, the observed result and remaining uncertainty; never claim installs, runs, convergence or results that were not actually performed.
