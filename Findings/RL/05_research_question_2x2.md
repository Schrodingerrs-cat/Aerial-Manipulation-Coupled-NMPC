# The proposal's 2×2 answered [optimal]

2026-11-27. The proposal asked: *which kinds of model error actually need RL, and which are already handled by classical methods?* It guessed the answer with a 2×2 table (error type × speed). This page fills that table in from the final lab-noise benchmark.

## How the evidence was collected

- **Task:** wall writing. Five sentences × 3 seeds = 15 runs per controller and condition, lab sensor noise.
- **Source:** `Code/results/am3_write/2026-11-27_1300_final_table_lab.json`, built from the `bench_turing_1126/` folder.
- **Controllers:**
  - **N** nominal NMPC;
  - **NDF** NMPC + disturbance observer (the classical fix);
  - **L2** NMPC + learned correction with the proposal's 2 channels (normal force, pitch torque);
  - **L2W** L2 trained with wall randomisation;
  - **L9W** 9-channel learned correction;
  - **L9W22** L9W fine-tuned to 2.2× motor lag;
  - **R21** pure RL (outside the proposal's design; shown for reference).
- **Cells:** each condition is placed in the 2×2. "Fast" means the error acts on the fast dynamics or appears suddenly.
- **Cell values:** runs completed (out of 15 per condition) and trace RMS error in mm over the completed runs.

## The answer

| | Parametric (mass, thrust, damping, motor lag) | Non-parametric (contact, friction, airflow) |
|---|---|---|
| **Slow** | Proposal: classical should be enough. **Mostly true.** NDF rescues every payload run that N loses (75/75 vs 45/75). Learning still improves accuracy 2–3× (200 g: L9W 1.2 vs NDF 4.0 mm). | Proposal: classical probably enough. **False for contact.** On a soft wall the observer makes things worse (N 13.4, NDF 17.8 mm); only the corrections trained with wall randomisation fix it (L2W 2.1, L9W 1.4 mm). |
| **Fast / sudden** | Proposal: classical mostly OK. **False.** Motor lag breaks the observer: NDF completes 0/45, and even 0/15 at 1.5× lag, which plain N completes 15/15. Only the corrections fine-tuned for lag and pure RL survive slow motors (L9W22 11/15, R21 14/15). | Proposal: RL should help. **Only for the combination.** A sudden gust and a fast approach are handled best by plain N (1.2 / 1.1 mm); no learning needed. The combined case (payload + lag + gust) is completed only by L9W/L9W22 (15/15; N and NDF 0/15). |

## Detail

| Cell | Condition | N | NDF | L2 | L2W | L9W | L9W22 | R21 |
|---|---|---|---|---|---|---|---|---|
| param, slow | payload 100 g | 0 | 15, 2.8 | 15, 3.1 | 15, 2.4 | 15, 1.2 | 15, 1.3 | 15, 2.3 |
| | payload 200 g | 0 | 15, 4.0 | 15, 4.5 | 15, 3.8 | 15, 1.2 | 15, 1.9 | 7, 2.8 |
| | thrust 90 % | 15, 1.9 | 15, 2.0 | 15, 2.3 | 15, 1.8 | 15, 1.3 | 15, 1.2 | 15, 2.0 |
| | thrust 85 % | 15, 3.5 | 15, 2.0 | 15, 2.5 | 15, 1.8 | 15, 1.2 | 15, 1.2 | 15, 2.1 |
| | joint damping 3× | 15, 1.6 | 15, 2.0 | 15, 2.3 | 15, 2.0 | 15, 1.3 | 15, 1.2 | 15, 1.9 |
| param, fast | motor lag 2× (motor_slow) | 0 | 0 | 0 | 0 | 0 | 11, 1.3 | 14, 2.5 |
| | motor lag 1.5× | 15, 1.9 | 0 | 0 | 0 | 15, 1.3 | 15, 1.1 | 15, 2.0 |
| | motor lag 3× | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| non-param, slow | slow wind | 15, 1.5 | 15, 2.0 | 15, 2.4 | 15, 1.8 | 15, 1.4 | 15, 1.4 | 15, 2.7 |
| | wall k × 0.5 | 15, 13.4 | 15, 17.8 | 15, 17.1 | 15, 2.1 | 15, 1.4 | 15, 1.4 | 15, 1.7 |
| | wall k × 3 | 15, 1.5 | 15, 2.2 | 15, 4.1 | 15, 2.5 | 15, 1.4 | 15, 1.4 | 15, 2.0 |
| | wall μ × 0.5 | 15, 3.3 | 15, 2.1 | 15, 3.4 | 15, 1.9 | 15, 1.6 | 15, 1.7 | 15, 1.6 |
| | wall μ × 2 | 15, 5.9 | 15, 3.6 | 15, 6.2 | 15, 3.2 | 15, 1.6 | 15, 1.4 | 15, 2.6 |
| non-param, fast | gust (1 N, 3 s) | 15, 1.2 | 15, 1.9 | 15, 2.2 | 15, 1.8 | 15, 1.3 | 15, 1.4 | 15, 1.9 |
| | fast approach (3× touch-down) | 15, 1.1 | 15, 1.9 | 15, 2.2 | 15, 1.8 | 15, 1.3 | 15, 1.3 | 15, 2.6 |
| | combo (100 g + lag 1.5× + gust) | 0 | 0 | 0 | 0 | 15, 1.3 | 15, 1.3 | 8, 2.6 |

## What this means

1. **The axis that matters is not slow vs fast; it is whether the error lives where the observer can see it.**
   - The observer estimates a slowly varying force or torque. It handles a constant mass error.
   - It fails when the error is in the actuator dynamics (motor lag) or in how contact force depends on state (a soft wall). In those cases it adds error (N beats NDF at 1.5× lag and on the soft wall).
2. **The proposal's 2-channel learned correction (L2) is no better than the observer.** It fails exactly where NDF fails. The learned correction only beats the observer after two changes:
   - nine channels instead of two (so it can correct the joints and all forces, not just normal force and pitch);
   - training with wall and motor-lag randomisation.

   This is a design result, not a tuning detail.
3. **Plain NMPC is the best controller for sudden external disturbances** (gust, fast approach). Adding an observer or a learned correction costs it 0.2–1 mm there.
4. **Where learning wins:** accuracy under parametric error (2–3× better) and survival under actuator lag and combined errors.
5. **Limits:**
   - 3× motor lag defeats every controller.
   - Pure RL (R21) survives slow motors best but is the least accurate.
   - Its take-off fine-tune (R29) lost that robustness (see CONTEXT 2026-11-27 12:53).

## Contact stiffness outside the training range (2026-11-28)

The wall-randomised corrections (L2W, L9W) were trained on stiffness 0.4–3.5× nominal; R21 and N/NDF never saw stiffness changes.
Benchmark at lab noise, 15 runs each, run on Turing (`Code/results/am3_write/bench/2026-11-27_1322_stress_<ctrl>_lab_wall_k{02,6,10}.json`).
Each cell gives runs completed, trace RMS (mm), share of the letters inked (coverage), and normal-force RMS error against the 0.6 N design force.

| Controller | wall k × 0.2 | wall k × 6 | wall k × 10 |
|---|---|---|---|
| N | 15, **28.6**, 0.14, 0.83 N (pen sinks and drags: not writing) | 15, 1.52, 0.88, 0.40 N | 15, 1.53, 0.87, 0.42 N |
| NDF | **0** (loss of control) | 15, 2.27, 0.73, 0.31 N | 15, 2.25, 0.75, 0.32 N |
| R21 (pure RL) | 15, **1.64**, 0.86, 0.39 N | 15, 2.00, 0.76, 0.27 N | 15, 1.99, 0.76, 0.28 N |
| L2W | **0** (loss of control) | 15, 2.81, 0.71, 0.20 N | 15, 2.90, 0.69, 0.21 N |
| L9W22 | 15, 2.25, 0.86, **0.22 N** | 15, **1.47, 0.91**, 0.51 N | 15, **1.48, 0.90**, 0.55 N |

- **Very soft wall (outside every range):** only the two controllers that react to the measured force keep writing, R21 and L9W22. The observer (NDF) and the 2-channel correction (L2W) lose control; plain N stays up but does not write.
- **Stiff walls:** every controller completes, so stiffness above the training range is harmless here. L9W22 traces best but presses lightly with the largest force error (0.5 N, mean 0.8 N); L2W holds force best (0.2 N) but traces worst.
- **Contact losses** (pressing-to-not-pressing transitions) are in the files but on stiff walls they count micro-bounces around the 0.08 N threshold (hundreds per run for N and L9W22), so they are not reported as a headline number.
- **The proposal's guess** that contact error needs learning is right only for a soft wall, and only for controllers with enough channels or pure RL.

## Not yet covered

- Force error for the older 17-condition bench (it predates the force metrics); a rerun is needed if the report wants it for every condition.
