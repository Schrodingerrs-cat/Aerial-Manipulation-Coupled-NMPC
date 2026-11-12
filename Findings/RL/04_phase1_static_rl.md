# Phase 1: best RL writer on the known model (started 2026-11-11 19:00 EST)

User decision (2026-11-11 ~18:45 EST): start simple and static. Drone and arm are known (mass, links, inertias), so no unknown payload in Phase 1.
Get the RL writer to a high success rate with good accuracy first. Then compare it with NMPC on the same test. Only after that, add robustness (payloads, link lengths), and then the NMPC + RL combination.

Roadmap:
1. Fixed test before training.
2. Score R12 on it.
3. Disturbance observer (TDE idea) in the inner loop.
4. R15 (PPO) and S15 (SAC) on Turing.
5. Sentence video.
6. Phase 2 NMPC on the same test.
7. Phase 3 comparison.
8. Phase 4 robustness and hybrid.

## Test (fixed before any Phase-1 result): `Code/scripts/am3_write_bench.py`
- **Sentences:** HELLO WPI, AERIAL ROBOT, THE QUICK BROWN, FOX JUMPS OVER, THE LAZY DOG. Together they use all 26 letters.
- **Setup:** letter height 5 cm (about 20 cm per 4-letter word), pen speed 50 mm/s, 3 noise seeds each (15 episodes).
- **Sensing:** "real" noise is 1 mm position, 0.5 deg attitude, 0.01 m/s velocity, 0.02 rad/s gyro, 1e-3 rad joint angle, 0.02 rad/s joint rate, 5 rad/s rotor telemetry, 0.02 N force. The old harness noise (0.1 mm, 0.01 deg) was unrealistically small.
- **Latency:** one control period, unchanged.
- **Pass:**
  - all 15 completed
  - mean trace RMS <= 2 mm
  - drag <= 0.02

## R12 on the test (2026-11-11 19:20 EST)

| run | completed | trace RMS mean / max (mm) | coverage | drag |
|---|---|---|---|---|
| R12, old noise | 15/15 | 2.31 / 2.47 | 0.80 | 0.000 |
| R12, real noise | 15/15 | 2.32 / 2.50 | 0.80 | 0.001 |
| R12 + observer (not retrained), real noise | 15/15 | 2.59 / 2.77 | 0.78 | 0.002 |

R12 already completes every sentence, and realistic noise barely changes it. It misses the 2 mm target by about 0.3 mm.

## Where R12's error comes from (HELLO WPI, seed 0, real noise)
- **Zero mean:** mean yz error is -0.1 / -0.3 mm, so this is not an offset.
- **No lag:** the best time shift is 0 ms.
- **Grows with pen speed:** RMS 0.86 mm below 10 mm/s, 1.6 mm at 10-40, 2.0 mm at 40-70, 2.9 mm above 70 mm/s. The correlation of |e| with speed is 0.50; with reference acceleration it is 0.29.
- **Base wobble:** the base error during strokes is about 2 mm RMS per axis, which the arm has to absorb.
- **Friction hypothesis rejected (diagnostic only, not a result):** lowering the wall/pad friction from 0.15 to 0.001 makes it slightly worse (2.00 -> 2.47 mm).

## Inner-loop disturbance observer (`InnerLoop.observe`, `ocrl/am3_rl_direct.py`)
- **What it is:** a generalized-momentum observer on the full coupled model `M(x) nu' + b + G = tau_rot + tau_j + d`. This is the TDE idea of Singh et al., but it does not differentiate noisy rates.
  - It uses the measured rotor speeds, so motor lag and latency are inside the estimate.
  - d is subtracted from the commanded base force, body moment and joint torques.
- **Off by default:** R12 parity is bit-exact (2.002 / 7.138 mm on HELLO WPI, seed 0).
- **History of attempts:**
  1. Base-only observer at gain 10: unstable.
  2. With realistic noise, even gain 3 oscillated, driven by attitude noise. Cause: the arm's reaction on the soft joint servo was counted as a base disturbance, so the observer fought the arm mode.
  3. The full-model observer (arm coupling inside M) fixed it.
- **Sweep (scripted PD outer loop, real noise, 6 s):**

  | gains (force, moment, joint) | base error last 2 s, no payload | 100 g | 200 g |
  |---|---|---|---|
  | off | 17 mm | 135 mm | lost (failsafe) |
  | 10, 10, 0 | 18 mm | 51 mm | 96 mm, survives |

- **Payload estimate is correct:** at 100 g the observer reports d_z = -0.98 N and a moment of 0.21 N m. The remaining offset with payload is the arm sagging, because the joint part of the observer is off. With joint gain > 0 under the untrained R12, the error went up (2.4-3.8 mm) and one run was lost at gain 10/10. That is expected: R12 learned to press the pen using exactly what the observer cancels. The joint part is left at 0 until a policy is trained with it.
- **Chosen:** force 10, moment 10, joint 0.

## Runs on Turing
- **R15_write15_s0 (job 2510960, PPO):**
  - Warm start from R12, with the input widened 75 -> 300 (4-step observation history; the new inputs start at zero weight, so step 0 behaves exactly like R12).
  - Observer on, realistic sensor noise in training.
  - Extra reward `+1.0 * exp(-(e/1 mm)^2)` while tracing.
  - lr 1e-4 with decay, 30M steps.
  - Env kind `write15`; the options are in `KIND_KW`, and `am3_write_eval.make` applies them from config.json.
- **S15_write15_sac_s0 (job 2510984, SAC):**
  - Same env, from scratch: lr 3e-4, batch 512, 8 gradient steps per vector step of 30 envs, 12M steps.
  - Question: can off-policy SAC learn the task where from-scratch PPO could not (R10)?
  - Cost note: about 25 ms per gradient step on one CPU thread, so the replay ratio is low (0.27).

Next, when they finish: score both on the test, render the sentence video, and run NMPC (N) on the same test (running locally now, `results/am3_write/bench/N_real.log`).

## NMPC (N) on the test (2026-11-11 20:43 EST)

| run | completed | trace RMS mean / max (mm) | coverage | drag |
|---|---|---|---|---|
| N, real noise | 15/15 | 2.07 / 2.22 | 0.87 | 0.005 |
| R12, real noise | 15/15 | 2.32 / 2.50 | 0.80 | 0.001 |

NMPC is slightly better than R12 (0.25 mm, higher coverage), and also just misses 2 mm. Neither passes yet. Log: `Code/results/am3_write/bench/N_real.log`.
