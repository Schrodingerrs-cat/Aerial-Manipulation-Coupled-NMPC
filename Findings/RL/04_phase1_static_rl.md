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

## R15 on the test (2026-11-12 18:20 EST): first Phase-1 pass

R15 finished on Turing (30M steps, 5.3 h, reward/step 22.3, flat for the last ~5M steps).

| run | completed | trace RMS mean / max (mm) | coverage | drag | max tilt |
|---|---|---|---|---|---|
| **R15 final, real noise** | **15/15** | **1.74 / 1.86** | 0.83 | 0.000 | 4.2-5.9 deg |
| N (NMPC), real noise | 15/15 | 2.07 / 2.22 | 0.87 | 0.005 | 7.4-9.6 deg |
| R12, real noise | 15/15 | 2.32 / 2.50 | 0.80 | 0.001 | |

- R15 meets every pass condition (15/15, mean <= 2 mm, drag 0). It is 25% better than R12 and 16% better than NMPC, with a calmer base (tilt 4-6 deg vs 7-10 deg).
- What changed from R12: 4-step observation history, inner-loop observer (force/moment), realistic sensor noise in training, a sharp 1 mm reward term. Not separated yet (one ablation each would be needed to say which part helps).
- Log: `Code/results/am3_write/bench/R15_final_real.log`. Checkpoints 10M and 20M are being scored.
- Video under the same conditions (HELLO WPI, seed 0, real noise; N vs R12 vs R15): rendering, `Media_results/Write_bench_HELLO_WPI_NMPC_vs_R12_vs_R15_2026-11-12/`.
- Phase 3 started: the same test with each mismatch case (payload 100/200 g, kf -10%, slow motors, 3x joint damping, gust, wind) for N and R15. Logs `bench/{N,R15}_real_<case>.log`.

### R15 checkpoints (2026-11-12 18:30 EST)
10M 2.24 mm, 20M 1.89 mm, 30M 1.74 mm (all 15/15). Still improving at the end, so **R16** continues R15 on Turing (job 2514759, warm start from R15 final, lr 5e-5 with decay, 30M).

### SAC (S15) at 6M steps (2026-11-12 18:35 EST)
0/15: "no contact acquired" or loss of control in every episode. Reward/step 10.7 vs PPO-from-R12 22.3. From scratch it has not learned the contact task, the same as from-scratch PPO (R10). It is slow on Turing (0.5M steps per 86 min) and the 24 h limit will stop it near 8M; its last checkpoint will be scored for the record. `am3_write_eval.make` now loads SAC zips (reads `algo` from config.json).

### Video (2026-11-12 19:05 EST)
`Media_results/Write_bench_HELLO_WPI_NMPC_vs_R12_vs_R15_2026-11-12/`: compare.mp4 (N | R12 | R15) plus one file per controller. HELLO WPI, letter height 5 cm, 50 mm/s, real sensor noise, seed 0. All three write the full text; R15 1.61 mm, R12 2.00 mm, N 2.05 mm on this run. The first R15 clip went black after 47 s (an OpenGL context left open by the earlier clips, not the controller); it was re-rendered alone with identical metrics, and the script now closes each renderer.

## Phase 3: same test under model mismatch (real noise, 15 episodes each)

| case | N completed / RMS (mm) | R15 completed / RMS (mm) |
|---|---|---|
| nominal | 15/15, 2.07 | **15/15, 1.74** |
| payload 100 g | 0/15 | 0/15 |
| payload 200 g | 0/15 | 0/15 |
| thrust -10% (kf90) | **15/15, 2.70** | 1/15 |
| motor lag x2 | 0/15 | 0/15 |
| joint damping x3 | running | 15/15, 1.84 |
| gust 1 N (y) | running | 15/15, 1.80 |
| sine wind | running | 15/15, 2.32 |

First read (N rows still filling in):
- Both known-model controllers break on payload and slow motors. That is the Phase-4 robustness target, as planned.
- R15 handles disturbances it can see (damping, gust, wind) but not a thrust loss. NMPC's model-based feedforward tolerates -10% thrust; R15's does not, even with the force observer on (it never makes contact). That is the first concrete place where the model helps and RL fails.

### R15 with the observer switched off (2026-11-12 19:00 EST): ablation and the kf90 cause

| case | R15 observer on (as trained) | R15 observer off |
|---|---|---|
| nominal | 15/15, 1.74 mm | 15/15, 2.55 mm |
| kf90 | 1/15 | 15/15, 4.52 mm, drag 0.075 |
| payload 100 g | 0/15 | 12/15, 25 mm (not writing) |
| payload 200 g | 0/15 | 0/15 |
| motor lag x2 | 0/15 | 0/15 |

- The observer is worth about 0.8 mm of R15's nominal accuracy.
- Under kf90 with the observer on, R15 backs off from the wall: base -6 mm in x, pad 2.9 mm short of the reference, pad force 0.09 N (contact needs > 0.15 N for 60 ms), so contact is never declared. With it off the base sags 6 mm in z instead and contact is made. The observer itself reads the thrust loss correctly (d_z = -1.6 N); the policy has never seen an episode where the observer carries a large bias, so its behaviour there is untrained. Phase 4 training (R17) includes this.
- `am3_write_bench.py --dob on|off` now overrides the observer (default: as trained). The old `--dob` flag became `--dob on`.

## Phase 4 started (2026-11-12 19:20 EST)

### RL: R17 (Turing job 2514876)
R15 + domain randomization, ramped in over the first 300 episodes per env. The ranges are R13's (payload 0-250 g, thrust 0.88-1.08, motor lag 1-2x, joint damping 0.5-4x, sine wind, steps), so they cover every bench case. Everything else is R15's (history 4, observer, real noise, 1 mm reward). Warm start from R15, lr 1e-4 with decay, 30M. Env kind `write15_dr`.
S15 (SAC) was cancelled to free the slot: 0/15 at 6M, the last checkpoint.

### NMPC: offset-free NMPC "ND" (`ocrl/am3_nmpc_dob.py`, new; N's frozen files unchanged)
- **What:** N plus the generalized-momentum observer, but with N's own predictor (contact model included) as the model, so d is exactly what N's prediction lacks. d (world force, body moment, joint torques) enters the prediction as a constant over the horizon: `M(x) nu' = rhs(x, u) + d`. Costs, constraints, weights and solver settings are N's. The acados build is N's with the parameter vector extended by 9.
- **Timing:** the command that acted on the period that just ended is the previous pending command (one-period latency), so the observer uses that, not the current one.
- **Parity:** `ND:0` (all gains 0) reproduces N exactly on HELLO WPI seed 0 (2.048 / 6.855 mm, coverage 0.897).
- **First checks (HELLO WPI seed 0, real noise):**

  | ND variant | nominal | payload 100 g | kf90 |
  |---|---|---|---|
  | base gains 10, joints 0 | 2.23 mm | loss of control in 1 s | 2.29 mm |
  | base 10, joints 10 | 2.29 mm on 6/11 strokes* | survives, 2.50 mm on 6/11 strokes* | 2.39 mm on 7/11* |

  \* These stopped at the harness's 400 s wall-clock guard because 7 acados processes ran at once. Run alone, `ND:10,10` solves in 9 ms mean, 0 failed QPs over 70 s. The full test is now running one process per chain.
- **Joint channel is required for payload:** without it the arm sags under the payload (q3 -1.87 -> -1.50 rad in 0.5 s), N's prediction is wrong, the rotor commands chatter 180-1800 rad/s and control is lost. With it, the observer reads the payload correctly: d_z = -1.0 N (0.1 kg x g), joint torques 0.20 / 0.13 N m.
- N itself under payload 100 g: the arm drifts (q2 -0.23 -> -0.45, q3 -1.87 -> -1.10 rad) and ends in "recovery timeout".

## Phase 4 results (2026-11-17 14:30 EST)

### ND fix: the observer was cancelling the pen force
- **Symptom:** every ND sentence hit the 400 s wall-clock guard, locally and on Turing (job 2515310, one core per process), after 1-8 of 11 strokes. Timing alone showed ND is real-time (9 ms solve, 0 failed QPs). So simulated time was the problem: a 410 s sim run wrote 3 strokes.
- **Cause:** a phase trace showed ND looping trace -> recover -> settle_resume about 15 times on stroke 0 (N: one pass). The observer used N's contact model inside rhs, so every error in predicted pen force went into d. The offset-free prediction then cancelled it and the pen eased off below the 0.15 N contact threshold.
- **Fix** (`am3_nmpc_dob.py`, `observe`): the wall-normal force in rhs is now the measured one, through the predictor's existing normal-force input d[0] = Fm - Fn_model. d then holds only non-contact error. After the fix, stroke 0 runs approach -> settle -> trace -> end, as N does.

### Writing test, real noise, 15 sentences per cell (completed / mean trace RMS mm)

| case | N | R15 | R16 | ND (fixed) | R17 (DR) |
|---|---|---|---|---|---|
| nominal | 15, 2.07 | 15, 1.74 | 15, 1.76 | 15, 2.62 | 15, 2.40 |
| payload 100 g | 0 | 0 | | **15, 2.65** | 0 (no contact) |
| payload 200 g | 0 | 0 | | **15, 3.05** | 0 (loss of control) |
| motor lag x2 | 0 | 0 | | 0 (loss of control < 4 s) | **15, 2.93** |
| damping 3x | 15, 2.31 | 15, 1.84 | | 15, 2.77 | 15, 2.43 |
| kf90 | 15, 2.70 | 1 | 1 (others time out) | 15, 2.59 | 9, 2.23 |
| gust | 15, 2.03 | 15, 1.80 | | 15, 2.66 | 15, 2.43 |
| sine wind | 15, 2.23 | 15, 2.32 | | 15, 2.70 | 15, 2.99 |

Drag is at most 0.008 everywhere. Logs: `Code/results/am3_write/bench/*_real*.log`. The Turing ND logs (before the fix) are in `bench_turing_1113/`.

Reading:
- **R16** (R15 plus 30M steps at lr 5e-5) gives no gain: 1.76 vs 1.74 mm. R15 is converged on the known model.
- **ND** is the first controller to write with a payload (100 g and 200 g) and keeps N's thrust tolerance. It costs 0.55 mm on nominal (2.62 vs 2.07): the observer passes sensor noise into the prediction. It still fails slow motors.
- **R17** is the first controller to survive motor lag x2 (15/15). It is worse than R15 on nominal (2.40 vs 1.74) and fails both payloads. Its training reward fell from 19 to about 7-10, below the scripted PD's 10.6.
- **ND and R17 fail on different cases.** This is the concrete case for the hybrid NMPC+RL step.

### Why R17 fails on payload: the joint channel of the RL observer
The RL inner loop's observer had its joint channel off: it oscillated at gain 3 under noise in the R15 sweep. ND showed the arm sags under payload without that channel. Test, with no retraining (R17, HELLO WPI seed 0, payload 100 g, real noise):

| joint gain (1/s) | result |
|---|---|
| 0 (as trained) | no contact acquired, 4/11 strokes; joint 1 mean 2.6 deg off, joint 2 sags 3 deg |
| 2 | **completed 11/11, 2.84 mm**, tilt 5.3 deg |
| 5 | completed 11/11, 3.10 mm |

No oscillation at 2 or 5 here.

### Retraining (Turing, submitted 2026-11-17 14:15 EST)
New env kind `write15_drj` is `write15_dr` with the joint gain at 2 (`dob_kj`, `KIND_KW`). 20M steps, lr 1e-4 with decay, target_kl 0.02.
- R18_write15drj_s0 (job 2532420): warm start from R17.
- R19_write15drj_fromR16_s0 (job 2532421): warm start from R16, the best nominal writer.

### R18/R19 mid-training check (2026-11-17 16:18 EST; real noise, 15 each)

| checkpoint | nominal | payload 100 g | motor lag x2 |
|---|---|---|---|
| R18 at 11.5M (from R17) | 6/15, 3.42 (no contact) | 8/15, 3.46 | 3/15, 4.13 |
| R19 at 8M (from R16) | 15/15, 2.41 | **13/15, 2.77** | 0/15 |

Mid-run only: the randomization is still ramping and lr is still high, so these are not results. R19 now writes with payload, which R16 could not do (0/15). R18 has lost nominal contact reliability for now. Final checkpoints are benched on all 8 cases.

### R18 final (2026-11-17 17:55 EST; real noise, 15 each)

| case | R18 (from R17, joint gain 2, DR) |
|---|---|
| nominal | 8/15, 2.82 |
| payload 100 g | **13/15, 3.24** |
| payload 200 g | 1/15 |
| motor lag x2 | **14/15, 3.52** |
| damping 3x | 8/15, 2.97 |
| kf90 | 10/15, 2.83 |
| gust | 8/15, 2.91 |
| sine wind | 7/15, 3.48 |

All failures except payload 200 g end with "no contact acquired".
- **Payload and motor lag together:** R18 is the first policy to handle both. The joint-channel fix works.
- **Fails late in long sentences:** HELLO WPI and THE LAZY DOG complete on every seed. THE QUICK BROWN and FOX JUMPS OVER fail on every seed, at stroke 11-19.

**Cause** (THE QUICK BROWN, seed 0, nominal; per approach):
- **Fading force:** R18's peak approach force falls from 0.36-0.41 N at the start to 0.26 N at stroke 16. Approach time grows from 2-3 s to 8 s, and then the no-contact timeout fires.
- **R15 shows the same drift and survives:** base x 507.7 -> 506.0 mm, z -10 mm over the sentence. But R15 presses at 0.30-0.42 N throughout and finishes 20/20. DR made R18 press more gently, so the same drift costs it contact.
- **Training never shows this:** training writing episodes are 1-2 letters, 10-20 s (`am3_rl_wall.py` reset). Test sentences are 9-15 letters, about 100 s.

**Retrain R20** (Turing job 2533832, submitted 2026-11-17 18:00 EST):
- New env `write15_drjl` = `write15_drj` with `write_max=5`: writing episodes have 1-5 letters, 13-64 s.
- Warm start from R18, 20M, lr 1e-4 with decay, target_kl 0.02.

### R19 final (2026-11-17 19:18 EST; real noise, 15 each) [optimal]

| case | R19 (from R16, write15_drj, 20M) |
|---|---|
| nominal | 15/15, 2.70 |
| payload 100 g | 15/15, 3.18 |
| payload 200 g | **0/15** (flips in 2-3 s) |
| motor lag x2 | 15/15, 3.09 |
| damping 3x | 15/15, 2.79 |
| kf90 | 15/15, 3.04 |
| gust | 15/15, 2.70 |
| sine wind | 15/15, 2.84 |

- **Most robust controller so far:** 7 of 8 cases complete on every run. N, ND and R17 each fail at least one of payload or motor lag.
- **Accuracy cost:** RMS is 2.7-3.2 mm, above the 2 mm pass line. R15 was 1.74 mm nominal with no robustness. So R19 is robust but not yet accurate.

**Why 200 g fails** (HELLO WPI, seed 0; scratchpad traces `pl_sweep.py`, `pl_trace.py`):
- **Sweep:** 120-180 g all complete (RMS 2.89-3.13 mm). Max tilt grows 5 -> 6 -> 9 deg. 200 g and 220 g flip before the first stroke.
- **Nominal (0 g):** smooth, tilt < 2 deg, shoulder torque -0.4 N m.
- **With payload:** at 180 g the arm and base oscillate during the approach (shoulder angle swings -13 to -39 deg, tilt 3-10 deg). The payload alone takes the static shoulder torque to about -0.8 N m, against a 1.0 N m servo limit. At 200 g the oscillation pushes the shoulder into saturation (-1.00 N m), the arm drops, and the base flips (40 deg).
- **The observer's joint gain sets the oscillation:** R19 was trained with gain 2. With gain 1 it flips even at 180 g. With gain 4, 180 g and 200 g both complete HELLO WPI. Full 8-case bench at gain 4 below.
- **Mass feedforward would not help here:** the observer already reads the payload correctly (d_Fz = -1.8 N for 180 g). The limit is servo torque margin, not estimation.

**R19 with joint gain 4, no retraining** (`am3_write_bench.py ... --kj 4 --name R19kj4`, 2026-11-17 19:35 EST):

| case | R19 (gain 2, as trained) | R19 gain 4 |
|---|---|---|
| nominal | 15/15, 2.70 | 15/15, 2.70 |
| payload 100 g | 15/15, 3.18 | 15/15, 3.22 |
| payload 200 g | 0/15 | **1/15**, 3.29 |
| motor lag x2 | 15/15, 3.09 | 14/15, 3.12 |
| damping 3x | 15/15, 2.79 | 15/15, 2.79 |
| kf90 | 15/15, 3.04 | 15/15, 3.05 |
| gust | 15/15, 2.70 | 15/15, 2.71 |
| sine wind | 15/15, 2.84 | 15/15, 2.91 |

- **Only a partial fix:** at 200 g, gain 4 gets the drone writing (2-16 strokes instead of 0) but it still flips mid-sentence (tilt 40 deg) or drifts off the base (255 mm).
- **Other cases:** unchanged except motor lag, which drops to 14/15.
- **Conclusion:** 200 g sits at the shoulder torque margin, and the policy has never learned to keep away from it.

**Retrain R21** (Turing job 2534328, submitted 2026-11-17 19:55 EST):
- New env `write15_drjt` = `write15_drjl` (writing of up to 5 letters) + joint gain 4 + `tau_w=1`.
- `tau_w` adds a reward penalty on the largest joint torque command above 80 % of the 1.0 N m servo limit, so the policy learns to keep torque margin.
- Warm start from R19, 20M steps, lr 1e-4 with decay, target_kl 0.02.
- **Reward change, disclosed:** the task, metrics and pass criteria are unchanged.

### R20 final (2026-11-17 21:34 EST; from R18, write15_drjl, up to 5 letters; real noise, 15 each) [optimal]

| case | R18 (parent) | R20 | R19 |
|---|---|---|---|
| nominal | 8/15, 2.82 | **15/15, 2.53** | 15/15, 2.70 |
| payload 100 g | 13/15, 3.24 | 15/15, 2.94 | 15/15, 3.18 |
| payload 200 g | 1/15 | **5/15, 3.71** | 0/15 |
| motor lag x2 | 14/15, 3.52 | **0/15** | 15/15, 3.09 |
| damping 3x | 8/15, 2.97 | 15/15, 2.67 | 15/15, 2.79 |
| kf90 | 10/15, 2.83 | 15/15, 2.65 | 15/15, 3.04 |
| gust | 8/15, 2.91 | 15/15, 2.54 | 15/15, 2.70 |
| sine wind | 7/15, 3.48 | 15/15, 3.21 | 15/15, 2.84 |

- **Longer training texts fixed the late-sentence contact loss:** every case except 200 g and motor lag is now 15/15. This confirms the R18 diagnosis.
- **Best RL accuracy with robustness so far:** nominal 2.53 mm.
- **200 g:** best RL result so far (5/15). The failures are mid-sentence flips (tilt 40 deg) or base drift (255 mm).
- **Motor lag regressed, 14/15 -> 0/15:** the drone flips (tilt 40 deg) mid-sentence, after 4-9 strokes. Not yet diagnosed.
  - Hypothesis: training samples motor lag x1-2 uniformly, and x2 is the edge of that range. Longer episodes give more time for a slow-motor instability to grow.
- **No single policy passes every case yet.** R19 fails 200 g; R20 fails motor lag and is 5/15 at 200 g. R21 (from R19, torque-margin penalty) is still training.

**Videos** (2026-11-17 22:10 EST; HELLO WPI, seed 0, real noise; combined compare.mp4 + ND.mp4, R19.mp4, R20.mp4):
- `Media_results/2026-11-17_2210_Write_HELLO_WPI_payload200g_ND_vs_R19_vs_R20/`:
  - ND completes (2.88 mm, tilt 7.5 deg).
  - R20 completes (3.60 mm).
  - R19 flips at 2.6 s, before writing.
- `Media_results/2026-11-17_2210_Write_HELLO_WPI_motor_slow_ND_vs_R19_vs_R20/`:
  - R19 completes (2.80 mm).
  - R20 flips at stroke 9/11.
  - ND flips at 1.6 s.
- Each video matches its bench row for seed 0.

### R21 final (2026-11-18 12:05 EST; from R19, write15_drjt = up to 5 letters + joint gain 4 + torque-margin penalty; real noise, 15 each) [optimal]

Bench ran on Turing (job 2535274, after training job 2534328). Logs are `Code/results/am3_write/bench/2026-11-17_2230_R21_real*`. Turing's clock wrote 1720 in the names; they were renamed to the submit time, 22:30 EST.

| case | N | ND | R19 | R20 | **R21** |
|---|---|---|---|---|---|
| nominal | 15/15, 2.07 | 15/15, 2.62 | 15/15, 2.70 | 15/15, 2.53 | **15/15, 1.87** |
| payload 100 g | 0/15 | 15/15, 2.65 | 15/15, 3.18 | 15/15, 2.94 | **15/15, 2.32** |
| payload 200 g | 0/15 | **15/15, 3.05** | 0/15 | 5/15, 3.71 | 5/15, 2.75 |
| motor lag x2 | 0/15 | 0/15 | **15/15, 3.09** | 0/15 | 11/15, 2.47 |
| damping 3x | 15/15, 2.31 | 15/15, 2.77 | 15/15, 2.79 | 15/15, 2.67 | **15/15, 1.94** |
| kf90 | 15/15, 2.70 | 15/15, 2.59 | 15/15, 3.04 | 15/15, 2.65 | **15/15, 1.97** |
| gust | 15/15, 2.03 | 15/15, 2.66 | 15/15, 2.70 | 15/15, 2.54 | **15/15, 1.90** |
| sine wind | 15/15, 2.23 | 15/15, 2.70 | 15/15, 2.84 | 15/15, 3.21 | **15/15, 2.64** |

- **First policy under the 2 mm pass line while staying robust:**
  - nominal 1.87, damping 1.94, kf90 1.97, gust 1.90 (max 1.95-2.04).
  - Most accurate controller on 6 of 8 cases, N and ND included.
- **Torque margin and longer texts together:** they cut RMS by about 0.8 mm against R19 on every case. Holding torque margin also keeps the arm steadier while tracing.
- **Still failing:**
  - 200 g: 5/15 (ND 15/15).
  - Motor lag: 11/15 (R19 15/15).
  - Every failure is a flip (tilt 40 deg) or base drift (250 mm) mid-sentence.
- **No single controller passes all 8 cases.** ND fails motor lag, and every RL policy fails part of 200 g.

**Videos** (2026-11-18 11:59 EST; HELLO WPI, seed 0, real noise; compare.mp4 + ND, R19, R20, R21 each), in `Media_results/2026-11-18_1159_Write_HELLO_WPI_{nominal,payload200g,motor_slow}_ND_R19_R20_R21/`:

| case | ND | R19 | R20 | R21 |
|---|---|---|---|---|
| nominal | done, 2.75 | done, 2.36 | done, 2.36 | **done, 1.85** |
| 200 g | **done, 2.88** | flips at 0/11 | done, 3.60 | drifts off at 9/11 |
| motor lag | flips at 0/11 | **done, 2.80** | flips at 9/11 | flips at 9/11 |

- **Common breaking point:** every mid-sentence RL failure in these videos happens at stroke 9 of 11, the final "PI". The base is at its furthest travel there.
- **Next diagnosis target:** that stroke.

**Retrain R22** (Turing job 2536573, bench job 2536574 chained; TAG `2026-11-18_1200_R22_write15drjtw_fromR21_s0`):
- New env `write15_drjtw` = R21's env with a wider randomization range: payload 0-280 g (was 250), motor lag x1-2.5 (was x2).
- Reason: the test cases (200 g, lag x2) sat at or near the edge of the training range. 280 g keeps the static shoulder torque at about 0.9 of the servo limit.
- Warm start from R21, 20M steps.

## R22 final (2026-11-18 16:50 EST)

`2026-11-18_1200_R22_write15drjtw_fromR21_s0`: R21 fine-tuned with wider randomization (`dr_wide`: payload up to 0.28 kg, motor lag up to 2.5×). Turing jobs 2536573 (train, 3 h 22 min) and 2536574 (bench). Logs: `Code/results/am3_write/bench/2026-11-18_1200_R22_*`.

| condition | R21 | R22 |
|---|---|---|
| nominal | 15, 1.87 | 15, 2.47 |
| payload 100 g | 15, 2.32 | 15, 2.77 |
| payload 200 g | 5, 2.75 | **0** (12 failsafe aborts by tilt, base error up to 264 mm) |
| motor lag ×2 | 11, 2.47 | **15, 2.97** |
| damping ×3 | 15, 1.94 | 15, 2.55 |
| thrust −10 % | 15, 1.97 | 15, 2.28 |
| gust | 15, 1.90 | 15, 2.53 |
| sine wind | 15, 2.64 | 15, 2.81 |

- Gain: slow motors back to 15/15 (as R19; ND 0/15, R20 0/15, R21 11/15) while keeping the other 6 conditions at 15/15.
- Cost: tracing RMS up ~0.5 mm everywhere; no condition under the 2 mm bar.
- 200 g regressed: in the video the drone falls at t = 5.3 s, during approach, before writing (R21 failed late, at stroke 9/11). Wider payload range did not teach a torque-feasible posture; consistent with the 242 g static shoulder limit (91 % used at 200 g).
- Conclusion: more randomization alone does not fix 200 g. Next step needs the torque limit addressed directly (posture / feasibility), not wider DR.
- Videos: `Media_results/2026-11-18_1639_Write_HELLO_WPI_{motor_slow,payload200g}_ND_R21_R22/`.

## Smoothness: NMPC vs RL (2026-11-19 09:00 EST) [optimal]

Question (Sahil): the goal was an RL controller as smooth as NMPC, but the bench never measured smoothness. Added `smooth_metrics` to `scripts/am3_write_eval.py` (reported by the bench, averaged over completed episodes). Writing part only (first trace sample to the end). Command measures are total variation per second (commands are piecewise constant, so this does not depend on the control rate: N 50 Hz, RL 100 Hz); state measures use a common 50 Hz grid.

Bench, no mismatch, real sensor noise, 15 episodes each (`bench/2026-11-19_0846_smooth_*`). Lower = smoother.

| | trace RMS mm | rotor cmd change, krad/s per s | joint torque cmd change, N m per s | body angular accel RMS, rad/s² | joint accel RMS, rad/s² | pen velocity error RMS, mm/s |
|---|---|---|---|---|---|---|
| N | 2.07 | 21.1 | 2.76 | 38.0 | 71.8 | 27.4 |
| ND | 2.62 | 27.2 | 3.72 | 54.1 | 95.9 | 34.5 |
| R15 | **1.74** | **3.3** | **0.55** | **5.5** | **9.4** | 21.7 |
| R21 | 1.87 | 3.5 | 0.70 | 6.8 | 12.5 | **19.6** |
| R22 | 2.47 | 3.5 | 0.69 | 7.0 | 12.5 | 20.1 |

(Accuracy columns reproduce the earlier bench runs exactly, so the new metrics did not change the episodes.)

- RL is 6-8x smoother than NMPC in commands and 5-7x in drone/arm accelerations under realistic sensing. ND (observer) is rougher than N.
- Cause, NMPC on HELLO WPI seed 0, median rotor-command jump per 20 ms step: zero sensor noise 1.2 rad/s, old harness noise (0.1 mm) 28 rad/s, realistic noise (1 mm, 0.5 deg) **360 rad/s** (mean command 1030 rad/s). N re-solves from raw noisy measurements with no state filter or input-rate cost strong enough to reject it; motor lag filters most of it, so it still writes. RL was trained with realistic noise and has an observer + 4-step history: 29 rad/s.
- So "NMPC is the smooth reference" held only for near-perfect sensing. With realistic sensors RL already meets and exceeds NMPC smoothness.
- Pen velocity error is large for all (20-35 mm/s on a 50 mm/s pen). Not yet checked whether it is pad vibration or how the reference velocity is defined (min-jerk timing); compare across controllers only.
- Figure: `Media_results/2026-11-19_0850_smoothness_N_vs_R21_signals.png` (rotor 1 command, shoulder torque, roll rate; 3 s while writing).
- Open: give NMPC a state estimator (e.g. EKF/low-pass on the measured state) or stronger input-rate cost, as a new variant (N is frozen), and re-measure.

## NF: NMPC with a state filter (2026-11-19 10:20 EST) [optimal]

Why: N's roughness comes from re-solving on raw noisy measurements (previous section). Fix on the NMPC side, so the RL-vs-NMPC comparison is fair.
What: `ocrl/am3_nmpc_filt.py` (new; N's frozen files unchanged). Fixed-gain predictor-corrector, steady-state Kalman form, N's own predictor as the model:
`x_pred = step(x_hat, u_applied)`, `x_hat = x_pred + alpha (x_meas - x_pred)`. One scalar alpha for all states. Bench name `NF:alpha`.

Tuning on a held-out sentence ("ROBOTS WRITE", seed 7; not in the bench), real noise:

| alpha | completed | trace RMS mm | rotor cmd change krad/s per s | angular accel rad/s² |
|---|---|---|---|---|
| N | yes | 2.09 | 20.9 | 38.0 |
| 1.0 (= N, parity) | yes | 2.07 | 20.9 | 38.2 |
| 0.5 | yes | 1.56 | 8.3 | 17.9 |
| 0.3 | yes | 1.39 | 4.6 | 10.9 |
| 0.15 | yes | 1.97 | 2.1 | 5.7 |
| 0.08 | no (loss of control: estimate lags) | | | |

Full bench, no mismatch, real noise, 15 episodes (`bench/2026-11-19_1010_smooth_NF03`, `_NF015`):

| | trace RMS mean / max mm | coverage | rotor cmd change | joint torque cmd change N m/s | angular accel | joint accel | pen vel err mm/s |
|---|---|---|---|---|---|---|---|
| N | 2.07 / 2.22 | 0.87 | 21.1 | 2.76 | 38.0 | 71.8 | 27.4 |
| **NF 0.3** | **1.43 / 1.74** | **0.94** | 4.6 | 0.67 | 10.8 | 19.5 | 11.9 |
| NF 0.15 | 2.31 / 2.72 | 0.78 | **2.1** | **0.34** | **5.5** | **9.9** | **9.6** |
| R15 | 1.74 / 1.86 | 0.83 | 3.3 | 0.55 | 5.5 | 9.4 | 21.7 |
| R21 | 1.87 / 1.95 | 0.81 | 3.5 | 0.70 | 6.8 | 12.5 | 19.6 |

- NF 0.3 passes the Phase-1 bench (15/15, mean <= 2 mm, no drag) and is the most accurate controller so far (18 % better than R15). It is ~4.5x smoother than N; RL (R15) is still ~1.4x smoother in commands and ~2x in accelerations.
- NF 0.15 is as smooth as R15 but less accurate (2.31 mm): the usual filter trade-off, lag vs noise.
- Consequence for the comparison: most of RL's smoothness advantage over NMPC came from NMPC not filtering its measurements. RL's remaining case rests on the mismatch conditions (next).
