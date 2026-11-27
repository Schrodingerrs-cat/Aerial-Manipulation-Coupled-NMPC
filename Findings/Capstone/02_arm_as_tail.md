# Arm as a "cheetah tail" while carrying a block [capstone]

2026-11-27. The professor suggested using the load like a cheetah's tail. The drone carries a block in the hand, with no swinging payload; the block's mass is known (the capstone's analytical estimate). The question: can moving the arm and block help the drone move between waypoints faster or more smoothly than holding the arm still?

## Setup

- Plant: the frozen aerial manipulator (1.05 kg body + 0.47 kg arm) with a block of 0 / 150 / 250 g at the hand, modelled as a solid sphere.
- Task: a square route with 0.6 m sides. The first move is fore-aft (in the arm's pitch plane), the second sideways. Each move is a minimum-jerk segment taking T seconds, followed by a 1.5 s dwell. Lab sensor noise; 3 seeds per setting.
- Controller: the same coupled NMPC (acados SQP-RTI, 50 Hz) in every mode, with the block in its model. Only the arm terms of the cost change:
  - **rigid**: joints frozen by a heavy joint-rate penalty. The block is dead weight. This is the fair baseline.
  - **hold**: the block is kept still in the world. This turned out to be an anti-tail: the arm rotates against the body's pitch.
  - **tail s**: the arm is free, and a spring of square-root weight s pulls the block back to its home offset (s = 0.05, 0.3, 1.0).
  - **unaware**: rigid, with no block in the model.
- Code:
  - `Code/ocrl/am3_tail.py`: the model with the block and the controller modes;
  - `Code/scripts/am3_tail_run.py`: the study runner;
  - `Code/scripts/am3_tail_video.py`: videos.

  Results are in `Code/results/tail/` and the frozen files are untouched.

## Results

### 1. A moderate tail is best; more freedom is not better

Cells give runs completed, tracking error (mm RMS) and peak tilt (°).

**150 g block:**

| T (s per side) | rigid | tail 0.05 | tail 0.3 | tail 1.0 |
|---|---|---|---|---|
| 1.5 | 3/3, 4.3, 22 | 3/3, 1.5, 15 | 3/3, **1.1, 12.5** | 3/3, 6.6, 25 |
| 1.2 | 3/3, 2.8, 17 | 3/3, 2.4, 18 | 3/3, **2.0, 17** | 3/3, 5.8, 24 |
| 1.0 | 3/3, 7.5, 24 | 2/3, 6.5, 29 | 3/3, **5.4, 23** | 3/3, 9.6, 32 |
| 0.9 | 2/3, 12.5, 32 | 0/3 | **3/3, 7.6, 28** | 1/3, 12.9, 38 |

**250 g block:**

| T (s per side) | rigid | tail 0.05 | tail 0.3 | tail 1.0 |
|---|---|---|---|---|
| 1.5 | 3/3, 3.2, 21 | 3/3, 3.5, 18 | 3/3, **1.1, 13** | 3/3, 4.8, 22 |
| 1.2 | 3/3, 3.1, 18 | 3/3, 6.1, 20 | 3/3, **2.1, 17** | 3/3, 5.4, 23 |
| 1.0 | 3/3, 9.2, 28 | 0/3 | 3/3, **5.8, 26** | 2/3, 8.5, 29 |
| 0.9 | **0/3** | 0/3 | **3/3**, 9.5, 29 | 0/3 |

- **Too loose (0.05):** the arm wanders 200–230° in total joint travel. It costs 7–10 % more rotor energy and fails at speed.
- **Too stiff (1.0):** the arm behaves almost like "hold" and does worse than rigid.
- **Moderate (0.3):** best in every row, and the only mode that completes all 24 runs per mass.

Finer spring sweep at 250 g (`Code/results/tail/2026-11-27_1313_tail_spring_fine_mb0.25.jsonl`): every spring from 0.2 to 0.5 reaches 0.9 s per side (3/3); 0.15 manages 1/3; no spring reaches 0.8 s.
Between 0.3 and 0.5 the error is the same (9.2–9.5 mm at 0.9 s), and 0.5 does it with less joint travel (137° vs 205°) and 1 % less energy. The optimum is broad, so the result does not depend on a finely tuned weight. The tail0.3 rows reproduce the earlier sweep exactly (deterministic per seed).

### 2. The tail extends the speed limit

Fastest move time with 3 of 3 runs completed:

| block | rigid | tail 0.3 |
|---|---|---|
| none | 0.9 s | 0.8 s |
| 150 g | 1.0 s | 0.8 s |
| 250 g | 1.0 s | 0.9 s |

Carrying 250 g with the arm used as a tail, the drone flies as fast as the rigid arm with no block. The arm helps even with no block, because it is 31 % of the vehicle's mass and is already a tail.

### 3. What it costs

- **Rotor energy:** within 1–2 % of rigid at the same speed, so the tail is not paid for in thrust.
- **The servos:** in every run at T ≤ 1.0 s, the tail mode hits the 1 N·m servo torque limit and uses 90–230° of joint travel. The real margin is servo torque. On hardware, the servo's speed and torque limits and its heating decide whether the benefit is available.

### 4. Load awareness matters more than the tail

From v1, 150 g, 1 seed: the rigid arm with no block in its model ("unaware") tracks about 10× worse (13.7 vs 1.2 mm at slow speed) and fails from T = 1.5 s. Knowing the block's mass is the first requirement; the tail is the second.

## Failures and caveats

- **"hold" fails from 2.0 s.** Keeping the block still in the world sounds like stabilisation, but it turns the arm into an anti-tail.
- **Comparing the wrong spring would miss the effect.** The first sweep (v1) compared rigid with the weak 0.05 spring and concluded that the tail does not extend the speed limit. That was true for that spring, but the 0.3 spring changes the answer.
- **Solver stalls.** Eight runs stalled inside acados. All of them were at settings where other seeds lost control; one was reproduced alone, with tilt at 45° and rising before the stall. The runner records a stall as a failure after 300 s.
- **Not shown here:**
  - no wall contact during the moves;
  - no swing;
  - the block is rigidly attached at the hand;
  - routes are flown at constant height;
  - simulation only.

## Videos

- `Media_results/2026-11-27_1227_Tail_square_T0.9_150g_NMPC_rigid_vs_tail0.3/` (seed 0): the rigid arm tips past 30° on the third side; the tail arm swings joint 1 by about 35° and completes.
- `Media_results/2026-11-27_1240_Tail_square_T0.9_250g_NMPC_rigid_vs_tail0.3/` (seed 1): the rigid arm loses control and the tail arm completes. Seed 1 was used because rigid seeds 0 and 2 stall inside acados after losing control.

## Learned tail (RL)

Can a policy learn the tail by itself? Two PPO policies were trained the same way, differing only in what the arm may do:
- **free:** joint targets within ±0.8 rad of home, with a weak home term in the reward.
- **rigid:** no arm action; the joint targets stay at home.

Training setup (both policies):
- Interface: the R9 rung (commanded acceleration, body rates, joint targets) with the inner disturbance observer on.
- Routes: random waypoint routes, 3–5 hops of 0.3–0.8 m, moves of 1–2 s.
- Block: random 0–200 g, with its mass given as an input.
- Training: 10M steps, warm-started from R9.

Code is `Code/ocrl/am3_rl_tail.py` and the runs are `Code/results/am3_rl/2026-11-27_1039_T1_tail_{free,rigid}_fromR9_s5_local`. The policies were evaluated on the same square route, dwell, metrics and lab noise as the NMPC, from rest, 3 seeds: `Code/results/tail/2026-11-27_1311_tail_rl_final_rest.jsonl`.

The table gives the fastest move time with 3 of 3 runs completed, and the tracking error (mm RMS) at T = 1.2 s:

| block | RL rigid | RL free | NMPC tail 0.3 |
|---|---|---|---|
| none | 0.9 s; 14.2 | 0.9 s; 9.8 | 0.8 s; 2.0 |
| 150 g | 1.2 s; 31.2 | **0.9 s; 10.5** | 0.8 s; 2.0 |
| 200 g | 1.2 s; 33.3 | **1.0 s; 11.0** | - |
| 250 g (outside training) | 1.5 s; - | 1.5 s; - | 0.9 s; 2.1 |

- **Yes, the policy learns it.** With a block, the free-arm policy flies 1–2 speed steps faster than the rigid one, with about a third of its error. It swings the arm through 118–129° of joint travel on fast routes. This matches the NMPC result from a completely different method.
- **But the NMPC is 3–10× more accurate and slightly faster.** The RL drone never meets the settle criterion (within 2 cm and 5 cm/s) on routes at or below 1.5 s: it keeps jittering at 3–10 cm/s around the waypoint.
- **With no block, the free policy is worse at speed** (41.9 vs 30.4 mm at 1.0 s; jerk 27.6 vs 10.0). It swings the arm when there is little to gain. The NMPC tail does not have this problem.
- **The RL "rigid" arm is servo-held, not locked.** The joint targets are fixed, but the servos give way under inertial load (50–90° of total joint travel with a block, versus 8–20° for the NMPC's rigid mode). It is a weaker baseline than the NMPC's rigid arm.
- **Evaluation bug found and fixed.** The first evaluation used the training start state (random tilt and rates). Seed 2 began tilted 17° and crashed in every slow case. Starting from rest, as the NMPC does, removed every slow-speed failure.
- **Video:** `Media_results/2026-11-27_1311_Tail_square_T1.0_150g_NMPCtail_vs_RLrigid_vs_RLfree/` (150 g, 1.0 s per side, seed 0). The NMPC tail completes at about 10 mm; RL rigid flips (60° tilt) on the third side; RL free completes with larger excursions.

## Bottom line

Using the arm and block as a tail works in both the model-based and the learned controller. It lets the drone carry 150–250 g as fast as, or faster than, the rigid arm carries nothing, at the same rotor energy, paid for in servo torque. The model-based version, with a moderate spring, is the better one.
