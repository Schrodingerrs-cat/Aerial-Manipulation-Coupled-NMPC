# End-to-end RL control: paper analysis and our design (30 Oct 2026)

A **separate track** from L (the learned NMPC correction). Here the policy *is* the controller, compared head-to-head with N (coupled NMPC) and B (decoupled baseline) on the tasks we already built.

## 1. Reference paper: Singh et al., arXiv 2606.16621 (Jun 2026)

*"Reinforcement Learning with Inner-loop Dynamics Estimator for Aerial Manipulation under Uncertainty"* — IIIT Hyderabad. **Quadrotor + 3-DoF Dynamixel XM430-W210 arm**, 3.0 kg. Our robot is the same class: 3-DoF arm, XM430-W210-class servos, 1.52 kg. Closest published match to our platform.

### Architecture: hierarchical, not end-to-end-to-motors

| layer | rate | what it does |
|---|---|---|
| RL policy | 100 Hz | 6-DOF end-effector goal → **motion references** |
| thrust-vector decomposition | — | desired accel + yaw → desired attitude + collective thrust (Mellinger–Kumar) |
| inner-loop controller | 300 Hz | tracks those references, compensates unknown dynamics → body torques + joint torques |

**The policy never commands motors.** This is the single most important design choice for us: it is what makes the problem learnable at modest sample counts. An end-to-end motor-speed policy has to learn attitude stabilisation from scratch; this one inherits it.

### Observation (33-dim)

`[ (p_g − p_b)/10 , vec(R_B[:,0:2]) , v_b/10 , ω_b , (p_g − p_e)/10 , vec(R_Eᵀ R_G) , θ , θ̇ ]`

Two details worth copying:
* **Rotation matrix columns, not quaternions.** Quaternions have a sign ambiguity (q and −q are the same rotation) which creates a discontinuity the network has to waste capacity on. Our current `am3_rl.build_obs` uses a rotation vector `2·q_xyz·sign(q_w)` — acceptable, but the 6-element form is strictly better behaved.
* **Goal-relative, normalised by 10.** Everything the policy sees is an error, not an absolute position.

### Action (10-dim)

`[ a_b,d (3) , ω_b,d (3) , ψ_d (1) , θ_d (3) ]` — desired base acceleration, body rates, yaw, joint positions.

### The actual contribution: a model-free inner loop (Time-Delay Estimation)

Write the coupled attitude+joint dynamics as `M_η η̈ + H_η = τ_η` with `η = [(φ,ϑ,ψ), θ]`. Replace the true inertia by a **constant diagonal** `M̄_η` and sweep everything else — including all the off-diagonal coupling that is hardest to model — into a lumped unknown:

```
M̄_η η̈ + H̄_η = τ_η ,    H̄_η = (M_η − M̄_η) η̈ + H_η
```

Control law and estimator:

```
τ_η = M̄_η ( η̈_d − K_p e_η − K_d ė_η ) + Ĥ̄_η
Ĥ̄_η(t) = τ_η(t−L) − M̄_η η̈(t−L)            ← one step of memory, no model at all
```

Closed loop: `ë + K_d ė + K_p e = M̄⁻¹(Ĥ̄ − H̄)`, bounded because the estimation error over one sample interval is bounded.

This is classic Time-Delay Control. It is ~20 lines, needs no system model, and **the coupling term we spent the whole project modelling is exactly what it lumps and cancels**. That makes it a genuinely interesting comparison point for our thesis: our NMPC *models* `M_bq`; TDE *estimates its effect* from one step of delay.

### Reward design — three ideas we should take

1. **Two-scale exponential shaping**: `r = r₁e^(−r₂x²) + r₃e^(−r₄x²)`, one wide term (keeps gradients alive when far away) plus one narrow term (gives precision near the goal). Penalties are `p = p₁(e^(−p₂x²) − 1)`, always ≤ 0.
2. **Asymmetric progress reward**: `r_prog = 50·Δe` when approaching, `100·Δe` when receding. Moving away costs twice what approaching pays, which the authors say "strongly discourages limit cycles and oscillatory behaviour".
3. **Multiplicative coupling to prevent reward hacking**: `r_total = r_prog + (r_pos · r_near + r_general)/100`. Secondary rewards (smoothness, low velocity) are *gated* by positional accuracy — the agent cannot farm them without first getting near the goal.

> **This third idea directly fixes the bug I hit today.** Our L reward gave `+1` per step unconditionally, so stalling to the 900-step cap scored ~150 more than finishing the task. I patched it by crediting unused time on completion. Multiplicative gating is the more principled fix: make the survival/smoothness terms *conditional on task progress* so they cannot be collected by doing nothing. I will use gating in the new env rather than the patch.

### Training setup, and the gap we cannot close

PPO, MLP [512, 256, 128] ELU, **8192 parallel environments** on an RTX 4090 (Isaac Gym / Aerial Gym), horizon 32, γ 0.99, lr 1e-4 adaptive, KL 0.008.

We have **MuJoCo on CPU, ~8 parallel processes**. That is a throughput gap of roughly **three orders of magnitude**. We cannot replicate their scale, so we must not promise their breadth of task. This is the main constraint on what we attempt.

Domain randomisation at every reset: payload 0.1–0.5 kg, link lengths ×0.8–1.2, motor time constants ×0.95–1.05, joint stiffness ×0.5–1.5, joint damping ×0.9–1.1. We already have an equivalent in `am3_mismatch.py`.

### Results, and an honest calibration

Figure-eight trajectory, payloads 200/400 g, speeds 0.5/1.0 m/s. End-effector RMSE / success rate:

| condition | RL+PID | RL+INDI+PID | theirs (RL+TDE) |
|---|---|---|---|
| 200 g, 0.5 m/s | 0.242 / 85 % | 0.195 / 95 % | **0.180 / 95 %** |
| 200 g, 1.0 m/s | 0.315 / 70 % | 0.220 / 90 % | **0.151 / 95 %** |
| 400 g, 1.0 m/s | 0.392 / 60 % | 0.285 / 85 % | **0.205 / 90 %** |

40.9 % mean RMSE reduction vs RL+PID, 26.4 % vs RL+INDI+PID. The authors state plainly that there is no significance testing and it is one hardware study.

**Calibration we must keep in front of us: their end-effector RMSE is 150–205 mm. Our NMPC tracks the pad to 0.5 mm RMSE on the wall task and 0.11 mm in free flight.** Different tasks and theirs is real hardware with real sensing, so this is not a fair head-to-head — but it sets the expectation. **End-to-end RL is very unlikely to beat our NMPC on precision.** If we want a defensible RL result, the claim should be about robustness, model-freedom, or behaviour under mismatch — not accuracy.

## 2. Second paper: Dimmig & Kobilarov, arXiv 2407.00889 (Jul 2024)

DreamerV3 (model-based RL) for **non-prehensile pushing** with a fixed gripper, Isaac/Aerial Gym, depth images, 32 envs with randomised friction. Less directly applicable — different task, vision-based, velocity-command action space.

One finding is valuable as a warning: on the repetitive alternating-goal task the agent **found a shortcut** — "the vehicle learns to fly quickly around the table continually pushing to the right (regardless of if the object is being moved)". It scored well without doing the task. On randomised goals, performance dropped by a factor of ten.

Together with our stalling bug, that is two independent instances of the same failure: **a reward that can be satisfied without performing the task.** Any reward we write gets checked against this before training.

## 3. Our design

### What we reuse (this is the efficiency win)

Our `Episode` harness takes any object with `compute(t, xm, Fm, task) → (u, info)`. So an RL controller is a **drop-in**, and the moment it exists it works with `am3_compare.py` (matched video), `am3_live.py` (interactive viewer), `am3_traj_run.py` (trajectory demos) and `am3_eval.py` (the same frozen metrics N and B are scored on). Nothing needs rebuilding, and the comparison is apples-to-apples by construction.

### Architecture (following the paper)

```
policy:  obs → [ a_b,d (3), ω_b,d (3), ψ_d (1), θ_d (3) ]       50 Hz
inner:   geometric thrust/attitude (reuse am3_ctrl_b's base) + joint PD → [Ω_cmd (4), τ_joint (3)]
later:   swap the inner loop for TDE to replicate the paper's ablation
```

The inner loop already exists inside `am3_ctrl_b.py` — its geometric controller converts a desired acceleration into thrust, attitude and rotor speeds. So our first RL controller is literally the paper's **RL+PID baseline**, and adding TDE reproduces their contribution on our platform.

### The goal ladder (what the user asked for)

Each rung reuses a reference generator we already have, and is scored with the metrics N and B were scored on.

| rung | task | existing infrastructure | success = |
|---|---|---|---|
| **R1** | hold hover | `Task` hover phase | base error ≤ 20 mm, no chatter, 30 s |
| **R2** | follow a base trajectory | `LoopTask` (figure-eight, Lissajous) | compare directly against N's 0.128 mm |
| **R3** | move the arm while hovering | `free`/`free_big` waypoints | B diverges here; does RL? |
| **R4** | base trajectory **and** arm motion together | `LoopTask` with `arm_laps > 0` | the coupling test |
| **R5** | draw a line on the wall in contact | `line` path, full task machine | the actual project task |

R1–R4 need no contact, so the plant can run at a coarser timestep and the env is ~5× faster than the correction env.

### Reward (gated, per the paper)

```
r_pos   = 10·exp(−10·e²) + 2·exp(−2·e²)            two-scale, e = base position error
r_near  = smoothness + low-velocity + upright terms        all ≤ 0 or small
r_prog  = 50·Δe if approaching else 100·Δe                 asymmetric
r_total = r_prog + (r_pos · r_near + r_general)/100        multiplicative gate
```
No unconditional per-step survival bonus. Checked against the question: *can this be maximised without doing the task?*

### What we will not claim

* We cannot match 8192-environment training. Results will be from ~10⁶ steps, not ~10⁸.
* Precision competitive with the NMPC is not expected, and a loss there is not a failure of the method — it is the expected outcome, and the paper's own numbers predict it.
* Any RL-vs-NMPC claim is scored on the **same frozen criteria** as N and B, with paired statistics, because we already found today that a 2-scenario difference on 20 samples does not replicate.

---

# R1 log (30 Oct 2026): first attempt failed, why, and the fixes

## Attempt 1 (invalid): policy learned to crash

900k steps, 45,485 episodes. **No episode reached the 1000-step cap.** Mean episode length *fell* as training progressed: 42 → 23 → 18 → 17 steps. Mean return -62 → -29.

Cause: a reward that is negative per step plus a crash penalty (-20) smaller than the accumulated negative return of surviving. Dying early was the optimum. This is the mirror image of the stalling defect found in the L reward the same day (there surviving paid too much; here it paid too little). The two failures are related: in both, the reward could be improved by doing something other than the task.

Two concrete defects:
1. **Position reward on the wrong scale.** I normalised by 5 cm, so the "wide" Gaussian was 0 beyond ~15 cm and the agent saw no gradient from where it starts. Fixed: `10·exp(-(e/0.5)²) + 10·exp(-(e/0.05)²)` with e in metres.
2. **Exploration far too violent.** Uniform-ish noise on ±6 m/s², ±4 rad/s, ±0.5 rad yaw and the full joint range killed the vehicle in ~48 steps before anything could be learned. Fixed: body-rate range ±1.5 rad/s, yaw ±0.3, accel ±5, joint authority 5 % of range (`arm_frac`), and `log_std_init = -1.5`.

## The inner loop itself was marginal

The scripted PD expert, through the same action interface, **failed 7 of 30 episodes** even from mild starts (61 mm offset, 8° tilt). With the attitude bandwidth borrowed from baseline B (11.6 rad/s) and no motor-lag inversion at 100 Hz, the loop loses ~33° of phase to the 40-60 ms rotor lag. Gain sweep over 20 seeds:

| wn [rad/s] | zeta | survived | final error |
|---|---|---|---|
| 5.0 | 1.0 | 20/20 | 20.2 mm |
| 7.0 | 1.0 | 20/20 | 2.9 mm |
| 9.0 | 1.0 | 20/20 | 2.0 mm |
| 11.6 | 1.0 | **16/20** | 1.4 mm |
| 11.6 | 1.5 | 18/20 | 1.8 mm |

Chose **wn = 8.0, zeta = 1.0** for margin below the cliff. Verified on 40 fresh seeds: 40/40, 2.4 mm.

## What the reward now ranks (10 episodes each, reward/step)

| policy | return | reward/step |
|---|---|---|
| scripted PD | 8905 | 10.8 |
| zero action | 415 | 1.1 |
| noise, std 0.22 | 72 | 0.5 |
| noise, std 1.0 | 11 | 0.2 |

Every policy earns positive reward per step, so terminating is never attractive, and better behaviour ranks strictly higher.

## Guards added so this cannot fail silently overnight

* **Preflight** now fails if a competent controller earns ≤ 0 reward/step ("surviving must pay").
* **Collapse detector**: after 400k steps, if the mean length of the last 500 episodes is below 60, the run saves `model_collapsed`, logs `COLLAPSE`, and exits 2 instead of burning the allocation.
* Progress lines now print mean episode length, which is the number that exposed this.

## Honest status

Attempt 2 is training. It has not yet shown that the policy learns; that is the next check, against the scripted-PD return of ~8900 and the zero-action return of ~415. The PD expert reaches 2.4 mm, so beating it is a real bar, not a formality.

---

# R1 attempt 2: result, and the decision taken (30 Oct 2026, ~19:40; made autonomously per the user's instruction)

## Result (3M steps, 40 held-out starts, seeds 100-139, identical episodes for every row)

| policy | survived | position error, last 200 steps | roughness |
|---|---|---|---|
| zero action | 2/40 | 1048 mm | 0.000 |
| scripted PD expert | 40/40 | 2.4 mm | 0.001 |
| RL, 1.0M steps | 40/40 | 97.3 mm | 0.003 |
| RL, 1.6M steps | 40/40 | 16.4 mm | 0.004 |
| RL, 2.4M steps | **38/40** | 12.5 mm | 0.012 |
| **RL, final (3M)** | 40/40 | **10.2 mm** | 0.008 |

The policy learned to hover robustly on unseen starts, smoothly, and well inside the 20 mm base-error criterion used for the NMPC gate. It is **not** at expert accuracy: 10.2 mm against 2.4 mm. The 2.4M checkpoint lost two episodes, so training has mild late-stage instability; a "latest checkpoint" is not automatically the best one.

## Decision rule, written before the numbers came in

| observation | action |
|---|---|
| error still falling across checkpoints | continue training |
| error plateaued, survival 40/40 | the reward is the limit: sharpen it |
| survival dropped at later checkpoints | lower the learning rate |
| error <= ~5 mm, survival 40/40 | R1 done, move to R2 |

Observed: error falling (16.4 -> 12.5 -> 10.2 mm, roughly -20 % per 0.7M steps) *and* a survival dip at 2.4M. The first and third rows both apply, and the exit bar (5 mm) is not met.

## Why two continuations instead of one

Falling error says "train longer", but the narrow reward Gaussian (5 cm scale) is almost flat at 10 mm, so the slow decay could equally be the reward limiting progress. Those two explanations predict different things, so I test both with a controlled comparison: **same starting checkpoint, same steps, same learning rate (1e-4, lowered for the instability), only the reward differs.**

| run | reward | tag |
|---|---|---|
| A | unchanged | `R1_cont_A` |
| B | unchanged plus a third term `10·exp(-(e/0.01)²)` (1 cm scale) | `R1_cont_B_fine` |

Evaluation is in millimetres on the same 40 starts, which does not depend on the reward, so A and B are directly comparable even though their rewards differ.

Reading the outcome: if B beats A clearly, the reward was the limit. If A and B are similar, more training was the lever and the fine term is unnecessary. If neither improves, the limit is elsewhere (action authority, the inner loop's own steady-state error, or observation noise) and I look there next.

## A mistake of mine, recorded

I launched both continuations with `--collapse-after 10000000`, which **disables** the collapse detector I had just added. There was no good reason. Warm-started runs are the ones most likely to degrade, and the detector exists for exactly that. I am monitoring episode length by hand instead, and will keep the detector on for later runs.

---

# R1: A/B result (30 Oct 2026, ~20:25)

Same starting checkpoint (3M steps, 10.2 mm), 4M more steps each, lr 1e-4. 40 held-out starts.

| run | reward | survived | error | roughness |
|---|---|---|---|---|
| start | - | 40/40 | 10.2 mm | 0.008 |
| A (seed 1) | unchanged | 40/40 | **20.8 mm** | 0.010 |
| **B (seed 2)** | + `10·exp(-(e/1 cm)²)` | 40/40 | **6.0 mm** | 0.011 |
| PD expert | - | 40/40 | 2.4 mm | 0.001 |

Episode length stayed ~1000 throughout in both (no collapse; the detector was off, checked by hand).

**Reading it.** More training on the flat reward made the policy *worse* (10.2 -> 20.8 mm): with no gradient inside ~1 cm the error random-walks. The 1 cm term fixed that (6.0 mm). Consistent with "the reward was the limit".

**Confound, stated plainly:** A and B also used different seeds (1 vs 2), one run each. A 3.5x gap is large but n=1 per arm cannot exclude seed luck. Replicates with the seeds swapped (`R1_rep_A_s2`: no fine term; `R1_rep_B_s1`: fine term) are running; the claim stands only if the ordering survives the swap.

**Cost of the fine term:** roughness is 0.011 against the expert's 0.001, i.e. the policy still dithers its actions ~10x more than a PD controller. Harmless in simulation, relevant for any move to hardware.

---

# R2 setup: base trajectory tracking (30 Oct 2026, ~20:40)

## Environment change, and the check that it did not disturb R1

`HoverEnv` now tracks `self.pref / self.vref / self.aref` instead of the constant `BASE`; the defaults reproduce R1 exactly, and `TrackEnv` (new, `ocrl/am3_rl_direct.py`) overrides them with a moving reference. Verified after the refactor on 20 held-out starts: scripted PD **2.4 mm** (before: 2.4), R1 policy B **6.0 mm** (before: 6.0). Unchanged.

## TrackEnv

* Reference: a Lissajous figure in the y-z plane, the same maths as `LoopTask` in `ocrl/am3_traj.py` (smooth-rate ramp, exact lap count, starts and ends at rest at the hover point). Arm held still.
* Re-drawn every episode: amplitude y in [3, 20] cm, z in [1, 8] cm, duration 12-30 s, 1-3 laps, shape in {lemniscate, lissajous 2:3, sine 1:3}. Figures needing more than 2 m/s^2 are rejected (the NMPC demos peak at ~0.1 m/s^2; 2 m/s^2 keeps training inside the tilt limit).
* Observation adds the reference velocity and two look-ahead points (+0.25 s, +0.5 s): that is how the policy can learn acceleration feedforward without being given the acceleration. 40 dims in total.
* Reward: same as R1 with the **fine 1 cm term on by default** (the lesson from R1 A/B), and position/velocity measured against the moving reference.
* Training: PPO, 8 envs, 6M steps, lr 3e-4, collapse detector on.

## Is the task solvable? Scripted PD **with reference-acceleration feedforward**, same action interface, 6 starts each

| trajectory | peak ref accel | PD+ff survived | PD+ff RMS error | zero action |
|---|---|---|---|---|
| `fig8_slow` (the NMPC video figure-eight: 3 laps / 30 s, +-15 x +-5 cm) | 0.11 m/s^2 | 6/6 | **6.2 mm** | 0/6 |
| `liss_mid` (lissajous 2:3, 2 laps / 20 s) | 0.30 | 6/6 | **16.3 mm** | 0/6 |
| `fig8_fast` (3 laps / 15 s) | 0.52 | 6/6 | **25.7 mm** | 0/6 |

Solvable, and error grows with speed as expected for a simple PD. There is headroom for a policy that uses the look-ahead points to anticipate.

**Calibration:** the NMPC holds ~0.13 mm on a comparable figure-eight (with the arm also moving). This expert is ~50x worse at the same task. A result near the expert would be a good RL outcome; matching the NMPC's precision is not expected and not the claim.

## Notes

* The training-log reference was misleading in attempt 2 (`ref: PD 1319` came from 200-step preflight episodes, printed next to 1000-step training returns). It now prints **reward per step**: preflight shows zero 1.25, PD 15.8 for R2.
* The `R1_rep_*` runs were launched before this script change and still print the old format.

## Replicate check (2026-10-30 21:25) - the "replicates" were duplicates (bug, fixed)
`r1_replicates.json` gave results identical to the originals (A 20.8 mm both, B 6.0 mm both) and the weights compared equal (max abs diff 0.0) between R1_cont_A (seed 1) and R1_rep_A_s2 (seed 2), likewise B.
Cause: `--init-from` uses `PPO.load`, which restores the saved `seed=0`; `_setup_model` then re-seeds torch and calls `env.seed(0)`, so every warm start used identical torch and env streams regardless of `--seed`.
Fix: `model.set_random_seed(a.seed)` after the warm-start load (scripts/am3_train_direct.py). Verified: first-episode returns now differ between seeds.
Consequence: A (flat, 20.8 mm) vs B (fine term, 6.0 mm) is a paired comparison at ONE shared seed, n=1 per condition. Not yet evidence that the fine term generalises.
Relaunched genuine replicates: R1_rep2_A_s3 (flat, seed 3) and R1_rep2_B_s4 (fine, seed 4), 4M steps, collapse detector on. Decision rule unchanged: the claim holds only if B beats A by a clear margin in both pairs (seed 0 pair, seed 3/4 pair).

## R2 intermediate eval, model_5500k (n=4 starts, seeds 100-103, deterministic policy; not yet n=8)
| trajectory | zero | PD+ff expert RMS mm | R2 policy RMS mm | survived |
|---|---|---|---|---|
| fig8_slow (0.11 m/s^2) | 0/4 | 6.2 | 9.4 | 4/4 |
| liss_mid (0.30) | 0/4 | 16.3 | 10.2 | 4/4 |
| fig8_fast (0.52) | 0/4 | 25.6 | 11.7 | 4/4 |
Reading: the learned policy tracks all three named trajectories, beats the scripted PD+feedforward on the two faster ones, loses on the slowest (9.4 vs 6.2). Caveats: n=4, one training seed, tilt max 17.8 deg vs 7.6 deg for PD (more aggressive), training reward/step still noisy (6-11 vs PD 15.8). The NMPC reference for these trajectories is not in this comparison. Next: n=8 on model_final, then R3 (arm motion at hover).

## Results 2026-10-31 (genuine replicates, R2 final; seeds 100+, deterministic policy)
R1 hover, 40 starts, error over last 200 steps: PD 2.4 mm. Flat reward: 20.8 (seed 0 pair) and 10.4 (rep2 seed 3). Fine 1 cm term: 6.0 (seed 0 pair) and 7.2 (rep2 seed 4). All 40/40 survive. Fine term wins both pairs (by 14.8 and 3.2 mm), but the second margin is small, and no RL policy reaches PD (2.4 mm). Claim stands weakly: fine term helps, magnitude seed-dependent, n=2 per condition.
R2 final (6M steps), 8 starts: survived 7/8 on every trajectory; RMS mm (PD+ff / R2): fig8_slow 6.1 / 19.2, liss_mid 16.4 / 24.6, fig8_fast 25.7 / 33.3; tilt max ~22 deg vs 8.6 deg. The final model is WORSE than PD+ff everywhere and loses a start. The earlier 5.5M checkpoint (n=4) looked better than PD on two trajectories; with n=4 and one seed that was not reliable. Checkpoints at 4.5M-5.5M had noisy training reward (6-11 per step), i.e. training had not converged and checkpoints differ a lot.
Videos: Media_results/RL_R1_hover_2026-10-31, RL_R2_fig8_fast_2026-10-31. Live viewer: scripts/am3_rl_live.py.
Next (decision): R2 is not good enough to build on. Investigate before more steps: per-checkpoint eval on separate validation seeds, lower final-phase learning rate / larger batch for a smoother end of training, and check why tilt is 2.5x PD (action-rate penalty weight in TrackEnv).

## R3 - bigger 3-D trajectories, finer reward and observation (2026-10-31, decided autonomously after the user's feedback)
User feedback after running R1/R2 final in the live viewer: R1 fine; R2 fairly good but the base stays slightly below the green reference and does not reach it; wants bigger trajectories in all directions, stronger reward/penalty, longer training, keep improving, keep logging.
Diagnosis (signed error, held-out start 100, steady part after 2 s; mm, x/y/z):
- fig8_slow: PD mean (2.2, 0.2, 0.4) std (1.5, 4.5, 0.8); R2 policy mean (-4.0, -1.8, **-10.2**) std (13.2, 4.1, 7.6); along-track lag 0.8 mm.
- fig8_fast: PD mean (1.7, 1.0, 2.2); R2 mean (-4.9, -1.7, **-10.0**) std (25.8, 7.7, 15.9); lag -0.5 mm.
=> the user's observation is a constant z bias of about 10 mm (not lag), plus an x wobble of 13-26 mm in the axis the reference never moves in.
Causes found in the code: (1) position error enters the network as error/0.5 m, so a 10 mm offset is a 0.02 input; (2) the 1 cm reward term exp(-(e/0.01)^2) has almost no gradient at 10 mm; (3) no persistent-offset signal (no integral).
Changes (new class Track3Env in ocrl/am3_rl_direct.py; TrackEnv and R1/R2 untouched; HoverEnv gets lin_w, default 0 = unchanged):
- observation: + position error / 2 cm (clipped +-5) and a leaky error integral (tau 3 s) / 2 cm; reference velocity /0.8, look-ahead /0.5 (larger motions).
- reward: fine term weight 10 -> 20, plus linear near-target penalty lin_w * min(e, 3 cm) / 1 cm per step (lin_w 0.4; max 1.2 per step vs ~19 max reward, capped so far-away states are not made worse than terminating).
- trajectories: 3-D Lissajous in x, y, z (forward/back, left/right, up/down), amplitudes Ax 0 or 0.1-0.4, Ay 0.1-0.5, Az 0.03-0.25 m, harmonics 1-3 per axis, random phases, T 15-35 s, 1-3 laps, peak accel <= 2 m/s^2, start/end at rest at hover. Old R2 was y-z only, |Ay| <= 0.2, |Az| <= 0.08.
- trainer: --env track3, --lin-w, --lr-decay (lr * max(0.1, remaining progress)).
Named held-out evaluation set (scripts/am3_eval_track.py --env track3): big_fig8 (+-0.5 x +-0.2 m), circle_xy (+-0.3 x +-0.4 m horizontal circle), fwd_up (+-0.35 x +-0.25 m), lissa3d (3-D Lissajous). PD+ff feasibility (3 starts): 6.3-8.2 mm RMS on the first three, 20.1 mm on lissa3d; all survive. This is the bar.
Runs launched 08:46 (all 15M steps, 8 envs, collapse detector on, save every 1M): R3_big_s0 (lin_w 0.4, fine 20, seed 0), R3_big_s1 (same, seed 1), R3_big_strong_s2 (lin_w 0.8, fine 30, seed 2).
Logged reference reward/step: zero -0.18, PD 8.94 (not comparable with R2's 15.8 because trajectories are bigger and the penalty is new).
Decision rule: at intermediate checkpoints compare on the four named trajectories against PD+ff using fresh start seeds; judge by RMS, mean signed error per axis (bias) and survival. Success = survival 100% and RMS within 2x of PD+ff with no z bias > 3 mm.

### R3 progress log
- 09:43 4M steps: R3_big_s0 / s1 complete full-length episodes (mean length 2057 / 1765 steps), reward/step 3.30 / 2.66; strong variant (lin_w 0.8, fine 30) mean length 271.
- 10:01 5M steps: s0 5.34, s1 5.12 reward/step (PD 8.94); strong variant 0.27, mean length 592. Stopped R3_big_strong_s2 at 10:05 (SIGTERM, interrupt checkpoint saved): too-strong penalty makes surviving less valuable than precision (consistent with arXiv 2311.06144, which reports that overly large penalties prioritise stabilisation over tracking). Keep lin_w 0.4 / fine 20.
- Literature check (web): integral term against steady-state offset (arXiv 2311.06144) - supports the leaky-integral observation; PPO with PID expert knowledge for convergence (J. System Simulation 2025, PPO-SAG) - motivates a BC warm start. No paper found combining BC + PPO + integral observation for this task; treated as our own design to be tested.
- BC warm start (scripts/am3_bc_init.py): 80 episodes of the PD+ff expert on random Track3Env references, expert executed with action noise sigma 0.2 on the acceleration channels, clean expert actions as labels (DAgger-lite coverage), critic regressed on discounted returns; 40 epochs; actor MSE 1e-5, critic MSE 0.37 (return-normalised). Saved results/am3_rl/R3_bc_s0/model_bc.zip.
- 10:06 launched R3_bcppo_s0: PPO fine-tune from the BC model (seed 5, lr 1e-4 constant, 10M steps, same reward/obs as R3_big). Question it answers: does starting at expert level and improving beat PPO from scratch (R3_big_s0/s1)?

### R3 evaluation at 4M steps (n=4 starts, seeds 100-103; RMS mm after 2 s; bias = mean signed error x,y,z mm) - results/am3_rl/eval/r3_eval_4M.json
| trajectory | PD+ff | R3_big_s0 | R3_big_s1 | strong_s2 |
|---|---|---|---|---|
| big_fig8 | 8.0 | 47.3 [-14.8 18.3 -3.9] | 99.9 [24.4 40.8 -3.0] | 0/4 survive |
| circle_xy | 6.9 | 47.2 | 60.3 | 0/4 |
| fwd_up | 6.3 | 48.4 | 104.9 | 0/4 |
| lissa3d | 20.0 | 55.9 | 134.3 | 0/4 |
Reading: far from the bar at 4M (reward/step 3.3 vs PD 8.9). Bias is the same on every trajectory (s0 about (-14, +18, -4) mm), so it is a drift/offset of the policy, not a trajectory-following failure; the integral observation has not yet removed it. Training is not converged (reward/step still rising at 5M). The strong variant never learned to fly. Next: re-evaluate at later checkpoints; BC+PPO run (R3_bcppo_s0) evaluated at its 1M checkpoint.

### R3 BC warm start result (10:40; n=4 starts, seeds 100-103; results/am3_rl/eval/r3_eval_bc.json)
RMS mm / survived / bias z mm. PD+ff: big_fig8 8.0, circle_xy 6.9, fwd_up 6.3, lissa3d 20.0 (all 4/4, bias z about 0.5).
- **R3_bc_s0 (pure behaviour cloning, no RL): 10.6 / 8.6 / 9.4 / 22.1, 4/4 on all, tilt max 7.6 deg, z bias +3.2..3.8 mm.** A neural policy flying the big 3-D trajectories at roughly PD level (within about 1.1-1.5x), using only observations. The from-scratch PPO runs were at 47-134 mm at 4M steps, so BC is a far better starting point.
- R3_bcppo_s0 at 1M steps of PPO fine-tuning (lr 1e-4, log_std -2, no KL cap): RMS 7.0-7.3 mm where it survives (better than PD on big_fig8 and circle_xy, z bias -6.4) BUT 2/4 starts lost on every trajectory, tilt max 44 deg. Training reward/step 9.43 (> PD 8.94) with mean episode length 458: precision improved while stability was lost. The update is too aggressive for a warm-started policy (critic only roughly fitted, exploration std 0.135).
Decision: run R3_bcppo2_s6 with a conservative fine-tune: log_std -3 (std 0.05), lr 3e-5, target_kl 0.01 (the warm-start path ignored --target-kl before; fixed: custom_objects now carries it). Stopped R3_big_s1 (from scratch, 2.7 reward/step at 4M, 100 mm RMS) to free CPU; R3_big_s0 and R3_bcppo_s0 keep running for comparison.
Selection rule going forward: a checkpoint counts only if it survives 100% of starts on all four trajectories; among those, lowest mean RMS.

### R3 mid-training evaluation (11:30; n=4 starts; results/am3_rl/eval/r3_eval_mid2.json) and decisions
RMS mm / survived (PD+ff: big_fig8 8.0, circle_xy 6.9, fwd_up 6.3, lissa3d 20.0, all 4/4):
- **R3_big_s0 @9M (from scratch, PPO): 3.9 (2/4), 4.7 (2/4), 11.6 (3/4), 3.6 (2/4)**; bias z -1.8 mm, x,y ~0; tilt max 36-43 deg. Where it survives it is MORE precise than PD+ff and the z bias is gone (the user's "stays lower" complaint addressed by the fine-scale error input / integral / linear penalty), but it loses 1-2 of 4 starts.
  Failure analysis (big_fig8, seeds 100-105): every loss happens in the first 1-2 s (steps 103 and 220), starts 29-53 cm from the reference moving 0.4 m/s; the policy tilts hard to catch up and crosses the 60 deg limit (tilt>30 deg at steps 62 and 93). Starts that survive track for the full 31 s with max tilt 16-30 deg. So: precise tracking, over-aggressive transient recovery.
- **R3_bcppo2_s6 (BC init + conservative PPO, log_std -3, lr 3e-5, target_kl 0.01): BROKEN.** RMS 88-108 mm at 500k steps and 302-319 mm at 1M with a growing offset (bias (+27,+33,-71) then (+123,+135,-229) mm) although tilt stays 7.6 deg and training reward/step fell 5.8 -> 3.3. The BC critic (return-normalised MSE 0.37) gives poor advantages and even small PPO updates push the actor consistently off. Stopped. R3_bcppo_s0 (lr 1e-4) also stopped: collapsed to mean episode length 117. Conclusion: PPO fine-tuning from a BC actor with an only-roughly-fitted critic is unstable here; would need critic warm-up with the actor frozen (not tried yet). The pure BC policy remains a valid 4/4-survival fallback (8.6-22 mm).
Decision: fix the transient directly in the reward. New `tilt_w` option (penalty per step of tilt_w * max(0, tilt - 20 deg) / 10 deg; default 0 so older runs are unchanged). Fine-tune the precise 9M policy with it: **R3b_tilt_s7**: --init-from R3_big_s0/model_9000k.zip, tilt_w 1.0, lr 5e-5, target_kl 0.02, 8M steps (11:42). The critic comes from the same reward family, avoiding the BC critic problem. R3_big_s0 keeps running to 15M with lr decay as the untouched control.
Next evaluation: both on all four trajectories plus the same starts, with the selection rule (100% survival, then lowest RMS).

### R3 evaluation 12:10 (n=6 starts, seeds 100-105; results/am3_rl/eval/r3_eval_mid3.json)
| trajectory | PD+ff RMS | R3b_tilt_s7 @1M (tilt_w 1) | R3_big_s0 @12M (control) |
|---|---|---|---|
| big_fig8 | 8.0 (6/6) | 3.3 (3/6) | 3.3 (3/6) |
| circle_xy | 6.9 (6/6) | 3.3 (3/6) | 3.3 (3/6) |
| fwd_up | 6.3 (6/6) | 72.6 (4/6; one start ran away, max err 314 mm) | 3.4 (3/6) |
| lissa3d | 20.0 (6/6) | 3.3 (3/6) | 3.4 (3/6) |
Bias z -2.3 to -2.8 mm, x,y within 1.4 mm. tilt max 42-44 deg for both.
Reading: both learned very precise tracking (3.3 mm RMS where they survive, about 2-6x better than PD+ff, and the same 3.3 mm on all four trajectories, which says error is dominated by the noise floor / controller floor not by trajectory difficulty) but both lose exactly half the starts. The tilt penalty of weight 1 (cost 2 per step at 40 deg vs ~10 reward per step for catching up, and a flat death cost of 20) did not change survival: it is too weak a signal against the incentive to rush back to the reference (asymmetric progress reward 50-100 per metre). Stopped R3b_tilt_s7.
Decision: dose-response on the survival incentive, both fine-tuned from R3_big_s0 @12M (lr 5e-5, target_kl 0.02, 6M steps, 12:15): **R3c_tilt6_s8** (tilt_w 6, termination penalty 100) and **R3d_tilt3_s9** (tilt_w 3, termination penalty 60). New option `bad_pen` (default 20 = unchanged). Question: does an unambiguous cost on high-tilt recovery fix the start transient while keeping the 3.3 mm precision? If neither works, next candidate is a start-capture curriculum (smaller start offsets early, widening) or a reference ramp from the start state; both would be explicit task changes and are documented as such.

### R3 evaluation 13:00 (n=6 starts, seeds 100-105; results/am3_rl/eval/r3_eval_mid4.json)
RMS mm (survived) per trajectory big_fig8 / circle_xy / fwd_up / lissa3d. PD+ff: 8.0 / 6.9 / 6.3 / 20.0, all 6/6.
- **R3_big_s0 final (15M, PPO from scratch): 2.4 / 2.4 / 2.5 / 2.6, each 4/6.** 3-8x more precise than PD+ff (lissa3d: 2.6 vs 20.0), bias (0.9, 0.1, -1.8) mm, tilt max 34 deg. Improved from 3/6 at 12M to 4/6 at 15M (lr decayed to 10%).
- R3d_tilt3_s9 @1M (tilt_w 3, bad_pen 60, from 12M): 2.6 / 2.6 / 2.8 / 3.0, each 4/6, tilt max 32 deg.
- R3c_tilt6_s8 @1M (tilt_w 6, bad_pen 100): 3.3 / 8.9 / 4.1 / 2.4, survived 4/6, 4/6, 4/6, 3/6, tilt max 41-44 deg: the heavier penalty is not better (stopped 13:05).
Reading: precision is no longer the problem; survival of 2 of 6 starts is. Same two failure starts as before (far, fast starts: 29-53 cm offset, 0.4 m/s). Hypothesis: with start offset sigma 0.15 m per axis those starts are the rare tail of the training distribution and are barely practised. The tilt/termination penalties are not the lever (3 and 6 give the same survival as 1 and 0).
Decision: widen the TRAINING start distribution only (evaluation starts unchanged, so the test gets no easier): **R3e_wide_s10**, init from R3_big_s0 final, p0_sigma 0.30 m (was 0.15), v0_sigma 0.40 m/s (was 0.20), tilt_w 3, bad_pen 60, lr 5e-5, target_kl 0.02, 8M steps (13:08). Comparison set: R3_big_s0 final (nothing), R3d (penalties only, running), R3e (penalties + wide starts). If survival goes to 6/6 with RMS <= 5 mm: success by the stated rule; then render the comparison video, run n=20 evaluation, and push-recovery tests.

### R3 evaluation 13:30 and the cause of the lost starts (results/am3_rl/eval/r3_eval_mid5.json)
- R3e_wide_s10 @1M (wide training starts + tilt_w 3 + bad_pen 60): survived 3/6, 4/6, 3/6, 3/6; RMS 2.0-2.2 mm (one 17 mm average from a late failure on circle_xy). R3d_tilt3_s9 @4.5M: 4/6 on all, RMS 2-9 mm average. **Neither improved survival.** The hypothesis that the far-start tail was under-sampled is NOT supported: widening the training starts made no difference (and made training episodes shorter, mean length 1340). Penalties (tilt_w 1/3/6, bad_pen 20/60/100) all gave 3/6-4/6.
- Direct inspection of the first 150 steps of big_fig8 for seeds 100-107 with R3_big_s0 final: both lost starts (seed 101 lost at step 102, seed 103 at step 106) have the body-rate command saturated (|a_w| = 1.0, i.e. 1.5 rad/s) and large yaw command (0.75-0.78), and tilt rises to 60 deg within about 1 s. All surviving starts keep |a_w| <= 0.84 and tilt <= 27 deg. Commanded acceleration is saturated (1.0 = 5 m/s^2) in every start. Start offset / velocity / initial tilt do not separate lost from surviving starts (lost: 362 mm, 0.43 m/s, 4.6 deg and 289 mm, 0.46 m/s, 9.2 deg; survived: up to 367 mm, 0.46 m/s, 13 deg). The scripted expert never needs more than ~2.5 m/s^2 and uses zero body rate (tilt <= 8 deg).
- Conclusion: the failure is the learned policy using an over-wide action authority in the first second (rate command into a flip), not a reward-weight or start-distribution problem.
Decision (R4): bound the action envelope inside the environment, as input constraints (the NMPC has them too): commanded acceleration +-3.5 m/s^2 (was 5), body rate +-0.6 rad/s (was 1.5); tilt_w 3 and bad_pen 60 built in. New env `Track4Env` (kind `track4`) in ocrl/am3_rl_direct.py; Track3Env and the old runs are untouched. Fairness check: the scripted PD+ff expert is unchanged under the limits (8.0 / 6.9 / 6.3 / 20.0 mm RMS, 6/6 on all four, results/am3_rl/eval/r4_expert_only.json). This IS a change of the policy interface, disclosed here: the R3 numbers (precision 2.4 mm, 4/6 survival) and R4 numbers are for different action ranges.
Runs (13:49): **R4_scratch_s0** (from scratch, 15M steps, lr decay, seed 0) and **R4_warm_s1** (warm start from R3_big_s0 final, 8M steps, lr 5e-5, target_kl 0.02, seed 1). Stopped R3d, R3e (no longer informative). Success rule unchanged: 100% survival, then lowest RMS, goal RMS <= 5 mm (PD+ff is 6-20 mm).
Note: tool gotcha (twice): `ps | grep` patterns that contain the run name also match the shell running the command; kill by PID.

### R4 result at 1M steps (14:10; n=6 starts, seeds 100-105; results/am3_rl/eval/r4_eval_warm1M.json): success rule met
R4_warm_s1 @1M (Track4Env: acc +-3.5 m/s^2, body rate +-0.6 rad/s, tilt_w 3, bad_pen 60; warm start from R3_big_s0 final):
| trajectory | PD+ff RMS / survived | R4_warm_s1 RMS / survived | bias x,y,z mm | tilt max |
|---|---|---|---|---|
| big_fig8 | 8.0 / 6 of 6 | 2.1 / 6 of 6 | -0.8 -0.4 -1.1 | 18.7 |
| circle_xy | 6.9 / 6 of 6 | 2.0 / 6 of 6 | -0.8 -0.4 -1.0 | 18.8 |
| fwd_up | 6.3 / 6 of 6 | 2.2 / 6 of 6 | -0.8 -0.4 -0.9 | 19.1 |
| lissa3d | 20.0 / 6 of 6 | 2.1 / 6 of 6 | -0.7 -0.4 -1.0 | 18.7 |
Training at 1M: mean episode length 2446 (full-length), reward/step 15.28 (PD 8.92). R4_scratch_s0 at 1M: reward -0.60, mean length 358 (still learning, as all from-scratch runs at this point).
Reading: the action-envelope bound removed the lost starts (R3: 3/6-4/6; R4: 6/6 on all four) while keeping the precision (about 2.1 mm, 3-10x better than PD+ff) and cutting tilt from 34-44 deg to 19 deg. The z bias is now about -1 mm (user's "stays lower" complaint: about 10 mm in R2, about 1 mm now). Caveats: n=6, one training seed, one checkpoint; the bound makes it a different policy interface from R2/R3. Next: n=20 evaluation (running), comparison videos (running: RL_R4_big_fig8 / lissa3d / circle_xy, seed 101), push-recovery test, later checkpoints and the scratch run as replicate.

## R4_warm_s1 @1M — n=20, push recovery, videos (2026-10-31)

**n=20 held-out starts (seeds 100–119), deterministic policy.** Survival 19/20 on all four trajectories, so the 100%-survival rule is NOT met at n=20 (it was at n=6). The single loss is the same start (seed 111) on every trajectory, terminating at step ~113, i.e. an initial-condition failure, not a trajectory failure. RMS over survivors: big_fig8 2.9 mm, circle_xy 2.8, fwd_up 2.9, lissa3d 2.8 (PD+ff 8.0 / 6.9 / 6.2 / 20.0); bias z about -1 mm; tilt max about 19 deg. Precision claim stands on survivors; the robustness claim does not yet.

**Push recovery** (0.5 s force at t=8 s on big_fig8, `eval_push`, RL vs PD+ff): both survive every case up to 2 N. At 1 N lateral peak error 41-45 mm vs 139-149 mm; at 2 N 87-105 vs 282-298 mm; at 4 N x 347-374 vs 681-725 mm. Both lose the drone at 4 N along +/-y (peak 154-160 mm RL vs 306-310 mm PD). RL has smaller peak error everywhere and recovers in <1.4 s for pushes up to 2 N; PD takes 1.0-2.7 s. The 4 N y loss is shared with PD, so it is a limit of the interface/envelope rather than an RL-only weakness (cause not diagnosed).

**Videos:** `Media_results/RL_R4_{big_fig8,lissa3d,circle_xy}_2026-10-31/` rendered (RMS after 2 s: RL 2.2-2.3 mm vs PD 6.0 / 19.6 / 7.1 mm; zero action terminated). Frames not yet visually inspected.

Next: evaluate 4M and 6.5M checkpoints at n=20 and R4_scratch_s0; target 20/20 survival and RMS <=5 mm.

### Later checkpoints at n=20 (R4_warm_s1) — survival gets WORSE, precision better
| ckpt | survival (each traj.) | RMS mm (4 trajs) | max err mm |
|---|---|---|---|
| 1M | 19/20 (seed 111) | 2.8-2.9 | ~20 |
| 4M | 18/20 (seeds 111, 118) | 2.7-3.0 | ~20 |
| 6.5M | 18/20 (seeds 111, 118) | 2.3-2.6 | ~13 |

No checkpoint meets the 100%-survival rule at n=20, so no policy is yet "selected". The losses are the same two initial conditions on every trajectory (terminate at step 113-163, ~1.2-1.6 s), independent of the reference. Their position/velocity offsets are mild (seed 108, with 0.44 m offset, survives); initial attitude is not an obvious discriminator (seed 113, 19.8 deg tilt, survives). In a quick replay of seed 111 the actions saturate at +/-1 and change sign every few steps, whereas seed 108 stays smooth, i.e. an out-of-distribution observation drives chatter. (My replay's printed state was stale, so tilt/error columns from that probe are not evidence.) Mechanism not proven.

Implication: precision keeps improving with training but the tail-risk starts do not; training reward is not penalising these ~5-10% starts enough. Candidate fixes (not yet tried; disclose if adopted): hard-start oversampling in training (distinct from eval seeds), a larger bad_pen, or action-rate penalty to remove chatter. R4_scratch_s0 is the independent check of whether this is warm-start inherited.

### Final models at n=20 (2026-11-01) — neither passes; longer training made survival worse
| run | survival | RMS survivors (mm) | lost seeds |
|---|---|---|---|
| R4_warm_s1 final (8M) | 16/20 | 3.5-4.4 | 101, 111, 116, 117 |
| R4_scratch_s0 final (15M) | 17-18/20 | 4.2-15.2 (max err up to 1.4 m, tilt up to 52 deg) | 105, 118 (+102 on fwd_up) |

Warm-start survival trend: 19/20 (1M) -> 18/20 (4M, 6.5M) -> 16/20 (8M). Scratch is not better: it loses different starts and has large transients on survivors. So the lost-start problem is not warm-start inherited; it is a generic training weakness, and more steps alone do not fix it. Best candidate so far remains the 1M checkpoint (19/20, 2.8-2.9 mm) but it does not meet the 100% rule. Next: hard-start oversampling / action-rate penalty (to be disclosed if adopted).

## R5 - trajectory + moving arm (started 2026-11-03 18:52 EST)
Added Track5Env (see SESSION_CONTEXT). Baseline check before training: PD+ff with the arm moving survives 16/20 (big_fig8) and 15/20 (lissa3d), RMS ~300 mm; with arm still it was 6-20 mm. At 50-100% arm amplitude PD (even with gains x2 and an integral term) lost the drone in ~3 s on 2 of 3 starts; the drone stays level (tilt <10 deg) while position runs away, motors strongly unbalanced, so this is arm-load coupling the outer-loop PD cannot cancel. I therefore limited the arm amplitude to 10-35% of the half-range (my choice; disclose, increase later). Policy gives a +-0.1 rad residual on top of the joint reference (design choice, not full joint authority). Training: Turing jobs 2452401/2452402, 15M steps, R4 recipe (lr-decay, tilt_w 3, bad_pen 60). Not yet done: hard-start oversampling / action-rate change (not applied in R5; R4 survival tail issue may recur - evaluate at n=20 before trusting).

### R5 results (2026-11-04, eval at 2026-11-04 13:29 EST; Turing jobs 2452401/2452402 finished ~02:30 UTC, 2.7 h / 2.3 h for 15M steps)
20 held-out starts (seeds 100-119), deterministic policy, arm moving (amplitude 10-35% of half-range):
| trajectory | PD+ff survive / RMS mm | R5_s0 final | R5_s1 final |
|---|---|---|---|
| big_fig8 | 16/20 / 304 | 20/20 / 4.7 | 20/20 / 7.3 |
| circle_xy | 15/20 / 286 | 20/20 / 4.5 | 20/20 / 7.1 |
| fwd_up | 16/20 / 299 | 20/20 / 4.4 | 20/20 / 7.6 |
| lissa3d | 15/20 / 294 | 20/20 / 5.7 | 20/20 / 8.2 |
Tilt max ~18 deg (PD ~9 deg); z bias ~-1.5 mm; joint tracking RMS 0.43-0.47 deg (s0) / 0.85 deg (s1) vs PD 0.63-0.78 deg. Both seeds meet the survival rule at n=20 (the R4 tail problem did not appear); s0 meets RMS <=5 mm on 3 of 4 trajectories (5.7 on lissa3d), s1 does not (7-8 mm). Caveats: only the final checkpoint tested; n=20 on seeds 100-119 which were also used for R4 diagnosis (not for R5 design); fresh seeds 1000+ (n=50, big_fig8 and lissa3d) launched; arm amplitude limited as disclosed above; the RMS goal was set for the arm-still task.

### R5 fresh seeds (1000-1049, n=50, big_fig8 and lissa3d; 2026-11-04 13:45 EST)
PD+ff: 33/50 and 30/50 survive, RMS ~290-300 mm. R5_s0 final: 50/50 and 50/50, RMS mean 13.6 / 17.1 mm but median 3.3 / 3.2 mm (a few starts with large transients, max error 57-76 mm). R5_s1 final: 49/50 and 49/50, RMS 17.5 / 20.0 mm, median 4.2. So on fresh seeds survival holds for s0 (100%) but not s1 (1 loss), and the 5 mm RMS goal is NOT met on mean RMS (typical start ~3-4 mm, tail starts 50-90 mm). The n=20 result (4.4-8 mm) was optimistic. Tail behaviour (slow settling on some starts) is the open issue; only final checkpoints evaluated, no checkpoint selection yet.

## R5 tail diagnosis and R6 (2026-11-08 17:48 EST)
**Which starts are bad:** on the 50 fresh seeds, RMS correlates with the yaw-joint amplitude (r = 0.46), not with the initial offset or speed (r about 0.1). Nearly all of the 12 starts above 10 mm RMS have q1 amplitude 10-12 deg, the top of the trained range.
**Mechanism (replay of seeds 1037, 1024):** when q1 reaches about 20-23 deg off nominal, y error grows 170-500 mm, the y-acceleration action is pinned at its limit 30-40% of the time and the rate action saturates, while the vehicle stays nearly level. The inner loop transfers the thrust moment with the COM at the NOMINAL posture; a 20 deg q1 offset moves the COM about 2 cm sideways (about 0.3 N m), and the attitude PD has no integral, so holding level needs more commanded tilt than the +-3.5 m/s^2 action limit (R4) gives.
**Intervention (proves the cause):** InnerLoop(aware=True) uses the COM at the measured joint angles (Track6Env). Scripted PD+ff, big_fig8, seeds 1000-1019:
| inner loop | arm amplitude (fraction of half-range) | survive | median RMS mm |
|---|---|---|---|
| fixed | 0.10-0.35 | 12/20 | 254.9 |
| fixed | 0.35-0.50 | 1/20 | 398.5 |
| aware | 0.10-0.35 | 20/20 | 13.5 |
| aware | 0.35-0.50 | 20/20 | 26.3 |
**Correction to earlier claims:** the "PD+ff about 300 mm" baseline in the R5 results was mostly caused by this inner-loop flaw, not by the difficulty of the task. RL's margin over PD on R5 was overstated. RL had been compensating for a defect in the interface. Also, the arm sinusoid reaches 2 x amplitude from nominal, so fraction 0.5 is the largest swing inside the operational range; the earlier statements about 50-100% included references outside it.
**Change from plan (disclosed):** the inner loop now uses measured joint angles for the COM. This is available from encoders; PD and RL get the same loop. R6 = Track6Env, arm amplitude 0.10-0.50, 15M steps, R4 recipe, 2 seeds, Turing jobs 2485439/2485440. R5 results stay as recorded for the old interface. Saved R5 checkpoints were not evaluated further, because they are all trained on the flawed loop.

### Corrected baseline and R5 push test (2026-11-08 17:55 EST)
**Track6Env (fixed inner loop, arm 10-50%), seeds 1000-1019:** PD+ff 20/20, RMS 18.4 mm (big_fig8) and 23.7 mm (lissa3d). R5_s0 unretrained on this loop: 20/20, RMS 40 mm, bias x 6.4 mm; worse than PD, as expected, since it learned to compensate for a defect that is now gone. Not evidence about RL on the fixed loop; R6 is. (eval/r5_on_track6_n20.json)
**R5_s0 push test (old loop, start seed 100, big_fig8):** RL survived all 18 cases. Peak error 33-45 mm at 1 N lateral, 88-147 mm at 2 N, 330-593 mm at 4 N; recovery under 0.25 s at 1 N but 3-4.7 s at 2-4 N lateral (R4 at 2 N: 0.8-1.4 s). **The PD column is INVALID:** every PD row shows LOST with NaN peak because PD on the old loop crashed on start seed 100 before the push at t = 8 s (same start crashes at step 303 unpushed). No RL-vs-PD push conclusion can be drawn. Repeat on track6 after R6. (eval/push_R5_s0_final.json)

### R6 results (2026-11-08 20:07 EST)
Track6Env (posture-aware inner loop, arm 10-50%), final checkpoints at 15M steps, 2 seeds, trained on Turing. Evaluated locally on seeds 1000-1049, deterministic. PD+ff uses the same inner loop and starts. Files: `Code/results/am3_rl/eval/r6_eval_{big_fig8,circle_xy,fwd_up,lissa3d}_n50.{json,log}`.

| path | PD+ff survive / mean mm | s0 survive / mean / median / worst mm | s1 survive / mean / median / worst mm |
|---|---|---|---|
| big_fig8 | 50/50, 18.4 | 49/50, 3.5, 2.5, 29.7 | 50/50, 5.3, 2.9, 83.1 |
| circle_xy | 50/50, 18.3 | 49/50, 3.4, 2.4, 32.4 | 50/50, 5.5, 2.7, 96.8 |
| fwd_up | 50/50, 18.6 | 49/50, 3.6, 2.5, 40.7 | 50/50, 6.4, 2.8, 127.3 |
| lissa3d | 50/50, 24.0 | 49/50, 3.5, 2.6, 31.8 | 50/50, 5.2, 2.9, 76.9 |

**Against the rule (100% survival, then lowest RMS; goal <= 5 mm):** s1 is selected, but its mean is 5.2-6.4 mm, so the 5 mm goal is NOT met on the mean (median is under 3 mm; a few starts give 77-127 mm). s0 meets 5 mm but fails survival: seed 1008 crashes after 64-65 control steps on all four paths, so the cause is the start, not the path (not yet investigated).
**Tradeoffs:** joint RMS is about 0.9-1.0 deg for RL vs 0.3-0.6 deg for PD+ff; peak tilt about 18.5 deg vs 9 deg.
**Push test (big_fig8, one start, seed 1000; `eval/push_R6_s{0,1}_final.{json,log}`):** valid PD column this time. RL survived all 18 cases on both seeds; PD+ff lost 4 N +y. Peak error 1 N lateral: RL 37-52 mm vs PD 141-239 mm. 4 N lateral: RL 431-817 mm vs PD 755-1501 mm. One start only: supporting evidence, not a statistic.
**Not done yet:** R6 videos (required before calling this rung done), seed 1008 diagnosis, RL vs NMPC.

### R6 early-crash diagnosis (2026-11-09 08:30 EST)
Scripts: `Code/results/am3_rl/eval/diag_r6/`. Path big_fig8, final checkpoints.
**Seed 1008 is hard for both seeds.** Start: 34 cm above the reference, 16.5 deg tilt (96th percentile of seeds 1000-1049). s0 crashes at 0.65 s (tilt > 60 deg). s1 does not crash, but at 1.2 s it is 918 mm off and has sunk to z = 0.80 m; that is its 83 mm RMS. PD+ff recovers smoothly on the same start (body rate about 1 rad/s vs 6-8 for RL).
**So s1's 50/50 overstates its reliability.** Early-crash test, first 2 s only, 300 unseen starts (seeds 2000-2299): s0 5/300, s1 3/300, PD+ff 0/300. Full-length runs were not done on these starts, so these are lower bounds.
**What happens in the s0 crash:** the acceleration channels are at their limit in the first 0.2 s, yaw action sits at about -1 from 0.24 to 0.42 s, roll rate climbs to 6 rad/s, then the rate actions saturate. Same signature as the R3/R4 crashes (actions pinned, body rate far above commanded).
**Cause not proven.** Crash starts are mostly not extreme: 4 of 8 have initial speed around or above the 95th percentile, 2 have high tilt, 2 show nothing unusual (2019, 2217). Small sample.
**Status of the rung:** videos show the intended behaviour on a typical start (RL_R6_*_2026-11-09). The 100% survival rule is still not met once tested on more starts, so R6 is NOT done by the criteria.

### R7 design, written before any result (2026-11-09 08:42 EST)
**Intervention test on trained R6 (first 2 s, 300 unseen starts 2000-2299, big_fig8; `Code/results/am3_rl/eval/diag_r6/intervene_results.jsonl`).** Crashes s0 / s1:
| override | crashes | excursions > 0.6 m |
|---|---|---|
| none | 5 / 3 | 4 / 4 |
| yaw action = 0 | 0 / 0 | 3 / 0 |
| body-rate actions = 0 | 2 / 1 | 1 / 0 |
| yaw and rate = 0 | 0 / 0 | 0 / 0 |
| acceleration clipped to +-0.7 of range | 1 / 0 | 0 / 1 |
| acceleration clipped to +-0.5 of range | 0 / 0 | 0 / 0 |
Yaw is the strongest single channel. These are overrides of a policy not trained for them, so they show which channels matter, not final performance.
**Change from R6 (disclosed):** Track7Env fixes the yaw reference at 0 and ignores the policy's yaw output. One change only. No path has a yaw target. PD already flies yaw 0, so its numbers are unchanged (checked: identical to 6 decimals on 2 starts). Everything else is the R6 recipe: 15M steps, 2 seeds, Turing.
**Pass criteria, fixed now:** same as before: 100% survival, then lowest RMS, goal <= 5 mm mean. Tests: (1) 50 starts, seeds 1000-1049, four paths, against PD 18-24 mm; (2) NEW stricter survival test: full-length runs on 300 starts, seeds 2000-2299, big_fig8, not used in training. R7 passes survival only if test 2 has 0 crashes. Note seeds 1008 and 2000-2299 were used to diagnose R6, so they are no longer untouched for design; a further fresh block (seeds 3000-3299) will be run as the final check.
**If yaw alone is not enough:** next is R8, adding the tighter acceleration limit (it also removed crashes above).

### R7 results (2026-11-09 11:24 EST)
Track7Env (fixed yaw), final checkpoints, seeds as pre-registered. Files: `Code/results/am3_rl/eval/r7_eval_*_n50.*`, `r7_surv2000_{0..5}.*`, `push_R7_s{0,1}_final.*`.
| test | PD+ff | R7 s0 | R7 s1 |
|---|---|---|---|
| 50 starts, big_fig8 | 50/50, 18.4 mm | 50/50, 3.6 mm (worst 21) | 49/50, 4.4 mm (worst 36) |
| 50 starts, circle_xy | 50/50, 18.3 | 50/50, 5.0 (worst 87) | 49/50, 4.4 (worst 38) |
| 50 starts, fwd_up | 50/50, 18.6 | 50/50, 10.0 (worst 317) | 49/50, 4.8 (worst 45) |
| 50 starts, lissa3d | 50/50, 24.0 | 50/50, 3.5 (worst 22) | 49/50, 4.4 (worst 30) |
| 300 starts full length, big_fig8 | 300/300, 17.7 | 298/300, 2.9 (median 2.4) | 300/300, 3.4 (median 2.2, worst 104) |
**Verdict: R7 does NOT pass.** The rule was 100% survival. s0 loses seeds 2016 and 2174 (about 0.65 s). s1 passes the 300 starts but loses seed 1017 on all four paths (about 0.9 s). The fresh block 3000-3299 was not run, to keep it unseen for the next round.
**Better than R6:** s1 mean is 4.4-4.8 mm, inside the 5 mm goal on all 4 paths (R6 s1: 5.2-6.4). First-2 s crashes on seeds 2000-2299 went from 5/3 to 2/0. Push: RL survived all 18 on both seeds; PD lost 4 N +y.

### R8 design, written before any result (2026-11-09 11:28 EST)
**Override test on trained R7 (first 2 s, 350 starts: seeds 1000-1049 and 2000-2299; `diag_r6/intervene_results_R7.jsonl`). Crashes s0 / s1:** none 2 / 1; body rate 0: 0 / 1; body rate halved: 0 / 1; acceleration clipped to 0.7 of range: 0 / 1; to 0.5 of range: 0 / 0 (and 0 excursions). Only the 0.5 clip clears both seeds.
**Change from R7 (disclosed):** Track8Env limits commanded acceleration to +-1.75 m/s^2. One change only; as planned in the R7 design. PD+ff goes through the same limit, so its baseline is re-measured on track8.
**Known risk, stated now:** a 4 N sideways push needs about 2.6 m/s^2 to cancel, more than 1.75. R8 may lose 4 N pushes that R7 survives. The push test will report it either way.
**Pass criteria, unchanged:** 100% survival then lowest RMS, goal <= 5 mm mean. Tests: 50 starts seeds 1000-1049 on four paths; full-length 300 starts on 2000-2299; if both pass, fresh seeds 3000-3299 as the final check (not looked at before). Seeds 1000-1049 and 2000-2299 have now informed the design, so only the 3000 block counts as unseen.

### R8 results (2026-11-09 14:23 EST)
Track8Env (fixed yaw, acceleration +-1.75 m/s^2), final checkpoints. PD+ff re-measured under the same limit. Files: `eval/r8_eval_*_n50.*`, `r8_surv2000_{0..5}.*`, `push_R8_s{0,1}_final.*`.
| test | PD+ff | R8 s0 | R8 s1 |
|---|---|---|---|
| 50 starts, big_fig8 | 50/50, 18.5 mm | 50/50, 6.8 (median 5.6, worst 31) | 50/50, 6.7 (median 4.2, worst 39) |
| 50 starts, circle_xy | 50/50, 18.4 | 50/50, 6.6 (5.5, 30) | 50/50, 7.4 (4.1, 68) |
| 50 starts, fwd_up | 50/50, 18.7 | 50/50, 6.9 (5.6, 33) | 50/50, 8.6 (4.3, 112) |
| 50 starts, lissa3d | 50/50, 24.0 | 50/50, 7.2 (6.2, 30) | 50/50, 7.0 (4.7, 46) |
| 300 starts full length, big_fig8 | 300/300, 17.7 | 300/300, 5.7 (5.2, 27) | 300/300, 4.8 (3.9, 67) |
**Survival: first version with no lost start on either seed. Accuracy: 5 mm mean goal NOT met** on the 50-start tests (R7 s1 was 4.4-4.8 mm). The limit traded accuracy for survival. Joint RMS 0.6-0.7 deg (R7 about 0.8), peak tilt 13 deg (R7 19).
**Push (one start, seed 1000):** RL survived all 18 on both seeds, so the feared loss at 4 N did not happen, but peak errors are larger than R7 (4 N +y: about 1.3 m vs 0.7-0.8 m). PD+ff under the limit lost 3 of 18 (4 N +x, +y, -y).

### R8 final check on fresh seeds (2026-11-09 14:40 EST)
Seeds 3000-3299, full length, big_fig8, never used for design (`eval/r8_fresh3000_{0..5}.*`). PD+ff 300/300, 17.0 mm. R8 s0 300/300, 5.65 mm (median 4.95, worst 35.8). R8 s1 300/300, 4.82 mm (median 4.01, p99 26.0, worst 44.2).
**Verdict.** Survival rule: PASSED. No lost start on either seed in 50 x 4 paths + 300 + 300 fresh runs. Selected by the rule: R8 s1. Accuracy goal (5 mm mean): met on big_fig8 at n=300 (4.8 mm on both blocks), NOT met on the 50-start tests (6.7-8.6 mm on seeds 1000-1049; why that block is worse than the 300-start blocks was not investigated); circle_xy, fwd_up and lissa3d have no 300-start test. So the accuracy goal is only partly met; this is not claimed as a full pass. The fresh test covers one path.

### R9 design, written before any result (2026-11-09 15:09 EST)
**Change from R8:** acceleration limit +-2.5 m/s^2 (R8 1.75, R7 3.5); nothing else. Aim: recover R7's accuracy without its crashes. Override test on R7 at about this limit (0.7 of range) still left seed 1017 crashing on s1, so crashes may return. **Same pass criteria and tests as R8**, with PD re-measured on track9. Seeds 3000-3299 have been used for R8, so a new unseen block, 4000-4299, is the final check for R9. R8 stays selected unless R9 passes survival on all tests and has lower RMS.

### RL vs NMPC on the same task, first numbers (2026-11-09 15:40 EST)
`Code/scripts/am3_rl_vs_nmpc.py`: N unchanged, same plant, starts, noise, paths, arm motion and survival limits as track8; RL = R8 s1. Raw table: `Code/results/am3_rl/eval/vsnmpc_summary.txt`, blocks `vsnmpc_*.json`. Seeds 1000-1049.
**Harness note (disclosed):** N's model contains the wall and the free-flight plant does not; at RL's nominal pose the pad is at the wall face, so N first crashed in 0.3 s. Fix: N works in a frame shifted 0.5 m in x. N itself not edited. N keeps one control period of latency (Episode convention); RL's inner loop has none. The Episode 25 cm failsafe is not used.
**Incomplete:** fwd_up block 1030-1039 did not finish (first run ran over 20 min, rerun timed out at 200 s after 4 starts), so fwd_up has 40 of 50 starts. Why it was slow is unknown. Timing under 20 parallel processes is not valid for real-time claims. N's joint error is large on some starts (up to about 10 deg in rerun log): N tracks the pad, not the joints, so this needs checking before calling the arm task equal.

### RL vs NMPC: no-offset starts and videos (2026-11-09 17:45 EST)
Starts on the reference, no random offset; arm motion still random; seeds 1000-1049. `Code/results/am3_rl/eval/vsnmpc_nooffset_summary.txt`.
| path | N survive, median / mean / worst | RL (R8 s1) | PD+ff |
|---|---|---|---|
| big_fig8 | 50/50, 0.13 / 0.51 / 6.6 mm | 50/50, 3.47 / 3.87 / 20.6 mm | 50/50, 16.8 / 17.5 / 42.2 mm |
| lissa3d | 50/50, 0.14 / 1.18 / 33.4 mm | 50/50, 3.84 / 4.22 / 18.1 mm | 50/50, 22.3 / 23.5 / 44.5 mm |
**So N's losses on the RL starts come from the random start error (N loses 22-28% with it, 0% without).** They are not explained by offset size alone: on big_fig8 crashes occur at 0.2-2.2 s; correlation of loss with start distance +0.21, tilt +0.15, body rate +0.34; the smallest lost start is 8 cm off and the largest survivor 44 cm. Mechanism not established. N was designed for near-reference starts, and these random starts are what RL is trained on; both facts matter when reading the comparison.
Videos: `Media_results/NMPC_vs_RL_{big_fig8,lissa3d,circle_xy}_seed{1000,1006}_2026-11-09` (script `am3_rl_vs_nmpc_video.py`). Seed 1000 is typical; seed 1006 is a start N loses on every path and RL's worst on big_fig8 and lissa3d. Seeds chosen before rendering; 1006 chosen knowing N loses it.

### R9 results (2026-11-09 18:20 EST)
Track9Env (fixed yaw, acceleration +-2.5 m/s^2), final checkpoints, PD+ff re-measured under the same limit. Files: `eval/r9_eval_*_n50.*`, `r9_surv2000_{0..5}.*`, `push_R9_s{0,1}_final.*`.
| test | PD+ff | R9 s0 | R9 s1 |
|---|---|---|---|
| 50 starts, big_fig8 | 50/50, 18.5 mm | 50/50, 4.6 (median 3.2, worst 35) | 49/50, 5.8 (3.4, 38) |
| 50 starts, circle_xy | 50/50, 18.4 | 50/50, 4.5 (3.0, 34) | 49/50, 5.8 (3.3, 40) |
| 50 starts, fwd_up | 50/50, 18.7 | 50/50, 4.9 (3.1, 41) | 49/50, 6.3 (3.4, 46) |
| 50 starts, lissa3d | 50/50, 24.0 | 50/50, 4.5 (3.2, 33) | 49/50, 5.6 (3.5, 36) |
| 300 starts full length, big_fig8 | 300/300, 17.7 | 300/300, 3.4 (3.1, 27) | 300/300, 4.0 (2.8, 45) |
s0 meets both rules on these tests. s1 loses seed 1006 on all four paths (1.7-1.9 s), so reliability depends on the training seed. Push: RL lost 0 of 18 on both seeds; PD+ff lost 2. Final check pending on unseen seeds.

### R9 final check on unseen seeds (2026-11-09 18:41 EST)
Files: `eval/r9_fresh4000_{0..5}.*` (300 starts, big_fig8, seeds 4000-4299), `eval/r9_fresh4300_{circle_xy,fwd_up,lissa3d}.*` (50 starts each, seeds 4300-4349). The 4300 block is an addition beyond the pre-registered check.
| test | PD+ff | R9 s0 | R9 s1 |
|---|---|---|---|
| big_fig8, 300 | 300/300, 17.5 mm | 300/300, 3.51 (median 2.99, worst 36.6) | 300/300, 4.01 (2.90, 54.5) |
| circle_xy, 50 | 50/50, 19.5 | 50/50, 3.60 (3.05, 9.4) | 50/50, 5.86 (3.45, 42.6) |
| fwd_up, 50 | 50/50, 19.7 | 50/50, 3.63 (3.03, 8.7) | 50/50, 6.17 (3.48, 45.8) |
| lissa3d, 50 | 50/50, 24.9 | 50/50, 3.71 (3.24, 9.2) | 50/50, 5.22 (3.51, 27.2) |
**Verdict: R9 s0 passes survival and the 5 mm mean goal on every test run.** It replaces R8 s1 as selected. **Limit on this claim:** one of two training seeds. s1 lost seed 1006 on all four paths on the earlier block, so the result depends on the training seed; "RL reliably learns this" is not shown. 1/350-type loss rates cannot be excluded for s0 from these samples.

### RL (R9 s0) vs NMPC on the same task (2026-11-09 19:08 EST)
Same protocol as before; seeds 1000-1049, RL = R9_turing_s0 (`eval/vsnmpc9_*`, summary `vsnmpc9_summary.txt`). N results are identical to the R8 comparison, as expected. Survival, median / mean RMS (survivors only):
| path | N | RL (R9 s0) | PD+ff |
|---|---|---|---|
| big_fig8 | 36/50, 0.13 / 1.26 mm | 50/50, 3.15 / 4.63 | 50/50, 17.6 / 18.5 |
| circle_xy | 39/50, 0.13 / 0.45 | 50/50, 3.03 / 4.47 | 50/50, 17.6 / 18.4 |
| fwd_up (40 starts) | 29/40, 0.13 / 0.49 | 40/40, 3.06 / 5.27 | 40/40, 18.6 / 19.5 |
| lissa3d | 36/50, 0.14 / 1.53 | 50/50, 3.23 / 4.46 | 50/50, 22.6 / 24.0 |
**fwd_up block 1030-1039 hung again** and timed out after 25 min, the same block as last time, so this is reproducible and not random load. Those 10 starts are missing. Videos: `Media_results/NMPC_vs_RL9_*_2026-11-09` (seeds 1000, 1006).

### R10 wall line: design, written before any result (2026-11-09 19:28 EST)
**Change of task (requested by the user, 2026-11-09 19:28 EST):** the wall rung uses Aryan's start: exact hover equilibrium, base at the reference, pad 5-8 cm from the wall, no random offset beyond his scenario spread (gap0 0.05-0.08 m, y/z within 8 mm). Random starts from the free-flight rungs are not used here.
**Why Aryan's N starts cleanly:** that equilibrium start, then 1 s hover. His scenarios vary it only by gap and about 8 mm; our random starts were 20-40 cm off, which N was not designed for.
**Env (`ocrl/am3_rl_wall.py`):** same plant with the wall, same Task state machine and contact logic as N and B, same 10 ms-period latency convention, base holds the fixed reference, joints follow IK of the Task's pad reference plus a policy residual (+-0.25 rad). Stops mirror the Episode failsafe (tilt 40 deg, base 25 cm, force 4 N, other contacts). Fixed yaw and acceleration +-2.5 m/s^2 kept from R9.
**Joint gains changed (disclosed):** the free-flight gains 1.57/0.016 oscillate (12 rad/s limit cycle) once the harness latency is on. kp 1.0 / kd 0.05 chosen from a sweep with latency on; scripted PD then reaches the wall with 100% contact on the 3 lines but tracks poorly (40-60 mm pad error) and stops during trace.
**Score:** same Episode harness and `am3_acceptance_v2` criteria as N and B, on the line scenarios of `am3_domain_heldout/heldout2` (24 scenarios, not used in training). A first 5M-step run checks whether it learns; no claim until scored.

### R10 baselines: N and B on the 24 wall-line scenarios (2026-11-09 19:30 EST)
`scripts/am3_eval_wall.py N|B --block b --blocks 6`, Episode harness, `am3_acceptance_v2`. Rows: `results/am3_rl/eval/wall_{N,B}_{0..5}.json`.
| | completed | pass all | track RMS med / worst | base max med | contact med |
|---|---|---|---|---|---|
| N | 24/24 | 24/24 | 0.41 / 0.95 mm | 0.6 mm | 1.00 |
| B | 24/24 | 0/24 | 19.3 / 23.3 mm | 39 mm | 0.73 |

### R10_wall_s0 (from scratch): did not learn to hold the base (2026-11-09 21:05 EST)
Turing job 2493385 died at 4.0M of 5M steps on an NFS write error in the episode log (`BlockingIOError`; the write is now guarded). Reward/step rose to 3.6 (scripted PD reference 8.9), but mean episode length was about 130 steps.
Diagnosis (`results/am3_rl/eval/diag_r10/why_end.py`, model_4000k, 8 starts): every episode ends in the "position" phase at about 2 s, base more than 25 cm off, tilt 12-15 deg. Learning base hold and the wall task together from scratch is too much at once; the track rungs were always warm-started.
Scripted PD for reference: reaches trace on 6/6 but the base drifts 9-12 cm sideways in contact and a finger touches the wall. It has no contact-force compensation, so it is a weak reference, like B.

### R10b: warm start from R9 s0 with a widened input (2026-11-09 21:05 EST)
The wall observation is the R9 observation (55) with 20 entries appended. The first layer of the policy and value nets gets zero weights for the new entries, so at step 0 the policy flies exactly like R9 (`am3_train_direct.py --init-from`, automatic when the sizes differ). Everything else as R10.
Check before training: the widened policy survives 8/8 starts for 11 s, tilt 0.5 deg, but holds a steady 40 mm base offset in the wall posture and never reaches the wall ("no contact acquired"). Turing job 2493494, 10M steps, seed 0.
