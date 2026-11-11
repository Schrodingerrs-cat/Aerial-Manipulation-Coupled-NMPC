# Writing on the wall with a moving base (started 2026-11-10 18:45 EST)

Goal (Sahil, 2026-11-10): one general contact-writing capability - e.g. a sentence on the wall, words about 20 cm - done by NMPC, by RL, and by NMPC + RL where NMPC falls short.
This is outside the frozen plan (fixed hover point); approved by Sahil 2026-11-10. Results here are capability results, not gate evidence.

## Pieces
- `Code/ocrl/am3_write.py`: single-stroke capital font A-Z (glyph 0.6 x 1, scaled by letter height), `text_strokes`, `WriteTask`, `WriteEpisode`.
  Per stroke: position/travel with pen up (gap 12 mm) -> approach until contact is measured -> settle -> trace -> release -> lift. Contact detection, settle, recovery and limits are the ones in am3_task.Task.
  Base reference follows the pen with the hover offset (pB = BASE + pen displacement in y, z; vB its rate), so letter size is not limited by the arm. Reading direction is -y (drone faces +x).
  Pen speed 25 mm/s mean along a stroke (min-jerk timing), travel 50 mm/s. NMPC weight table gets `lift`, `travel` -> free-flight weights at run time (frozen file untouched).
- `Code/scripts/am3_write_eval.py`: metrics: completed, strokes, trace_rms/max (pad vs reference during trace), coverage (reference points within 2 mm of ink), contact (trace), drag (pressing while pen up), base_max (vs moving reference), tilt.
- `Code/scripts/am3_write_video.py`: live render, cameras follow the base; red = ink, green = reference.
- `Code/scripts/am3_write_mismatch.py`: same under unknown model mismatch (payload on the hand, thrust, motor lag, joint damping, gusts/wind).
- RL env: `WallEnv(write_p=...)` (`--env write`): 70 % episodes write 1-2 random letters of random height 3-10 cm with the moving base, 30 % the fixed-base shapes. The wall env now reads the base reference from the task (fixed hover when absent); parity on heldout_00 bit-exact (2.5002318862282995 mm before and after).

## First results (2026-11-10 18:45 EST)
| controller | text | letter height | completed | trace rms | coverage | contact | drag | base max err |
|---|---|---|---|---|---|---|---|---|
| N (NMPC) | L | 6 cm | yes | 0.41 mm | - | 1.00 | - | 0.9 mm |
| N | OM | 10 cm (word 16 cm) | yes, 2/2 strokes | 0.36 mm | 1.00 | 1.00 | 0 | 0.7 mm |
| RL R10c (no moving-base training) | OM | 10 cm | yes, 2/2 | 3.44 mm | 0.87 | 1.00 | 0 | 24 mm |
Video: `Media_results/Write_OM_10cm_NMPC_2026-11-10/`.
Bugs found and fixed: glyph arcs joined with a repeated point gave zero-length fillet segments (NaN path); the RL controller must be attached to the episode plant.

## Plan
1. RL: R12_write_s0 on Turing (job 2503426, 30M, warm from R10c). Score on held-out texts and the fixed-base shapes.
2. NMPC: longer text (sentence), and mismatch suite to find where it degrades.
3. NMPC + RL: residual policy on top of NMPC trained under randomized mismatch, only if step 2 shows NMPC failing.

## Correction: drone and arm must write together (2026-11-10 19:11 EST)
Sahil's review of the first OM video: the base copied the pen exactly, so the arm stayed still ("moving the hand with a frozen finger"). Wanted: hand and fingers together -
the drone keeps moving while the arm writes, heads for the next letter while the arm finishes the current one, and either can move when the other does.
Change (`WriteTask.plan_base`): base path = pen path over global arclength (strokes + pen-up travels), Gaussian-smoothed (SIGMA_B 0.10 m) so it looks ahead, then pushed so the
pen-base offset stays within the arm's reach (3.5 cm y, 3 cm z). The base is indexed by pen progress, so it waits if contact takes longer than planned. Pen speed is now a parameter.
| N, OM 10 cm | pen speed | time | trace rms | coverage | arm motion relative to base (y, z) | joint ranges |
|---|---|---|---|---|---|---|
| coordinated | 25 mm/s | 31.7 s | 0.36 mm | 1.00 | 48, 64 mm | 10, 29, 36 deg |
| coordinated | 50 mm/s | 20.9 s | 0.54 mm | 1.00 | 48, 64 mm | |
| coordinated | 75 mm/s | 17.3 s | 0.79 mm | 0.996 | 48, 64 mm | |
R12/R13 restarted on the coordinated task with pen speed 25-75 mm/s: jobs 2503635 (R12_write_s0) and 2503636 (R13_write_dr_s0). The earlier frozen-arm runs were cancelled after ~25 min.

## NMPC + RL hybrid H1 (2026-11-10 20:15 EST)
`Code/ocrl/am3_hybrid.py`: u = u_NMPC + residual (rotor speeds +-60 rad/s, about 5 % of hover; joint torques +-25 % of the continuous limit). Zero residual = plain NMPC;
PPO starts near zero, so training starts from the NMPC. Env steps exactly the Episode.run loop (sensors, failsafe, one-period latency, 50 Hz). Tasks: 70 % random 1-2 letters
(3-10 cm, 25-75 mm/s, coordinated drone + arm), 30 % fixed-base shapes; mismatch randomized every episode (same sampler as R13: `am3_rl_wall.sample_mismatch`).
Reward: alive 2, pad error, base error, contact in contact phases, pen drag when up, force over 2 N, action size; -100 on failure, +100 when done. Plain NMPC earns 2.25/step (nominal).
HybridCtrl is an Episode controller, so H is scored by the same harness as N and RL. acados built on Turing (2026-11-10) so hybrids can also train there.
Run: H1_hybrid_s0 locally (20 envs, 10M steps, started 20:12 EST).

## 2026-11-11 11:25 EST — R12 / R13 scored, H1 diagnosed, next runs (R14, H2)

R12 = PPO, writing + shapes, nominal model, 30M steps. R13 = same plus domain randomization (DR), 30M steps. Both on Turing.
Held-out words never trained on as words (training sees 1-2 random letters, 3-10 cm). Results in `Code/results/am3_write/`.

**Held-out words, 10 cm letters, 50 mm/s (trace RMS mm / coverage), all completed**

| word | N | R12 | R13 |
|---|---|---|---|
| OM | 0.54 / 1.00 | 1.77 / 0.88 | 4.00 / 0.72 |
| HI | 0.67 / 1.00 | 1.38 / 1.00 | 4.06 / 0.83 |
| WPI | 0.55 / 1.00 | 1.83 / 0.87 | 3.74 / 0.60 |
| ZEN | 0.60 / 1.00 | 2.07 / 0.86 | 5.84 / 0.57 |
| HI at 20 cm | 0.72 / 1.00 | 1.38 / 1.00 | 4.34 / 0.94 |

20 cm is outside the training range (3-10 cm); all three complete it.

**Model mismatch, OM 10 cm, 25 mm/s**

| case | N | R12 | R13 |
|---|---|---|---|
| nominal | 0.36 | 1.13 | 4.10 |
| payload 100 g | FAIL | FAIL | **4.50 OK** |
| payload 200 g | FAIL | FAIL | FAIL |
| thrust -10 % | 1.42 | 2.00 | 4.67 |
| motor lag x2 | FAIL | FAIL | **4.00 OK** |
| damping x3 | 0.38 | 1.19 | 3.33 |
| gust y | 0.40 | 1.50 | 4.76 |
| wind sine | 1.04 | 4.96 (cov 0.44) | 4.03 |

Reading: N is the most accurate wherever its model holds. R12 is a big step up from R10c (1.1-2 mm vs 3.9 mm) but not robust.
R13 is the only controller that survives payload 100 g and slow motors, at the cost of ~4 mm error and 5-10 % pen drag.

**H1 (hybrid NMPC + residual) failed to learn; cause found.** Zero-residual NMPC under the training DR: 5/12 episodes lose control within ~1.5 s
(every payload >= 0.15 kg), one ran the full 600 s cap. So H1's -8 reward/step is the randomized plant, not the residual; PPO had almost no
recoverable episodes to learn from. H1b (smaller exploration noise) behaved the same and stalled on Turing (long episodes); cancelled.

**Next runs (Turing, submitted 11:20 EST):**
- R14_write_drc_ft_s0: fine-tune R12 with DR ramped in over the first 300 episodes per env (`write_drc`), lr 1e-4, 20M. Goal: R12 accuracy + R13 robustness.
- H2_hybrid_s0: hybrid with (1) episode cap = stroke time + margin, (2) same DR ramp, (3) joint residual 0.25 -> 0.5 tau_c
  (0.25 kg on the hand needs ~0.6 N m the NMPC does not model), log_std -2.5, 10M.

## R14 and H2 results (2026-11-11 17:40 EST)
R14 = R12 fine-tuned with DR ramped over 300 episodes (write_drc, lr 1e-4, 20M steps, Turing job 2507364; final train reward 8.38/step).
Scored with the same tests as R12/R13 (logs in `Code/results/am3_write/logs_1111/*R14*`):

| Test (trace RMS, mm) | N | R12 | R13 | R14 |
|---|---|---|---|---|
| Held-out OM/HI/WPI/ZEN, 10 cm | 0.54-0.67 | 1.38-2.07 | 3.74-5.84 | 4.14-4.58 |
| HI 20 cm | 0.72 | 1.38 | 4.34 | 4.54 |
| payload 100 g | fail | fail | 4.50 | 5.54 |
| motor_slow | fail | fail | 4.00 | 3.94 |
| payload 200 g | fail | fail | fail | fail |
| other 5 mismatch cases | ok | ok | 3.3-4.8 | 3.9-4.4 |

Result: R14 ends up in the same place as R13 (robust, about 4 mm). Fine-tuning from R12 did not keep R12's accuracy once DR was on.
H2 check (16 identical full-DR episodes, seed 7): plain NMPC completes 7/16, H2 at 5M completes 8/16; both lose control on the same heavy-payload episodes within about 100 steps. The residual does not yet prevent the early loss of control.
