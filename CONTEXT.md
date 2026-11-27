# CONTEXT.md — start here (people and AI agents)

Last updated: **2026-11-21 19:30 EST** (Sahil + Claude). Branch: **`payload-swing`**; all recent work is there, and `main` is old.

This file is the short, current picture. Read it first, then `CLAUDE.md` (rules), then the files it points to.
`SESSION_CONTEXT.md` is the long chronological log; use it only for history.

---

## 1. One project, two courses: keep them apart

| | Capstone RBE 594 | RBE 595 Optimal Control & RL |
|---|---|---|
| Team | Ashish Sukumar + Sahil Gajera | Aryan Shah + Sahil Gajera |
| Advisor | Prof. Rosendo | course professor |
| Focus | analytical load mass (product of exponentials, no Bayesian estimator), payload-aware manipulation | NMPC vs RL vs NMPC+RL for writing on a wall in contact, robustness |
| Next deadline | capstone presentation **Mon 2026-11-23**, final **2026-11-27** | final presentation (show failures as well as successes, push the limits, lessons, next steps) |

Code, results and videos are shared. Framing and claims stay per course. New notes are tagged `[capstone]` or `[optimal]`.

## 2. The robot and the task (this repo)

- **Robot:** 1.52 kg quadrotor with a 3-joint arm (yaw, shoulder, elbow; wrist fixed at 20°) and a soft pen pad, simulated in MuJoCo 3.10.
  - Links: mount 5 cm, upper arm 16 cm, forearm 14 cm.
  - This is the same arm geometry as Ashish's 4-joint flight robot, with the wrist joint locked.
  - Design: `Code/config/am3_design_v3.json`. Model: `Code/model/aerial_manipulator_v3.xml`.
- **Task:** write whole sentences on a wall ("HELLO WPI", "THE QUICK BROWN", ...). The pad presses on the wall while the drone flies along the text.
  - Code: `Code/ocrl/am3_write.py`. Story: `Findings/Write/01_wall_writing.md`.

## 3. Controllers

| Name | What it is | Code |
|---|---|---|
| **N** | Coupled NMPC: full arm + drone model with contact, acados SQP-RTI at 50 Hz | `Code/ocrl/am3_nmpc.py`, `am3_ctrl_n.py` (frozen) |
| **ND** | N plus a generalized-momentum disturbance observer. It estimates unknown force, moment and joint torques online and feeds them into the NMPC model (offset-free). Command name `ND:10,10` | `Code/ocrl/am3_nmpc_dob.py` |
| **RL (R-series)** | PPO policy that sends commands to a fast inner loop. The inner loop has the same kind of disturbance observer. Trained on Turing with randomized payload, motor lag, thrust error, damping and wind | env `Code/ocrl/am3_rl_wall.py`, inner loop `Code/ocrl/am3_rl_direct.py`, training `Code/scripts/am3_train_direct.py` |
| B | Older decoupled baseline (geometric + separate arm control) | `Code/ocrl/am3_ctrl_b.py` |

**RL history:**
- R12 → R15: best accuracy, no robustness.
- R17: domain randomization.
- R18/R19: observer joint channel on, so it holds payloads.
- R20: trained on up to 5 letters, which fixes contact loss late in long sentences.
- **R21**: adds a torque-margin penalty; best so far.
- **R22** (2026-11-18): wider randomization range. More robust (finishes all sentences in 7 of 8 conditions, incl. slow motors) but ~0.5 mm less accurate, and 200 g got worse (0/15, falls before writing starts).

Every round's design, cause and evidence: `Findings/RL/04_phase1_static_rl.md`.
Plain-language RL story up to R10: `Findings/RL/00_RL_story.md`.

## 4. The benchmark and current results [optimal + capstone update]

**How it is run:** `Code/scripts/am3_write_bench.py CTRL --noise real [--case CASE] --name NAME`.
- 5 sentences × 3 sensor-noise seeds = 15 runs per cell.
- Realistic sensor noise.
- Conditions are defined in `Code/scripts/am3_write_mismatch.py`.

**Pass:** 15/15 completed, tracing RMS ≤ 2 mm, no dragging.

Each cell is completed/15 and mean tracing RMS in mm:

| condition | N | ND | R19 | R20 | **R21 (best RL)** | R22 |
|---|---|---|---|---|---|---|
| nominal | 15, 2.07 | 15, 2.62 | 15, 2.70 | 15, 2.53 | **15, 1.87** | 15, 2.47 |
| payload 100 g | 0 | 15, 2.65 | 15, 3.18 | 15, 2.94 | 15, 2.32 | 15, 2.77 |
| payload 200 g | 0 | **15, 3.05** | 0 | 5, 3.71 | 5, 2.75 | 0 |
| motor lag ×2 | 0 | 0 | **15, 3.09** | 0 | 11, 2.47 | **15, 2.97** |
| damping ×3 | 15, 2.31 | 15, 2.77 | 15, 2.79 | 15, 2.67 | 15, 1.94 | 15, 2.55 |
| thrust −10 % | 15, 2.70 | 15, 2.59 | 15, 3.04 | 15, 2.65 | 15, 1.97 | 15, 2.28 |
| gust | 15, 2.03 | 15, 2.66 | 15, 2.70 | 15, 2.54 | 15, 1.90 | 15, 2.53 |
| sine wind | 15, 2.23 | 15, 2.70 | 15, 2.84 | 15, 3.21 | 15, 2.64 | 15, 2.81 |

**Honest summary:**
- RL (R21) is the most accurate controller and is under 2 mm on 4 conditions.
- ND is the only controller that handles 200 g; it fails slow motors.
- R22 (wider training) trades accuracy for robustness: completes 7/8 conditions, but none under 2 mm, and 0/15 at 200 g.
- **No controller passes all 8 conditions yet.**

**Why 200 g is hard:** the shoulder servo limit is 1.0 N·m. From the analytical model, the static shoulder torque is 0.47 N·m plus 2.19 N·m per kg of load, so the limit is 242 g and 200 g already uses 91 %. The RL policies' arm swing then saturates the shoulder and the drone flips. NMPC plans within the torque limit.

### Smoothness and the filtered NMPCs (2026-11-19)

The bench now also measures smoothness (`smooth_metrics` in `Code/scripts/am3_write_eval.py`). With realistic sensor noise, plain N/ND re-plan from raw noisy measurements and their rotor commands chatter (median jump 360 rad/s per step); RL had been 6-8x smoother. Two new NMPC variants (`Code/ocrl/am3_nmpc_filt.py`, N's frozen files untouched):
- **NF** (`NF:0.3`): N + fixed-gain state filter (predict with N's model, blend 30 % of each measurement).
- **NDF** (`NDF:0.5,f`): ND + the same filter, predicting with the observer's disturbance estimate; observer runs on the filtered state.

Completed/15, trace RMS mm; last row = rotor command change (krad/s per s, no mismatch; lower = smoother):

| condition | N | ND | NF | **NDF** | R15 | R21 | R22 |
|---|---|---|---|---|---|---|---|
| nominal | 15, 2.07 | 15, 2.62 | **15, 1.43** | 15, 2.32 | 15, 1.74 | 15, 1.87 | 15, 2.47 |
| payload 100 g | 0 | 15, 2.65 | 0 | 15, 2.64 | | 15, 2.32 | 15, 2.77 |
| payload 200 g | 0 | 15, 3.05 | 0 | 15, 3.40 | | 5, 2.75 | 0 |
| motor lag ×2 | 0 | 0 | 0 | 0 | | 11, 2.47 | **15, 2.97** |
| damping ×3 | 15, 2.31 | 15, 2.77 | 15, 2.87 | 15, 2.52 | | 15, 1.94 | 15, 2.55 |
| thrust −10 % | 15, 2.70 | 15, 2.59 | 0 | 15, 2.31 | | 15, 1.97 | 15, 2.28 |
| gust | 15, 2.03 | 15, 2.66 | 15, 1.59 | 15, 2.33 | | 15, 1.90 | 15, 2.53 |
| sine wind | 15, 2.23 | 15, 2.70 | 15, 2.57 | 15, 2.33 | | 15, 2.64 | 15, 2.81 |
| rotor cmd change | 21.1 | 27.2 | 4.6 | 11.3 | **3.3** | 3.5 | 3.5 |

- Filtering makes NMPC smooth and (on the known model) the most accurate, but the nominal-model filter breaks under model error (NF). Adding the observer (NDF) restores robustness to 7/8 at 2.4x less chatter than ND, still ~3x rougher than RL.
- **No NMPC variant survives motor lag ×2** (a lag is not a constant disturbance the observer can learn). RL handles it (R22 15/15). This is the clearest case for RL / NMPC+RL.
- Details: `Findings/RL/04_phase1_static_rl.md` (sections Smoothness, NF, NDF). Figure `Media_results/2026-11-19_0850_smoothness_N_vs_R21_signals.png`.

## 5. Capstone analytical work in this repo [capstone]

- **`Code/ocrl/am3_kin_poe.py`:** product of exponentials with the space frame on the drone body.
  - FK, space Jacobian, closed-form IK: 0.07 ms vs 1.17 ms numerical, exact.
  - `payload_mass()`: closed-form mass from joint torques.
- **Checks:**
  - `Code/scripts/am3_kin_check.py`: FK matches the NMPC model to 4e-16 m; IK round trip 4e-15 rad.
  - `Code/scripts/am3_mass_check.py`: mass estimate 99.4 g / 199.6 g for 100 / 200 g loads. With no load and thrust −10 % it reads −0.2 g, where a vertical-force reading gives 170 g.
- **Write-up:** `Findings/Capstone/01_analytical_kinematics_mass.md`.
- **Ashish's own results** (4-joint flight robot, calibrated joint torques, 96 % mass accuracy, tracking 38.7 → 19.8 cm) are **not in this repo**. Source: his `Analytical_Method_Results.pdf`.
- **Slides** (WPI template):
  - Latest: `Presentations/2026-11-18_1430_Capstone_Analytical_Load_Mass.pptx`, 12 slides, Times New Roman, talk script in the speaker notes.
  - Build scripts are next to it.
  - Advisor style: at most two videos per slide, every number explained in words, large simple text.

## 6. Videos (committed)

`Media_results/`. Newer folders start with `YYYY-MM-DD_HHMM`.

| Folder | What it shows |
|---|---|
| `2026-11-18_1639_Write_HELLO_WPI_{motor_slow,payload200g}_ND_R21_R22/` | ND, R21, R22: R22 finishes slow motors (ND, R21 fail); at 200 g only ND finishes, R22 falls at 5 s |
| `2026-11-18_1159_Write_HELLO_WPI_{nominal,payload200g,motor_slow}_ND_R19_R20_R21/` | ND, R19, R20, R21 on the same sentence |
| `2026-11-17_2210_Write_HELLO_WPI_{payload200g,motor_slow}_ND_vs_R19_vs_R20/` | ND, R19, R20 on the two hard cases |
| `Write_HELLO_WPI_payload100g_N_vs_ND_vs_R17_2026-11-17/`, `..._motor_slow_...` | N, ND, R17 under payload and motor lag |

Each folder has `compare.mp4` (all controllers side by side) plus one mp4 per controller.

## 7. Running things

- **Setup:**
  - Python 3.10 with `Code/requirements-pinned.txt`.
  - acados built from source into `Code/third_party/acados`; the build line is in that file.
  - Always: `cd Code; unset MUJOCO_GL; export OMP_NUM_THREADS=1 PYTHONPATH=.:scripts`.
  - Import stable_baselines3 before acados.
- **Trained policies (committed):** `Code/checkpoints/{R15,R19,R20,R21}_*/model_final.zip`, each with the `config.json` it needs. All other checkpoints are gitignored; they live in `Code/results/am3_rl/` on Sahil's PC and on Turing.
- **Examples:**
  ```
  python scripts/am3_write_bench.py checkpoints/R21_write15drjt_fromR19_s0/model_final.zip --noise real --case payload200g --procs 8 --name R21
  python scripts/am3_write_bench.py ND:10,10 --noise real --name ND
  python scripts/am3_write_video.py MY_TAG "HELLO WPI" --height 0.05 --speed 0.05 --noise real --case payload200g --ctrl ND:10,10 checkpoints/R21_write15drjt_fromR19_s0/model_final.zip
  python scripts/am3_kin_check.py && python scripts/am3_mass_check.py
  ```
- **Training:** runs on the WPI Turing cluster (CPU, academic partition): `slurm/am3_rl_turing.sh`, bench `slurm/am3_bench_turing.sh`. It runs under Sahil's account. Nothing running on Turing (R22 finished 2026-11-18 15:39). R22 model: `Code/results/am3_rl/2026-11-18_1200_R22_write15drjtw_fromR21_s0/` (local + Turing, not committed).

## 8. Rules (from `CLAUDE.md`, non-negotiable)

- Simulation is the authority. No manufactured success: never change physics, metrics or thresholds to look better.
- Report failures honestly, with their cause.
- Frozen files (`Code/config/am3_freeze_v2.json`) are not edited.
- Keep code simple.
- Timestamp every note; name new outputs `YYYY-MM-DD_HHMM_...`.

## 8b. RBE595 pipeline status vs the proposal (2026-11-19 12:00 EST) [optimal]

Proposal promised: (1) nominal NMPC, (2) NMPC + disturbance observer, (3) **NMPC + learned model correction** (RL outputs a correction to the NMPC prediction; NMPC computes the commands), tested on payload, contact stiffness, approach speed (peak impact force) and outside the training range.
- (1) N, NF done. (2) ND, NDF done. Pure RL (R15-R22) done as an extra baseline.
- (3) **L, training on Turing since 2026-11-19 ~11:45 EST**: base NDF, PPO, motor-lag curriculum 1.4 -> 2.5x. Both variants (Sahil's choice):
  L2 = the proposal's 2 channels (normal force, pitch torque), job 2539092, TAG `2026-11-19_1142_L2_hybrid_s0`;
  L9 = all 9 observer channels (stated deviation), job 2539093, TAG `2026-11-19_1142_L9_hybrid_s0`. ~4-5 h each.
  Code: `Code/ocrl/am3_hybrid_l.py`, `Code/scripts/am3_train_hybrid.py`, `slurm/am3_hybrid_turing.sh`. Zero action reproduces NDF exactly (checked). Bench name `NDFL:<zip>`.
- Stress tests (new cases in `Code/scripts/am3_write_mismatch.py`): wall_soft/stiff (contact stiffness 0.5x/3x), wall_slippery/grippy (friction 0.5x/2x), approach_fast (3x touch-down speed), motor_lag15, motor_lag3 (outside training), kf85 (outside training), combo (100 g + lag 1.5x + gust). Bench reports `F_peak_N`. Running on Turing for N, NDF, R21, R22 (jobs 2539098-2539101, names `2026-11-19_1150_stress_*`).
- **Update 2026-11-20 19:15 EST.** Stress benches done (logs `Code/results/am3_write/bench/2026-11-19_1145_stress_*`, the names say 1145 not 1150).
  Completed/15, trace RMS mm (fail = 0/15):

  | case | N | NDF | R21 | R22 | L2 |
  |---|---|---|---|---|---|
  | wall_soft | 15, 11.6 | 15, 15.4 | 15, 1.77 | 15, 2.42 | 15, 14.2 |
  | wall_stiff | 15, 2.21 | 10, 2.21 | 15, 1.94 | 15, 2.51 | 14, 3.62 |
  | wall_slippery | 15, 3.91 | 15, 2.55 | 15, 1.69 | 15, 2.30 | 14, 3.74 |
  | wall_grippy | 14, 6.18 | 14, 3.84 | 15, 2.55 | 15, 3.08 | 15, 6.06 |
  | approach_fast | 15, 2.06 | 15, 2.31 | 15, 1.88 | 15, 2.48 | 15, 2.83 |
  | motor_lag15 | 12, 2.29 | fail | 15, 2.02 | 15, 2.64 | fail |
  | motor_lag3 | fail | fail | fail | fail | fail |
  | kf85 | 13, 7.63 | 15, 2.39 | 15, 2.07 | 9, 2.30 | 13, 3.09 |
  | combo | fail | fail | 10, 2.57 | 5, 3.15 | fail |

  Peak impact force: N up to 7.9 N, NDF 5.8 N, RL < 2.5 N except combo (R21 8.5 N, R22 5.3 N).
- **L2 finished** (10M steps; training completion fell 0.76 -> ~0.30 as the lag curriculum passed 1.5x). Bench `2026-11-20_1810_L2_*`: **worse than NDF** on the original 8 cases
  (nominal 12/15 2.80 mm vs NDF 15/15 2.32; payload200g 15, 4.58; motor_slow fail) and on the stress set (above). Several runs hit the 400 s wall-clock guard. Likely cause: most training episodes were at motor lag NDF cannot survive and a model correction cannot fix, so the policy learned a harmful correction. Not yet checked: an earlier checkpoint (1000k, completion 0.71).
- **L9 froze** at 1.5M steps (no log for 22 h, killed at 24 h). Fix: HybridEnv truncates episodes past the 400 s guard (commit 5d18aeb). Resumed from model_1500k: job 2542974 (curriculum restarts at lag 1.4).
- **New run L2W_lag14**: L2 + wall stiffness 0.4-3.5x / friction 0.4-2.5x randomization, motor lag capped at 1.4x (`WALL=1 LAGMAX=1.4`), job 2542977, TAG `2026-11-20_1911_L2W_lag14_hybrid_s0`. Note: the wall_soft/stiff stress cases are inside its training range.
- **Noise finding (2026-11-20 19:30 EST).** The shaking and hard-pressing look in the videos since 2026-11-12 come from the 'real' sensor noise (1 mm, 0.5 deg, white): with the old harness noise
  NDF writes HELLO WPI at 0.81 mm (vs 2.21), coverage 100 %, peak force 0.65 N, rotor-command change 1.1 vs 11.4 (video `Media_results/2026-11-20_1921_*`). Sahil approved benchmarking at three levels:
  nominal (0.1 mm, 0.01 deg), **lab** (spec-based, main result: 0.3 mm, 0.1 deg, pose error correlated tau 50 ms; `LabSensors` in `Code/scripts/am3_write_bench.py`, sources in its comments), real (stress).
  Noise generator checked: std and autocorrelation match, identical noise at 50 and 100 Hz sampling. One-sentence check at lab noise: N 1.22 mm, NDF 1.86, R21 1.89, all complete.
- **Benches running (2026-11-20 19:59)**: N, NDF, R21, R22, L2 x {lab, nominal} x 17 conditions, jobs 2543098-2543117 (9 CPUs each, PROCS=1), names `2026-11-20_1959_[stress_]<ctrl>_<noise>[_case]`.
  Account cs551 is DENIED on partition short (course accounts): only academic, 64 CPUs per user.
- **L2W_lag14 stopped** at 500k (job 2542977 cancelled; it trained at 'real' noise). Restart only if NDF still fails the soft wall at lab noise, and then train at lab noise.
- **Lab-noise results so far (22:15)**: N, NDF, R21 done; R22, L2 running. Soft wall still breaks NMPC at lab noise: N 15/15 13.4 mm, NDF 15/15 17.8 mm (peak 4.5-4.7 N); R21 1.75 mm.
  Nominal case: N 15/15 1.17 mm (best accuracy), NDF 13/15 1.92, R21 15/15 1.86. Rotor-command change: N 5.6, NDF 3.2, R21 1.0. Motor lag: N and NDF fail, R21 14/15 (motor_slow) and 15/15 (lag 1.5x).
- **L2W_lab submitted (22:17)**: job 2543462, TAG `2026-11-20_2217_L2W_lab_hybrid_s0`: L2 + wall randomization + lag cap 1.4 + lab noise (justified: soft wall is a model error, not noise).
  The 10 nominal-noise bench jobs (2543108-2543117) are HELD so L2W starts first; release them (`scontrol release`) once it runs.
- **2026-11-21 16:50 EST.**
  - L2W_lab finished (10M steps, training completion 0.97 at the end). Bench at lab noise: jobs 2567364/2567365, names `2026-11-21_1645_[stress_]L2W_lab*`.
  - **L9 froze again** at 4M (one worker at 100 % CPU inside a single step for 19 h; py-spy not permitted on Turing). Cancelled; it also used the dropped settings ('real' noise, lag to 2.5).
    Fix: step watchdog in HybridEnv (faulthandler, 120 s: dumps the stuck stack to the job .err and kills the worker) + auto-resume loop in `slurm/am3_hybrid_turing.sh`. Tested locally.
    Replacement **L9W_lab** (same settings as L2W_lab, 9 channels): job 2567363, TAG `2026-11-21_1645_L9W_lab_hybrid_s0`.
  - **Bench harness bug found**: the Episode's 400 s wall-clock guard aborted NMPC sentences that were simply slow on Turing (completed ones take 350-440 s). Most NMPC/L2 "12-14/15" counts at lab noise
    (and some at 'real' noise, e.g. NDF wall_stiff 10/15) were this guard, not control failures. Guard raised to 1500 s in `am3_write_bench.py`. Lab re-runs of the affected cases: jobs 2567370-2567373 (`2026-11-21_1646_rerun_*`).
    Low-noise level re-submitted with the fix, all 17 conditions: jobs 2567374-2567377 (`2026-11-21_1646_lownoise_{NDF,N,R21,R22}`); the earlier `2026-11-20_1959_*_nominal*` jobs were cancelled and their partial logs deleted.
  - Genuine failures at lab noise (loss of control): N payload 100/200 g, motor_slow; NDF motor_slow, lag 1.5x, combo; all motor_lag3; R21 payload200g 8/15, combo 7/15. Soft wall: N 13.4 mm, NDF 17.8, L2 17.1, R21 1.75.
  - Videos done: `Media_results/2026-11-21_1646_Write_HELLO_WPI_{nominal,wall_soft}_lab_*` (SUMMARY lines in the READMEs).
- **2026-11-21 18:45 EST. L2W_lab benchmark (lab noise, 15 runs per condition) — L2W FIXES THE SOFT WALL:**
  - wall_soft 15/15, 2.11 mm (NDF 17.8 mm, N 13.4 mm, R21 1.75 mm). Also 15/15: nominal 1.79, gust 1.78, wind 1.84, kf90 1.82, kf85 1.82, damping3x 1.99, approach_fast 1.77,
    slippery 1.91, stiff 2.52, grippy 3.18, payload100g 2.39, payload200g 3.78 (N loses control on both payloads). Peak force <= 1.75 N everywhere.
  - L2W FAILS every motor-dynamics case (0/15, loss of control): motor_slow, motor_lag15, motor_lag3, combo. The 2 channels (extra normal force + 1 more) cannot represent rotor lag.
    This is the question for L9W (9 channels, all generalized forces).
  - Cost: arm joint acceleration ~30 (NDF 15, R21 8): the learned correction makes the arm busier.
  - L9W_lab (job 2567363): watchdog fired once at ~1.2M steps (an acados `solve()` hung > 120 s inside HPIPM, stack in `am3L_2567363.err`); auto-resume restarted from the 1M checkpoint. Works as designed.
  - Lab re-runs with the 1500 s guard so far: N kf85 15/15 3.49 mm, N motor_lag15 15/15 1.95 mm (was a guard abort, not a failure), N grippy 15/15 5.90, N slippery 15/15 3.31; R22 kf85 6/15 (genuine).
- **2026-11-21 18:55 EST. Timing + prediction at contact** (`Code/scripts/am3_write_timing.py`, `Code/results/am3_timing/2026-11-21_1831_write_timing_pred_lab.json`;
  "HELLO WPI", lab noise, seed 0, one process, Intel Ultra 9 275HX single thread; not comparable to Turing or to Aryan's machine):
  - Real time: N / NDF / L2W median cycle 9.6 / 10.9 / 12.0 ms, p99 28-34 ms; **~3.5 % of cycles exceed the 20 ms budget** for all three (L2W's policy adds ~1-2 ms).
    No solver failures, retries or fallbacks here. R21: 1.1 ms median, max 7.4 ms, never over its 10 ms period. (One acados solve hung > 120 s during L9W training on Turing: rare, but a real-time hazard.)
  - Prediction at contact (pen writing phase; model's predicted pad depth and normal force vs the plant, 20/100/200 ms ahead):
    normal wall: N errors <= 0.44 mm, <= 0.24 N (true force ~0.47 N). NDF/L2W similar.
    **soft wall: N's model predicts the pad 6.9 mm shallower and 1.2 N less force than reality at 200 ms (true force ~3.0 N: it presses 6x harder than on the normal wall).**
    NDF's observer fixes the 20 ms force error (0.19 N) but not the horizon (6.9 mm, 1.4 N at 200 ms): a constant disturbance cannot represent a stiffness error that grows with depth.
    **L2W: <= 0.35 mm, <= 0.13 N bias over the whole horizon, true force 0.36 N.** This is the mechanism behind L2W fixing the soft wall.
- Turing project dir `~/sem3/Aerial-Manipulation-Coupled-NMPC` is NOT a git clone: sync changed files with rsync.
- **2026-11-21 19:30 EST. User approved (a) force-controlled writing as an extra variant, (b) RL retraining at lab noise, (c) L2W at all 3 noise levels.**
  - **Force-controlled writing** (`ocrl/am3_write.py`, `--force F` in the bench, `FORCE=` in `slurm/am3_bench_turing.sh`; default off = depth task, unchanged:
    N nominal reproduces 1.216 mm exactly). While tracing, d(gap_ref)/dt = K_F (F_meas - f_ref); f_ref 0.6 N (= model wall 600 N/m x 1 mm depth ref), K_F 3e-3 m/(N s), gap_ref in +-8 mm.
    Local check (HELLO WPI, lab, seed 0): neutral on the normal wall (N 1.26, NDF 1.85, L2W 1.60, R21 2.06 mm); soft wall N 14.5 -> 6.4 mm, NDF 19.1 -> 9.6 mm (still FAIL, peaks 3-4 N);
    L2W 2.5, R21 1.9. Why it cannot fix N: on the soft wall N's real pad depth stays ~8.5 mm whatever the reference (gap_ref raised by up to +7 mm, no clipping): the arm drifts to another posture
    (shoulder 76 % of its torque limit, 9 % saturated; joints 2/3 moved ~28 deg), base pitched 13 deg and 14 mm off. The model error is inside the NMPC; a task-level force loop cannot reach it.
    Full force benches at lab noise (17 conditions): N, NDF, L2W, R21: jobs 2568337-2568344, names `2026-11-21_1927_force06_[stress_]{N,NDF,L2W,R21}_lab*`.
  - **RL at lab noise**: `LabSensors` in `ocrl/am3_rl_direct.py` (same SIG_LAB/TAU_LAB; correlated error advances 5 ms per call, the env measures twice per 10 ms step; checked: 0.30 mm std, 10 ms autocorr 0.82),
    env kinds `write15_drjt_lab` / `write15_drjtw_lab` (identical to R21/R22 otherwise; checked). Fine-tunes, 20M steps, lr 1e-4 decay, as R21/R22:
    R23 = R21 settings from R21 (job 2568255, TAG `2026-11-21_1920_R23_write15drjt_lab_fromR21_s0`), R24 = R22 settings from R22 (job 2568256, TAG `2026-11-21_1920_R24_write15drjtw_lab_fromR22_s0`).
    Fixed on the way: `reset(seed)` rebuilt a plain `Sensors`, it now keeps the sensor class.
  - L2W at nominal and 'real' noise: jobs 2568251-2568254, names `2026-11-21_1920_[stress_]L2W_{nominal,real}*`.
  - Queue order (64-CPU cap): low-noise benches, L2W benches, force benches; R23/R24 (32 CPUs each) likely start when L9W finishes (~07:00 11-22).
  - When done: bench R23/R24 at lab (17 conditions) + videos; L9W bench + videos; force-mode videos (soft wall N vs L2W).
- Still open: final table + videos; report; optional deadline-enforced timing run for the writing task.

## 9. Open items before Monday 2026-11-23 (capstone)

- Merge Ashish's analytic-mass results and video into the deck (his video file is not in this repo).
- Decide how much RL/NMPC writing material the capstone talk needs (slides 10–11 of the latest deck).
- R22 done (see table). 200 g deprioritized (near the 242 g hardware limit). Smoothness study done (NF, NDF). Next candidate: NMPC+RL hybrid aimed at motor lag, NDF as the NMPC base.
- **2026-11-23 18:28 EST. Next step after the capstone: slow motors [optimal].** At lab noise, L9W finishes all 17 conditions at 1.2-1.6 mm except motor_slow (2x lag) and motor_lag3 (0/15). R23 (lab-noise R21) is worse than R21 (kf90 5/15, kf85 0/15, payload200g 0/15). R24 was trained (20M steps) but not yet benchmarked.
  - **L9W_lag22** (job 2587660): fine-tune of L9W final (`--init`, new option in `am3_train_hybrid.py` / `INIT=` in `slurm/am3_hybrid_turing.sh`), same settings (9 channels, wall randomisation, lab noise), motor-lag curriculum 1.4 -> 2.2x, 6M steps.
    Question it tests: can a 9-channel model correction cover a 2x motor lag that NDF cannot survive (L2 with lag up to 2.5x from scratch failed)? motor_lag3 stays outside training as an out-of-range test.
  - R24 lab benches: jobs 2587661 (8 main conditions) and 2587662 (9 stress conditions).
- **2026-11-25 20:49 EST.** L9W_lag22 finished (6M steps; training completion stayed 0.92-0.95 while the lag curriculum went to 2.2x; no collapse as with L2). Lab benches: jobs 2595430/2595431, names `2026-11-25_*_[stress_]L9W22_lab*`. Local motor_slow check running.
  - **R24 lab bench (done, completed/15, mm):** nominal 15, 2.22; payload100g 15, 2.69; payload200g 0; motor_slow 15, 2.63; damping3x 15, 2.33; kf90 13, 2.41; gust 15, 2.25; wind 15, 2.59;
    wall_soft 15, 2.12; stiff 15, 2.29; slippery 15, 1.98; grippy 15, 2.90; approach_fast 15, 2.26; lag15 15, 2.36; lag3 0; kf85 0; combo 7, 3.00.
    Lab-noise retraining did not help: R24 is ~0.4 mm less accurate than R21 and still fails 200 g, kf85, lag3. R21 stays the pure-RL reference.
- **2026-11-25 20:49 EST. L9W_lag22 on motor_slow (2x motor lag), local, lab noise, 15 runs** (`Code/results/am3_write/bench/2026-11-25_2049_local_L9W22_lab_motor_slow.json`):
  **12/15, 1.34 mm** (old L9W 0/15; R21 14/15, 2.49 mm; NDF 0/15). The 3 failures are loss of control (tilt 42-47 deg). Costs: peak normal force up to 7.2 N (old L9W nominal 3.9 N, R21 3.5 N),
  tilt up to 18 deg, rotor-command change 17.6 (R21 1.6, NDF 3.2, old L9W nominal 15.2). It no longer fails, but it flies harder. Full 17-condition Turing bench pending (does it lose anything on the other cases?).
- **2026-11-25 21:10 EST.** Video `Media_results/2026-11-25_2059_Write_HELLO_WPI_motor_slow_lab_L9W_vs_L9W22_vs_R21/` (old L9W fails at 1 s; L9W_lag22 1.43 mm; R21 2.73 mm). Fixed `am3_write_video.py`: two policies of one series got the same file name (the first render showed L9W_lag22 twice; deleted).
- **2026-11-26 10:25 EST. L9W_lag22 full lab bench (Turing, 17 conditions, 15 runs) and final table** (`Code/results/am3_write/2026-11-26_1025_final_table_lab.md`, script `Code/scripts/am3_final_table.py`, data `Code/results/am3_write/bench_turing_1126/`):
  - L9W_lag22: 15 conditions 15/15 at 1.13-1.65 mm, **motor_slow 11/15 at 1.30 mm** (old L9W 0/15; local run 12/15), motor_lag3 0/15 (outside training). Only one condition with 0/15, fewest of all controllers.
  - Cost of the fine-tune: payload200g 1.20 -> 1.91 mm (still 15/15); rotor-command change ~15 % higher (15 -> 17.6). Other cases equal or slightly better (lag 1.5x 1.34 -> 1.13, grippy 1.63 -> 1.40).
  - **Correction to the 2026-11-25 note:** high peak normal forces (4-8 N, at touch-down/stroke starts) are NOT new with L9W_lag22: the old L9W already peaked at 3.9-12.6 N (stiff wall 12.6 N). Peak force is a shared weakness of the 9-channel correction (R21 <= 3.5 N except combo), still to be explained.
  - Summary (conditions 15/15 | conditions 0/15): N 12|5, NDF 13|4, R21 13|1, R22 13|2, R23 11|4, R24 12|3, L2 13|4, L2W 13|4, L9W 15|2, L9W_lag22 15|1.
- **2026-11-26 11:10 EST. Take-off from the ground (user request) [optimal + capstone demo].** `Code/ocrl/am3_takeoff.py` (frozen files untouched; the stand is added to a copy of the model at run time).
  - Scene: 4-post launch stand (top 0.33 m, above the frozen Episode's 0.3 m divergence floor), rotor arms rest on the posts, 0.45 m behind the hover point.
  - Arm fold: q = (0, -45, -120) deg, inside the operational range, same elbow branch as writing. (A straight L at q2 = -90 was tried first: outside the operational range and needed 1.2 N m at the shoulder with 200 g, over the 1 N m servo; the arm sagged and N/NDF lost control.)
  - Phases before the writing task: ground 1.5 s (rotors spin up open-loop to 85 % of each rotor's static weight share: the folded arm puts the CoM ~4 cm forward, equal speeds tipped the drone onto its nose; servos hold the fold with gravity feed-forward + PD 1.5 N m/rad),
    takeoff 3 s (min-jerk climb to 1.2 m), unfold 3 s (base forward to the hover point, pen to the writing start), then hover/position/approach/... as before. The controller is reset and takes over at the start of 'takeoff' (`GroundStart` wrapper; same for every controller).
  - Local checks ("HI", lab noise, seed 0): N 0.99 mm (tracking during climb <= 2 mm, tilt <= 4 deg); NDF 2.07 mm; NDF + 200 g 3.68 mm; L9W_lag22 1.21 mm; N + 200 g FAILS in unfold (N also fails 200 g from hover).
    **R21 FAILS take-off** (drifts, arm hits a stand post / failsafe at 0.5-0.7 s): it was only ever trained from hover.
  - **R25 training (job 2598256)**: R21 settings + half the writing episodes start on the stand (env kind `write15_drjt_to`, `takeoff_p=0.5`), fine-tune from R21, 20M steps, TAG `2026-11-26_1104_R25_write15drjt_to_fromR21_s0`.
    In training the spin-up is done in reset (as `GroundStart`), the policy acts from take-off on; rotor-arm/stand-pad contact is allowed only in ground/takeoff, any other contact ends the episode.
  - Tools: `am3_write_video.py --takeoff` (cameras follow the drone back to the stand).
- **2026-11-26 18:10 EST. R25 FAILED to learn take-off; cause found; R26 submitted.**
  - R25 (20M steps, 6.5 h): ~50 % of episodes ended at ~0.6 s for the whole run (= every take-off episode), no improvement; full-length episodes fell 48 % -> 33 % (hover writing also got worse).
  - Cause (diagnosed in the env): every R25 take-off ended with a finger of the hand touching the prop1 disc. The (-45, -120) fold left 0.5 cm to the propeller discs once the joints move by a few degrees
    (the policy's joint residual); the NMPC holds the fold exactly, so it never touched. The scripted PD reached the writing phases from the stand, so env and task were fine.
  - Fix: fold searched over the operational range for clearance under +-8 deg joint and +-10 deg yaw wiggle: **(0, -35, -110)**, 6.1 cm to the discs, 8.5 cm above the floor, shoulder 0.72 N m with 200 g (only pose meeting all).
    Re-checked ("HI", lab): N 0.99 mm, NDF + 200 g 3.68 mm, L9W_lag22 1.28 mm; R21 still fails (drifts, tilt 30 deg, failsafe at 3.5 s, no contact).
  - **R26** (job 2599953): same as R25 with the new fold, TAG `2026-11-26_1810_R26_write15drjt_to_fromR21_s0`. Check script: `Code/scripts/am3_takeoff_check.py CTRL [TEXT] [CASE]`.
  - The take-off video `Media_results/2026-11-26_1104_Takeoff_*` used the old (-45, -120) fold; re-render with R26.
- **2026-11-26 19:15 EST. R26/R27 stopped (not learning take-off); R28 with mid-take-off starts.**
  - R26 (Turing, 50 % take-off) at 3M: ~50 % of episodes still end < 1.2 s, flat. R27 (laptop, 30 %, seed 7) at 4.6M: climbed ~2.8 s at 2.5M, then regressed (arm link hits a stand pad again).
    Trace: on the stand R21-based policies tilt 6-10 deg and drift sideways while rising slowly, then saturate the acceleration commands; every episode dies near the stand, so the climb itself is never learned.
  - **R28**: env kind `write15_drjt_tomid` (`mid_p=0.5`): half the take-off episodes start part-way along the climb/unfold (random time, base on the reference with its velocity, arm at the IK of the pen reference),
    half on the stand as before. Scripted PD passes from every start; R21 passes some. Runs: Turing job 2600210 (20M, seed 3, TAG `2026-11-26_1912_R28_write15drjt_tomid_fromR21_s3`),
    laptop (8M, 16 envs, seed 7, `Code/results/am3_rl/2026-11-26_1912_R28_write15drjt_tomid_fromR21_s7_local`).
  - Take-off test of a checkpoint (from the stand only): `Code/scripts/am3_takeoff_env_eval.py MODEL [N]`.
- **2026-11-26 19:51 EST. Real cause of every RL take-off failure: an observation bug. Fixed; R29 submitted.**
  - Track9's look-ahead inputs (reference 0.25 s and 0.5 s ahead) were computed from the fixed HOLD reference (the hover point), not the task. While writing the base stays within cm of the hover point, so it never mattered;
    on the stand it told the policy the target was 0.45 m ahead and 0.87 m up -> it accelerated forward, drifted ~7 cm and the arm hit a stand pad (R25-R28: 0/10 from the stand at R28 3M).
  - Fix (`am3_rl_wall.WallEnv._extra_obs` + `TakeoffWriteTask.base_ahead`): during takeoff/unfold the look-ahead follows the take-off path. Writing phases unchanged.
  - **With the fix, plain R21 (no take-off training) takes off and writes** ("HI", lab, Episode harness): 4/4 strokes, 1.89 mm, coverage 0.63; tracking during climb <= 19 mm, unfold <= 39 mm (N: ~2 mm).
  - R28 runs (both trained with the bug) stopped. **R29** = R28 settings with the fix: Turing job above, TAG `2026-11-26_1951_R29_write15drjt_tomid_lookfix_fromR21_s3`, 20M steps.
  - Next: `scripts/am3_takeoff_env_eval.py` on R29 checkpoints (from the stand), check its hover writing vs R21, lab bench, take-off video R21 vs R29 vs NMPC controllers.
- **2026-11-27 10:40 EST. R29 works; capstone 'cheetah tail' study started.**
  - **R29** (take-off-trained R21, look-ahead fix): take-off from the stand 10/10 (`am3_takeoff_env_eval.py`), base error at the approach 2-3 mm (R21 with the fix: also 10/10, 6-9 mm). Training: <=1 % early terminations throughout.
    Turing: R29 lab bench (17 conditions, jobs 2601835/6) and a take-off bench for N, NDF, L9W22, R21, R29 (6 conditions, jobs 2601837-41; `--takeoff` / `TAKEOFF=1` new in the bench). Take-off video with 5 controllers rendering locally.
  - **Tail study (capstone; professor: use the load like a cheetah's tail; arm holds a block, no swing)** [capstone]
    - Model-based (`Code/ocrl/am3_tail.py`, `Code/scripts/am3_tail_run.py`, results `Code/results/tail/`): coupled NMPC with the block in its model (mass added to the hand at solver build; frozen files untouched),
      square route (0.6 m sides) at decreasing move times, lab noise. Modes: rigid (joints frozen by a joint-rate penalty), hold (block still in the world), tail (arm free, weak home spring), unaware (rigid, no block in the model).
    - v1 (150 g, 1 seed): rigid and tail both complete down to 1.0 s per 0.6 m and both fail at 0.8 s (the tail does NOT extend the speed limit); at 1.5 s the tail halves the tracking error (2.2 vs 4.6 mm rms) and the peak tilt (16 vs 25 deg);
      at 1.0 s the tail tilts more (32 vs 24 deg). Tail costs 5-8 % more rotor energy and runs the servos at their torque limit; with a weak home spring the arm wanders 60-95 deg even on slow moves.
      'hold' fails from 2.0 s (keeping the block still in the world rotates the arm against the body's pitch: an anti-tail). 'unaware' is 10x less accurate (13.7 vs 1.2 mm) and fails from 1.5 s: load awareness matters most.
    - v2 running: 3 seeds, tail spring 0.05/0.3/1.0, blocks 150 and 250 g.
    - **2026-11-27 12:30 EST, v2 150 g done** (`Code/results/tail/2026-11-27_1217_tail_sweep_v2b_mb0.15.jsonl`, 3 seeds, lab noise; completed / rms mm / peak tilt deg):
      | T s | rigid | tail 0.05 | tail 0.3 | tail 1.0 |
      |---|---|---|---|---|
      | 1.5 | 3/3, 4.3, 22 | 3/3, 1.5, 15 | 3/3, **1.1, 12.5** | 3/3, 6.6, 25 |
      | 1.2 | 3/3, 2.8, 17 | 3/3, 2.4, 18 | 3/3, **2.0, 17** | 3/3, 5.8, 24 |
      | 1.0 | 3/3, 7.5, 24 | 2/3, 6.5, 29 | 3/3, **5.4, 23** | 3/3, 9.6, 32 |
      | 0.9 | 2/3, 12.5, 32 | 0/3 | **3/3, 7.6, 28** | 1/3, 12.9, 38 |
      - A moderate arm spring (0.3) is best everywhere: the only mode that completes all 12 runs, lower error, tilt and jerk than rigid, same rotor energy (within 1 %).
        Too weak (0.05) wanders (206-234 deg of joint travel), costs 7-10 % energy and saturates the servos; too strong (1.0, approaching 'hold') is worse than rigid. Hold -> rigid -> tail 0.3 <- tail 0.05: an optimum, not 'more freedom is better'.
      - Cost: at T <= 1.0 s tail 0.3 runs the servos at their torque limit (tau_max 1.0 N m) - the margin is the servo.
    - **2026-11-27 12:33 EST, v2 250 g done** (`Code/results/tail/2026-11-27_1226_tail_sweep_v2b_mb0.25.jsonl`):
      | T s | rigid | tail 0.05 | tail 0.3 | tail 1.0 |
      |---|---|---|---|---|
      | 1.5 | 3/3, 3.2, 21 | 3/3, 3.5, 18 | 3/3, **1.1, 13** | 3/3, 4.8, 22 |
      | 1.2 | 3/3, 3.1, 18 | 3/3, 6.1, 20 | 3/3, **2.1, 17** | 3/3, 5.4, 23 |
      | 1.0 | 3/3, 9.2, 28 | 0/3 | 3/3, **5.8, 26** | 2/3, 8.5, 29 |
      | 0.9 | **0/3** | 0/3 | **3/3**, 9.5, 29 | 0/3 |
      - With the heavier block the tail 0.3 arm is the ONLY mode that flies 0.9 s per side: here the tail does extend the speed limit (v1 compared the weak 0.05 spring, which does not).
        Same energy as rigid at each T (1.06-1.16 x hover); servos at the torque limit from 1.2 s on. 5 runs hit the 300 s stall guard, all in modes/speeds whose other seeds lost control.
    - Video (seed 0, 150 g, 0.9 s per side): `Media_results/2026-11-27_1227_Tail_square_T0.9_150g_NMPC_rigid_vs_tail0.3/` (rigid tips past 30 deg on the 3rd side and the failsafe stops it; the tail swings joint 1 by about 35 deg and completes, 0.6 mm at the dwell).
      Renderer `Code/scripts/am3_tail_video.py` (NMPC modes or RL checkpoints; green route, blue base path, orange block path).
    - **2026-11-27 12:50 EST, speed limit and no-block control** (`Code/results/tail/2026-11-27_1233_tail_{limit_mb0.15,limit_mb0.25,noblock}.jsonl`, 3 seeds each).
      Fastest move time (s per 0.6 m side) with 3/3 completed:
      | block | rigid | tail 0.3 |
      |---|---|---|
      | none | 0.9 | 0.8 (0.7 not run) |
      | 150 g | 1.0 (0.9: 2/3) | 0.8 (0.7: 2/3) |
      | 250 g | 1.0 | 0.9 (0.8: 1/3) |
      - The arm-as-tail buys one to two speed steps at every mass, and with 250 g it flies as fast as the rigid arm with NO block. It helps even without a block: the arm itself is 0.47 kg of the 1.52 kg vehicle, already a tail.
      - Energy stays within 1-2 % of rigid at equal T; the price is servo torque (at the 1 N m limit in every fast run) and 90-230 deg of joint travel.
    - Video 250 g, 0.9 s per side, seed 1: `Media_results/2026-11-27_1240_Tail_square_T0.9_250g_NMPC_rigid_vs_tail0.3/` (rigid loses control, tail completes; seed 1 because rigid seeds 0 and 2 stall inside acados after losing control - noted in its README).
      - Bug found: the v2 run (10:15) hit its timeout with no output; one run (tail 0.05, T 0.9, seed 2) stalls inside acados after the drone has lost control (tilt 45 deg and rising, warm start blown up).
        `am3_tail_run.py` now runs each run in its own process with a 300 s wall-clock limit (a stall is recorded as a failure with that reason) and writes rows as they finish (.jsonl).
    - Learned tail (`Code/ocrl/am3_rl_tail.py`, kinds `tail_free` / `tail_rigid`): R9-rung interface, random waypoint routes (1-2 s hops), random block 0-200 g with its mass as an input, inner-loop observer on, acceleration limit 4 m/s^2.
      Training both locally from R9 s0 (10M steps, 8 envs each): `Code/results/am3_rl/2026-11-27_1039_T1_tail_{free,rigid}_fromR9_s5_local`.
- **2026-11-27 13:05 EST. Turing benches back (R29 lab, take-off) [optimal].** Files `Code/results/am3_write/bench_turing_1127/` (R29 lab copied into `bench_turing_1126/` for the table);
  final table regenerated with R29: `Code/results/am3_write/2026-11-27_1300_final_table_lab.{md,json}` (`am3_final_table.py` now lists R29).
  - R29 (R21 fine-tuned with take-off): 11 conditions 15/15, 3 at 0/15. Same or better accuracy than R21 where both finish (payload100g 1.98 vs 2.32 mm, wall_grippy 2.08 vs 2.56),
    but it LOST thrust-deficit robustness: motor_slow 14 -> 0, kf85 15 -> 0, kf90 15 -> 9, combo 8 -> 5. Failures are mostly not crashes but unfinished writing (kf85 coverage 8-48 %); motor_slow ends in loss of control.
    Likely forgetting during the take-off fine-tune (to check: thrust-deficit share in the take-off env). Do not present R29 as a strict upgrade of R21.
  - Take-off from the stand (6 conditions, 15 runs each; completed / RMS mm; hover-start value in brackets where different):
    | | nominal | payload100g | motor_slow | gust_y | wall_soft | payload200g |
    |---|---|---|---|---|---|---|
    | N | 15, 1.11 | 0 | 0 | 15, 1.16 | 15, 13.7 | 0 |
    | NDF | 15, 1.84 | 15, 2.72 | 0 | 15, 1.83 | 15, 17.9 | 15, 4.00 |
    | L9W22 | 15, 1.30 | 15, 1.32 | 8 (11), 1.27 | 15, 1.33 | 15, 1.41 | 15, 1.90 |
    | R21 | 15, 1.86 | 15, 2.33 | **0 (14)** | 15, 1.91 | 15, 1.78 | **0 (7)** |
    | R29 | running | running | 0 | running | running | 0 |
    - For N, NDF and L9W22 the take-off adds nothing measurable (same counts and RMS as starting in hover; they fail where they fail from hover). The learned-only R21 loses its stress cases when it must also take off.
