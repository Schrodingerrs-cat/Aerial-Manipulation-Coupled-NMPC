# CONTEXT.md — start here (people and AI agents)

Last updated: **2026-11-20 19:15 EST** (Sahil + Claude). Branch: **`payload-swing`**; all recent work is there, and `main` is old.

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
- Turing project dir `~/sem3/Aerial-Manipulation-Coupled-NMPC` is NOT a git clone: sync changed files with rsync.
- Still open: NF/NDF/L timing vs the 20 ms real-time budget; final table + videos; report.

## 9. Open items before Monday 2026-11-23 (capstone)

- Merge Ashish's analytic-mass results and video into the deck (his video file is not in this repo).
- Decide how much RL/NMPC writing material the capstone talk needs (slides 10–11 of the latest deck).
- R22 done (see table). 200 g deprioritized (near the 242 g hardware limit). Smoothness study done (NF, NDF). Next candidate: NMPC+RL hybrid aimed at motor lag, NDF as the NMPC base.
