# CONTEXT.md — start here (people and AI agents)

Last updated: **2026-11-18 16:50 EST** (Sahil + Claude). Branch: **`payload-swing`**; all recent work is there, and `main` is old.

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

## 9. Open items before Monday 2026-11-23 (capstone)

- Merge Ashish's analytic-mass results and video into the deck (his video file is not in this repo).
- Decide how much RL/NMPC writing material the capstone talk needs (slides 10–11 of the latest deck).
- R22 done (see table). Next RL step: make 200 g work within the shoulder torque limit.
