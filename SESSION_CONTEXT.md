# SESSION_CONTEXT.md — living handoff note

**Purpose.** Working context for this repository so a session can be resumed after an interruption, or handed to another person or model without re-deriving everything. Keep appending to the session log at the bottom; correct the state sections in place when they go stale. Last updated: **2026-10-27**.

**Read this first, then `Plan.md`.** This file says where things stand; `Plan.md` says what the project is required to do.

---

## CURRENT STATE (updated 2026-11-10 20:15 EST) - read this first on resume
- Standing rules from user (2026-11-01): timestamp (date+time) every entry; comparison videos = one combined + separate per-controller; keep this file updated every 2-3 h so a resumed session reads it instead of re-caching.
- RL track: see Findings/RL/00_RL_story.md (plain-language history) and Findings/RL/03_endtoend_rl_design.md (evidence). R4 tracks big 3D trajectories at ~2-3 mm but loses 1-4 of 20 held-out starts; no checkpoint passes the 100%-survival rule. No training running. Next: hard-start oversampling / action-rate penalty (awaiting user go-ahead).
- 2026-11-03 18:40 EST: Turing login works from this PC (`ssh turing.wpi.edu`, key installed). ~/sem3 on Turing is empty; nothing submitted yet.
- 2026-11-03 18:52 EST: **R5 (trajectory + moving arm) training RUNNING ON TURING**: jobs 2452401 (R5_turing_s0) and 2452402 (R5_turing_s1), partition academic, 15M steps, `--env track5 --lr-decay`, 30 envs, 24 h limit (resubmit with RESUME=1 if timed out). Repo/env on Turing: `~/sem3/Aerial-Manipulation-Coupled-NMPC`, conda env `am3rl`. Logs: `am3rl_<jobid>.out`, `Code/results/am3_rl/R5_turing_s*/train.log`. Check: `ssh turing.wpi.edu` then `module load slurm; squeue -u sgajera`. Account CPU cap is 64 (2 jobs x 32).
- Track5Env added in Code/ocrl/am3_rl_direct.py (joint reference sinusoids 10-35% of half-range, policy gives +-0.1 rad residual, joint tracking reward). Baseline PD+ff with moving arm: 15-16/20 survive, RMS ~300 mm (vs 7-20 mm arm still); at 50-100% arm amplitude it crashes within ~3 s, so amplitude was limited to 10-35% (my design choice, disclosed). Results: Code/results/am3_rl/eval/r5_expert_only.json.
- Slurm fixes found 2026-11-03: unset MUJOCO_GL (osmesa breaks import); source /etc/profile.d/modules.sh in batch shells. Tarball am3_turing.tar.gz rebuilt 18:47 EST.
- NEXT when resuming: read Turing train logs; copy checkpoints back (scp); evaluate with `am3_eval_track.py --env track5 --n 20` (selection: 100% survival then lowest RMS, joint_rms_deg also reported); then videos (combined + separate); log in Findings/RL/03 with timestamps.
- 2026-11-04 13:40 EST: R5 jobs FINISHED on Turing (both 15M steps). Checkpoints copied to Code/results/am3_rl/R5_turing_s{0,1}. n=20 eval: both seeds 20/20 survival on 4 trajectories, RMS 4.4-5.7 mm (s0) / 7-8 mm (s1) vs PD+ff ~300 mm. Fresh-seed eval (seeds 1000+, n=50) written to results/am3_rl/eval/r5_eval_s*_final_fresh1000_n50.json. Videos (combined + separate) in Media_results/RL_R5_*_2026-11-04. am3_rl_video.py now writes separate videos. Nothing running on Turing. Next: fresh-seed result, push test (needs --env track5 support in am3_eval_push.py), larger arm amplitude (curriculum), then wall line.
- 2026-11-04 14:00 EST: R5 fresh seeds (1000+, n=50): s0 50/50 survive, mean RMS 13-17 mm, median 3.3; s1 49/50; 5 mm goal not met on mean. Partner review: Findings/Partner_repo_review.md. Speaker notes for the weekly deck: Findings/Presentation_speaker_notes.md. Live viewer supports track5.
- 2026-11-05 21:07 EST: MIDTERM REPORT (NMPC only, due; draft final.tex = docs/main.tex in Aryan's repo) being revised section by section with Sahil. Done: abstract, intro, literature (rewritten in chat), Coupled Dynamic Model (figures only), Nominal NMPC (verdict: leave). Next: Experimental Setup, Results, Limitations, Conclusion. Figures: docs/images/{system_overview,arm_reaction,wall_forces}.png from Code/scripts/am3_report_figs.py.
- 2026-11-05 21:16 EST: report: Experimental Setup reviewed: keep text, 2 small edits (verification precision is relative < 1e-8, not 1e-10; add held-out sentence) + wall_paths.png. B v5 (GATE_v3_B) ran after freeze v2, no freeze v3. Next: Results, Timing (state enforced-deadline failure), Limitations, Conclusion.
- 2026-11-05 21:21 EST: report: Results reviewed: gate, wall table and 24-row circle grid all match origin. Fixes given: N pre-motion 0.22 mm not 0.16; Timing rewritten (enforced deadlines lose control); N failed case cause by inspection only; table note on timed reference. Next: Limitations, Conclusion.
- 2026-11-05 21:32 EST: report: Sahil chose NOT to apply the Results/Timing fixes (kept draft wording). Limitations and Conclusion reviewed: leave. Section-by-section review finished; compile on Overleaf still to do.
- 2026-11-08 17:48 EST: RL: R5 tail root-caused to the inner loop using a fixed nominal COM. Fix = Track6Env (posture-aware InnerLoop). PD baseline with fix 20/20, 13.5 mm (the old ~300 mm was an artifact). R6 training on Turing: jobs 2485439 (R6_turing_s0), 2485440 (s1), 15M steps, arm 10-50%. Next: scp checkpoints back, eval `--env track6 --n 50 --start 1000`, push test with --env track6, videos, then RL vs NMPC.
- 2026-11-08 17:55 EST: fixed-loop baseline PD+ff 20/20, 18-24 mm (arm 10-50%). R5 push: RL survives all 18, PD column invalid (PD crashed before the push). R6 jobs running on Turing since about 17:48 EST.
- 2026-11-08 20:07 EST: R6 FINISHED (19:48 EST, both seeds 15M) AND EVALUATED; checkpoints local in Code/results/am3_rl/R6_turing_s{0,1}, nothing running on Turing or locally. n=50, seeds 1000+, 4 paths, arm 10-50%: PD+ff 50/50, 18-24 mm; s0 49/50, 3.4-3.6 mm (seed 1008 crashes after ~65 steps on every path, not investigated); s1 50/50, 5.2-6.4 mm, median <3, worst 77-127 mm. s1 is selected by the rule but misses 5 mm on the mean. Push (one start): RL survived all 18 on both seeds, PD lost 4 N +y. Full table in Findings/RL/03 (R6 results). NEXT: render R6 videos with am3_rl_video.py (combined + separate; required before calling this rung done, add Media_results README entry), diagnose seed 1008, then RL vs NMPC (needs Aryan's commits; ask user before pulling), then wall line.
- 2026-11-09 08:30 EST: R6 videos rendered: Media_results/RL_R6_{big_fig8,lissa3d,circle_xy}_2026-11-09 (s1, seed 1000, combined + separate, RL 2.7-2.9 mm vs PD 17 mm). Seed 1008 diagnosis in Findings/RL/03: hard for both seeds (s0 crashes 0.65 s, s1 near-crash 918 mm off). NEW: first-2 s crash test on 300 unseen starts: s0 5/300, s1 3/300, PD 0/300, so s1's 50/50 overstates reliability; R6 rung NOT done by the 100% rule. Cause not proven (actions pin, body rate 6-8 rad/s). NEXT: R7 aimed at early crashes (not designed yet), and RL vs NMPC (needs Aryan's commits; ask user before pulling).
- 2026-11-09 08:37 EST: GIT: Aryan's 2 commits (f15ae25) merged into branch rl-endtoend-r6 and pushed to origin via SSH (HTTPS remote has no credentials here: use git@github.com:Schrodingerrs-cat/Aerial-Manipulation-Coupled-NMPC.git). main untouched. Backup: ~/arm_optimal/am3_pre_merge_backup_2026-11-09.tar.gz. Conflicts: took Aryan's am3_ctrl_b.py (B v5) and am3_compare.py, kept both README sides. Consequences: moving-base B no longer exists (am3_traj_run refuses B; old one in commit bd93354); our am3_compare N/D/L options are gone (same commit). Our moving-base edits make am3_ctrl_n.py, am3_nmpc.py, am3_episode.py and am3_rl.py differ from Aryan's freeze v2 hashes (needs a freeze decision before gate claims). Checks after merge: model XML changes are visual only; R6/PD 12 eval runs identical to 6 decimals; N line 0.34 mm, B line 17.7 mm. Checkpoints are gitignored (local + Turing).
- 2026-11-09 08:42 EST: R7 SUBMITTED ON TURING: jobs 2489574 (R7_turing_s0), 2489575 (R7_turing_s1), 15M steps, --env track7 --lr-decay (R6 + yaw reference fixed at 0, one change). Why: overriding yaw alone on R6 took 300-start early crashes from 5/3 to 0/0. Pass criteria written before results in Findings/RL/03 (R7 design). NEXT: scp R7 back, 50-start eval on 4 paths with --env track7 --start 1000, full-length 300-start survival on 2000-2299 and on fresh 3000-3299, push test, videos (combined + separate). If still crashing: R8 = add acceleration limit. Then RL vs NMPC.
- 2026-11-09 11:28 EST: R7 evaluated: does NOT pass (s0 298/300, 50/50 on seeds 1000+, mean 3.5-10 mm; s1 300/300 but loses seed 1017 on all paths, mean 4.4-4.8 mm). PD 300/300, 17.7 mm. Table in Findings/RL/03. R8 SUBMITTED ON TURING: jobs 2491344 (R8_turing_s0), 2491345 (R8_turing_s1), 15M, --env track8 --lr-decay (R7 + acc limit +-1.75 m/s2), about 2.2 h. NEXT on resume: scp R8 back (sem3/Aerial-Manipulation-Coupled-NMPC/Code/results/am3_rl/R8_turing_s{0,1}); from Code/ run am3_eval_track.py with --env track8 --n 50 --start 1000 for the 4 paths, and big_fig8 in 6 blocks of 50 from --start 2000 (out r8_surv2000_k.json); push test --env track8 --start 1000 (risk: 4 N pushes). PD baseline must be read from these runs (limit changes it). If both pass: fresh seeds 3000-3299, then videos, then RL vs NMPC. All on branch rl-endtoend-r6.
- 2026-11-09 14:40 EST: R8 DONE and evaluated. Survival PASS on both seeds (50 x 4 paths, 300 on 2000+, 300 fresh on 3000+, no lost start). Selected R8_turing_s1 (Code/results/am3_rl/R8_turing_s1/model_final.zip). Accuracy partly met: big_fig8 4.8 mm at n=300 (both blocks), 6.7-8.6 mm on 50-start tests; PD 17-24 mm. Push: RL 18/18, PD 15/18 under same limit. Videos Media_results/RL_R8_*_2026-11-09 (combined + separate). Results in Findings/RL/03. NEXT: user decides between one more accuracy round or RL vs NMPC on the same task, then wall-line rung.
- 2026-11-09 15:40 EST: R9 RUNNING ON TURING: jobs 2492993, 2492994 (R9_turing_s{0,1}, --env track9, acc limit 2.5, submitted 15:09, about 2.2 h); criteria in Findings/RL/03, final unseen block 4000-4299. RL vs NMPC first numbers: Code/results/am3_rl/eval/vsnmpc_summary.txt, script Code/scripts/am3_rl_vs_nmpc.py (N in frame shifted 0.5 m so its model wall is clear). fwd_up has 40/50 starts, block 1030-1039 unfinished; investigate slow runs. Check N's joint error (N tracks pad, up to about 10 deg). NEXT: scp R9 back and run the R8 test set with --env track9; finish NMPC comparison; then wall line.
- 2026-11-09 17:45 EST: NMPC vs RL videos done: Media_results/NMPC_vs_RL_*_2026-11-09 (combined + separate; seeds 1000, 1006). No-offset test: N 50/50 on 2 paths (median 0.13 mm), so N's 22-28% losses on RL starts come from random start errors; mechanism not established. R9 still training at 17:45 (watcher). Next: R9 eval (--env track9, test set in Findings/RL/03).
- 2026-11-09 19:08 EST: R9 evaluated: R9_turing_s0 SELECTED (Code/results/am3_rl/R9_turing_s0/model_final.zip): survives all tests and under 5 mm on all paths incl. unseen seeds 4000-4299 and 4300-4349. s1 lost seed 1006 earlier, so training-seed dependent. Compare with NMPC: eval/vsnmpc9_summary.txt; videos Media_results/NMPC_vs_RL9_*_2026-11-09. fwd_up start 1034 hangs N reproducibly (not investigated). NEXT: investigate why N loses random starts and the hang; more RL seeds to test training reliability; then wall-line rung.
- 2026-11-09 21:10 EST: WALL RUNG (R10) started, user asked for Aryan's predefined start (equilibrium, pad 5-8 cm off wall). Env Code/ocrl/am3_rl_wall.py, scorer Code/scripts/am3_eval_wall.py (24 held-out line scenarios, Episode harness, acceptance_v2). Baselines: N 24/24 pass (0.41 mm), B 0/24 (19 mm). R10_wall_s0 from scratch died at 4M (NFS write error, now guarded) and never learned base hold. R10b = warm start from R9 s0 with widened input (zero weights for 20 new obs): Turing job 2493494, 10M, seed 0. R9_turing_s2 job 2493382 also running; R9_turing_s3 still to requeue. Committed 6e29386 on rl-endtoend-r6.
  NEXT: scp R10b_wall_s0 back, run diag_r10/why_end.py on checkpoints, then am3_eval_wall.py results/am3_rl/R10b_wall_s0/model_final.zip --block b --blocks 6 --out wall_R10b_b.json; wall videos (combined + separate) need an Episode recorder for RLCtrl.
- 2026-11-10 10:40 EST: WALL: R10b_wall_s0 (10M, warm from R9) completes 24/24 held-out scenarios, contact 1.0, base about 5 mm; passes all criteria on 7/24, the rest fail only tracking RMS (line 2.9-3.0, line_fast 3.0-3.1, line_T1 5.5 mm; limit 3). Cause found: joint D-term on qd (no velocity feedforward) gives kd/kp = 0.05 s lag. Option vel_ff (env wall_ff): R10b + ff with no retraining = 16/24 (line, line_fast 2.5-2.7 mm; line_T1 4.5 mm fails). R10c_wallff_s0 (warm from R10b, wall_ff, 10M) Turing job 2496214; R10b_wall_s1 job 2496041. R9_turing_s2 finished (not yet evaluated); R9_turing_s3 cancelled for cores. NEXT wall: scp R10c, eval with am3_eval_wall.py (reads wall_ff from config), videos with am3_rl_wall_video.py --ff.
- 2026-11-10 10:40 EST: NEW TASK (professor): payload swing exploitation. Branch payload-swing. Findings/Payload/01_swing_exploit_AB.md. Quad (no arm, 1.05 kg) + 0.2 kg load on 0.6 m rigid string; 2 m A->B: exploit plan + load-aware TVLQR settles 2.79 s vs best normal drone 4.35 s, swing-suppressed 4.02 s. Video Media_results/Payload_AB_2m_UvsSvsE_2026-11-10. NEXT payload: NMPC closed loop, sweep distance/load, obstacles (zigzag).
- 2026-11-10 16:50 EST: WALL evals (24 held-out, results/am3_rl/eval/wall_*): R10c_wallff_s0 16/24 (line 2.50, line_fast 2.31 mm pass; line_T1 3.83 mm fails 3 mm limit); R10b_wall_s1 0/24, never acquires contact (local optimum, reward 11.5 vs 15.6) -> warm-start recipe works on 1 of 2 seeds. SUBMITTED: 2502683 R10d_wallff_cont_s0 (R10c + 10M, wall_ff, lr-decay), 2502684 R10c_wallff_s2 (wall_ff from R9 s0, seed 2). NEXT: scp, eval (--block b --blocks 6), videos am3_rl_wall_video.py --ff.
- 2026-11-10 16:50 EST: PAYLOAD v2: the 1.8->2.8 s gap was the model (single 0.13 s lag, 3.9 deg tilt rms) not the controller (NMPC with that model = LQR, 2.83 s). New 17-state model with the attitude loop as designed (wn 15, zeta 0.9, motor lag 0.05, T lag 0.06; 0.4 deg rms). E+LQR now settles 1.62 s, E+NMPC 1.66 s, U best 4.35, S 4.05; thrust 23 vs 53 N s. Video Media_results/Payload_AB_2m_UvsSvsE_attmodel_2026-11-10. v1 result kept as results/payload/ab_2m_v1_singlelag.json. NEXT: distance/load sweep, then obstacles.
- 2026-11-10 17:36 EST: WALL SHAPES (user: longer shapes, M/OM, isometric view). Extra path M (one stroke, 8x6 cm, registered in am3_rl_wall.py, outside frozen task file). am3_eval_wall.py --shapes = held-out circle, circle_T4, triangle, triangle_T5 + M (38). Results: N 38/38 (0.4 mm), B 0/38 (13-19 mm), R10c (lines only) 8/38: completes all, M passes 8/8 (2.7 mm), others 3.0-3.3 mm vs 3 mm limit. R10d cancelled; R11_shapes_s0 (env wall_shapes = lines+circles+triangles+M, warm from R10c, 20M) Turing job 2502737; R10c_wallff_s2 job 2502684 running. Videos: Media_results/Wall_NvsBvsRL10c_{line,linefast,lineT1}_* (old camera), Wall_iso_NvsBvsRL10c_{circle,M}_* (isometric). Bigger letters/OM need the moving base (fixed-base patch ~8x6 cm) - ask user. NEXT: scp R11 and s2, eval lines + --shapes, iso videos.
- 2026-11-10 17:36 EST: PAYLOAD sweep (1/2/3 m, 0.1-0.4 kg): E fastest everywhere (1.41-1.76 s vs U best 3.77-5.91). OBSTACLE zigzag (2 posts, 3 m): E 2.93 s no hit; U best collision-free 5.69 s; S 6.00 s; U <= 4 s hits. Planner now has string-tension floor and 60 deg swing bound (needed: 111 deg / 87 deg plans). Video Media_results/Payload_zigzag_posts_UvsSvsE_2026-11-10. NEXT: obstacle-aware NMPC tracker, more layouts.
- 2026-11-10 18:51 EST: WRITING (user goal: one general contact-writing capability - sentences, words ~20 cm - by NMPC, RL, and NMPC+RL; moving base APPROVED by Sahil, outside frozen plan). Findings/Write/01_wall_writing.md. Code: ocrl/am3_write.py (font A-Z, WriteTask, WriteEpisode), scripts/am3_write_{eval,video,mismatch}.py. N writes OM 10 cm: 0.36 mm, coverage 1.0, base err 0.7 mm; RL R10c zero-shot 3.44 mm, cov 0.87. Mismatch (OM): N fails payload 100 g (no contact), 200 g and motor lag x2 (crash), degrades kf 0.9 (1.4 mm) and wind (1.0 mm); R10c fails the same three. Turing: R12_write_s0 job 2503426 (env write, 30M, warm R10c), R13_write_dr_s0 job 2503479 (env write_dr = + domain randomization), R11 cancelled (superseded). R10c_wallff_s2: 0/24 (completes, 3.1-3.2 mm). NEXT: hybrid = RL residual on NMPC commands (needs acados on Turing: no cmake there; or train locally), score R12/R13 on held-out texts + mismatch suite, sentence demo video.
- 2026-11-10 20:15 EST: WRITING now coordinated (user correction: drone and arm must move together - base follows smoothed look-ahead pen path within arm reach; pen speed 25-75 mm/s). Video Media_results/Write_OM_10cm_coordinated_NMPC_vs_RL10c_2026-11-10 (N 0.54 mm, R10c 3.87 mm at 50 mm/s). RUNNING: Turing R12_write_s0 (2503635) and R13_write_dr_s0 (2503636), 30M each, ETA ~1-4 am; LOCAL H1_hybrid_s0 (NMPC + RL residual, ocrl/am3_hybrid.py, env hybrid, 10M, 20 envs, log results/am3_rl/H1_hybrid_s0/train.log). acados now built on Turing (am3rl env). NEXT: score R12/R13/H1 on held-out texts (am3_write_eval.py) + mismatch suite (am3_write_mismatch.py, HybridCtrl needs a make() hook), sentence video N vs RL vs H.
- 2026-11-05 21:07 EST: THIS LOCAL REPO IS 2 COMMITS BEHIND origin (git@github.com:Schrodingerrs-cat/Aerial-Manipulation-Coupled-NMPC, main f15ae25, 2026-11-04): B v5 strict separation, freeze v2, gates v2/v3. Local Findings/ show the OLD v1 gate (N 30/30, B 1/30); the report correctly uses the new one (N 29/30, B 0/30). Judge the report against origin. Not pulled, because local has uncommitted RL work.

## 1. What this project is

RBE 595-F07 (Optimal Control & RL) course project, which doubles as Sahil Gajera's capstone. Co-author: Aryan Shah. Progress report **Nov 5 2026**, final report **Dec 14 2026**.

A 1.52 kg quadrotor with a 3-joint arm (yaw, shoulder, elbow; wrist fixed) in MuJoCo. Research question, from `proposal.pdf`: **which kinds of model error actually need RL, and which are already handled by classical methods?** Three controllers are compared on the same plant:

| | |
|---|---|
| **N** | coupled NMPC — full 9×9 mass matrix, rotor lag, contact in the prediction (CasADi → acados SQP-RTI, 50 Hz) |
| **B** | tuned decoupled baseline — geometric hover + separate joint-space arm, no base–arm coupling (200 Hz) |
| **Nd** | N with a deliberately decoupled prediction model (ablation) |
| **D / L** | N plus a classical EKF / learned SAC correction of two prediction channels `d = (F̂ₙ, τ̂_pitch)` |

### The scope change that matters

**`proposal.pdf` is out of date and the report must say so.** It describes NDT spot inspection with a probe, perpendicular pointing, a force setpoint and a 3 s dwell. The authoritative `Plan.md` (25 Oct 2026) replaced all of that with: the drone holds a **fixed hover-position reference** while the arm's own end effector (a 12 mm pad, no probe, no rod) traces trajectories on a wall in sustained contact. **No pointing objective, no force setpoint, no dwell.** The two correction channels survive.

### Document hierarchy

`Plan.md` (authoritative spec) → `CLAUDE.md` (working rules + current state) → `TODO.md` (checklist) → `Findings/STATUS.md` (gate table) → `Findings/Codebase/*.md` (design, controllers, gate) → `Media_results/README.md` (experiment log, newest entries last).

---

## 2. Non-negotiable working rules (from `CLAUDE.md`)

- A controller is done only when the **actual simulation visibly demonstrates it**, with a reproducible run and a real recording. Good metrics alone are never success.
- Never tune physics, initial conditions, thresholds or the baseline to manufacture a result. **Do not weaken B to flatter N.**
- A failed experiment is not proof of impossibility — diagnose and retry; separate confirmed limitations from unresolved failures.
- No silent deviation from `Plan.md`; substantive changes need approval first.
- **Frozen files** (listed in `Code/config/am3_freeze_v1.json`) are not edited without a new freeze version *and* equivalence evidence.
- Keep code minimal and close to the maths. `Code/ocrl/*.py` are deliberately dense 18–110 line modules; match that style, do not refactor into layers.
- Report the command run, the observed result, and the remaining uncertainty. Never claim an install, run or result that did not happen.

---

## 3. Environment — how to run anything

The clone is **not runnable out of the box**: `.gitignore` excludes `Code/third_party/`, `Code/results/am3_rl/`, `*.npz`, `*.zip`.

Set up once (already done on this machine, 2026-10-27):

```bash
# 1. venv matching Code/requirements-pinned.txt (== the freeze manifest)
python3 -m venv /home/pinaka/arm_optimal/venv_am3
/home/pinaka/arm_optimal/venv_am3/bin/pip install \
  casadi==3.8.1 mujoco==3.10.0 numpy==2.2.6 matplotlib==3.10.9 scipy==1.15.3 pandas==2.3.3
# torch / stable-baselines3 / gymnasium are NOT installed: only needed for the SAC policy L

# 2. acados at the pinned commit, into the repo
git clone https://github.com/acados/acados.git Code/third_party/acados
cd Code/third_party/acados && git checkout 8ff7c2a17e235d6cad6671538e923ccb7428dc3e
git submodule update --recursive --init
mkdir -p build && cd build && cmake -DACADOS_WITH_QPOASES=ON -DCMAKE_BUILD_TYPE=Release .. && make install -j8
/home/pinaka/arm_optimal/venv_am3/bin/pip install -e ../interfaces/acados_template

# 3. t_renderer — NOT produced by the acados build; code generation fails without it
curl -sL -o Code/third_party/acados/bin/t_renderer \
  https://github.com/acados/tera_renderer/releases/download/v0.2.1/t_renderer-v0.2.1-linux-amd64
chmod +x Code/third_party/acados/bin/t_renderer
```

**Every run needs these, from `Code/`:**

```bash
cd Code
export ACADOS_SOURCE_DIR=$PWD/third_party/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib
P=/home/pinaka/arm_optimal/venv_am3/bin/python
```

### Traps
- A **different** acados exists at `/home/pinaka/acados` (commit `9420fc8bd`) and the shell's default `ACADOS_SOURCE_DIR`/`LD_LIBRARY_PATH` point at it. Always override, or the wrong library loads.
- Rendering: leave `MUJOCO_GL` **unset** (default GLFW works with `DISPLAY=:1`). `egl` throws an EGLError; `osmesa` works.
- MuJoCo free-camera **azimuth 0 looks from −x toward the wall; 180 is behind the wall.** Static demo cameras must be widened or a moving drone leaves frame.
- Changing the NMPC parameter vector changes the codegen hash → full acados rebuild (~1–2 min). Normal.

### Still missing
- `Code/results/am3_rl/` — the paused SAC run (3 seeds, ~100k of 200k steps) never transferred. **Ask Aryan for the checkpoints or retrain.** This blocks controller L.
- `Findings/Planning/Plan_before_direct_EE_hover_2026-10-25.md` is linked from `Plan.md` but absent (removed in the 26 Oct cleanup).

---

## 4. Verification status

Aryan's inherited results were **independently reproduced** on this machine on 2026-10-27 (fresh venv, freshly built acados):

| check | result |
|---|---|
| Predictor vs MuJoCo (`scripts/am3_check_pred.py`) | FK 3.3e-16, Jacobians 2.8e-16, 9×9 M 1.0e-9, bias 1.1e-8, accelerations 5.4e-6 — matches the claimed `1e-9..5e-6`. Base–joint coupling block non-zero (0.033). |
| **Baseline B on `line`** | **bit-exact** — base RMS 4.892 mm, trace RMS 3.182 mm, peak Fₙ 1.1758 N, 382 frames, every digit identical |
| **NMPC N on `line`** | within a few percent — base RMS 0.178 → 0.174 mm, trace RMS 0.461 → 0.348 mm, peak Fₙ 0.6501 → 0.6486 N, 0 solver failures |

N is not bit-exact because acados generated code is compiled `-O3 -march=native`, so the QP differs in the last bits and feedback amplifies it. B is pure NumPy + MuJoCo, hence exact.

**Hardware finding:** this machine is much faster than the one the timing claims came from — N median cycle **9.3 ms vs 16.1 ms**, over-20-ms fraction **1.3 % vs 3.2 %**. N is still not real-time compliant (p99 24.6 ms, 115 ms first call), but the gap is far smaller than `Findings/Codebase/nominal_gate_v1.md` assumes, and the remedies listed there but never tried (bounded QP iterations, fewer substeps/stages, OpenMP build — the build prints `acados was compiled without OpenMP`) could plausibly close it. **Never compare timing numbers across machines without saying which machine.**

Evidence: `Code/results/am3_runs/verify1027_N_line/`, `Media_results/AM3_BvsN_wall_line_REPRO_2026-10-27/`. `Media_results/MY_TAG/` and `Code/results/am3_runs/MY_TAG_*` are Sahil's own independent re-run of the same command — a second reproduction, keep them.

---

## 5. Changes made to the repository (freeze v2 territory)

Four frozen files were edited, **each additively, each defaulting to the previous behaviour, each with a bit-exact parity re-run**. A new freeze version should be recorded before the next gate.

| file | change | why |
|---|---|---|
| `ocrl/am3_nmpc.py` | parameter vector `NP` 21 → 24, new `vB(3)` base-velocity reference; residual `S["vB"]*(x[7:10] - p[21:24])/SIG["vB"]` | base velocity was driven to **zero**, which fights a moving reference (lag ≈ 83 ms × speed ≈ 12 mm at 0.14 m/s) |
| `ocrl/am3_ctrl_n.py` | `p[I_P["pB"]] = r.get("pB", BASE)`, `p[I_P["vB"]] = r.get("vB", 0.0)` | feed the moving reference through |
| `ocrl/am3_ctrl_b.py` | `pB`/`vB` from `ref.get(...)`; position, velocity and IK targets relative to `pB` | so a B-vs-N comparison under base motion is meaningful rather than B just sitting at the old point |
| `ocrl/am3_episode.py` | failsafe uses `pref = task.ref().get("pB", BASE)` instead of the constant `BASE` | **real bug**: the loss-of-control monitor measured error against the *fixed* hover point, so a drone legitimately flying a large figure tripped the 250 mm abort while perfectly on reference (seen: both controllers "losing control" at 3.7 s with tilt 0–1° and N at 0.139 mm). Would also have broken the planned long-line wall test. |

**Parity evidence — repeat this check after any further edit to these files:**

```bash
$P scripts/am3_run.py parity_check_N N line && $P scripts/am3_run.py parity_check_B B line
# compare log.npz arrays against results/am3_runs/parity_v3_{N,B}_line → expect max abs diff 0.0
```
Stored: `Code/results/am3_runs/parity_v{2,3}_{N,B}_line`. Both rounds returned max abs diff **0.0** on the logged state, command, force and pad arrays, with every summary metric identical.

New files, **not** frozen: `Code/ocrl/am3_traj.py`, `Code/scripts/am3_traj_run.py`.

---

## 6. Demo capability added (outside the frozen plan)

`Plan.md` deliberately pins the base reference. `am3_traj.py` adds a **moving-base demonstration** track for the presentation. These are **capability tests, not gate evidence** — no frozen criterion covers them. Always label them that way.

`LoopTask` drives base and arm from one shared clock: base on a Lissajous `y = A sin(n_y θ)`, `z = C sin(n_z θ + φ)`, arm circling the moving base. One smoothstep envelope ramps the rate 0 → 1 → 0 so motion starts and ends at rest while completing exactly the requested laps; the same envelope scales the arm-circle radius, so the pad spirals out of and back into the base-relative origin without a velocity step.

```bash
# figure-eight + arm circles, B vs N
$P scripts/am3_traj_run.py TAG --mode loop --duration 30 --base-laps 3 --arm-laps 10 --ctrls B,N
# hover, arm only            → --base-laps 0
# arm holds a fixed offset   → --arm-laps 0 --r-arm 0.08
# arm hanging, uncommanded   → --arm-laps 0 --r-arm 0.0
# open-air Lissajous         → --shape lissajous --no-wall --arm-down --scale 1.5
```
Flags: `--shape {lemniscate,lissajous,sine}`, `--base-laps`, `--arm-laps`, `--r-arm`, `--scale`, `--duration`, `--no-wall` (wall `contype`/`conaffinity` = 0 and alpha 0 *after* the plant is built, so contact calibration is unaffected), `--arm-down` (start pose `q = (0, −10, −35)°`, pad 400 mm below the base, inside operational joint ranges), `--ctrls`, `--no-video`.

### Watching a run live

`Code/scripts/am3_live.py` opens an interactive MuJoCo window (orbit/zoom while it runs) instead of rendering offscreen. It reuses the same `Episode` record hook the offscreen recorder uses, so the run is identical to the one the videos and metrics come from; it only paces to the wall clock, draws trails into `user_scn` and prints a status line. `--speed` scales playback, close the window or Ctrl-C to stop, and the metrics print on exit.

```bash
$P scripts/am3_live.py N line                 # wall-contact task
$P scripts/am3_live.py N loop                 # figure-eight base + circling arm
$P scripts/am3_live.py B loop --no-wall --arm-down --arm-laps 0 --r-arm 0 --speed 0.5   # baseline diverging, slow motion
```

### Results (all 30 s, same plant/reference/seed/noise per pair)

| demo | N | B |
|---|---|---|
| Moving base, waypoint diamond | 0.175 mm RMS, tilt 1.67° | not run |
| Figure-eight ×3 + arm circles ×10 | 0.128 mm, tilt 0.65° | **diverged @ 15.05 s**, tilt 41° |
| Hover, arm only moving | 0.127 mm, tilt 0.32° | **diverged @ 13.55 s**, tilt 40° |
| Big figure-eight (±34.5 cm), arm held at 8 cm offset | 0.129 mm, tilt 0.66° | **diverged @ 11.94 s**, tilt 41° |
| Open-air 2:3 Lissajous, no wall, arm hanging uncommanded | 0.126 mm, tilt 1.21° | **diverged @ 4.82 s**, tilt 40° |

Media in `Media_results/AM3_{TRAJ,LOOP,FREE}_*`. Presentation explainer: `Media_results/AM3_FREE_lissajous_BvsN_2026-10-27/NOTES.md`.

**Interpretation, with its limits.** In free flight B does not merely track worse than on the wall task — it diverges every time, and fastest when the arm simply hangs. When B fails, N is using ~half the thrust envelope and a fifth of the joint-rate limit, so this is a **modelling** failure, not saturation. But: **B's gains were searched for the fixed-hover wall-contact task and it was never retuned for free-flight manoeuvring.** The defensible claim is "B as tuned for the original task diverges here", never "no decoupled controller could do this".

---

## 7. Where the research actually stands

From `TODO.md` / `Findings/STATUS.md`: items 3, 4, 5, 7, 8 partial; item 9 open.

- **Done:** design v3, predictor validation, B and N on all three wall paths, matched recordings, freeze v1 + nominal held-out gate (**N 30/30, B 1/30, Nd 0/30** on frozen criteria), timing and deadline trials.
- **Open frontier (item 8):** mismatch suite and the N vs D vs L comparison. On the 20-scenario mismatch development list: **N 11/20, D 13/20** joint successes — a modest win for the classical EKF. **L has never been evaluated**; its SAC training was paused at ~100k of 200k steps and the checkpoints are not in the repo.
- Known unresolved: N is not real-time compliant; the nominal predictor matches the plant by construction, so N's nominal accuracy is an upper bound; sliding/reversal/contact-loss predictor validation still open.

### Sensible next steps
1. **Long line on the wall** — needs the moving base, because the contact patch is capped at |y| ≤ 5 cm by yaw authority. The failsafe fix in §5 was a prerequisite.
2. Writing "OM" on the wall — multi-stroke with pen-up/pen-down through the contact state machine. Moderate work.
3. **Ball / curved surface — substantial, not a quick job.** The gap and contact normal are hard-coded to a plane (`n = (−1,0,0)`) in *both* the MuJoCo model and the CasADi predictor (`am3_pred.py`). A sphere needs `g = ‖p_T − c‖ − R − r_pad` with a state-dependent normal, plus re-validation of contact onset, sliding and impact. Not a config change.
4. Chase real-time compliance — cheap and high value given the hardware finding in §4.
5. Resolve L: get Aryan's checkpoints or retrain.

---

## 8. Session log

### 2026-10-27 (session 1)
- Pulled prior memory from `~/.claude/projects/-home-pinaka-ACP/memory`, scanned the repo, rewrote project memory under `-home-pinaka-arm-optimal/memory`.
- Built the environment (§3); found and fixed two undocumented blockers: missing `t_renderer` and the conflicting acados at `/home/pinaka/acados`.
- Reproduced the predictor check and the B/N line comparison (§4). Found that this machine is ~1.7× faster, which materially changes the real-time story.
- Added the moving-base capability (§5, §6): base-velocity reference, `LoopTask`, Lissajous shapes, wall removal, arm-down start.
- Found and fixed the fixed-`BASE` failsafe bug, which was producing false "loss of control" reports.
- Produced six demo media sets; logged every one in `Media_results/README.md` labelled as outside the frozen plan.
- **Not done:** long line on the wall, "OM" writing, curved surface, aggressive-trajectory variant, anything involving L.

## 2026-11-10 22:35 EST — handoff (PC shutting down)
- Local H1_hybrid_s0 stopped at ~2.6M steps (model_interrupt.zip saved). It was not learning: reward/step -7 to -9 vs plain NMPC (zero residual, nominal) ~2.3 on 8/8 completed episodes. Likely cause: exploration noise log_std -1.5 (~13 rad/s rotor noise) crashes episodes. DR-on NMPC baseline check timed out (not measured).
- Turing: R12_write_s0 (2503635, 12M/30M, 20.2/step, ETA ~02:30 EST), R13_write_dr_s0 (2503636, 16.5M/30M, 10.0/step, ETA ~00:30 EST), H1b_hybrid_s0 (2505671, PENDING; log_std -3.0, 10M steps).
- Next: copy R12/R13/H1b back, score with am3_write_eval.py + am3_write_mismatch.py (add HybridCtrl loader to make()), render NMPC vs RL vs hybrid sentence video.

## 2026-11-11 11:25 EST
- R12, R13 scored (table in Findings/Write/01_wall_writing.md). R12 1.1-2 mm not robust; R13 ~4 mm, only one surviving payload 100 g + motor_slow.
- H1/H1b dead: plain NMPC under DR loses control on payload >= 0.15 kg; H1b stalled. Code: scale_mismatch + dr_ramp (am3_rl_wall), write_drc env, hybrid episode cap + D_TAU 0.5 tau_c.
- Turing: R14_write_drc_ft_s0 (2507364), H2_hybrid_s0 (2507365), both PENDING (priority) at 11:20.
