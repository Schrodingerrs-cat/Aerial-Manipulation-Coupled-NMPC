# The RL story in plain words
Written 2026-11-01 09:55 EST. Times are given only where logs recorded them; older steps have dates only.
Details and raw numbers: `03_endtoend_rl_design.md`.

## What we are trying to do
Train one neural network (PPO) that flies the drone + arm by itself, with no NMPC. It is separate from the NMPC work. Ladder of goals:
hover -> follow a path -> move the arm -> path + arm -> draw a line on the wall.
Success rule (set before looking at results): 100% of held-out starts survive, then lowest error; goal RMS <= 5 mm.
How it works: the network outputs *references* (acceleration, body rates, yaw, joint targets); a classic inner loop turns them into motor commands.

## Timeline

| When | What happened | Result |
|---|---|---|
| early (date unlogged) | **Attempt 1.** Reward let the policy learn to crash. | Invalid. Fixed the reward. |
| early | Found the inner loop itself was marginal. | Fixed before more training. |
| 2026-10-27..30 | **R1: hover.** 3M steps, 40 held-out starts. | Worked. You confirmed hover looked fine. |
| 2026-10-30 21:25 | **Bug:** two "replicate" runs were identical (seed restored by load). | Fixed (`set_random_seed`), relaunched. |
| 2026-10-30/31 | **R2: follow a small figure-8.** | Precise (~2 mm) but sat ~10 mm *below* the green line. Your feedback: too small, too low, want bigger moves in all directions. |
| 2026-10-31 | **R3: big 3D paths** (left/right/up/down/fwd/back), finer reward, bias fix. | z bias fixed, but some starts crash. Tried many fixes: stronger penalties, tilt penalty, wide starts, behaviour-cloning start. None fixed the crashes. Several were dropped (listed in file 03). |
| 2026-10-31 | **Diagnosis:** crashing starts had the body-rate command saturated (policy over-steering). | Cause of R3 failures. |
| 2026-10-31 13:49 | **R4: cap the action size** (accel +-3.5 m/s2, rate +-0.6 rad/s). Two runs: warm (from R3) and scratch. | At 1M steps: 6/6 starts survive, ~2.1 mm vs PD 6-20 mm, z bias ~-1 mm. |
| 2026-10-31 evening | **More checking (20 starts).** Push tests; 3 comparison videos. | 19/20 survive at 1M. Pushes: RL better than PD everywhere, both lose at 4 N sideways. |
| 2026-10-31 15:48 / 16:45 | Warm (8M) and scratch (15M) finished. | Survival got *worse*: warm 16/20, scratch 17-18/20. |

## Where we are now (2026-11-01 09:55 EST)
- **Good:** accuracy. 2-3 mm on big 3D paths, 3-7x better than PD+feedforward. Bias fixed. Gentle tilt (~19 deg at 1M). Better than PD at being pushed.
- **Not good:** reliability. 1-4 of 20 random starts crash within ~1.5 s, even though those starts are not extreme. Same starts fail on every path, so it is about the start, not the path. More training does not fix it.
- **Best candidate:** warm run at 1M steps (19/20, 2.8 mm). It does **not** pass the 100% rule, so nothing is officially "selected".
- Nothing is training now.

## Why it probably happens (not proven)
On the bad starts the actions slam to +-1 and flip sign (chatter). Looks like the network meets an observation it rarely saw and over-reacts. Training reward barely punishes a few lost starts because most episodes go well.

## Next plan
1. Train with hard starts over-sampled (not the eval seeds), plus a penalty on action jumps (anti-chatter), and maybe a bigger crash penalty. These are changes from the plan, so they will be logged as such.
2. Re-check with 20 starts, then a fresh set of seeds, so we do not fool ourselves.
3. If 20/20 and <=5 mm: make the videos (combined + separate), then climb the ladder: arm motion, path + arm, line on the wall.
4. Only then rebuild the Turing tarball and run longer jobs there (needs the Turing hostname from you).
5. Housekeeping: README/STATUS.md still claim D beats N (p=0.688) - to be corrected.

## Things you can do now
- Fly it yourself: `python scripts/am3_rl_live.py --env track4` (mouse pushes) using the 1M checkpoint `results/am3_rl/R4_warm_s1/model_1000k.zip`.
- Videos: `Media_results/RL_R4_*_2026-10-31/` (one start that survives; combined only so far).

## Decision for you
Go ahead with the plan above, or change goals (e.g. accept 95% survival for now and move up the ladder)?

## Update 2026-11-08 20:07 EST: R6 (path + arm moving, up to 50% of joint range)
The earlier "PD about 300 mm" was mostly a flaw in our inner loop. With that fixed, PD gets 18-24 mm and RL was retrained (R6).
On 50 fresh starts and 4 big paths: seed 1 survives all, 5.2-6.4 mm mean (under 3 mm median). Seed 0 is more accurate, 3.4-3.6 mm, but loses one start (the same one on every path).
So RL is 3-5x better than PD and survives pushes better, but neither seed fully meets "100% survival and <= 5 mm". Videos are still to do.

## Update 2026-11-09 14:40 EST: R7 and R8
R7 fixed the yaw command: more accurate (4.4-4.8 mm on seed 1) but still lost 1-2 starts. R8 also limited acceleration to +-1.75 m/s^2: no lost start on either seed across 50 x 4 paths, 300 starts and 300 fresh starts. Cost: error rose to 4.8 mm on 300-start big_fig8 and 6.7-8.6 mm on the 50-start tests. PD is 17-24 mm. So survival passes, the 5 mm goal is only partly met.
