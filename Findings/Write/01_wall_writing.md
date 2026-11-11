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

## Correction: drone and arm must write together (2026-11-10 19:40 EST)
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
