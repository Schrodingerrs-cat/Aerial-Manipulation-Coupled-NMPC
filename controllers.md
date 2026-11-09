 Here are the controllers in the repo, all in Code/ocrl/:

  Name: B (am3_ctrl_b.py)
  What it is: Baseline: geometric hover controller plus separate joint-space arm
  control. It deliberately ignores arm–base coupling. Gains are tuned
  (config/am3_b_gains_v3.json).
  Rate: 200 Hz
  State: Frozen (v1). 30/30 completed, 1/30 meet the criteria in the held-out gate.
  ────────────────────────────────────────
  Name: N (am3_ctrl_n.py, am3_nmpc.py)
  What it is: Coupled NMPC: full 9×9 mass matrix, rotor lag and pad–wall contact in

  the prediction. CasADi model, acados SQP-RTI.
  Rate: 50 Hz
  State: Frozen (v1). 30/30 in the held-out gate. Not real-time compliant.
  ────────────────────────────────────────
  Name: Nd (same files, coupling="decoupled")
  What it is: The NMPC ablation: identical costs, constraints and solver settings,
  but a block-diagonal mass matrix.
  Rate: 50 Hz
  State: Frozen (v1). 0/30: failsafe aborts plus acados solver hangs.
  ────────────────────────────────────────
  Name: Z (am3_ctrl_corr.py)
  What it is: N with zero correction. It is a parity check and reproduces N exactly

  (zero difference in pad position and commands).
  Rate: 50 Hz
  State: Test only.
  ────────────────────────────────────────
  Name: D (am3_ctrl_corr.py + am3_ekf.py)
  What it is: N plus a classical EKF that estimates two prediction corrections:
  normal force and pitch moment.
  Rate: 50 Hz
  State: Mismatch dev list: 13/20 joint successes (N: 11/20). Not frozen, no
  held-out run.
  ────────────────────────────────────────
  Name: L (am3_ctrl_corr.py + am3_rl.py)
  What it is: N plus a SAC policy that outputs the same two corrections.
  Rate: 50 Hz
  State: Training is paused at about 100k of 200k steps for 3 seeds. Not yet
  evaluated beyond a smoke test.

  All of them run on the same MuJoCo plant, with the same noisy sensors, one-period
  command latency and task state machine. The held-out comparison is only on the
  nominal model.

  Two caveats still apply. B runs at 200 Hz and N at 50 Hz, so the B/N gap includes
  rate as well as architecture. And, as we discussed, the physical cause of B's
  drift hasn't been isolated yet.
