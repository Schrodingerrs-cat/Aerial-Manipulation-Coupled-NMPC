# Controllers for design v3 (25 Oct 2026 plan)

All controllers act on the same MuJoCo plant (`Code/ocrl/am3_plant.py`, design v3, coupling never removed), receive the same noisy state and force measurements (`am3_episode.Sensors`: 0.1 mm / 0.01 deg / 1 mm/s estimator-grade state noise, 10 mN force noise), use the same task logic and contact policy (`am3_task.py`) and the same actuator limits (rotor speeds 180..1800 rad/s with lag 40/60 ms, servo envelope 1 N m continuous / 3 N m stall). Commands pass through a one-control-period computation latency.

## Baseline B (`am3_ctrl_b.py`)
Coupling-neglecting but competent: geometric hover controller + separate joint-space arm control.
* Base: position PID (gravity support with the correct total mass 1.52 kg), SO(3) attitude PD+I about the **declared nominal locked-posture COM** with its inertia (computed once from the predictor at the contact posture), moment trim initialised from rotor-speed telemetry at hover, kinematic prediction over the command latency, partial inversion of the motor lag, 4x4 rotor allocation.
* Arm: position-only IK of the pad reference at the nominal base pose, joint PD + feedforward of the arm's own model (joint block of the decoupled predictor: M_qq, gravity, velocity terms at the reference), slow task-space integral correction.
* Contact: the wall force implied by the policy's indentation reference (k x indentation) is fed forward into thrust and joint torques; no force feedback loop.
* Omitted by construction: arm-base inertial cross blocks (M_bq), reaction of arm motion on the base, COM shift away from the nominal posture, joint optimisation of the seven inputs.
* **Rate: 200 Hz with one-period (5 ms) latency**, because its computation takes about 1 ms. A 50 Hz B with 20 ms latency was tried first and was unstable for every gain set (linearised closed-loop radius > 1, `Code/results/am3_b_tuning/linear_search*.json`); that is a consequence of a light arm under torque control with 40 ms of effective delay, and it is the reason B runs at its natural rate. The B/N difference therefore includes architecture, model and rate.
* Tuning (equal-or-larger budget than N): linearised closed-loop scans, 56 grid points, 80 + 120 + 120 random-search samples scored on development scenarios only (`Code/results/am3_b_tuning/`, `Code/config/am3_b_gains_v*.json`). Earlier gain sets are kept; v2 diverged on 5/16 development scenarios, v3 completed 16/16 after the policy-force feedforward was extended to the approach phase.

## Coupled NMPC N (`am3_nmpc.py`, `am3_ctrl_n.py`)
* 23-state predictor with the full 9x9 mass matrix, gravity and velocity bias, rotor lag, spherical-pad wall contact with regularised Coulomb friction, built only from the design config (not from MuJoCo); one CasADi formulation drives IPOPT (reference) and acados SQP-RTI (production). N=10 stages of 20 ms, RK4 with 4 substeps.
* Cost (normalised least squares): base position/velocity about the **fixed** hover reference, pad position and velocity along the task reference, small attitude/rate/joint-rate terms, input variation. No orientation reference, no force setpoint, no alignment term, no terminal LQR (terminal = running residual).
* Soft constraints with named normalised slacks: tilt 15 deg, operational joint ranges, joint rate, normal force <= 4 N, closing speed during approach, elbow/wrist height below the rotor plane; hard rotor-speed and joint-torque bounds.
* The measured state is propagated one period with the pending command before solving. Failures retry once from a reset, then fall back to the previous command (logged).
* **Model ablation Nd**: identical costs, constraints, solver settings, only the predictor uses block-diagonal M and per-block bias (gravity kept). In development runs Nd diverged ~0.4 s after the arm started to move (abort logged in `Code/results/am3_runs/dev_Nd_*`).
* IPOPT/acados parity at four task states: cost agreement 1e-9 relative, first input within 1e-4 rad/s (`Code/results/am3_predictor/solver_parity.json`).

## Known limits
Weights of N were set once from a first guess (no tuning search); N is at the sensor-noise floor in nominal runs, so extra tuning would not change the nominal comparison. All nominal runs use a predictor that matches the plant by construction up to integration and contact calibration; mismatch robustness is not yet evaluated (D/L comparison).
