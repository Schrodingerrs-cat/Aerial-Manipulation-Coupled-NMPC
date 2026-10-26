# Plan: coupled aerial-manipulator control for hover and direct end-effector wall trajectories

**Authoritative revision: 25 October 2026.** This plan implements the user's clarified goal: **the arm's own end effector moves along a trajectory in contact with a wall while the drone maintains a fixed hover-position reference and compensates for arm motion and contact forces.** A long probe, prescribed tool orientation and precision force-setpoint tracking are not requirements of this task.

This replaces the [21 October plan](Findings/Planning/Plan_before_direct_EE_hover_2026-10-25.md). Preserve that archive, the supplied originals, all R2/R3 code, recordings, audits and experimental history. Prior results belong to their actual model and task; they do not establish success on this revised task.

## 1. Goal, scope and honest comparison

The robot must hover in free flight, acquire wall contact with its own end effector, execute a feasible time-parameterized trajectory while maintaining contact, stop and release safely. The drone compensates by changing rotor speeds, thrust and attitude. Its world-frame **base-position reference remains fixed during the trace**. It is not welded to space, externally supported or translated along the wall to substitute for arm motion.

The end effector may rotate as its joints move. There is **no perpendicular-pointing objective, no desired end-effector orientation, and no orientation-based success threshold**. Mechanical contact feasibility, collision avoidance, actuator limits and bounded forces remain physical requirements. These protect the task; they are not a return to the removed orientation objective.

The user requests a comparison showing why coupled control matters. The scientifically valid experiment is:

- The **same fully coupled physical plant** is controlled by a competently tuned controller that neglects selected arm coupling effects.
- Our controller accounts for the coupled drone–arm dynamics and wall contact.
- Compare hover drift, attitude excursions, arm/path tracking, contact loss and actuator demand under matched tasks.

**Do not make the baseline fail on purpose.** An uncompensated controller may remain stable with greater error, especially for slow motion or high feedback gains. A larger transient is not automatically instability. If both perform well, report that result and test predeclared, physically feasible movement/contact conditions. Do not weaken baseline gains, give it the wrong total mass, withhold sensors, remove gravity support or select only failure clips to manufacture an improvement. “Works perfectly” means satisfying measured tolerances, not claiming zero error or guaranteed stability.

### 1.1 Relationship to the proposal

[proposal.pdf](proposal.pdf) motivates nominal NMPC versus classical and learned model correction for aerial contact. The current task explicitly replaces the earlier force-and-alignment spot-inspection specification with **direct end-effector trajectory tracking and hover stabilization under contact**. Keep that scope change visible in the report.

There are two distinct comparisons:

| Comparison | Question | Controllers |
|---|---|---|
| Required foundational comparison | Does explicitly modeling coupling improve hover and arm/contact tracking? | Coupling-neglecting baseline B versus coupled nominal NMPC N |
| Subsequent proposal comparison | Does model correction improve robustness to mismatch beyond nominal modeling? | N versus observer-corrected D versus learned-corrected L |

Do B/N and the required recordings first. Do not use the extra D/L work to bypass unresolved assembly or nominal-control problems. Preserve the two correction channels from the proposal for D/L unless a later explicit decision changes them; a normal-force correction channel does not require a normal-force tracking setpoint.

## 2. Current assets, evidence and design corrections

The robot source is [Model_candidate_2026-10-21](Model_candidate_2026-10-21). The supplied main XML has a free drone base and four arm hinges. The working three-active-joint derivative is `Code/model/aerial_manipulator.xml`, with q4 fixed and the hand mass retained. Active joints remain q1 yaw, q2 shoulder and q3 elbow. Current v2 code/model and training results are separate, altered designs.

The original supplied inertial ledger is:

| Body | Mass kg | Local COM m | COM diagonal inertia kg m² |
|---|---:|---|---|
| drone | 1.05 | (0,0,0) | (0.0017969, 0.0017969, 0.003454) |
| link1 | 0.12 | (0,0,-0.025) | (0.00005, 0.00005, 0.00005) |
| link2 | 0.15 | (0,0,-0.08) | (0.0003321, 0.0003321, 0.0000243) |
| link3 | 0.10 | (0,0,-0.07) | (0.000169, 0.000169, 0.0000113) |
| hand | 0.10 | (0,0,-0.025) | (0.0000342, 0.0000342, 0.0000267) |

The supplied assembly totals 1.52 kg before added components. These are supplied simulation values, not identified hardware parameters. The separate bare-drone XML is a different 1.24 kg vehicle plus an unattached payload; do not mix its parameters into this robot. Its principal inertia orientation must be transformed before use in body axes.

### 2.1 Verified history and its limits

The first diagnostic recorded hover/arm motion with the fixed wrist, direct thrust/moment inputs and simulator bias compensation. It reported COM error, not proof of fixed base-position tracking under the final controller. Summing locked-system inertia does not validate every block of the articulated mass matrix or bias.

The v2 design moved the mount to (-0.08,-0.06,-0.035) m, rotated the arm, and added a 0.33 m lateral contact extension. The audit found a visible gap between the shifted root and the original yaw housing, with no represented connecting bracket. The extension is rigidly attached in the simulator rather than grasped by the fingers. Existing v2 gate passes and SAC results validate that configuration only. Reported timing misses also remain separate from functional passes.

### 2.2 Required physical assembly

Use the supplied arm as the foundation. Restore or design a visibly continuous, mechanically plausible mount joining the airframe to the actual joint root. Account for added mounting mass, COM and inertia. A parent-child body relation across empty space is insufficient evidence of a complete mechanical assembly.

**Remove the long lateral extension from the target design.** Contact is at a defined surface of the hand/end-effector itself. A flush or compact local contact surface may be used only as an explicitly documented part of that end effector; it must not be a standoff rod that relocates the task away from the hand. The original `ee` site is a location marker, not automatically a collision surface. Select a physically present surface, define its tool point/frame, and make its collision geometry agree with what the viewer shows.

Keep q4 mechanically fixed; choose its fixed angle for reach and collision clearance rather than constant wall-normal pointing. Preserve the fixed hand inertia. Finger geometry is rigid unless explicitly remodeled; do not claim an active grasp or hidden additional joint.

The implementation agent may adjust joint ranges, mount transform, fixed wrist angle and necessary geometry within this three-active-joint design. Every change has a physical rationale and updated inertials. Distinguish mechanical joint limits from operational restrictions and configuration-dependent collision constraints. Do not merely hide old geometry while retaining the old contact site or dynamics.

## 3. Task specification and feasible trajectory generation

### 3.1 Hover and trajectory references

World z is upward. For the primary wall x=x_wall, choose outward normal n=(-1,0,0), horizontal tangent t_h=(0,1,0), vertical tangent t_v=(0,0,1). The drone base-position reference p_B,ref is fixed throughout a trace. Attitude may change to provide the required lateral force; no zero-roll/pitch requirement is imposed.

Let p_0 lie on the wall and c_h(s), c_v(s) describe a smooth planar path for s in [0,1]:

\[
p_{surf,ref}(s)=p_0+c_h(s)t_h+c_v(s)t_v,
\]
\[
\dot p_{surf,ref}=p'_{surf}(s)\dot s,\qquad
\ddot p_{surf,ref}=p''_{surf}(s)\dot s^2+p'_{surf}(s)\ddot s.
\]

Use smooth start/stop progress, for example s(xi)=10xi³-15xi⁴+6xi⁵ with xi=(t-t_start)/T clipped to [0,1]. Generate position, velocity and acceleration consistently. Lines, circles/ellipses, smooth splines and rounded multi-segment paths use the same interface. Sharp corners require rounding or an explicitly timed stop.

Start with a short horizontal segment, then a feasible curved path and direction changes. Select size, location and speed from the actual contact workspace, not from the old 33 cm probe's workspace. Freeze each comparison task before evaluating it. Do not shorten paths or slow only the baseline/controller selectively after seeing outcomes.

### 3.2 Contact without a force-setpoint or alignment task

The primary controlled outputs are base position and end-effector trajectory **while remaining in contact**. Normal force is a physical reaction that must be handled, logged and bounded. It need not follow a precision setpoint.

Choose and document one common contact-maintenance policy, such as a small bounded normal approach/indentation reference with measurement-based contact persistence. A lower force threshold can distinguish contact from noise; an upper force bound limits damaging impacts. Neither is a force-tracking setpoint. An optional low-weight preload objective is an engineering choice that must be declared and shared across B/N/D/L, not silently turned into the research objective.

Do not enforce exact zero gap and independently prescribe an incompatible force in a compliant model. For a compliant surface, slight indentation may be required. If native contact uses penetration regularization, state its interpretation and calibrate it. End-effector rotation is allowed, but the chosen physical surface must remain capable of sliding contact without the fingers or hand unexpectedly striking the wall.

### 3.3 Kinematic and dynamic feasibility

For a fixed base pose, coordinated yaw, shoulder and elbow motion can locally place a three-DOF end effector in 3D when its position Jacobian has full rank. Yaw alone usually traces an arc; shoulder and elbow adjust depth and height to produce a straight line. This does not guarantee arbitrary paths, singular configurations or feasible forces.

Check the full path and transitions for position reach, joint speed/torque, collision clearance, wall-edge avoidance, rotor reserve and reasonable base motion. End-effector orientation is evaluated only where it affects physical contact geometry or clearance, never against an arbitrary pointing target.

The whole-system COM moves when the arm moves even if the base stays still:

\[
p_G=\frac{\sum_i m_i p_{Ci}}{m},\qquad
m\ddot p_G=R A_F f+f_C-mg_0e_3+f_{ext}.
\]

Consequently, COM stabilization and base stabilization are distinct. Arm momentum and contact lever arms affect required moments. Sliding friction generally requires roll/pitch adjustments as well as thrust changes. Use full dynamic feasibility, not only total thrust-to-weight or a static posture check.

## 4. State, geometry and coordinate conventions

Use scalar-first Hamilton Q mapping body vectors to world, R=R(Q), body angular velocity omega, world base linear velocity v, and q=(q1,q2,q3). Let [a]_x b=a cross b. Gravity acceleration is -g0 e3 with g0=9.81 m/s².

\[
x_{mech}=(p_B,Q,v,\omega,q,\dot q),\qquad
\nu=(v,\omega,\dot q)\in\mathbb R^9.
\]

The mechanical model has 19 stored / 18 independent states. Four rotor-speed states give the production model 23 stored / 22 independent states. Inputs are four commanded rotor speeds and three joint torques: seven channels. A diagnostic wrench-actuated model has seven inputs too, but its first four are (T,Mx,My,Mz), not rotor speeds; never interchange them silently.

| Production quantity | Zero-based slice | Units/frame |
|---|---|---|
| p_B | 0:3 | m, world |
| Q | 3:7 | unit quaternion, body to world |
| v | 7:10 | m/s, world |
| omega | 10:13 | rad/s, body |
| q | 13:16 | rad, named joint axes |
| qdot | 16:19 | rad/s |
| Omega | 19:23 | physical rotor speed magnitudes, rad/s |
| u | 0:4 then 4:7 | rotor-speed commands then joint torques |

\[
\dot p_B=v,\qquad\dot R=R[\omega]_\times,\qquad
\dot Q=\tfrac12Q\otimes(0,\omega).
\]

Integrate and normalize attitude consistently in both solvers; account for normalization in derivatives. Maintain quaternion sign continuity and compare rotations rather than raw quaternion components. Rendering-only blade angles are not physical controller states.

### 4.1 Supplied-chain reference kinematics

For the original mount and the original `ee` site, with q4 fixed at beta,

\[
r_{ee}=-0.035e_3+R_z(q_1)\left[-0.05e_3+
R_y(q_2)\left(-0.16e_3+R_y(q_3)(-0.14e_3+R_y(\beta)(-0.064e_3))\right)\right].
\]

Then p_ee=p_B+R r_ee. This matches the supplied site's geometry, not automatically the final hand contact surface. The earlier inspection checked this chain against MuJoCo over 50 random joint configurations with maximum component error 2.23e-16 m. Revalidate after mounting/contact changes; this is not evidence of feasible trajectories.

The authoritative implementation composes the accepted model's actual joint-origin transforms and rotation axes, plus the fixed hand/contact transform. Do not keep the old v2 +Y probe offset or its pointing-axis output after changing the tool.

For a downstream point r_i in body coordinates, revolute-joint columns are

\[
J_{r,i}(:,j)=a_j\times(r_i-o_j),\quad
J_{\omega,i}(:,j)=a_j,
\]

and zero for non-ancestor joints. Use all three columns. The world position velocity is

\[
v_i=v+R(\omega\times r_i+J_{r,i}\dot q).
\]

Solve position-only IK with joint/clearance constraints and sensible posture regularization. Do not reinsert a perpendicular-pointing equation. Inspect scaled Jacobian singular values over each path and its approach/release.

## 5. Coupled dynamics: the physics shared by every controller

For each rigid body i, use mass m_i, its COM offset r_i(q), orientation R_i=R R_Bi(q), and COM rotational inertia I_Ci. With the mixed velocity convention above,

\[
J_{v,i}=[I_3\; -R[r_i]_\times\; R J_{r,i}],\quad
J_{w,i}=[0\;R\;R J_{\omega,i}],\quad I_i^W=R_i I_{Ci}R_i^T.
\]

\[
M(R,q)=\sum_i\left(m_iJ_{v,i}^TJ_{v,i}+J_{w,i}^TI_i^WJ_{w,i}\right).
\]

All base–joint cross blocks remain in the plant and full predictor. In this convention M can depend on R as well as q. Fixed-hand and mount masses count once; do not attach all arm mass to the base as a substitute for articulated dynamics.

Gravity and velocity bias are

\[
G=\sum_iJ_{v,i}^Tm_i g_0e_3,
\]
\[
b_v=\sum_i\left[J_{v,i}^Tm_i\dot J_{v,i}\nu+
J_{w,i}^T\left(I_i^W\dot J_{w,i}\nu+
\omega_i^W\times I_i^W\omega_i^W\right)\right].
\]

Jacobian derivatives follow Rdot and qdot with nu held fixed for this bias calculation. This includes transport terms needed for body angular quasi-velocities. Do not apply ordinary coordinate Christoffel formulas blindly or omit Coriolis/centrifugal coupling.

\[
M\dot\nu+b_v+G=
\tau_{rot}+S_q^T(\tau_a-\tau_{fr})+
\sum_{c\in contacts}J_c^Tf_c+\tau_{ext},\qquad
S_q=[0_{3\times6}\;I_3].
\]

For contact moments, add the corresponding angular-Jacobian term. Solve the linear system rather than explicitly inverting M in production. MuJoCo handles the actual contacts, limits and multibody reaction in the plant; the independent predictor implements its declared approximation.

### 5.1 Fixed-body inertias and frame handling

For fixed components expressed in a common frame,

\[
m=\sum_i m_i,\quad c=\sum_i m_i c_i/m,
\]
\[
I_C=\sum_i\left(A_i I_{Ci}A_i^T+m_i(\|c_i-c\|^2I_3-(c_i-c)(c_i-c)^T)\right).
\]

Transform MuJoCo principal inertias through `body_iquat` before using them in body axes. Distinguish inertia about the origin from inertia about the COM. Preserve off-diagonal terms. Mount or geometry changes require mass/COM/inertia reconciliation; collision/visual mesh dimensions do not automatically provide reliable inertial properties.

### 5.2 Rotors and joint actuators

For rotor i at r_i with body thrust axis a_i and reaction sign s_i,

\[
f_i=k_{f,i}\Omega_i^2,\quad
F_i^B=f_i a_i,\quad
\tau_i^B=r_i\times F_i^B+s_i\kappa_i f_i a_i,
\]
\[
\tau_{rot}=\left(R\sum_iF_i^B,\sum_i\tau_i^B,0_3\right)^T,
\quad\dot\Omega_i=(\Omega_{cmd,i}-\Omega_i)/t_{m,i}^{up/down}.
\]

Specify coefficient units, spin signs, motor lag, speed bounds and the feasible simultaneous thrust/moment set. The old 40 N collective cap alone is not evidence of four 10 N rotor limits. Source parameters or explicitly mark a coherent simulation assumption and test sensitivity. No fictitious independent lateral base force is allowed.

Joint commands pass through declared speed/torque limits and passive friction. Stall torque is not continuous torque. Use independently justified limits for each actuator, including the chosen motoring/braking approximation. Apply the same nominal saturation model to prediction and plant, or quantify the approximation. Log requested and applied commands separately.

## 6. Physical wall contact and its prediction

The plant uses native unilateral contact and friction on the selected **hand/end-effector contact surface**. Do not implement wall following by welding a contact point, overwriting the tool pose or applying hidden forces. Validate contact geometry against the visible assembly.

For a point-like or compact convex local surface, define signed gap g>0 in free space and v_c=-gdot. At the instantaneous contact point C, map forces with its material-point Jacobian J_C. The normal/tangential split is

\[
f_C=nF_n+f_t,\quad F_n\ge0,\quad
v_t=(I-nn^T)J_C\nu,\quad v_t^Tf_t\le0.
\]

A regularized spring/damper normal law and smooth friction can serve as a predictor approximation. Declare k, c, smoothing widths, contact gate and friction velocity scale, and measure precontact leakage and impact error. Native MuJoCo parameters are not automatically identical to a physical spring law. If a compliant contact model represents an assumed material, say so; do not call it a mechanical spring mechanism that has not been modeled.

If the local contact surface is spherical with center T and radius r,

\[
p_C=p_T-rn,\quad g=n^T(p_T-p_{wall})-r,
\quad J_C=J_T+r[n]_\times J_{wT}.
\]

The normal velocity equals n^T v_T, but tangential material velocity includes rotation. A spherical local model must match an actual compact end-effector surface, not reintroduce the rejected long probe. For a box/hand or multiple fingers, contact points can move or multiply; use the appropriate geometry and aggregate forces/moments rather than assuming one sphere without justification.

Require virtual power nu^T J_C^T f_C=v_C^T f_C. Calibrate and validate approach, sustained contact, sliding, sticking/breakaway, reversal and unloading. Refine plant timestep and predictor integration separately; compare peak force, impulse, gap, slip and state error. Two timestep results establish sensitivity, not a proven uncertainty bound.

## 7. Controllers and the coupling comparison

### 7.1 Baseline B: a competent coupling-neglecting controller

Use a geometric hover controller for the drone and joint-space tracking for the arm. Give the vehicle controller the correct total mass and a declared nominal locked-posture inertia; include gravity support and reasonable static compensation. Give the joint controller its nominal arm model. Both receive the same measurements and reference preview as the coupled controller where applicable.

The baseline omits configuration-dependent base–arm interaction in its prediction/feedforward and does not jointly optimize the seven inputs. It still receives feedback and must be tuned on an equal development budget to hold hover and track reasonable no-contact motion. Apply rotor allocation, motor lag, actuator limits and common contact-management logic identically.

This practical B/N comparison changes both controller structure and modeling. Describe it as a practical baseline comparison, not a pure causal isolation of one matrix term. To isolate coupling-model contribution, additionally implement an **NMPC model ablation** with the same optimization settings/costs/constraints and an explicitly decoupled approximation. Document exactly which blocks/bias terms are omitted; preserve a physically meaningful, solvable approximate model. Do not delete coupling from the simulation plant. Distinguish dynamic coupling omission from contact-wrench omission if studying both, rather than conflating their effects.

A zero-control/free-fall clip may be an optional physics illustration, clearly labeled. It is not the required baseline and is not evidence that coupled NMPC is necessary.

### 7.2 Coupled nominal NMPC N

Use CasADi/IPOPT as reference and acados for production. Start from a 20 ms control period and a documented horizon; integration resolution must follow current model/contact validation, not old R2 defaults.

\[
x_{j+1}=\Phi_h(x_j,u_j;\theta),\qquad x_0=\hat x,
\]
\[
\min_{X,U,s}\sum_{j=0}^{N-1}\ell(x_j,u_j,r_j)+\ell_N(x_N,r_N)+\rho_1\|s\|_1+\rho_2\|s\|_2^2.
\]

Let P=[t_h^T;t_v^T]. Principal residuals are

\[
e_B=p_B-p_{B,ref},\quad e_{vB}=v_B,\quad
e_t=P(p_E-p_{E,ref}),\quad e_{vt}=P(v_E-\dot p_{E,ref}).
\]

Add phase-appropriate gap/closing-speed terms, posture/rate regularization and input variation. Scale all terms by physical tolerances. **Remove old probe-pointing residuals, alignment constraints, alignment reward terms and force-setpoint RMSE objectives from the primary task.** Attitude can be regularized for feasible hover without prescribing zero tilt. Normal-contact persistence and bounded force remain objectives/constraints through the common contact policy.

Enforce joint position/rate/torque bounds, individual rotor bounds, total tilt, collision clearance and bounded contact approach/force. Give each softened inequality a named, normalized slack and log its physical violation. Soft constraints are not guarantees. Check relevant intersample states, not only control nodes, and independently evaluate physics-resolution extrema.

IPOPT and acados must share state/input conventions, dynamics, cost scaling, terminal costs, constraints, parameters and slack penalties. Test values and derivatives on identical feasible and violated trajectories. RTI or iteration-limited success status is not nonlinear convergence. Log every retry, residual and failed attempt. Distinguish independent completion from fallback operation.

### 7.3 Common task state machine

Use hover, arm positioning, approach, persistent acquisition, contact settling, trace, endpoint hold, release, retreat and done. A fixed three-second precision-force dwell is no longer a task requirement; choose a brief documented acquisition/settling test that demonstrates stable contact before motion.

Use measured-contact persistence/hysteresis and bounded recovery. During loss, pause path progress only through the shared declared policy, account for the lost interval and completion delay, and rejoin with bounded velocity/acceleration. Recovery must not erase impact peaks, reset failure history or make an incomplete path pass. Retreat must not reacquire intentionally released contact. Stop commands at done; no extra final step with stale references.

## 8. Classical and learned correction after nominal validation

Retain the proposal's correction contract in the coupled predictor:

\[
d=(\widehat F_n,\widehat\tau_{pitch}),\quad
D(x)=[J_C^Tn\;E_{pitch}],\quad
E_{pitch}=(0_3,e_2,0_3)^T,\quad \tau_d=D(x)d.
\]

This is a generalized prediction correction, not a claim of physically measured disturbance components. Hold it over the horizon with identical magnitude/rate limits for D and L. Zero correction must exactly reproduce N. Do not apply a second copy as a hidden plant force or motor-action residual.

The classical baseline is an augmented error-state EKF or the already implemented equivalent after audit. A 23-component production state has a 22-component tangent error; adding two correction states gives a 24-dimensional covariance. Fix a multiplicative attitude-error convention, use a three-dimensional attitude innovation, the inverse innovation covariance in the Kalman gain, and a consistent covariance reset after attitude injection.

SAC observes shared measured state/history, contact signal, path reference/progress, hover error, previous commands and correction. Remove obsolete alignment/force-target reward terms. Reward hover and timed path tracking in contact, penalizing loss, excessive force, actuator violations and correction effort. No hidden true mismatch parameters or privileged contact information for L alone.

Both methods receive the same measurement noise, delay, state access, recovery logic and tuning budget. Tangential friction and general payload errors need not lie in the two-channel span: quantify bounded oracle projection error in a units-consistent metric and do not interpret unrepresentability as proof against learning or estimation.

Existing policies trained on the long-probe, force-and-alignment task are historical checkpoints. They are not evaluated as final policies for the corrected task without explicit transfer/retraining and a fresh validation record. Do not delete or overwrite them.

## 9. Experiment matrix, metrics and acceptance

### 9.1 Matched scenarios

Run the same fully coupled plant, geometry, mass/inertia, initial state, actuator model, sensor seeds, disturbance realization and trajectory under each controller. Use separate development, validation and held-out scenario lists. Include repeated trials; exact deterministic repeats alone are not independent robustness evidence.

| Case | What it isolates |
|---|---|
| Hover, arm stationary | Baseline controller validity and flight model |
| Hover, each joint then all joints moving | Arm inertial reaction without wall forces |
| Contact, stationary end effector | External wall reaction and acquisition |
| Contact, straight segment | Coordinated arm motion while base holds position |
| Contact, smooth curved path and reversals | Changing coupling, sliding force and acceleration |
| Matched mismatch/disturbance trials | Robustness and later N/D/L comparison |

Choose a predeclared feasible range of arm speeds/accelerations, path sizes and contact loads. Sweep it for both controllers. Do not increase speeds until only the baseline breaks and present that cherry-picked case as general evidence. If B remains stable, report its actual errors and margins; our contribution can be improved disturbance rejection or tracking rather than rescue from instability.

### 9.2 Required metrics

- Base-position RMS/max error relative to the fixed reference; base velocity and settling time.
- COM motion separately, with clear distinction from base hover error.
- Attitude/tilt and rates, reported as response and physical limits, not an end-effector alignment score.
- Timed tangential end-effector tracking RMS/max, along-/cross-track error, endpoint error and complete path coverage.
- Contact-valid fraction, longest uninterrupted contact, loss/recovery counts/duration and completion delay.
- Normal/tangential reaction forces, peak force, impulse and unexpected contacts; no required force-target RMSE.
- Joint tracking, speed/torque, applied versus commanded inputs, individual rotor reserve, clipping and clearance.
- Solver failures/retries/slacks and complete-cycle timing, including estimation, references, inference and data transfer.

A geometric trace alone is insufficient: a stationary point can have zero distance to the curve while failing timed tracking. Normal force alone is insufficient: good contact can coexist with unacceptable base drift. Report task-level failures alongside conditional errors, not just successful intervals.

### 9.3 Freeze and pass criteria

Before held-out comparisons, preregister hover/path tolerances, required contact fraction, force bounds, actuator/clearance limits, episode/recovery timeout and completion/retreat conditions. Derive force bounds from the accepted end-effector contact model; **do not retain the old 1.5*F* rule when there is no F* task target**. Remove orientation-based checks and the old mandatory three-second force/alignment dwell from evaluator and configuration.

A nominal gate can retain the planned 30 cases and 95% joint-success target (at least 29/30), but its revised scenario composition and criteria must be frozen first. Record exactly what changed from old gates. A functional pass does not imply real-time compliance.

Keep the 20 ms full-cycle target and report median/p95/p99/max plus deadline misses. Test measured/injected delayed command application, not only a constant ideal delay, before real-time claims. Do not change the pass rule after seeing timing data. Preserve every freeze manifest; a code edit after freezing requires equivalence evidence or a new candidate/version, never retrospective overwriting of the old manifest.

## 10. Required comparison videos and visual integrity

Produce the requested B/N comparison **using actual closed-loop viewer recordings**. This is a required deliverable alongside the metrics, not an optional final polish step.

Record at least:

1. Arm movement in free flight: B and N, same motion and initial conditions.
2. Wall-contact trajectory: B and N, same physical surface, path and timing.
3. A curved-contact trajectory with N, showing the entire assembly and the hand's actual contact.

Provide synchronized side-by-side videos using identical camera, scale, lighting and simulation-time alignment. Both panels run at the same displayed simulation speed. Disclose slower-than-real-time execution, acceleration of playback and any cuts. If one controller aborts, show the abort time and cause; do not loop or conceal its final state. Preserve raw uninterrupted recordings and compositing commands so the comparison can be reconstructed.

Overlay controller identity, model/config tag, simulation time, base-position error, path error, contact state, measured normal force and phase. Show an overview that makes the arm mount visible and a contact close-up. A side-by-side diagnostic plot of base drift and path error should accompany the video.

Draw actual 3D end-effector positions. If showing a wall-plane projection, label it as a projection and distinguish contact-valid segments from detached motion. Do not project all path samples onto the wall and imply they are physical contact marks. Desired and actual curves must have distinct colors. No scripted poses, mock successes or postprocessed smoothing of the robot trajectory.

Rotor visualization must follow physical motor speeds, with explicit visual slowdown/blur if used. The end effector must visibly touch the wall at the modeled surface, and the mount must visibly connect to the drone. Video frame rate cannot establish impact peaks; use physics-rate logs from the same run.

## 11. Validation and implementation sequence

| Stage | Work | Exit evidence |
|---|---|---|
| 0 | Preserve old models/results; reconcile current goal across docs | This plan, updated task contract and versioned historical archive |
| 1 | Repair physical mount and remove long extension from target design | Connected geometry, mass/inertia ledger, direct hand-contact definition and viewer inspection |
| 2 | Validate workspace and actuation | Position-only path feasibility, all collision pairs, rotor/servo reserve and accepted parameters |
| 3 | Independently validate coupled plant/predictor | FK/Jacobians, full 9x9 M, gravity/bias/acceleration parity, actuator and contact tests |
| 4 | Implement shared trajectory/contact logic and competent baseline B | Hover and no-wall motion checks, declared coupling omissions and tuning budget |
| 5 | Update coupled NMPC N for the clarified task | No alignment/force-setpoint objective; physical contact/path/hover development checks |
| 6 | Run matched B/N trials and recordings | Full metric table, same-run logs and synchronized comparison videos, including failures |
| 7 | Freeze candidate and nominal gate; assess timing | Revised held-out gate, honest deadline accounting and delayed-actuation trials |
| 8 | Revalidate observer/SAC interfaces and train/evaluate D/L | Zero-correction parity, shared sensing, task-correct reward, paired held-out results |
| 9 | Final report and reproduction | Evidence-linked conclusions, limitations, raw recordings, manifest and clean-run instructions |

No code is changed by this planning revision. Another implementation agent may be active; inspect its current files and coordinate before editing the same modules. Do not stop running jobs or delete checkpoints simply because they are now historical; identify what task/model each job uses and handle it explicitly.

### 11.1 Independent checks

- Full inertia/COM accounting, inertial-frame transforms and fixed-body reduction.
- FK and all three joint columns against independent MuJoCo quantities and finite differences, including nonzero base attitude.
- M symmetry/positive definiteness, nonzero coupling blocks, bias/acceleration agreement at simultaneous joint/body motion.
- Gravity power, velocity-bias power and momentum checks in appropriately isolated configurations.
- Contact material velocity, virtual power, friction dissipation and moving/multiple-contact geometry where relevant.
- Collision coverage including adjacent/fixed bodies where default engine filtering skips pairs. A sampled grid is not a continuous certificate without a valid relative-motion bound and verified coverage.
- Rotor signs, feasible simultaneous wrench, motor response and servo saturation.
- Contact onset, sliding/reversal, loss/re-impact and timestep refinement for both peaks and impulse.
- Solver cost/constraint/slack parity, finite-difference derivative checks with appropriate step study, first-failure handling and done behavior.
- Evaluator counterexamples: completed-but-drifting base, detached tool tracing the curve, partial path, projected false contact, and missed deadlines.

## 12. Research basis and limitations

Use the audited [SOURCE_TECHNICAL_REFERENCE.md](SOURCE_TECHNICAL_REFERENCE.md) and original PDFs. The following are adaptations, not claims of an exact published implementation:

| Source | Adopt | Do not assume |
|---|---|---|
| P10, reference Section 3 | Jacobian-based kinetic energy and articulated coupling; proper quasi-velocity treatment | Its special two-joint task-flatness/COM design condition |
| P08, Section 4 | General n-joint model, distinction between COM and base coordinates, task feasibility | Universal higher-DOF task inversion or contact stability from free-flight flatness |
| P06, Section 8 | Arm/base coupling and trajectory-control concepts | Printed inconsistent blocks/gains or a transferable global quaternion proof |
| P12, Sections 12.3–12.8 | Moving-arm contact trajectories with stabilized base; hybrid motion/contact ideas and disturbance estimation | Its omnidirectional actuation, delta-arm geometry, independent six-axis FT sensor or light/slow-arm approximation |
| P01/P09, Sections 9–10 | Interaction-aware prediction, frames and actuator feasibility | Arbitrary six-axis body-wrench authority on a conventional quadrotor |
| P03, Section 7 | Structured learned observations and history | Free-flight symmetry after introducing a wall, or automatic learned superiority |

P12 supports the task concept: its moving delta arm executes a triangular contact trajectory with the base at a fixed reference. Our serial arm, propulsion and sensing differ. Adapt the principle, re-derive the equations, and validate our own system. The comparison tests a hypothesis; no paper guarantees that our coupling-neglecting baseline must become unstable.

## 13. Documentation, remaining inputs and definition of finished

Reconcile `CLAUDE.md`, `TODO.md`, `Findings/STATUS.md`, model-generation code, references, constraints, reward, evaluator and media labels to this plan before reporting the new task complete. Use named model versions; retain `am_*` code where appropriate instead of an unnecessary directory rewrite. No Pelican/ROBOTIS asset migration is required.

Open inputs needing evidence are: accepted mount/contact geometry; component mass/inertia; joint operational limits; fixed wrist angle; rotor and servo data or declared assumptions; contact/friction calibration; sensor noise/delay; feasible path domain; contact-maintenance policy; and numerical success/timing criteria. Each has a recorded source, assumption, validation and freeze status. Do not silently inherit old probe-specific parameters or assert hardware fidelity from simulation consistency.

Every experiment stores source/config/model/solver hashes, command, software versions, hardware/workload, random seeds, raw state/force/input/timing logs, summary and recordings. The original v2 33 cm-probe results remain explicitly labeled historical; corrected geometry and task require new validation.

**Finished means:** the actual directly mounted arm end effector tracks complete wall trajectories in sustained physical contact; the drone holds its fixed position reference within declared tolerances by active control; physical limits and timing claims are independently evaluated; matched B/N comparisons and real synchronized recordings show the measured outcome without manufactured baseline failure; and subsequent N/D/L research results are reproducible under the revised task contract. No claim of perfect tracking, guaranteed instability without coupling compensation, or hardware readiness is made without supporting evidence.
