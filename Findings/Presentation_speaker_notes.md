# Speaker notes and background for the 12-slide weekly progress deck
Written 2026-11-04 (evening EST). For Sahil. Sources: the slide PDF, and Ashish's repo (`oct4th-updates`: READMEs, code headers, LITERATURE.md). I read those; I did not run his code, so his numbers are as reported, not independently checked.
Rough timing: 12 slides, about 8-10 minutes, then questions.

---
## 0. The story in 30 seconds (say this first, or if you are cut short)
"We built a drone with a soft arm in MuJoCo. A model-predictive controller (NMPC) that knows where the arm is going flies it to within about 1 millimetre, 10-20x better than the geometric controller. When the arm holds an unknown load, we estimate the load's mass with a Kalman filter from physics (22.8 cm error down to 2.1 cm). What is left is the soft arm swinging, and we use reinforcement learning to correct the NMPC's model for that: 0.77 cm against 2.2 cm. It is worse with no load and one policy crashed twice, so we are fixing that next."

---
## 1. Slide-by-slide notes

### Slide 1 - Title
who, what. One line: "This week we moved from last week's rigid arm to a soft arm, and added loads, random arm motion, literature work, and RL."

### Slide 2 - This week at a glance (5 cards)
Walk the cards left to right, one sentence each:
1. Random arm motions: arm moves to a new pose every 1-2.5 s; NMPC error 0.13 cm.
2. Payload: unknown load in the gripper; 0.4 kg gives 22.8 cm without adaptation, 2.1 cm with our estimate.
3. Various trajectories: line, spiral, zigzag, random, zigzag-into-spiral; no crashes in 160 flights.
4. Literature: we replicated L1-NMPC (97-99% error reduction in a paper-like test) and built the newest 2026 method as a baseline.
5. Deep RL: RL correcting the NMPC's model, 0.77 cm vs 2.20 cm.
Careful: "0 crashes in 160 flights" is task 3 (no RL). Task 5 had 2 crashes in 120 flights; you show that on slide 11. Do not let the audience mix them.

### Slide 3 - Task 1: random arm motions
"The arm picks a new random pose every 1-2.5 s (0.5-1 s for 'fast'). Both controllers are told where the arm is and where it is going. The geometric controller reacts to the current error; the NMPC predicts 0.5 s ahead including the arm's planned motion, 50 times a second."
Numbers: straight line 0.32 cm (geometric) vs 0.06 cm (NMPC). 160 flights, 4 kinds of path, no crashes.
Soft arm: each joint is a spring and light damper pulling towards its planned angle, like a real servo; it lags, sags under load and swings.
Why it matters: the arm moving shifts the centre of mass and pushes back on the drone; that coupling is the whole difficulty.

### Slide 4 - Task 2: unknown payload
"An unknown load sits in the gripper. The controller is never told its mass. A Kalman filter works it out every 0.1 s from the robot's momentum, with no training, and the NMPC's model updates with it."
Numbers: zigzag 0.4 kg: 22.8 cm to 2.1 cm. Final estimate within 0.001 kg. 480 flights, 0 crashes.
Honest point on the slide: even if the true mass is known, 1.6 cm remains, because the soft arm swings and the NMPC's model assumes the arm follows its plan. That leftover is what task 5 attacks.

### Slide 5 - Task 3: various trajectories
"Same comparison across path types. The NMPC stays at 0.06-0.13 cm; geometric is 0.3-2.4 cm. With the fast arm the NMPC is 0.20-0.24 cm."
Numbers are RMS error of the true centre of mass after the first 3 s, mean of 5 random arm motions never used for tuning.
Why it matters: shows the result is not one lucky path.

### Slide 6 - NMPC implementation and disturbance rejection
 "The NMPC computes thrust and three moments over a 0.5 s horizon, solved 50 times a second with IPOPT. Its model includes the arm's planned motion and the load estimate. We push the drone three times with 8 N for 0.25 s, without telling either controller."
Table: largest error 31-41 cm (geometric) vs 17-23 cm (NMPC); back within 1 cm in 2.5-2.8 s vs 0.7 s. Neither crashed.
In the video: the blue dots are the NMPC's predicted next 0.5 s.

### Slide 7 - Task 4: state of the art
"Four papers matter. Hanover 2022 (L1-NMPC): we replicated it. Yadav 2026: the newest method, closest to ours, built as our baseline. Kubel 2026: a neural network inside the MPC, no arm. Singh 2026: RL flies an aerial manipulator with loads and no MPC. The gap: nobody trains the correction inside an MPC's model with RL on the flight's result, and nobody has a soft arm swinging an unknown load."
Phrase to use: "we found no paper that does this", not "first ever".

### Slide 8 - L1-NMPC falls short on our task
"L1 works in the paper's setting, where the arm is still and the extra mass is in the body. With a load in a moving soft arm it barely helps: at 0.3 kg zigzag 16.3 cm to 15.6 cm, while the Bayesian estimate gives 1.1 cm."
Three reasons on the slide: it fights the load with the rotors (at their limit 23% of the flight); its filter lags the moving arm; it cannot tell the arm's joints about the load.

### Slide 9 - Our novel idea
Say: "Newest work trains a correction to predict the next 0.02 s well. We train it with RL on how well the robot actually flies. The NMPC still computes every command and keeps its limits; the RL policy only corrects the model. The load comes from physics (the Bayesian estimate), so RL only has to learn what physics leaves: mostly the swinging arm."
Why it is different: a better one-step prediction is not always a better flight; the controller only cares about the errors that change what it does.

### Slide 10 - Task 5: our method in flight
Say: "Left video: the 2026 learned model, about 7 cm. Right: ours, under 1 cm. Second video: Bayesian estimate alone versus ours, under 1 cm apart, so read the numbers printed on the screen."
Facts: SAC, 3 seeds of 150,000 steps; checkpoints chosen on separate validation flights, never on test flights; the policy never sees the load.

### Slide 11 - Results on 120 test flights
Say: "At 0.4 kg: ours 0.77 cm, Bayesian alone 2.20 cm (65% lower), learned-model baseline 7.74 cm (10x lower), even below the NMPC given the true mass (1.66 cm)."
Then say the weak points yourself (before anyone asks): worse with no load (0.39 vs 0.16 cm), and one of the three policies crashed in 2 flights (0.4 kg, fast arm).
Why it can beat "true mass known": the true mass fixes the weight; the swing of the arm is a different error, which the policy learned from the arm's angles and speeds.

### Slide 12 - Limits and next steps
Limits: no-load penalty; 2 crashes; stiffness is our choice. Next: remove the crashes and the no-load penalty and train more seeds; test with sensor noise and thrust error; stage 2 contact inspection with reading quality as reward; bring back wall pressing with videos that show the measurement.

---
## 2. How each piece works (what / how / why)

### Simulation and robot
MuJoCo physics: 1.52 kg quadrotor with a 4-joint arm (Ashish's model; our own repo's arm has 3 joints; see section 5). The "soft arm": each joint torque = Kp (planned angle - angle) + Kd (planned speed - speed) + arm weight. Kp = 6, 6, 3, 1 N*m/rad from shoulder to wrist; this is the team's choice, not a motor datasheet. Flights last 15 s; errors are measured after 3 s, on the true centre of mass (CoM).

### Geometric controller (the baseline)
What: the standard SE(3) tracking controller for quadrotors (Lee, Leok, McClamroch 2010 style). In the repo: `quadrotor_sim.SE3Controller`, plus thrust-offset compensation, gains: position loop natural frequency 2.2 rad/s, damping 0.9; attitude loop 15 rad/s, damping 0.8.
How: (1) position loop: desired force = -Kx*position error - Kv*velocity error + weight + mass*reference acceleration; (2) the desired force direction defines the desired drone orientation; (3) attitude loop turns the orientation error into a moment; (4) thrust = force along the drone's up axis.
Why it is the baseline: it is the classic controller; it works on the whole rotation group so it has no singularities. Weakness: it only reacts to error that has already happened and has no model of the arm moving or of a load.
For arm tasks both controllers are told the arm's position and motion (Task 1 README). The repo has three geometric variants (arm model fixed / aware / aware plus feedforward, in moving_arm.py); ask Ashish which one the slides use.

### NMPC (nonlinear model predictive control)
What: at 50 Hz it solves an optimisation: choose thrust and three body moments for the next 25 steps of 0.02 s (0.5 s) so the CoM follows the reference with least effort, subject to actuator limits; apply the first command, then re-solve.
Model (centroidal dynamics, exact for any arm motion): state = CoM position, orientation quaternion, CoM velocity, body rates (13 numbers); input = thrust + 3 moments (4 numbers). The arm's effect enters through the CoM offset, the inertia about the CoM, and terms from the arm's angular momentum and its derivative (the coupling). The arm is NOT an optimisation variable (the wrist inertia is too small for a 50 Hz optimiser); its planned motion is given to the model and a 500 Hz joint servo tracks it.
Extras: correction terms F_hat, tau_hat and the mass m_hat appear as model parameters (this is where the load estimate and the RL correction go in); delay compensation pushes the state one period ahead; solver IPOPT through CasADi, warm started.
Why it wins: it uses the arm's planned motion in advance, so it cancels the disturbance before the error builds. The code is rebuilt from the concepts of the team's own coupled-NMPC project (acados version, 3-joint arm).
Cost: heavier compute than geometric; real-time feasibility on hardware not shown here.

### Bayesian load estimate (a Kalman filter, not a neural network)
Idea: linear momentum of the whole robot over a short window T gives, for any arm motion:
  impulse - m0*(change in CoM velocity of the unloaded robot + g*T) = m_load * (change in hand velocity + g*T)
The left side comes from measured state and the commanded thrust; the right side is the unknown mass times a known quantity. So m_load is the single unknown in a linear equation; a scalar Kalman filter estimates it every 0.1 s, gives an uncertainty, and reopens its covariance when the load changes (grab/release). The estimate goes into the whole model: thrust, CoM, inertia, arm torques.
Why: the recent papers learn a model then adapt its linear last layer; here the linear structure is known from physics, so nothing needs to be trained.
Limits: assumes commanded thrust equals real thrust (best case; an earlier test with thrust 5% low put the estimate off by 0.093 kg and the error at 3.7 cm); call it "new for our project", not "new to the field" (Park et al. 2023 already do mass estimation with Kalman filters).
Note: slide 2 and 9 say "Bayesian"; it is a Kalman filter (Bayesian estimator for a linear Gaussian problem).

### L1-NMPC (Hanover et al. 2022; replicated)
What: NMPC + an L1 adaptive layer. A fast state observer on [velocity, body rate] estimates what the model is missing (matched and unmatched disturbance); a low-pass filter turns that into a correction added to the NMPC's command. Equations 8-13 of the paper.
Our replication: 97-99% error reduction for loads up to 60% of the robot's mass in a paper-like test (arm still, extra mass in the body, circle); the paper reports over 90%. It is one simulated flight per load, not the paper's real 750 g quadrotor. Observer/filter settings are not published, so ours were tuned on separate validation flights (observer 10 rad/s, filters 1 and 2 rad/s, 500 Hz) and we added anti-windup, which the paper does not have; without it L1 crashed.
Why it fails on our task: it only sees the drone, so it cannot tell the arm's joints to hold the extra weight; the load moves the CoM and the arm sags; its filter lags a moving arm; it saturates the rotors.

### Yadav et al. 2026 learned-model baseline (arXiv 2605.14805)
What the paper does: an attention network reads the last 0.15 s of flight; its linear last layer predicts the MPC model's one-step error; that layer is updated in flight by a Kalman filter that reopens when the load changes. Real 2.2 kg quadrotor, rigid 2-joint arm, loads up to 500 g.
Our version (baseline, not replication): encoder with attention over time and signals, 3 layers, width 64, 4 heads, then a 16-feature linear decoder; trained on 60 flights (loads 0, 0.2, 0.4 kg, 20 each); correction used on the first 4 stages of the horizon, fades over 2, then zero.
Important honesty point: the paper's full in-flight update crashed 29 of our 120 test flights (and 2 of 6 validation flights), so the table uses the offline decoder with no in-flight update. This is a weaker version than the paper's; say "built from the paper" not "replicated". Part of its gap to ours is structural: it corrects only the first stages of the horizon.

### Our RL method (task 5)
What: SAC (soft actor-critic, off-policy, entropy-regularised) outputs 6 numbers at 10 Hz: CoM acceleration and body angular acceleration, added to the NMPC's model over the whole horizon (scaled to at most 0.5 m/s^2 and 5 rad/s^2). Zero action = NMPC + Bayesian mass. The correction moves half-way to the new value each step (about 0.15 s), so it is smooth.
Observation: tracking error, speed error, attitude error, body rates, the arm's distance from its plan and its joint speeds, the load estimate, the current correction, last 3 steps (27 features). It never sees the true load.
Reward: exp(-(error/2 cm)^2) per step, small costs on the correction and its changes, -50 for a crash.
Training: load 0-0.45 kg, random paths and arm motions never used for testing; 3 seeds x 150,000 steps; checkpoint chosen on 6 validation flights; test = the same 120 flights as task 2 (4 paths, 5 loads, 2 arm speeds, 3 arm motions).
Why RL and not supervised: the controller only needs the correction that improves the flight; RL optimises the flight result itself.
Why it can lose with no load: the policy always adds a small correction even when none is needed (0.39 vs 0.16 cm). A fix: penalise the correction more at zero load, or train on more no-load flights.

---
## 3. Literature: who did what, and where we fit
| Paper | One-line idea | Relation to us |
|---|---|---|
| Hanover et al. 2022 (RA-L, arXiv 2109.04210) | L1 adaptive layer on NMPC for quadrotors; >90% error reduction with payloads | Replicated (task 4); falls short with a moving soft arm and a gripper load |
| Yadav et al. 2026 (arXiv 2605.14805) | Attention network learns the MPC model's one-step error; linear last layer updated by Kalman filter in flight; real 2.2 kg drone with rigid 2-joint arm | Our state-of-the-art baseline (built, not replicated); closest to ours |
| Kubel et al. 2026 (arXiv 2604.14678) | Neural network inside the MPC model trained on prediction error with an energy penalty; tilting-rotor drone, no arm; 15% better than earlier Neural MPC on a circle | Same idea family; no arm, no load; trained on prediction not result |
| Singh et al. 2026 (arXiv 2606.16621) | PPO RL flies an aerial manipulator with 200 and 400 g loads, delay-based estimator below; no MPC | RL replaces the controller; ours keeps the NMPC. Also the reference for the end-to-end RL project |
| Ujjawal et al. 2026 (arXiv 2603.11638) | Transformer for arm/drone dynamics error, linear correction updated by least squares; ~41% better than best learned baseline | Idea of adapting only a small linear part online |
| O'Connell et al. 2022, Neural-Fly (Science Robotics, arXiv 2205.06908) | Learn features offline, adapt linear coefficients online | Background for the "adapt only the last layer" idea |
| FlyAware (arXiv 2601.22686) | Notes Kalman-based payload estimators can take tens of seconds to converge | The speed our estimator had to beat (ours: 0.1 s updates) |
| Park et al. 2023 | Intertwined Kalman filters for mass and inertia of an aerial manipulator | Prior art for mass estimators: why we say "new for our project" |
| Pannocchia & Rawlings 2003 | Disturbance models for offset-free MPC | Idea behind the earlier wall-pressing disturbance observer (archived) |
| Actuators 2025; arXiv 2606.08533; 2609.11661; 2307.03705 | RL-tuned admittance with NMPC (contact); meta-RL for payloads; contact-aware NMPC without force sensing; ultrasound RL with image-quality reward | Related; 2307.03705 is the precedent for rewarding the reading quality (our stage 2) |

The gap statement (safe wording): "Among the papers we read in full, none trains the correction inside an MPC's model with reinforcement learning on the result of the flight, and none models an arm that sags and swings under an unknown load."
Where we are weaker than the literature: they have real hardware (Yadav, Hanover); we are simulation-only with perfect sensors.
Newer RL algorithms noted but not used: CrossQ (more sample efficient than SAC), FastTD3/FlashSAC (need very high simulation throughput; ours is about 589 steps/s on 12 cores).

---
## 4. Likely questions and honest answers
1. **Why is the soft arm important?** Real servo arms are not perfectly stiff; it makes the arm lag, sag under load and swing, which a rigid-arm model misses. Caveat: the stiffness values are our choice, not from a datasheet.
2. **Why does RL beat "true mass known"?** True mass fixes the weight; the remaining error is the arm's swinging, which RL learns from joint angles and speeds. 0.77 vs 1.66 cm at 0.4 kg.
3. **Is the 10x over the learned model fair?** Partly: it is our own build of Yadav et al., simplified, and with the in-flight update switched off because the full update crashed 29 of 120 flights. Say that.
4. **Only 3 seeds?** Yes. Seed 2 crashed twice; the spread between seeds matters. All 3 seeds beat Bayesian-alone at 0.2, 0.3, 0.4 kg. Next step: more seeds.
5. **Why worse with no load?** The policy adds a correction when none is needed; planned fix above.
6. **Crashes?** 2 of 120 flights, one policy, 0.4 kg, fast arm, flights that Bayesian-alone completes. A crash is the worst outcome; must be fixed before calling it safe.
7. **Does it work on a real drone?** Not tested. Simulation, perfect sensors, commanded thrust = real thrust. Next steps include sensor noise and thrust error.
8. **Why SAC?** Named in the proposal; off-policy so sample efficient for expensive NMPC-in-the-loop flights (about 589 steps/s). CrossQ is a possible upgrade.
9. **Why does NMPC beat geometric?** It uses the arm's planned motion in advance and respects actuator limits; geometric only reacts.
10. **How fast is the NMPC? Real time?** 50 Hz in the simulation with IPOPT; hardware real-time not shown. Do not quote a solve time unless Ashish confirms one; I did not find it in the files I read.
11. **Why is the reward "tracking error" and not contact?** Stage 1. Wall-pressing/inspection is stage 2 (archived tag `archive/press-wall-task5`), not in this week's results.
12. **What is new compared with Yadav et al.?** RL on the flight's result instead of one-step prediction error; soft arm; load from physics. Say "we found no paper that does this".
13. **Is the Bayesian estimate new?** New for our project; mass estimators already exist (Park et al. 2023).
14. **Why 0.5 s horizon, 50 Hz?** Long enough to see the arm's planned motion and the path ahead, short enough for the optimiser to finish within 20 ms.
15. **Why error after 3 s?** To exclude the start-up transient; flights last 15 s.
16. **Why the push test?** To check disturbance rejection; 8 N for 0.25 s; the controllers are not told.
17. **Why did L1 fail here?** Section 2.
18. **Course label:** the slides say RBE 594; check that matches what you want to show.

---
## 5. If they ask about OUR own RL work (Sahil's separate project)
Not on the slides. Do not mix the numbers up with the deck's.
- Different approach: RL replaces the controller (PPO), through a geometric inner loop; 3-joint arm; no NMPC. Follows Singh et al. 2026 and Dimmig & Kobilarov 2024.
- Latest (R5, 2026-11-04): drone follows big 3D paths while the arm moves; PD with feedforward drops to about 300 mm RMS error and loses 4-5 of 20 starts; our RL policy survived 20/20 starts at 4-8 mm RMS on seeds 100-119, but on 50 fresh seeds one seed lost 1 start and mean RMS was 14-20 mm (typical start 3-4 mm, a few starts settle slowly). The 5 mm goal is not met yet. Evidence: `Findings/RL/03_endtoend_rl_design.md`.
- Compatible direction: the end-to-end RL work could later serve as an alternative to the NMPC, or be combined as a residual on top of it.
- Different units: our errors are millimetres of base position; Ashish's are centimetres of centre of mass, so the numbers are not directly comparable.

---
## 6. Glossary
- **CoM**: centre of mass of drone + arm + load. **RMS**: root-mean-square error over time.
- **SE(3)**: 3D position and orientation. **NMPC**: nonlinear model predictive control. **IPOPT**: the optimisation solver used.
- **Kalman filter**: recursive estimator that combines a model with noisy measurements. **L1 adaptive**: fast observer + low-pass filter adding a correction.
- **SAC**: soft actor-critic RL algorithm. **PPO**: proximal policy optimisation (the RL algorithm in our own project).
- **Soft arm**: joints are spring-dampers toward their planned angles. **Centroidal model**: dynamics written about the CoM, so the arm's reaction forces appear only as torque and inertia terms.
