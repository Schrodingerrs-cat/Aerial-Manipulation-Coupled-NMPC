---

# Background for Sahil: NMPC, learning and the hybrid, in questions and answers

Not for the slides. Read it before the talk so you can answer questions in your own words.

## NMPC (nonlinear model predictive control)

**What is it?**
A controller that plans. Every 20 ms it does five things:
1. Measure the current state.
2. Use a model of the robot to predict the next 0.2 s (10 steps of 20 ms).
3. Search for the sequence of commands that keeps the drone and the tip closest to their paths, with the least effort, without breaking any limit.
4. Apply only the first command.
5. Throw the rest away and plan again from the new measurement.

Re-planning every step is what makes it a feedback controller: if something unexpected happens, the next plan starts from where the robot really is.

**What does it control?**
Seven commands, all chosen together in one plan:
- 4 rotor speeds;
- 3 arm joint torques (yaw, shoulder and elbow; the wrist is fixed).

**What is in its model?**
23 states:
- the drone's position, orientation and their velocities;
- the 3 joint angles and their speeds;
- the 4 rotor speeds (motors cannot change speed instantly);
- the contact with the wall: a spring-like wall that pushes back when the tip presses in, plus friction.

It also models how the arm's motion pushes the drone and the drone's motion swings the arm (the coupling).

**What is it trying to minimise (the cost)?**
- tip error from the letter path;
- drone error from its hover point;
- how hard it pushes the wall compared with the 1 mm target depth;
- how much the commands change (smoothness).

**What can it not break (the constraints)?**
- rotor speed minimum and maximum;
- joint torque limits;
- joint angle limits;
- tilt limits.

A PID cannot respect these limits directly; NMPC plans inside them.

**Why "nonlinear"?**
The drone rotates, and the arm's geometry changes a lot as it moves. A model linearised around hover (as in linear MPC or LQR) becomes wrong exactly during big arm motions and contact. So we keep the full nonlinear model.

**Why not PID, or a drone controller plus a separate arm controller?**
- They react only after an error has appeared.
- They do not know that the arm's motion pushes the drone.
- The figure-8 video shows the result: the separate controllers run 4 times faster (200 Hz), and they still drift until they crash at 15 s. The NMPC, at 50 Hz, stays within 0.3 mm, because it sees the push coming in its prediction.

**How is it solved so fast?**
- We use acados, an open-source solver made for real-time optimal control.
- It uses sequential quadratic programming: repeatedly approximate the problem by a simpler quadratic one, solve that, and refine.
- Each new solve starts from the previous plan, shifted by one step, so only a few refinements are needed.
- On this PC one solve takes about 10 ms of its 20 ms budget. It misses the deadline on about 3.5 % of steps; when that happens, the previous plan is used.

**What are its weaknesses?**
- **It is only as good as its model.** Any of these make its predictions wrong, and it fails:
  - an unknown load;
  - slower motors (delay);
  - a softer wall;
  - weaker rotors (bias).
- It needs a lot of computing.

**What is the "disturbance estimate" (NMPC + disturbance estimate)?**
- An observer compares what the model predicted with what really happened and estimates an extra unknown force.
- The NMPC then includes that force in its model.
- It fixes slowly changing errors, such as a load.
- It does not fix errors that depend on the state, such as a softer wall (the extra force changes with depth) or slow motors (the error is in the timing, not a force).

## Learned controller (reinforcement learning)

**What is it?**
A neural network that replaces the model and the planner.
- Inputs: 300 numbers, 100 times a second (tip and drone errors, speeds, joint angles, measured contact force, over the last 4 steps).
- Outputs: 10 numbers (drone acceleration and turn rates, and the 3 joint targets).
- A simple inner loop turns those into rotor speeds and joint torques.

**How does it learn?**
- Reinforcement learning, with the PPO algorithm. It tries actions in simulation and gets a reward for tracing well, staying stable and touching lightly.
- Over 20 million steps (about 3.5 h on 30 cores of Turing) it improves the actions that earned more reward.

**Why does it cope with slow motors?**
- During training we randomly changed the load, the motor speed, the wind and the thrust in every episode (domain randomisation).
- So it learned one behaviour that works across all of them, without being told which one it is in.

**Weaknesses:**
- Less precise: about 1.9 mm against 1.2 mm.
- No guarantees about its limits.
- It struggles near the shoulder's torque limit (200 g load: 7 of 15).
- It only knows what it saw in training: it fails with motors 3 times slower.

## Hybrid: NMPC + learned correction

**What is it?**
- The NMPC still plans and flies the drone.
- A small network adds a correction to the NMPC's model:
  - 2 numbers in the small version: an extra force at the tip;
  - 9 numbers in the larger version: extra forces and torques on the drone and on every joint.
- The NMPC then plans with the corrected model.

**How is it trained?**
The same reinforcement learning, but the network's action is the model correction, not the motor command. The reward is how well the NMPC then traces.

**Why is it better than either one alone?**
- It keeps the NMPC's precision, limits and structure.
- The network only has to learn what the model gets wrong. On the soft wall, the predicted tip depth error drops from about 7 mm to 0.06 mm, and the tracing error from 17.8 mm to 2.1 mm.
- The larger version handles every disturbance we tested at 1.2–1.4 mm, including all at once, except motors twice as slow.

**What is the difference from the learned controller?**

| | Learned controller | Hybrid |
|---|---|---|
| Who flies the drone | the network | the NMPC |
| What the network outputs | the commands | a correction to the NMPC's model |

**Cost:** about 2 ms more computing per step.

## Likely questions

**Is this only simulation?**
Yes, MuJoCo. The sensor noise is set to motion-capture level (0.3 mm, 0.1°); we also tested 1 mm and 0.5°. The next step is hardware.

**Why is writing a good test?**
It needs millimetre accuracy while touching a surface, which is the same skill as an inspection probe or a marking or cleaning tool. Every letter shape also exercises the arm in different directions.

**Was anything tuned on the test runs?**
No. The test runs use separate random seeds and sentences that were never used for tuning or training.

**Why did force control not fix the soft wall?**
The error is inside the model:
- the NMPC still predicts the wrong depth for a given force;
- so it keeps choosing the wrong commands, whatever target you give it.
