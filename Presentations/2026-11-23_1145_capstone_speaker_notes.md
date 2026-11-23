# Capstone talk notes, 2026-11-23 (Ashish + Sahil, 15 min)

Sahil: slides 1-4 and 13-19. Ashish: slides 5-12. About 45 s per slide.

## 1. Title

[Sahil] Good morning. We are Ashish and Sahil. Our project is a drone with a robot arm, an aerial manipulator, that has to carry objects and touch surfaces. We start with why that is hard, then the load: how a drone can use it and weigh it. Then the contact: a model-based controller, a learned controller, and a mix of both, compared on one test, tracing letters on a wall. We finish with what failed and what is next.

## 2. Why: Drones That Carry and Can Write

[Sahil] Drones are good at looking. Many real jobs need touching: inspecting a bridge with a probe, marking or cleaning a wall, picking up an object. For that we put an arm on the drone. Four things then push it off course. First, the drone and the arm are coupled: every motion of one moves the other, so controlling them separately is hard. Second, a load makes the drone heavier and pulls it forward. Third, the arm pushes the drone every time it moves. Fourth, when the arm's tip touches a wall, the wall pushes back. Our goal is a controller that knows the load and the contact. Our test is tracing letters on a wall with the arm's tip: it needs millimetre accuracy while touching, the same skill as an inspection probe or a marking tool. We do not use a real pen; the tip is a small pad that presses about 1 millimetre into the wall, and the trace shows where it touched.

## 3. The Plan: Carry, Then Touch

[Sahil] This is the road map, in five steps. Carry, which Ashish leads: step one, a load hanging on a string: can its swing help the motion instead of hurting it? Step two, a load in the gripper: can the drone weigh it with its own arm? Step three, carrying an object through wind. Touch, which I lead: step four, a model-based controller, NMPC, that plans with one model of drone, arm and wall. Step five, learning: a learned controller, and a mix where learning only corrects the model. How we test. For carrying: pick up an object of 0.3 to 0.5 kilograms in flight and carry it through gusts up to 6 metres per second, 30 test flights per method. For touching: trace 5 sentences that together use all 26 letters, under 17 conditions such as loads, slower motors, wind and softer or stiffer walls, 15 tries each, with realistic sensor noise. No test flight was used for tuning.

## 4. Step 1: A Hanging Load Can Help the Motion

[Sahil] Before the arm carries anything, we started with the simplest load: 200 grams hanging on a 60 centimetre string under the drone. In simulation the string is a rigid link on a ball joint, and the drone measures the string's angle. The task: fly 2 metres and stop, with drone and load both at rest. Left: a normal drone that ignores the load. To avoid a big swing it must move slowly; it is settled after 4.35 seconds. Right: our planner knows the load. It swings the load forward on purpose, up to 41 degrees, and catches it, so both arrive at rest after 1.62 seconds, 2.7 times faster. Settled means the drone within 5 centimetres of the goal and the swing under 2 degrees. Prof. Rosendo asked a fair question: is this more efficient, or does it just spend more thrust? We measured rotor power, which grows with thrust to the power 1.5. On the same 5-second clock, the swing move uses about 6 percent more energy than the slow move, and its busiest rotor peaks at 1.6 times hover thrust, half of the rotor's maximum. But it finishes 2.7 times sooner, so the energy spent until the job is done is 55 percent lower, if the drone uses that time for the next move. So: faster, not free, and cheaper per move. With sensor noise and a 10 millisecond delay it still settles in 1.7 seconds on average.

## 5. Step 2: The Drone Must Know Its Load

[Ashish] Now a load in the gripper. The flight controller sets thrust from the drone's mass. If it does not know the load, it drifts: in our test flights the path error is 38.7 centimetres on average, against 19.1 when it is told the true mass, half. So how do we find the mass in flight? First we tried a Bayesian filter: it compares the rotor push with how the drone accelerates. But wind, drag or a weak rotor look like extra load to it, so it was only 70 percent accurate. Following Prof. Rosendo's suggestion we now use the arm itself as a scale, with an analytical formula: 96 percent accurate, about 6 grams off on a 150 gram load.

## 6. The Idea: Use the Arm as a Scale

[Ashish] Hold a bag at arm's length: your shoulder feels the weight times the lever arm, the horizontal distance from shoulder to bag. The drone's arm works the same way. Its joint motors measure torque, and the joint angles and link lengths, which we know exactly, give the lever arm. So the mass is the only unknown: torque divided by g times the lever arm. For an arm with several joints we use the product of exponentials with the frame on the drone body: chaining the joint rotations gives the tip position, and its Jacobian gives, for every joint, the torque that one kilogram at the tip would cause, called a. One least-squares fit over all joints, m equals the sum of a times tau over the sum of a squared, gives the mass. Analytical means one formula solved directly: no filter, no search, nothing learned. Likely question, what if the drone tilts or accelerates? The drone's tilt from its IMU tells us which way is down in the arm's frame, so g is rotated into the body frame. The arm's own weight is already in the model, so only the extra torque counts, and we average over a short window so arm motion and sensor noise average out.

## 7. Results: The Arm Weighs Well, and the Drone Flies Better

[Ashish] On 300 test flights never used for tuning, with motor lag, wind, sensor noise and model errors. Left: how close each method's mass estimate is to the true mass. Ours, after a one-time servo calibration, 96.4 percent. The Bayesian filter 88 percent even after calibration, 70 before. Middle: distance from the path. Not knowing the load, 38.7 centimetres. With our estimate, 19.8, almost as good as being told the true mass, 19.1. [Sahil] On our writing robot the same formula reads 99.4 grams for a 100 gram load and 199.6 for 200. With no load but rotors 10 percent weaker than modelled, it reads about zero, because the arm does not feel the rotors; a thrust-based estimate would read 170 grams. [Ashish] The formula also works backwards: it tells us the shoulder can hold at most 242 grams; at 200 grams it already uses 91 percent of its torque.

## 8. Step 3: Carrying an Object Through Gusts

[Ashish] Now a harder carry: pick up an object, for example a package of 0.3 to 0.5 kilograms, mid-flight and carry it through gusts up to 6 metres per second. Three things go wrong at once: up to a third more mass, wind pushing the drone, and the object swinging on the arm and pulling the drone over. Our method has three steps. One, weigh the object with the arm, as before. Two, with the mass known, the wind is the only unknown left in the drone's momentum balance, so we can measure it. Three, damp each arm joint for its real inertia, including the servo's gearbox. For a fair test we compare against three published controllers, L1 adaptive, INDI and PID, all tuned the same way on separate flights. This is a different, stronger robot: 1.52 kilograms with a 4-joint arm on 3 to 4 newton-metre servos.

## 9. Same Flight: a Published Controller Flips, Ours Finishes

[Ashish] The same test flight for both: the object is picked up at 5 seconds and the gusts reach 6 metres per second. Left, the published L1 adaptive controller, tuned for this task. With each gust it tilts further, the object swings, a rotor hits its thrust limit, the red rotor disc, and it flips at 14.1 seconds. The tuned INDI and PID controllers flip on this flight too, at 15.8 and 16.3 seconds. Ours, right, stays under about 30 degrees of tilt and finishes. Every controller was tuned the same way, on development flights never used for testing.

## 10. Results: Heavier Objects, Stronger Wind

[Ashish] Over 30 heavy-object test flights each, the published controllers finish about half; ours finishes 83 percent. A version of ours that is told the true wind and mass also gets 83 percent, so our estimates are as good as knowing the truth here. With a 0.3 kilogram object, ours still finishes at least 90 percent of flights in gusts up to 7.5 metres per second, about twice the others. The flights that fail have a rotor at its thrust limit.

## 11. Why the Arm Can Be Damped: the Servo's Gearbox

[Ashish] Damping the arm first looked impossible: in our old model the arm shook at around 1000 degrees per second and the drone crashed at 12 seconds. The reason was the model, not the idea. A real 3 to 4 newton-metre servo has a gearbox with a ratio of about 350, which adds an inertia of about 0.005 kilogram metres squared, many times the wrist link's. With that in the model, the same damping keeps the arm shaking at about 20 degrees per second and the drone finishes. Lesson: model the real hardware.

## 12. Better Than Giving the Others 40 % Stronger Motors

[Ashish] Last check: we gave the published controllers motors 40 percent stronger, without adding their weight, which favours them. They finish 70 to 73 percent of the heavy-object flights. Ours, on the standard motors, still finishes 83 percent. To be honest about the limit: the flights we lose fail at the rotor thrust limit, and even a controller told the true wind and mass loses them.

## 13. Step 4: Why One Model for Drone + Arm (NMPC)

[Sahil] Now the touch half. Here the drone flies a figure-eight while the arm draws circles 5 centimetres in radius, shown at real speed. Both controllers get the same drone, the same path and the same sensors. Left: the usual approach, a drone controller and a separate arm controller, each ignoring the other. It even runs 4 times faster, at 200 hertz. Every arm motion pushes the drone, the drone controller does not expect it, and you can watch the error grow lap after lap until it tilts past 40 degrees and crashes at 15 seconds, 24 centimetres off. Right: our NMPC, nonlinear model predictive control, with one model of drone and arm together. It knows each arm motion will push the drone, counters it in advance, and stays within 0.3 millimetres. WHAT is NMPC: every 20 milliseconds it measures the state, then uses the model to predict the next 0.2 seconds, 10 steps of 20 milliseconds, and searches for the commands, 4 rotor speeds and 3 joint torques, that keep the drone and the tip closest to their paths with the least effort, without breaking any limit: rotor speeds, joint torques, tilt. It applies only the first command, then measures again and re-plans. Re-planning every step is what makes it feedback. WHY a model: it sees the arm's push before it happens, it handles limits directly, and it can include the wall contact. WHY nonlinear: the drone rotates and the arm's geometry changes a lot, so a model linearised around hover would be wrong exactly when it matters. WHY NOT PID or separate controllers: they only react after an error appears, and they do not know the coupling, which is what you see on the left. HOW it runs: our model has 23 states, the drone, the arm joints, the rotor speeds and the contact. We solve it with acados, a fast optimisation solver: about 10 milliseconds per plan on this PC, inside its 20 millisecond budget. The COST: it is only as good as its model, which is the next slide.

## 14. The Test: Tracing Letters on a Wall

[Sahil] This is the touch test: the NMPC traces HELLO WPI, shown at double speed. Red is where the tip pressed, green the letters to follow. The drone flies along the wall while the arm draws the letters. Letters are 5 centimetres tall, the tip moves at 5 centimetres per second and presses 1 millimetre into the wall. The sensors are as noisy as a motion-capture system, about 0.3 millimetres and 0.1 degrees. Each condition is 5 sentences times 3 noise seeds, 15 tries. With nothing disturbing it, the NMPC finishes all 15 sentences, 1.2 millimetres from the letters on average. Here the model is exactly right: the NMPC predicts the tip depth 0.2 seconds ahead to within a quarter of a millimetre. That is our reference.

## 15. Where the NMPC Breaks: Real-World Conditions

[Sahil] The NMPC is only as good as its model, and the real world does not match the model. We tested what a real drone meets. An unknown load, here 100 grams: its model thinks the arm is empty, the drone tilts and drifts, and in 0 of 15 tries does it keep the tip on the wall. Delay in the motors, motors twice as slow as the model expects: it tips past 40 degrees and loses control, 0 of 15. A wrong contact model, a wall half as stiff: it finishes, but 13 millimetres off the letters and pressing 6 times harder, because 0.2 seconds ahead it predicts the tip 7 millimetres shallower than it really is. A bias, rotors 15 percent weaker than modelled: 13 of 15, 3.7 millimetres. Sensor noise alone is not the problem: with noise three times larger, 1 millimetre, it still finishes 15 of 15, at 2.1 millimetres. Adding a disturbance estimate to the NMPC fixes the loads, but not the slow motors and not the soft wall. We also tried pressing to a target force instead of a depth: the soft-wall error dropped from 13 to 6 millimetres, still a failure. We have not yet tested delay in the sensors themselves; slow motors are our stand-in for delay. This is where learning comes in.

## 16. Step 5a: A Learned Controller Replaces the Model

[Sahil] The first learning approach throws the model away. A neural network flies the drone directly. WHAT: 100 times a second it reads 300 numbers, the tip and drone errors, speeds, joint angles and the measured contact force over the last 4 steps, and outputs 10: how the drone should accelerate and turn, and where the 3 joints should go. A simple inner loop turns those into rotor speeds and joint torques. HOW it learns: reinforcement learning, trial and error in simulation with the PPO algorithm, 20 million steps, about 3 and a half hours on 30 cores of the Turing cluster. During training we randomly change the load, the motor speed, the wind and the thrust, so it learns to cope without being told. RESULT: with motors twice as slow, the NMPC with a disturbance estimate, left, loses control. The learned controller, right, traces the sentence, and finishes 14 of 15 in the test. The cost: it is less precise, about 1.9 millimetres instead of 1.2, it has no guarantees, and it struggles with the heaviest load.

## 17. Step 5b: Keep the Model, Learn Only Its Error

[Sahil] The second learning approach is different, and this is the key difference from the last slide. There, the network replaced the planner. Here the NMPC still plans and flies the drone, with its model, its limits and its guarantees; a small network only adds a correction to the model: an extra force at the tip, 2 numbers, that the NMPC includes in its prediction. It is trained the same way, by reinforcement learning, rewarded for tracing well. Same soft wall as before. Left: NMPC with a disturbance estimate, 17.8 millimetres off and pressing hard. Right: with the learned correction, 2.1 millimetres, all 15 sentences, and a light touch. Why it works: its prediction of the tip depth 0.2 seconds ahead is now off by 0.06 millimetres on average, instead of 7. A larger correction, 9 numbers that can push on the drone and on every joint, handles even more, as the next table shows. The cost: about 2 milliseconds more computing per step.

## 18. All Controllers on the Same Tests

[Sahil] Same tests for every controller. Each cell: tries finished, and the average distance of the tip from the letters. Green: all finished within 3 millimetres. Yellow: some failed, or 3 to 5. Red: none finished, or more than 5. Load is 100 and 200 grams together, 30 tries; wind is a gust and a steady sine, 30 tries; all at once is a 100 gram load, motors 1.5 times slower and a gust together. The plain NMPC is the most precise when its model is right, but fails with a load, slow motors and all at once. The disturbance estimate fixes the load. The learned controller is the only one that copes with slow motors, but loses some heavy-load tries. The NMPC with the larger learned correction is the most accurate under every disturbance, about 1.2 to 1.4 millimetres, including all at once, but it still fails with motors twice as slow. Note: in the first run of the no-disturbance test, 2 of the disturbance-estimate tries were stopped by a stuck solver, not by control; the rerun finished 15 of 15.

## 19. What Failed, What We Learned, What Is Next

[Sahil] What failed. Much slower motors still beat every controller that uses a model; the learned controller copes, but not at three times slower. Pressing harder does not fix a wrong model: force control on the soft wall still missed. And the heaviest objects still fail when a rotor reaches its thrust limit. What we learned. If the drone knows its load, the load can even help: faster moves, better flights in wind. A correct model gives the most precise control. And the best use of learning was to correct the model, not to replace it. Next. Give the NMPC the load mass measured by the arm, which joins our two halves. Teach the learned correction to handle slow motors. And move to hardware, starting with measuring a real servo. Thank you, we are happy to take questions.

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
