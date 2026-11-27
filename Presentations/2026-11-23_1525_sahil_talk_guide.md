# Capstone talk guide for Sahil (2026-11-23 15:25)

Deck checked: `Presentations/Capstone_Carry_Touch_Write_with_arm_damping_loop.pdf` (19 slides, saved 15:23).

**Speaking order:**
- Sahil: 1–4.
- Ashish: 5–12.
- Sahil: 13–17.
- Ashish: 18–19.

**Contents:**
- Part A: what to fix in the deck.
- Part B: what to say on your slides, and the questions each one can trigger.
- Part C: the whole system as one story, so you can answer anything.

---

## Part A. Fixes for the PDF (every number was checked against the result files)

| Slide | Problem | Fix |
|---|---|---|
| 2 | "**Decoupled**: drone and arm move each other." This says the opposite of what you mean: the system is *coupled* (each moves the other). Controlling it as if it were decoupled is the mistake we show on slide 13. | "**Coupled**: drone and arm move each other." |
| 4 | Energy line still on the slide. | Delete "Energy: 6 % more over the same 5 s…". It is now in your notes (below). |
| 5 | "96 % accurate(analytical)" | Add a space: "accurate (analytical)". |
| 7 | Footnote says "Sahil: exact joint torques, writing robot." Wrong: the 99.4 g / 199.6 g / −0.2 g readings used the disturbance observer's *estimated* joint torques, with 1 mm / 0.5° sensor noise (`am3_mass_check.py`). | "Sahil: estimated joint torques with sensor noise, writing robot." |
| 8, 10, 12 | Still "parcel" (titles and text), but you asked for "object". | "Carrying an Object Through Gusts", "Heavier Objects, Stronger Wind", "heavy objects". |
| 12 | "7.5 vs 3–4.5 m/s" sits under the title "40 % stronger motors". These are the slide-10 numbers (standard motors for everyone), so it reads as if they were measured with stronger motors. | Ask Ashish. If they are not the stronger-motor numbers, delete that block or label it "(standard motors, from slide 10)". |
| 18 | The **"Slower motors" column was removed**. That is the one case where the learned controller wins (14/15) and where the "larger learned correction" fails (0/15). Without it, slide 16's claim has no row in the table, and the bottom row looks perfect. If someone asks "does anything fail?", the table hides the answer. | Put the column back (values below). |
| 18 | The row "NMPC + learned correction" (the small, 2-number one) was removed. But slide 17's result (17.8 → 2.1 mm) *is* that small correction, while the table shows the larger one (1.4 mm). Someone will ask why 2.1 became 1.4. | Re-add the row, or say it out loud: "slide 17 was the small correction; the table shows the larger one". |
| 19 | Old version, now contradicts slide 18 (see below). | Replace the text (below). |

**Slide 18: all rows and columns.** Each cell is tries finished, then the average tip distance from the letters.

| | No disturbance | Unknown load | Slower motors | Softer wall | Wind | All at once |
|---|---|---|---|---|---|---|
| NMPC | 15/15, 1.2 mm | 0/30 | 0/15 | 15/15, 13.4 mm | 30/30, 1.4 mm | 0/15 |
| NMPC + disturbance estimate | 15/15, 1.9 mm | 30/30, 3.4 mm | 0/15 | 15/15, 17.8 mm | 30/30, 1.9 mm | 0/15 |
| Learned controller | 15/15, 1.9 mm | 22/30, 2.5 mm | **14/15, 2.5 mm** | 15/15, 1.8 mm | 30/30, 2.3 mm | 8/15, 2.6 mm |
| NMPC + learned correction | 15/15, 1.8 mm | 30/30, 3.1 mm | 0/15 | 15/15, 2.1 mm | 30/30, 1.8 mm | 0/15 |
| NMPC + larger learned correction | 15/15, 1.3 mm | 30/30, 1.2 mm | **0/15** | 15/15, 1.4 mm | 30/30, 1.4 mm | 15/15, 1.3 mm |

**Slide 19: what contradicts the table, and replacement text.**

What is wrong in the current text:
- "Motors 3× slower, **or several problems at once: every controller**". Wrong: the larger learned correction finishes all at once 15/15 (slide 18).
- "Next: Larger learned correction for motor lag (**trained, testing now**)". Done: it was tested and still fails slower motors.
- "Learned controllers retrained at realistic noise". Done, and it did not help (worse on loads).
- "Next (**Nov 27**)": that is the RBE 595 date, not the capstone's.

Suggested replacement, short and memorable:

- **What failed:**
  - Much slower motors still defeat every controller that uses a model.
  - Pressing harder does not fix a wrong model (force control on the soft wall).
  - The heaviest objects still crash when a rotor hits its limit.
  - The NMPC misses its 20 ms deadline on about 3.5 % of steps.
- **What we learned:**
  - Know the load, and the load can help.
  - A correct model is the most precise.
  - Learn the model's error; keep the model.
- **What is next:**
  - Give the NMPC the load mass measured by the arm (joins the two halves).
  - Teach the correction to handle slow motors.
  - Measure a real servo; test on hardware.

The rest is correct as written:
- 15/15, 1.2 mm
- 0 of 15
- 13 mm
- 13 of 15 (rotors 15 % weaker)
- sensor noise alone 15/15 (2.1 mm at 1 mm noise)
- 14 of 15
- 7 → 0.06 mm
- 17.8 → 2.1 mm
- crash at 15 s
- within 0.3 mm
- 1.6 vs 4.4 s
- 96.4 / 88 / 69.9 %
- 38.7 / 19.8 / 19.1 cm
- 99.4 / 199.6 / −0.2 / 170 g
- 242 g

---

## Part B. Your slides: what to say, and the questions to expect



### Slide 1. Title (about 20 s)

**Say:**
>  Our project is a drone with a robot arm, an aerial manipulator, that has to carry objects and touch surfaces. We'll show how we made it aware of its load and of the surface it touches: first the carrying, then the touching, where we compare a model-based controller, a learned controller, and a mix of the two.

### Slide 2. Why (about 50 s)

**Say:**
> Drones are good at accessing space/location. Many real jobs need touching: inspecting a bridge with a probe, marking or cleaning a wall, picking up a package. So we put an arm on the drone, and four things then push it off course.
>
> First, drone and arm are coupled: every motion of one moves the other.
> Second, a load makes the drone heavier and shifts its balance forward.
> Third, every arm motion pushes the drone back.
> Fourth, when the arm's tip touches a wall, the wall pushes back.
>
> Our goal is a controller that knows the load and the contact. Our test is tracing letters on a wall. It needs millimetre accuracy while touching, which is the same skill an inspection probe or a marking tool needs.


### Slide 3. Plan (about 40 s)

> Two halves for the whole system and with total five steps. 

1st Carry, can a hanging load's swing help the motion, 
2n can the drone weigh its load with its own arm, and can it carry an object through gusts. 
4th Touch, is one model of drone plus arm enough to control contact, and 5th what can learning add.
>

--------------------**Say only if time:**
> The touch test is 5 sentences that together use all 26 letters, under 17 conditions, 15 tries each, with realistic sensor noise.

### Slide 4. Step 1, hanging load (about 60 s)

**Say:**
> Before the arm carries anything, we started with the simplest load: 200 grams hanging on a 60 cm string under the drone. The task: fly 2 metres and stop, with the drone *and* the load at rest.
>
> On the left is a normal drone that ignores the load. To avoid a big swing it has to move slowly, and it is settled only after 4.4 seconds.
> On the right, our planner knows the load. It swings the load forward on purpose and catches it at the goal, so both arrive at rest after 1.6 seconds: 2.7 times faster.
>
> We also compared energy. The swing move looks aggressive, and over the same few seconds it does use a little more energy, about 6 percent, because the rotors push harder for a short time. But it is done 2.7 times sooner, so the energy per move is about half, 55 percent less. Faster, and cheaper per move.

**If asked:**
- **"How did you compute the energy?"**
  - In the simulation we logged every rotor's actual thrust every 0.5 ms.
  - A rotor's ideal power grows with thrust to the power 1.5. This is momentum theory, the standard first-order rotor power model: P = T^1.5 / √(2ρA).
  - We integrated that power over time, divided by the power needed to hover for the same time, so the rotor size cancels.
  - Over the same 5 s window: normal drone 1.001 × hover, swing-aware 1.065 × hover, so about 6 % more.
  - Until the move is finished: 4.36 "seconds of hover" for the normal drone vs 1.96 for ours, so 55 % less.
- **"Is that real battery energy?"**
  - No. It is ideal rotor power. It ignores motor and electronics losses and blade drag.
  - Those losses also grow with thrust, so the "6 % more on the same clock" is, if anything, an underestimate.
  - The per-move advantage holds as long as the drone uses the saved time, for example for the next move.
- **"Peak thrust?"**
  - The busiest rotor peaks at 5.05 N for ours vs 3.07 N for the normal drone. Hovering needs 3.07 N per rotor; the rotor maximum is 10 N.
  - So ours peaks at about 1.6× hover, but still only half of what the rotor can give.
- **"Why not just fly faster with the normal drone?"**
  - We swept the normal drone's move time from 1.5 to 5 s and showed its best one (4.75 s).
  - Faster normal moves leave a big swing that takes longer to die out. At 1.8 s the swing reaches 65° and it settles only after 9.8 s.
- **"How is the load modelled?"**
  - The string is a rigid massless link on a ball joint, which assumes it stays taut. The planner also keeps the string tension above half the load's weight, so the plan never relies on a slack string.
  - The load is a 200 g point mass; the drone is 1.05 kg without the arm.
  - The drone is assumed to measure the string angle (an angle sensor or a camera).
- **"How is the motion planned?"**
  - It is a minimum-time trajectory optimisation (CasADi + IPOPT, 80 time steps) with the full drone + pendulum model.
  - Constraints: thrust 6–30 N, tilt ≤ 35°, altitude ±0.25 m, and both drone and load at rest at the goal.
  - A load-aware LQR feedback tracks that plan. An NMPC tracker gives the same result (1.66 s).
- **"Settled means?"** Drone within 5 cm of the goal and swing under 2° from then on.
- **"With noise?"** With sensor noise and a 10 ms delay it still settles in 1.7 s on average (10 seeds).
- **"What does it have to do with the arm?"** It is the first proof of the main idea: if the controller *knows* the load, the load stops being only a disturbance. The next question is how the drone gets to know its load, which is Ashish's part.

**Hand-over:**
> So knowing the load pays off. But how does the drone know its load? Ashish.

### Slide 13. Step 4, why one model (NMPC) (about 70 s)

**Say:**
>  Here the drone flies a figure-eight while the arm draws small circles. Both controllers get the same drone, the same path, the same sensors.
>
> On the left is the usual approach: a drone controller and a separate arm controller, each ignoring the other. It even runs 4 times faster. But every arm motion pushes the drone, the drone controller doesn't expect it, and you can see the error grow lap after lap until it tilts past 40 degrees and crashes at 15 seconds.
>
> On the right is our NMPC, with one model of drone and arm together. It knows each arm motion will push the drone and counters it before it happens:
>
> How it works: every 20 milliseconds it measures the state, predicts the next 0.2 seconds with the model, finds the best commands for the 4 rotors and 3 joints within all their limits, applies the first command, and repeats.

**If asked:** see Part C, section 3.

### Slide 14. The test (about 40 s)

**Say:**
> This is the touch test: the NMPC traces HELLO WPI, shown at double speed. Red is where the tip pressed, green the letters to follow. Letters are 5 cm tall, the tip moves at 5 cm per second and presses 1 mm into the wall. The sensors are as noisy as a motion-capture lab.
>
> With nothing disturbing it, it finishes all 15 sentences, about 1.2 millimetres from the letters. That's our reference.

**If asked:**
- **"Why 1.2 mm and not zero?"** Sensor noise (0.3 mm, 0.1°), the 20 ms control period, and the delay of one control step.
- **"How is 'distance from the letters' measured?"** Root-mean-square distance of the tip from the letter path, while it is writing.
- **"What counts as finished?"** The whole sentence is written without losing control and without losing the wall for good.

### Slide 15. Where the NMPC breaks (about 60 s)

**Say:**
> The NMPC is only as good as its model, and the real world never matches the model exactly. So we tested what a real drone meets.
>
> - An unknown load: 0 of 15. Its model thinks the arm is empty.
> - Motors slower than expected, which is a delay: 0 of 15. It tips over.
> - A softer wall: it finishes, but 13 mm off, pressing six times harder.
> - Rotors weaker than modelled, a bias: 13 of 15.
> - Sensor noise alone is not the problem: even with three times more noise it finishes 15 of 15.
>
> Each time, the cause is the same: its model is wrong. On the soft wall we measured it directly: 0.2 seconds ahead it predicts the tip 7 mm shallower than it really is.

**If asked:**
- **"Did you test sensor delay?"** Not separately. The controller always has one control step of delay (20 ms) built in; slower motors are our stand-in for a bigger delay.
- **"Why not just add an observer?"**
  - We did: the "disturbance estimate" (Part C, section 4). It fixes the loads.
  - It does not fix slow motors or the soft wall, because those errors are not a constant extra force.
- **"Force control?"** We also tried pressing to a target force instead of a depth. On the soft wall the error dropped from 13 to 6 mm, still a failure, because the error is inside the model.

### Slide 16. Step 5a, learned controller (about 60 s)

**Say:**
> The first learning approach throws the model away: a neural network flies the drone directly. 100 times a second it reads the errors, speeds, joint angles and contact force, and outputs where the drone should accelerate and where the joints should go. It learned by trial and error in simulation, reinforcement learning, while we randomly changed the load, motor speed, wind and thrust, so it learned to cope without being told.
>
> With slow motors, the NMPC with a disturbance estimate, on the left, loses control. while The learned controller, on the right, writes the sentence, and finishes 14 of 15 in the test.
>
> The price: it is less precise, about 1.9 instead of 1.2 mm, it gives no guarantees, and it struggles near the arm's torque limit.

### Slide 17. Step 5b, keep the model, learn only its error (about 60 s)

**Say:**
> The second approach is the opposite. The NMPC still plans and flies the drone, with its model and its limits. A small network only corrects the model: it adds an extra force at the tip that the NMPC includes in its prediction.
>
> Same soft wall. On the left, the NMPC with a disturbance estimate is 17.8 mm off and pressing hard. On the right, with the learned correction, it is 2.1 mm off, finishes all 15, and touches lightly. Its prediction of the tip depth is right again: off by 0.06 mm instead of 7.
>
> A larger version that can correct forces on the drone and every joint handles even more. Ashish will show all controllers side by side.

**If asked:**
- **"0.06 mm, really?"**
  - That is the average (bias) of the depth prediction 0.2 s ahead.
  - The typical scatter (RMS) is 0.36 mm, vs 6.9 mm before.
- **"How is this different from slide 16?"**
  - On slide 16 the network outputs the motor commands.
  - Here it outputs a correction to the model, and the NMPC still computes every command.

**Hand-over to Ashish (slides 18–19):**
> So: model-based is precise, learning is robust, and the best was to let learning fix the model.

---

## Part C. The whole system as one story

### 1. The robot and the simulation

**The robot (writing robot):**
- 1.52 kg quadrotor.
- 3-joint arm:
  - yaw, shoulder, elbow;
  - the wrist is fixed at 20°;
  - links 5 / 16 / 14 cm,
- At the tip: a 12 mm rubber pad.
- Rotors:
  - up to 10 N each;
  - speeds 180–1800 rad/s;
  - a motor can't change speed instantly (time constants 40 ms up, 60 ms down).
- Shoulder servo: 1 N·m continuous.

**The simulation:**
- MuJoCo 3.10 (a standard robotics physics engine).
- Physics steps every 0.25 ms.
- Contact between pad and wall is soft: about 600 N/m, so 1 mm of press gives about 0.6 N. Friction 0.15.

**Sensors.** The controllers never see the true state; they get noisy measurements. Three noise levels:

| Level | Position | Angle | Use |
|---|---|---|---|
| Nominal | 0.1 mm | 0.01° | |
| **Lab** (main results) | 0.3 mm | 0.1° | like a motion-capture system; the error drifts smoothly (50 ms correlation) instead of jumping every sample |
| Real / stress | 1 mm | 0.5° | white noise |

The sensors also give the joint encoders, rotor speeds and the contact force (±0.02 N).

**Test protocol:**
- 5 sentences that together use all 26 letters, × 3 noise seeds = 15 tries per condition.
- 17 conditions:
  - loads 100 / 200 g;
  - slower motors (×1.5, ×2, ×3);
  - weaker rotors (−10 %, −15 %);
  - joint damping ×3;
  - gust and sinusoidal wind;
  - softer / stiffer / slippery / grippy wall;
  - fast approach;
  - all at once.
- None of these runs were used for tuning or training.

### 2. Why one model (the story of N vs B)

- **The first instinct** is what most drones do: a normal flight controller for the drone, and a separate controller for the arm.
- **Why that fails.** The arm is 0.47 kg on a 1.05 kg drone. When it moves, its reaction pushes and tilts the drone, and its centre of mass shifts. The separate drone controller treats the drone as a rigid body with a fixed mass distribution, so it only reacts after the push has happened.
- **What we tried.**
  - We tuned that baseline (B) honestly: same sensors, same limits, 200 Hz (4× faster than our NMPC).
  - It still drifts and crashes whenever the arm moves fast (figure-8 video: 24 cm off, then it tips past 40° at 15 s).
- **The conclusion.** The coupling must be inside the controller's model. That led to one model of drone + arm, and to a controller that can use such a model: model predictive control.

### 3. NMPC: what, why, how

**What.** Every 20 ms the controller solves a small optimisation problem:
- "Find the next 0.2 s of commands (10 steps of 20 ms) for 4 rotor speeds and 3 joint torques that keep the tip on the letters and the drone at its hover point, with the least effort and smooth commands, without breaking any limit."
- It applies only the first command, then measures again and re-plans.
- Re-planning from the new measurement every time is what makes it a feedback controller (this is called receding horizon).

**The model (23 states):**
- drone position, orientation and their velocities;
- 3 joint angles and speeds;
- 4 rotor speeds (motor lag);
- the wall contact (spring-like pad with friction).

It also contains the full 9 × 9 mass matrix of drone + arm, which is exactly the coupling the separate controllers ignore. The model is written from the design numbers, not copied out of MuJoCo, so it is a real model, not the simulator itself.

**The cost (what it minimises):**
- tip error along the letters;
- drone error from its hover point;
- small terms on rates and on how much the commands change.

There is no force setpoint: it presses by following a depth reference 1 mm into the wall.

**Constraints (limits it plans inside):**

| Kind | Limits |
|---|---|
| Hard | rotor speed range, joint torque limits |
| Soft (penalised if broken) | tilt ≤ 15°, joint angle ranges, joint speed, contact force ≤ 4 N, approach speed, arm parts staying below the rotors |

**Why nonlinear?** The drone rotates and the arm's geometry and inertia change a lot. A linear model (linear MPC, LQR around hover) is wrong exactly during big arm motions and contact.

**Why MPC rather than PID or LQR?**
- It plans ahead (it sees the arm's push coming).
- It handles limits directly instead of clipping after the fact.
- It handles 7 coupled inputs at once.
- PID reacts after the error; LQR has no limits and is linear.

**How it is solved fast:**
- acados: an open-source real-time optimal-control solver.
- It uses SQP-RTI: one quadratic-programme refinement per control step, warm-started from the previous plan shifted by one step.
- Integration: 4th-order Runge–Kutta, 4 sub-steps per stage.
- We checked acados against IPOPT, a slow reference solver: same answer to 1e-9.
- Delay: the controller first predicts where the robot will be once the command takes effect (one step later), then plans from there.
- Timing on this PC:
  - median 9.6 ms of its 20 ms budget;
  - about 3.5 % of steps take longer than 20 ms;
  - in the simulation the computation does not slow the plant down, so this is a measured risk for hardware, not a failure we saw.
- Safety: if a solve fails, it retries once, then keeps the previous command.

**The weakness: it is only as good as its model.** The soft-wall measurement shows it: at 0.2 s ahead it predicts the tip 6.9 mm too shallow and the force 1.2 N too low. On the normal wall these errors are 0.24 mm and 0.09 N.

**Ablation.** An NMPC with the same settings but without the coupling terms in its model diverged about 0.4 s after the arm started moving. So the coupling, not just the "MPC" part, is what makes it work.

### 4. NMPC + disturbance estimate (NDF)

**What the disturbance is.** Anything the model doesn't know, expressed as extra forces and torques:
- 3 forces on the drone;
- 3 torques on the drone;
- 3 joint torques.

**How it is estimated.**
- A momentum observer, a standard robotics method, compares how the robot's momentum *should* change under the commands, according to the model, with how it *does* change.
- The difference is an estimate of the unknown force and torque. It is filtered with about a 0.1 s time constant.
- The NMPC then adds that estimate to its model. This is called offset-free MPC.

**A state filter.** We also added a filter that blends the model's prediction with each noisy measurement (50/50). Without it, the NMPC re-plans from raw noise and the rotor commands chatter. This version is called NDF.

**Why it fixes loads.** A load is a nearly constant extra weight and torque, which is exactly what the observer estimates. Results: 100 g and 200 g both 30/30.

**Why it can't fix slow motors or the soft wall.**
- Slow motors are an error in *timing*, not a force. By the time the observer sees it, the next commands are already wrong, and it gets worse the faster you move.
- A soft wall's extra force changes with depth. The observer holds it constant over the 0.2 s prediction, so the prediction is still wrong further ahead.

**Why not adaptive control or online system identification?** Possible next steps. They need the right parameter structure, for example knowing it is the wall stiffness. Learning the correction is the general version of that idea.

### 5. The learned controller (pure RL)

**Method.**
- Algorithm: PPO (Proximal Policy Optimisation), a standard, stable reinforcement-learning method for continuous control.
- Network: 2 hidden layers of 256.
- Training budget: 20 million steps, about 3.5 h on 30 cores of the Turing cluster.

**What the network sees and does.**
- Every 10 ms (100 Hz) it reads 300 numbers: tip and drone errors, velocities, joint angles, contact force, and the last 4 time steps.
- It outputs 10 numbers: drone acceleration and turn rates, and 3 joint targets.
- A simple inner loop turns those into rotor speeds and joint torques. That inner loop also has a disturbance observer.

**Reward.** For each time step:
- plus for tracing within a millimetre and for keeping contact;
- minus for tip error, pressing harder than 2 N, and using more than 80 % of a joint's torque;
- a large minus for crashing.

**Domain randomisation.** Every training episode has a random load, motor speed, thrust error, joint damping and wind. So it learns one behaviour that works across all of them, without being told which case it is in. That is why it copes with slow motors.

**Why PPO and not other methods?**
- We also tried SAC (off-policy) from scratch. After 6 M steps it never learned to make contact: 0 of 15.
- PPO from scratch failed as well. What worked was PPO built up in stages: first free flight, then writing, then randomisation, each stage starting from the previous policy.
- Imitation of the NMPC would only copy the NMPC, including its failures.

**Its weaknesses:**
- less precise (1.9 vs 1.2 mm);
- no guarantee of limits;
- near the shoulder's torque limit it fails: at 200 g it finishes 7–8 of 15, because its arm swings use up the shoulder's torque;
- it cannot handle what it never saw: motors 3× slower;
- fine-tuning it at the lab noise level made it worse on loads, so we kept the original.

### 6. NMPC + learned correction (the hybrid)

**Idea.** Keep everything good about the NMPC (model, limits, precision) and let learning fix only what the model gets wrong.

**How.**
- Every 20 ms a small network (2 × 256, PPO again) outputs a correction that is added to the NMPC's model for its prediction.
- The correction never goes to the motors directly; the NMPC computes every command.
- If the network outputs zero, the controller is exactly the NMPC + disturbance estimate (checked).

**Two versions:**

| Version | What it corrects |
|---|---|
| Small: 2 numbers | extra normal force at the tip, and a pitch torque |
| Larger: 9 numbers | forces and torques on the drone and on every joint |

**Training:**
- Rewarded for tracing well (same idea as above).
- Random loads and motor lag (capped at 1.4× slower), random wall stiffness (0.4–3.5×) and friction.
- Lab-level noise.

**Why it works on the soft wall.** The network learns the depth-dependent extra force the observer can't represent. The depth prediction error drops from 6.9 mm to 0.06 mm on average (0.36 mm RMS), and the tracing error from 17.8 to 2.1 mm.

**Results.**
- The larger version is the most accurate under every disturbance (1.2–1.4 mm), including all at once (100 g + motors 1.5× slower + gust).
- It still fails motors 2× slower, which was outside its training range: the lag was capped at 1.4×.

**Cost.**
- About 1–2 ms more per step.
- The arm moves more busily.

**Why this design?** It was the plan in the RBE 595 proposal: learning as a model correction, not a replacement. The results support it: precision from the model, robustness from learning.

### 7. The capstone link (Ashish's half)

- The arm-as-a-scale formula (product of exponentials) gives the load mass from joint torques. On our writing robot it reads 99.4 g for 100 g, using the observer's joint-torque estimate.
- **Next step:** feed that measured mass into the NMPC's model, so it *knows* the load instead of estimating it as a disturbance or learning it. That joins the two halves.

### 8. Quick answers to tough questions

- **"Is any of this on hardware?"** No, all simulation. Hardware is the next step, starting with measuring a real servo's inertia.
- **"Did you tune on the test?"** No. The test runs use separate seeds and sentences.
- **"Is the comparison fair?"** All controllers get the same plant, sensors, limits and tests. B runs 4× faster than the NMPC.
- **"What is the biggest open problem?"** Slow motors (actuator delay) for anything model-based, and the rotor thrust limit with heavy objects.
- **"Why simulation noise of 0.3 mm?"** It matches a lab motion-capture system. We also report 1 mm.
- **"Real-time on hardware?"**
  - The NMPC fits in 20 ms on a desktop CPU but overruns on 3.5 % of steps.
  - On an onboard computer that needs a shorter horizon or a faster solver.
  - The learned controller needs only about 1 ms.
