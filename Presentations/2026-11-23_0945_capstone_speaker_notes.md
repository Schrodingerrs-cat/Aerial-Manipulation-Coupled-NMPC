# Capstone talk notes, 2026-11-23 (Ashish + Sahil, 15 min)

Sahil: slides 1-4 and 14-20. Ashish: slides 5-13. About 45 s per slide.

## 1. Title

[Sahil] Good morning. We are Ashish and Sahil. Our project is a drone with a robot arm that has to carry things and touch things. Today we show how we made it aware of its load, of the wind, and of the surface it touches, and what worked and what did not.

## 2. A Drone With an Arm Gets Pushed Off Course

[Sahil] A drone that only looks is easy. Once it carries or touches something, three things push it off course. A load makes it heavier and moves its balance point forward. The arm itself pushes the drone every time it moves. And when the pen touches a wall, the wall pushes back. A normal flight controller ignores all three. Our goal is a drone that carries loads and can write on a wall with the pen within about 2 millimetres of the letters.

## 3. Our Plan: First Carry, Then Touch

[Sahil] We split the project in two halves. Carry: first a load hanging on a string, then weighing a load in the gripper, then carrying a parcel through wind; Ashish leads that part. Touch: writing on a wall, first with a controller that plans with a model, then adding learning; I lead that part. Each half has its own test, and every number we show comes from test runs that were not used for tuning.

## 4. 1. A Hanging Load Can Help the Motion

[Sahil] Prof. Rosendo asked whether the swing of a hanging load can help instead of hurt. The drone moves 2 metres with 200 grams on a 60 centimetre string. Left: a normal drone that ignores the load has to move slowly to avoid a big swing; it is settled after 4.4 seconds. Right: our planner knows the load. It swings the load forward on purpose and catches it, so both arrive at rest after 1.6 seconds, with less than half the rotor effort. So a known load can work for us. That raises the next question: how does the drone know its load?

## 5. 2. The Drone Has to Know Its Load

[Ashish] A load in the gripper changes the drone's mass. The controller sets thrust from the mass, so if it does not know the load, it drifts. In our test flights the path error doubles: 38.7 centimetres without knowing the load, 19.1 when it is told the true mass. Before, we estimated the mass with a Bayesian filter that watches the whole drone. Wind and drag look like load to it, so it was only 70 percent accurate. Following Prof. Rosendo's idea, we use the arm itself as a scale, and it is 96 percent accurate, about 6 grams off on a 150 gram load.

## 6. The Arm Is a Scale

[Ashish] Hold a bag at arm's length: your shoulder feels its weight times the lever arm. The arm's motors measure their torque, and the joint angles and link lengths give the lever arm exactly. So the mass is the only unknown: torque divided by g times the lever arm. For an arm with several joints, the product of exponentials, with the frame on the drone body, gives the lever arm at every joint, and one least-squares fit over all joints gives the mass. No filter, no search, nothing learned.

## 7. It Weighs Well, and the Drone Flies Better

[Ashish] On 300 test flights never used for tuning, with motor lag, wind, sensor noise and model errors: our estimate is 96 percent accurate, the Bayesian filter 88 percent even after calibration. With our estimate the path error is 19.8 centimetres, almost the 19.1 of a drone told the true mass. [Sahil] On our writing robot the same formula reads 99.4 grams for 100 and 199.6 for 200. And when the rotors are 10 percent weak but there is no load, it still reads zero, where a thrust-based estimate would read 170 grams.

## 8. The Same Formula Gives the Arm's Limit

[Ashish] The formula also works backwards: for a given load it gives the torque the shoulder needs. The shoulder motor's maximum is 1 newton-metre, so the heaviest load it can hold is 242 grams, and 200 grams already uses 91 percent of it. [Sahil] Our writing tests agree: the learned controller, which must handle the load by itself, finishes only 7 of 15 sentences with 200 grams.

## 9. 3. Carrying a Parcel Through Gusts

[Ashish] Now a harder carry: pick up a 0.3 to 0.5 kilogram parcel mid-flight and carry it through gusts up to 6 metres per second. That is up to a third more mass all at once, wind pushing the drone, and the parcel swinging on the arm and pulling the drone over. Our method has three steps. First, weigh the parcel with the arm, as before. Second, with the mass known, the wind is the only unknown left in the drone's momentum balance, so we can measure it. Third, damp each arm joint for its real inertia, including the servo's gearbox. This is a different, stronger robot: 1.52 kilograms with 3 to 4 newton-metre servos.

## 10. Same Flight: a Published Controller Flips, Ours Finishes

[Ashish] The same test flight for both: the parcel is picked up at 5 seconds and the gusts reach 6 metres per second. Left, the published L1 adaptive controller, tuned for this task. With each gust it tilts further, the parcel swings, a rotor hits its limit and it flips at 14.1 seconds. The tuned INDI and PID controllers flip on this flight too. Ours, right, stays under about 30 degrees of tilt and finishes. Every controller was tuned the same way, on flights we never used for testing.

## 11. Ours Finishes More Flights in Stronger Wind

[Ashish] Over 30 heavy-parcel test flights each, the published controllers finish about half; ours finishes 83 percent. A version of ours that is told the true wind and mass also gets 83 percent, so our estimates are as good as knowing the truth here. With a 0.3 kilogram parcel, ours still finishes at least 90 percent of flights in gusts up to 7.5 metres per second, about twice the others. The flights that fail have a rotor at its thrust limit.

## 12. Why the Arm Can Be Damped: the Servo's Gearbox

[Ashish] Damping the arm first looked impossible: in our old model the arm shook at around 1000 degrees per second and the drone crashed. The reason was the model, not the idea. A real servo has a gearbox with a ratio of about 350, which adds an inertia of about 0.005 kilogram metres squared, many times the wrist link's. With that in the model, the same damping keeps the arm shaking at about 20 degrees per second and the drone finishes. Lesson: model the real hardware.

## 13. Better Than Giving the Others 40 % Stronger Motors

[Ashish] Last check: we gave the published controllers motors 40 percent stronger, without adding their weight, which favours them. They finish 70 to 73 percent of the heavy-parcel flights. Ours, on the standard motors, still finishes 83 percent. To be honest about the limit: the flights we lose fail at the rotor thrust limit, and even a controller told the true wind and mass loses them.

## 14. 4. To Touch, Plan With One Model of Drone and Arm

[Sahil] Now the touch half. Writing needs millimetres, and the arm's motion pushes the drone. Here the arm moves its pen 6 centimetres back and forth twice a second. Left: separate drone and arm controllers, the usual approach. The drone controller does not expect the arm's push, drifts 25 centimetres and loses control. Right: one model of drone and arm together. The planner knows each arm swing will push the drone and counters it in advance, so the drone stays within 5 millimetres. Every 20 milliseconds it predicts the next 0.2 seconds and picks the 4 rotor speeds and 3 joint torques; this is model predictive control. One plan takes about 10 milliseconds.

## 15. The Writing Test

[Sahil] This is the test: the planner writes HELLO WPI, shown at double speed. Red is ink, green the letters to follow. The drone flies along the wall while the arm draws. Letters are 5 centimetres tall, the pen moves at 5 centimetres per second and presses 1 millimetre into the wall, and the sensors are as noisy as a motion-capture system. With nothing disturbing it, the planner finishes all 15 sentences, 1.2 millimetres from the letters on average. That is our reference.

## 16. When Its Model Is Wrong, the Planner Fails

[Sahil] The planner is only as good as its model. Here it carries 100 grams it does not know about: it cannot keep the pen on the wall, 0 of 15. With motors slower than its model expects, it tips over, 0 of 15. A softer wall is subtler: it finishes, but 13 millimetres off the letters, pressing 6 times harder. We measured why: on the soft wall its model predicted the pen 7 millimetres shallower than it really was. Adding a disturbance estimate fixes the loads, but not the soft wall and not slow motors.

## 17. 5. A Learned Controller Copes With Slow Motors

[Sahil] The other approach learns instead of modelling. A neural network is trained by trial and error in simulation, reinforcement learning, for 20 million steps, while we randomly change the load, the motor speed and the wind, so it learns to cope without being told. 100 times a second it reads the errors and speeds and outputs where the drone should accelerate and where the joints should go. With slow motors, the planner with a disturbance estimate, left, tips over. The learned controller, right, writes the sentence, and finishes 14 of 15 in the test.

## 18. Best of Both: Learn Only What the Model Gets Wrong

[Sahil] Our hybrid keeps the planner and lets a small network correct only its model, adding a small extra force inside the model. The planning stays model-based. On the soft wall, the planner with a disturbance estimate, left, is 17.8 millimetres off and pressing hard. With the learned correction, right, 2.1 millimetres, all 15 sentences, and a light touch. The reason: its prediction of the pen depth is right again, off by less than a tenth of a millimetre instead of 7.

## 19. All Controllers on the Same Tests

[Sahil] Same tests for every controller, 15 tries each. Each cell: sentences finished out of 15, and the average pen distance from the letters in millimetres. The plain planner is precise when its model is right, but fails with loads, slow motors and the soft wall. The learned controller is the only one that survives slow motors. The bottom row is a larger learned correction, 9 numbers instead of 2, finished last night: it is the most accurate under every disturbance, about 1.2 to 1.4 millimetres, even with 200 grams and with all problems at once; only the plain planner with nothing disturbing it is slightly better. It still fails with slow motors, which is what we work on next.

## 20. What Failed, What We Learned, What Is Next

[Sahil] To be honest about failures: no controller survives motors three times slower than expected, and slow motors still defeat every planner-based controller. The heaviest parcels still crash 17 percent of the time at the rotors' thrust limit, and the planner misses its 20 millisecond deadline on about 3.5 percent of steps. What we learned: knowing the load matters, for the swing, the parcel and the wind. A correct model gives the most precise control. And learning works best as a correction to the model, not a replacement. Next, for November 27: feed the measured load mass into the writing planner's model, which joins our two halves; handle slow motors in the learned correction; and measure a real servo's gearbox inertia to move toward hardware. Thank you.

