# Free-flight Lissajous: coupled vs decoupled control — what to say

## The demo in one sentence

A 1.52 kg quadrotor with a 3-joint arm hanging straight down flies a Lissajous figure in open air for 30 s. The **same plant, same reference, same sensor noise, same initial state** are given to two controllers: a coupled NMPC that models the drone–arm interaction, and a conventional decoupled controller that does not.

## Why this setup isolates the thing we care about

The arm is **not commanded to move**. It holds its hanging pose the whole flight. So nothing in this demo is about arm trajectory tracking — the only thing happening is that **a 0.47 kg articulated mass hangs 400 mm below the vehicle and swings as the drone manoeuvres**.

That swinging mass pushes back on the airframe. Three distinct effects:

1. **Inertial reaction** — accelerating the base accelerates the arm, and the arm pushes back.
2. **Centre-of-mass offset** — the system COM is not at the base origin, and it moves as the arm deflects. Thrust applied at the rotor plane therefore produces a moment about the true COM.
3. **Gyroscopic / Coriolis coupling** — base rotation and joint motion interact.

All three live in the off-diagonal blocks of the mass matrix, `M_bq`, which couple base acceleration to joint acceleration.

## What each controller knows

| | Coupled NMPC (**N**) | Decoupled baseline (**B**) |
|---|---|---|
| Model | full 9×9 mass matrix including `M_bq`, gravity and velocity bias, rotor lag | drone as a rigid body of the correct total mass; arm handled separately in joint space |
| Horizon | 10 stages × 20 ms, optimises all 7 inputs jointly | none — instantaneous feedback |
| Rate | 50 Hz | 200 Hz (4× faster) |
| Knows the arm reacts on the base | **yes** | **no** |

B is not a straw man: it has the correct total mass, gravity compensation, a nominal locked-posture inertia, the same sensors, the same actuator limits, and it runs **four times faster**. Its gains came from a tuning budget at least as large as N's. The *only* structural thing it lacks is the coupling term.

## What we implemented to make this demo possible

The research plan deliberately pins the drone's position reference to a fixed hover point — the inspection task is "hold still and move the arm". To fly a trajectory we added three things:

1. **A moving base reference.** The NMPC already carried the base reference as a per-stage parameter, so it could already accept a moving target. What it lacked was a base *velocity* reference — base velocity was being driven to zero, which fights a moving target and causes lag. We appended a 3-element velocity reference to the parameter vector.
2. **An analytic periodic task.** `LoopTask` drives the base along a Lissajous figure `y = A sin(n_y θ)`, `z = C sin(n_z θ + φ)` and the arm around a circle of the moving base, both off one shared clock. A single smoothstep envelope ramps the rate 0 → 1 → 0, so the motion starts and ends at rest while still completing exactly the requested number of laps.
3. **A reference-relative failsafe.** The shared loss-of-control monitor measured position error against the *fixed* hover point, so a drone legitimately flying a large figure tripped the 250 mm abort while perfectly on reference. It now measures against the current reference.

Everything defaults to the previous behaviour, and we verified that by re-running the original wall-contact benchmark: **the results came back bit-for-bit identical**, so none of the existing validated results were disturbed.

## The result

N holds the moving reference to about **0.13 mm RMS** for the full 30 s. B diverges and hits the 40° loss-of-control failsafe within about 5 s.

The honest reading: **this is a modelling failure, not an actuation failure.** When B loses control, N is using roughly half the available thrust and a fifth of the joint-rate limit. There is plenty of control authority left — B simply does not know the arm is pushing back, so it commands the wrong thing.

## Two caveats to state before you are asked

- **B was tuned for the original fixed-hover wall task and was never retuned for free-flight manoeuvring.** The defensible claim is "a competent decoupled controller tuned for the hover task diverges here", not "no decoupled controller could ever do this".
- **This demo is outside the frozen research plan** and is not covered by the 30-case acceptance gate. It is a capability demonstration, not validated evidence for the thesis.
