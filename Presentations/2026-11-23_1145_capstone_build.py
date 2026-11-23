"""Capstone deck v3, Mon 2026-11-23 (RBE 594, Ashish + Sahil), after Sahil's review of Ashish's merged deck (Capstone_Carry_Touch_Write_with_arm_damping.pptx).
Changes from v2: Sahil's title and slide-2 wording; plan slide as a plain pipeline (test details moved to the notes); hanging-load slide with the energy check
(results/payload/2026-11-23_1130_energy_ab_2m.json); Ashish's slides simplified, 'object' instead of 'parcel', his videos embedded; arm-limit slide folded into the
mass results; human-arm analogy; one-model slide uses the figure-8 run (AM3_LOOP_BvsN_figure8_2026-10-27); 'where it breaks' covers noise, delay and bias;
learned controller vs learned correction drawn as two different pipelines; table columns grouped (load = 100 g + 200 g, wind = gust + sine) from the Turing bench
files (Code/results/am3_write/bench_turing_1123/); bold is dark, red only for the one key phrase per slide.  Writes the .pptx and the talk notes (.md)."""
import os, re
HERE = os.path.dirname(os.path.abspath(__file__)); AS = os.path.join(HERE, "2026-11-23_1145_capstone_assets") + "/"
exec(open(os.path.join(HERE, "2026-11-18_1430_capstone_helpers.py")).read().replace("/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck/", AS))
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION

TNR, LGR = "Times New Roman", RGBColor(0xB4, 0xB8, 0xBC)
NOTES = []


def slide(title, notes):
    sl = prs.slides.add_slide(L["Title Only"]); sl.shapes.title.text = title
    for r in sl.shapes.title.text_frame.paragraphs[0].runs: r.font.name = TNR; r.font.size = Pt(34)
    sl.notes_slide.notes_text_frame.text = notes; NOTES.append((title, notes)); return sl


def lines(sl, x, y, w, h, paras, size=21, gap=14, color=DK, align=None):
    """Plain paragraphs; **text** = bold, !!text!! = bold red (one key phrase per slide)."""
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h)); tf = tb.text_frame; tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    for i, s in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph(); p.space_after = Pt(gap)
        if align: p.alignment = align
        for part in re.split(r"(\*\*.*?\*\*|!!.*?!!)", s):
            if not part: continue
            r = p.add_run(); f = r.font; f.name = TNR; f.size = Pt(size); f.color.rgb = color
            if part.startswith("**"): r.text = part[2:-2]; f.bold = True
            elif part.startswith("!!"): r.text = part[2:-2]; f.bold = True; f.color.rgb = RED
            else: r.text = part
    return tb


def note(sl, x, y, w, s, size=14): lines(sl, x, y, w, 0.6, [s], size=size, color=GR)


def video(sl, name, x, y, w, h):
    sl.shapes.add_movie(AS + f"v_{name}.mp4", Inches(x), Inches(y), Inches(w), Inches(h), poster_frame_image=AS + f"p_{name}.png", mime_type="video/mp4")


def img(sl, f, x, y, w=None, h=None): sl.shapes.add_picture(AS + f, Inches(x), Inches(y), Inches(w) if w else None, Inches(h) if h else None)


def bars(sl, x, y, w, h, title, cats, vals, colors, fmt, vmax):
    cd = CategoryChartData(); cd.categories = cats; cd.add_series("s", vals)
    ch = sl.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(x), Inches(y), Inches(w), Inches(h), cd).chart
    ch.has_legend = False; ch.has_title = True; ch.chart_title.text_frame.text = title
    t = ch.chart_title.text_frame.paragraphs[0].runs[0].font; t.size = Pt(18); t.bold = True; t.color.rgb = DK; t.name = TNR
    pl = ch.plots[0]; pl.gap_width = 55; pl.has_data_labels = True; dl = pl.data_labels; dl.number_format = fmt; dl.number_format_is_linked = False
    dl.position = XL_LABEL_POSITION.OUTSIDE_END; dl.font.size = Pt(18); dl.font.bold = True; dl.font.color.rgb = DK; dl.font.name = TNR
    for i, c in enumerate(colors):
        pt = pl.series[0].points[i]; pt.format.fill.solid(); pt.format.fill.fore_color.rgb = c
    va = ch.value_axis; va.maximum_scale = vmax; va.minimum_scale = 0; va.visible = False; va.has_major_gridlines = False
    ca = ch.category_axis; ca.tick_labels.font.size = Pt(16); ca.tick_labels.font.name = TNR; ca.tick_labels.font.color.rgb = DK; ca.format.line.color.rgb = LGR


def steps(sl, y, items, h=0.85, x0=0.55, w_total=12.25, size=19, fills=None):
    n = len(items); gap = 0.4; w = (w_total - gap * (n - 1)) / n
    for i, t in enumerate(items):
        x = x0 + i * (w + gap); box(sl, x, y, w, h, fill=fills[i] if fills else CARD)
        tb = lines(sl, x + 0.1, y + 0.1, w - 0.2, h - 0.15, [t], size=size, gap=0); tb.text_frame.paragraphs[0].alignment = PP_ALIGN.CENTER
        tb.text_frame.vertical_anchor = MSO_ANCHOR.MIDDLE
        if i: arrow(sl, x - gap + 0.05, y + h / 2, x - 0.05, y + h / 2)


# 1 title -------------------------------------------------------------------------------------------
sl = prs.slides.add_slide(L["Title Slide"])
sl.shapes.title.text = "Contact-Aware Control of a Drone With an Arm"
sl.placeholders[1].text = "Carry, Write, Inspect using an Aerial Manipulator\nAshish Sukumar and Sahil Gajera\nRBE 594 Capstone  |  November 23, 2026"
for ph in (sl.shapes.title, sl.placeholders[1]):
    for p in ph.text_frame.paragraphs:
        for r in p.runs: r.font.name = TNR
n1 = ("[Sahil] Good morning. We are Ashish and Sahil. Our project is a drone with a robot arm, an aerial manipulator, that has to carry objects and touch surfaces. "
      "We start with why that is hard, then the load: how a drone can use it and weigh it. Then the contact: a model-based controller, a learned controller, and a mix of both, "
      "compared on one test, tracing letters on a wall. We finish with what failed and what is next.")
sl.notes_slide.notes_text_frame.text = n1; NOTES.append(("Title", n1))

# 2 why ---------------------------------------------------------------------------------------------
sl = slide("Why: Drones That Carry and Can Write",
           "[Sahil] Drones are good at looking. Many real jobs need touching: inspecting a bridge with a probe, marking or cleaning a wall, picking up an object. For that we put an arm on the drone. "
           "Four things then push it off course. First, the drone and the arm are coupled: every motion of one moves the other, so controlling them separately is hard. "
           "Second, a load makes the drone heavier and pulls it forward. Third, the arm pushes the drone every time it moves. Fourth, when the arm's tip touches a wall, the wall pushes back. "
           "Our goal is a controller that knows the load and the contact. Our test is tracing letters on a wall with the arm's tip: it needs millimetre accuracy while touching, "
           "the same skill as an inspection probe or a marking tool. We do not use a real pen; the tip is a small pad that presses about 1 millimetre into the wall, and the trace shows where it touched.")
img(sl, "m_write_clean.png", 6.9, 1.55, w=6.0)
note(sl, 6.9, 4.25, 6.0, "Our robot tracing HELLO WPI on a wall (simulation)")
lines(sl, 0.55, 1.6, 6.1, 0.5, ["Four things push a drone with an arm off course:"], size=22)
lines(sl, 0.55, 2.3, 6.1, 3.3, ["**Coupled:** drone and arm move each other.", "**The load** makes it heavier and pulls it forward.", "**The arm** pushes the drone every time it moves.",
                                "**The wall** pushes back on the arm's tip."], size=22, gap=16)
box(sl, 0.45, 5.6, 12.45, 0.85, fill=CARD)
lines(sl, 0.7, 5.78, 12.0, 0.6, ["Goal: a controller that !!knows the load and the contact!!.  Test: trace letters on a wall."], size=22)

# 3 plan --------------------------------------------------------------------------------------------
sl = slide("The Plan: Carry, Then Touch",
           "[Sahil] This is the road map, in five steps. Carry, which Ashish leads: step one, a load hanging on a string: can its swing help the motion instead of hurting it? "
           "Step two, a load in the gripper: can the drone weigh it with its own arm? Step three, carrying an object through wind. "
           "Touch, which I lead: step four, a model-based controller, NMPC, that plans with one model of drone, arm and wall. Step five, learning: a learned controller, and a mix where learning only corrects the model. "
           "How we test. For carrying: pick up an object of 0.3 to 0.5 kilograms in flight and carry it through gusts up to 6 metres per second, 30 test flights per method. "
           "For touching: trace 5 sentences that together use all 26 letters, under 17 conditions such as loads, slower motors, wind and softer or stiffer walls, 15 tries each, with realistic sensor noise. "
           "No test flight was used for tuning.")
box(sl, 0.55, 1.65, 7.3, 0.55, fill=RED); lines(sl, 0.55, 1.72, 7.3, 0.45, ["CARRY: know the load"], size=20, color=WHITE, align=PP_ALIGN.CENTER)
box(sl, 8.25, 1.65, 4.55, 0.55, fill=DK); lines(sl, 8.25, 1.72, 4.55, 0.45, ["TOUCH: know the contact"], size=20, color=WHITE, align=PP_ALIGN.CENTER)
steps(sl, 2.45, ["**1** Hanging load", "**2** Weigh the load", "**3** Carry in wind", "**4** Plan with a model", "**5** Add learning"], h=1.25, size=22)
qs = ["Can the swing help the motion?", "Can the drone know its load?", "Can it carry an object through gusts?", "Is one model of drone + arm enough?", "What can learning add?"]
w = (12.25 - 0.4 * 4) / 5
for i, q in enumerate(qs): lines(sl, 0.55 + i * (w + 0.4), 3.95, w, 1.3, [q], size=20, color=GR, align=PP_ALIGN.CENTER)
lines(sl, 0.55, 5.4, 12.25, 0.8, ["Every result: test runs never used for tuning."], size=19, color=GR, align=PP_ALIGN.CENTER)

# 4 hanging load ------------------------------------------------------------------------------------
sl = slide("Step 1: A Hanging Load Can Help the Motion",
           "[Sahil] Before the arm carries anything, we started with the simplest load: 200 grams hanging on a 60 centimetre string under the drone. "
           "In simulation the string is a rigid link on a ball joint, and the drone measures the string's angle. The task: fly 2 metres and stop, with drone and load both at rest. "
           "Left: a normal drone that ignores the load. To avoid a big swing it must move slowly; it is settled after 4.35 seconds. "
           "Right: our planner knows the load. It swings the load forward on purpose, up to 41 degrees, and catches it, so both arrive at rest after 1.62 seconds, 2.7 times faster. "
           "Settled means the drone within 5 centimetres of the goal and the swing under 2 degrees. "
           "Prof. Rosendo asked a fair question: is this more efficient, or does it just spend more thrust? We measured rotor power, which grows with thrust to the power 1.5. "
           "On the same 5-second clock, the swing move uses about 6 percent more energy than the slow move, and its busiest rotor peaks at 1.6 times hover thrust, half of the rotor's maximum. "
           "But it finishes 2.7 times sooner, so the energy spent until the job is done is 55 percent lower, if the drone uses that time for the next move. "
           "So: faster, not free, and cheaper per move. With sensor noise and a 10 millisecond delay it still settles in 1.7 seconds on average.")
video(sl, "swing", 0.45, 1.45, 12.4, 3.36)
lines(sl, 0.55, 4.95, 12.3, 1.7, ["Setup: a **200 g** load on a **60 cm** string; fly 2 m and stop with drone and load at rest.",
                                  "Planning with the swing: at rest after !!1.6 s instead of 4.4 s!!.",
                                  "Energy: **6 % more** over the same 5 s, but **55 % less** per move, because it is done sooner."], size=20, gap=8)

# 5 know the load (Ashish) --------------------------------------------------------------------------
sl = slide("Step 2: The Drone Must Know Its Load",
           "[Ashish] Now a load in the gripper. The flight controller sets thrust from the drone's mass. If it does not know the load, it drifts: "
           "in our test flights the path error is 38.7 centimetres on average, against 19.1 when it is told the true mass, half. "
           "So how do we find the mass in flight? First we tried a Bayesian filter: it compares the rotor push with how the drone accelerates. "
           "But wind, drag or a weak rotor look like extra load to it, so it was only 70 percent accurate. "
           "Following Prof. Rosendo's suggestion we now use the arm itself as a scale, with an analytical formula: 96 percent accurate, about 6 grams off on a 150 gram load.")
img(sl, "ash_robot.png", 6.7, 1.5, w=6.2)
lines(sl, 0.55, 1.75, 5.9, 4.6, ["Without the load in its model, the drone drifts **twice as far** from its path.",
                                 "First try, a filter on the whole drone: wind looks like load, **70 %** accurate.",
                                 "Now: !!the arm as a scale!!, **96 %** accurate."], size=23, gap=26)

# 6 arm as scale (Ashish) ---------------------------------------------------------------------------
sl = slide("The Idea: Use the Arm as a Scale",
           "[Ashish] Hold a bag at arm's length: your shoulder feels the weight times the lever arm, the horizontal distance from shoulder to bag. The drone's arm works the same way. "
           "Its joint motors measure torque, and the joint angles and link lengths, which we know exactly, give the lever arm. So the mass is the only unknown: torque divided by g times the lever arm. "
           "For an arm with several joints we use the product of exponentials with the frame on the drone body: chaining the joint rotations gives the tip position, and its Jacobian gives, for every joint, "
           "the torque that one kilogram at the tip would cause, called a. One least-squares fit over all joints, m equals the sum of a times tau over the sum of a squared, gives the mass. "
           "Analytical means one formula solved directly: no filter, no search, nothing learned. "
           "Likely question, what if the drone tilts or accelerates? The drone's tilt from its IMU tells us which way is down in the arm's frame, so g is rotated into the body frame. "
           "The arm's own weight is already in the model, so only the extra torque counts, and we average over a short window so arm motion and sensor noise average out.")
img(sl, "f_human.png", 0.5, 1.5, h=3.6); img(sl, "f_poe.png", 5.9, 1.65, w=7.0)
lines(sl, 0.6, 5.25, 4.9, 0.5, ["Your shoulder", ], size=19, color=GR, align=PP_ALIGN.CENTER)
lines(sl, 5.9, 5.25, 7.0, 0.5, ["The drone's arm: same physics"], size=19, color=GR, align=PP_ALIGN.CENTER)
lines(sl, 0.55, 5.8, 12.3, 0.9, ["One joint: !!m = torque / (g · d)!!.   Several joints: the arm's geometry gives the torque per kg at each joint, and one fit gives m."], size=20)

# 7 results (Ashish) --------------------------------------------------------------------------------
sl = slide("Results: The Arm Weighs Well, and the Drone Flies Better",
           "[Ashish] On 300 test flights never used for tuning, with motor lag, wind, sensor noise and model errors. Left: how close each method's mass estimate is to the true mass. "
           "Ours, after a one-time servo calibration, 96.4 percent. The Bayesian filter 88 percent even after calibration, 70 before. "
           "Middle: distance from the path. Not knowing the load, 38.7 centimetres. With our estimate, 19.8, almost as good as being told the true mass, 19.1. "
           "[Sahil] On our writing robot the same formula reads 99.4 grams for a 100 gram load and 199.6 for 200. With no load but rotors 10 percent weaker than modelled, it reads about zero, "
           "because the arm does not feel the rotors; a thrust-based estimate would read 170 grams. "
           "[Ashish] The formula also works backwards: it tells us the shoulder can hold at most 242 grams; at 200 grams it already uses 91 percent of its torque.")
bars(sl, 0.4, 1.4, 4.4, 4.0, "Mass estimate accuracy, %", ["Ours", "Filter (calibrated)", "Filter"], (96.4, 88.0, 69.9), [RED, LGR, LGR], '0.0"%"', 118)
bars(sl, 4.9, 1.4, 4.4, 4.0, "Distance from path, cm", ["Unknown", "Ours", "True mass"], (38.7, 19.8, 19.1), [LGR, RED, DK], '0.0', 48)
lines(sl, 9.55, 1.7, 3.35, 4.2, ["Writing robot: 100 g reads **99.4 g**, 200 g reads **199.6 g**.", "Weaker rotors do not fool it.", "Limit: the shoulder holds at most **242 g**."], size=20, gap=20)
note(sl, 0.55, 5.75, 12.3, "300 test flights never used for tuning (motor lag, wind, sensor noise, model errors). Higher accuracy is better; lower distance is better.")

# 8 carry in wind (Ashish) --------------------------------------------------------------------------
sl = slide("Step 3: Carrying an Object Through Gusts",
           "[Ashish] Now a harder carry: pick up an object, for example a package of 0.3 to 0.5 kilograms, mid-flight and carry it through gusts up to 6 metres per second. "
           "Three things go wrong at once: up to a third more mass, wind pushing the drone, and the object swinging on the arm and pulling the drone over. "
           "Our method has three steps. One, weigh the object with the arm, as before. Two, with the mass known, the wind is the only unknown left in the drone's momentum balance, so we can measure it. "
           "Three, damp each arm joint for its real inertia, including the servo's gearbox. "
           "For a fair test we compare against three published controllers, L1 adaptive, INDI and PID, all tuned the same way on separate flights. "
           "This is a different, stronger robot: 1.52 kilograms with a 4-joint arm on 3 to 4 newton-metre servos.")
img(sl, "a_carry_ash.png", 6.75, 1.45, w=6.15)
note(sl, 6.75, 5.0, 6.15, "Ours carrying 0.4 kg through gusts (simulation)")
lines(sl, 0.55, 1.6, 6.0, 1.2, ["An object (e.g. a package) of 0.3–0.5 kg, picked up mid-flight, gusts up to 6 m/s."], size=21)
lines(sl, 0.55, 2.9, 6.0, 2.8, ["**1.** Weigh it with the arm.", "**2.** Mass known: !!measure the wind!!.", "**3.** Damp the arm joints."], size=23, gap=16)

# 9 vs published (Ashish) ---------------------------------------------------------------------------
sl = slide("Same Flight: a Published Controller Flips, Ours Finishes",
           "[Ashish] The same test flight for both: the object is picked up at 5 seconds and the gusts reach 6 metres per second. Left, the published L1 adaptive controller, tuned for this task. "
           "With each gust it tilts further, the object swings, a rotor hits its thrust limit, the red rotor disc, and it flips at 14.1 seconds. The tuned INDI and PID controllers flip on this flight too, at 15.8 and 16.3 seconds. "
           "Ours, right, stays under about 30 degrees of tilt and finishes. Every controller was tuned the same way, on development flights never used for testing.")
video(sl, "l1", 0.45, 1.45, 8.0, 4.5)
lines(sl, 8.75, 1.6, 4.15, 4.6, ["Published L1 controller: **flips at 14.1 s**.", "INDI and PID flip on this flight too.", "Ours: !!finishes!!, tilt under 30°."], size=21, gap=20)
note(sl, 0.55, 6.05, 12.3, "Same first test flight for both: 0.4 kg picked up at 5 s, gusts to ~6 m/s. Red rotor disc = at its thrust limit.")

# 10 carry results (Ashish) -------------------------------------------------------------------------
sl = slide("Results: Heavier Objects, Stronger Wind",
           "[Ashish] Over 30 heavy-object test flights each, the published controllers finish about half; ours finishes 83 percent. A version of ours that is told the true wind and mass also gets 83 percent, "
           "so our estimates are as good as knowing the truth here. With a 0.3 kilogram object, ours still finishes at least 90 percent of flights in gusts up to 7.5 metres per second, about twice the others. "
           "The flights that fail have a rotor at its thrust limit.")
bars(sl, 0.4, 1.4, 6.0, 4.1, "Heavy objects, gusts to 6 m/s: % finished", ["L1", "INDI", "PID", "Ours", "Told truth"], (53, 53, 47, 83, 83), [LGR, LGR, LGR, RED, DK], '0"%"', 105)
bars(sl, 6.6, 1.4, 4.6, 4.1, "0.3 kg: strongest gust, m/s", ["L1", "INDI", "PID", "Ours"], (3.8, 3.0, 4.5, 7.5), [LGR, LGR, LGR, RED], '0.0', 9.5)
lines(sl, 11.35, 1.7, 1.6, 4.0, ["!!83 %!! vs about 50 %", "**2×** the wind"], size=20, gap=24)
note(sl, 0.55, 5.75, 12.3, "Test flights not used for tuning. Left: 30 flights per bar (0.3–0.5 kg). Right: strongest gust with at least 90 % of 20 flights finished. Failures: a rotor at its thrust limit.")

# 11 armature (Ashish) ------------------------------------------------------------------------------
sl = slide("Why the Arm Can Be Damped: the Servo's Gearbox",
           "[Ashish] Damping the arm first looked impossible: in our old model the arm shook at around 1000 degrees per second and the drone crashed at 12 seconds. The reason was the model, not the idea. "
           "A real 3 to 4 newton-metre servo has a gearbox with a ratio of about 350, which adds an inertia of about 0.005 kilogram metres squared, many times the wrist link's. "
           "With that in the model, the same damping keeps the arm shaking at about 20 degrees per second and the drone finishes. Lesson: model the real hardware.")
video(sl, "armature", 0.45, 1.45, 8.0, 4.5)
lines(sl, 8.75, 1.6, 4.15, 4.6, ["Old model, no gearbox: the arm shakes at **~1,000 °/s**, crash.", "Real servo gearbox modelled: **~20 °/s**, finishes.", "Lesson: !!model the real hardware!!."], size=21, gap=20)
note(sl, 0.55, 6.05, 12.3, "Same flight (0.4 kg, gusts to ~3 m/s); only the servo model differs.")

# 12 stronger motors (Ashish) -----------------------------------------------------------------------
sl = slide("Better Than Giving the Others 40 % Stronger Motors",
           "[Ashish] Last check: we gave the published controllers motors 40 percent stronger, without adding their weight, which favours them. They finish 70 to 73 percent of the heavy-object flights. "
           "Ours, on the standard motors, still finishes 83 percent. To be honest about the limit: the flights we lose fail at the rotor thrust limit, and even a controller told the true wind and mass loses them.")
video(sl, "strong", 0.45, 1.45, 8.0, 4.5)
lines(sl, 8.75, 1.6, 4.15, 4.6, ["Others with 40 % stronger motors: **70–73 %** finished.", "Ours, standard motors: !!83 %!!.", "Our failures are at the thrust limit."], size=21, gap=20)
note(sl, 0.55, 6.05, 12.3, "Left: L1 with 40 % stronger motors flips at 16.6 s. Right: ours, standard motors, finishes.")

# 13 one model (Sahil) ------------------------------------------------------------------------------
sl = slide("Step 4: Why One Model for Drone + Arm (NMPC)",
           "[Sahil] Now the touch half. Here the drone flies a figure-eight while the arm draws circles 5 centimetres in radius, shown at real speed. Both controllers get the same drone, the same path and the same sensors. "
           "Left: the usual approach, a drone controller and a separate arm controller, each ignoring the other. It even runs 4 times faster, at 200 hertz. "
           "Every arm motion pushes the drone, the drone controller does not expect it, and you can watch the error grow lap after lap until it tilts past 40 degrees and crashes at 15 seconds, 24 centimetres off. "
           "Right: our NMPC, nonlinear model predictive control, with one model of drone and arm together. It knows each arm motion will push the drone, counters it in advance, and stays within 0.3 millimetres. "
           "WHAT is NMPC: every 20 milliseconds it measures the state, then uses the model to predict the next 0.2 seconds, 10 steps of 20 milliseconds, and searches for the commands, 4 rotor speeds and 3 joint torques, "
           "that keep the drone and the tip closest to their paths with the least effort, without breaking any limit: rotor speeds, joint torques, tilt. It applies only the first command, then measures again and re-plans. "
           "Re-planning every step is what makes it feedback. WHY a model: it sees the arm's push before it happens, it handles limits directly, and it can include the wall contact. "
           "WHY nonlinear: the drone rotates and the arm's geometry changes a lot, so a model linearised around hover would be wrong exactly when it matters. "
           "WHY NOT PID or separate controllers: they only react after an error appears, and they do not know the coupling, which is what you see on the left. "
           "HOW it runs: our model has 23 states, the drone, the arm joints, the rotor speeds and the contact. We solve it with acados, a fast optimisation solver: about 10 milliseconds per plan on this PC, inside its 20 millisecond budget. "
           "The COST: it is only as good as its model, which is the next slide.")
video(sl, "fig8", 0.45, 1.45, 12.4, 3.55)
lines(sl, 0.55, 5.15, 12.3, 1.5, ["Separate controllers: the error grows until the drone !!crashes at 15 s!!. One model of both: within **0.3 mm**.",
                                  "NMPC: every 20 ms, predict 0.2 s ahead with the model, pick the best commands, apply the first, repeat."], size=20, gap=10)

# 14 writing test -----------------------------------------------------------------------------------
sl = slide("The Test: Tracing Letters on a Wall",
           "[Sahil] This is the touch test: the NMPC traces HELLO WPI, shown at double speed. Red is where the tip pressed, green the letters to follow. The drone flies along the wall while the arm draws the letters. "
           "Letters are 5 centimetres tall, the tip moves at 5 centimetres per second and presses 1 millimetre into the wall. The sensors are as noisy as a motion-capture system, about 0.3 millimetres and 0.1 degrees. "
           "Each condition is 5 sentences times 3 noise seeds, 15 tries. With nothing disturbing it, the NMPC finishes all 15 sentences, 1.2 millimetres from the letters on average. "
           "Here the model is exactly right: the NMPC predicts the tip depth 0.2 seconds ahead to within a quarter of a millimetre. That is our reference.")
video(sl, "write", 0.45, 1.45, 7.4, 3.57)
lines(sl, 8.15, 1.6, 4.75, 4.9, ["Red: where the tip pressed. Green: the letters.", "5 cm letters, tip at 5 cm/s, pressing 1 mm into the wall.",
                                 "No disturbance: !!15 of 15!! sentences, **1.2 mm** from the letters."], size=21, gap=18)

# 15 where it breaks --------------------------------------------------------------------------------
sl = slide("Where the NMPC Breaks: Real-World Conditions",
           "[Sahil] The NMPC is only as good as its model, and the real world does not match the model. We tested what a real drone meets. "
           "An unknown load, here 100 grams: its model thinks the arm is empty, the drone tilts and drifts, and in 0 of 15 tries does it keep the tip on the wall. "
           "Delay in the motors, motors twice as slow as the model expects: it tips past 40 degrees and loses control, 0 of 15. "
           "A wrong contact model, a wall half as stiff: it finishes, but 13 millimetres off the letters and pressing 6 times harder, because 0.2 seconds ahead it predicts the tip 7 millimetres shallower than it really is. "
           "A bias, rotors 15 percent weaker than modelled: 13 of 15, 3.7 millimetres. Sensor noise alone is not the problem: with noise three times larger, 1 millimetre, it still finishes 15 of 15, at 2.1 millimetres. "
           "Adding a disturbance estimate to the NMPC fixes the loads, but not the slow motors and not the soft wall. We also tried pressing to a target force instead of a depth: the soft-wall error dropped from 13 to 6 millimetres, still a failure. "
           "We have not yet tested delay in the sensors themselves; slow motors are our stand-in for delay. This is where learning comes in.")
video(sl, "load", 0.45, 1.45, 7.4, 3.57)
lines(sl, 8.15, 1.5, 4.75, 5.0, ["Unknown load: **0 of 15**.", "Slower motors (delay): **0 of 15**.", "Softer wall: finishes, but **13 mm** off.", "Weaker rotors (bias): 13 of 15.",
                                 "Sensor noise alone: still 15 of 15.", "Cause: !!its model is wrong!!."], size=21, gap=10)
note(sl, 0.55, 5.15, 7.4, "Video: NMPC with an unknown 100 g load, real speed.")

# 16 learned ----------------------------------------------------------------------------------------
sl = slide("Step 5a: A Learned Controller Replaces the Model",
           "[Sahil] The first learning approach throws the model away. A neural network flies the drone directly. "
           "WHAT: 100 times a second it reads 300 numbers, the tip and drone errors, speeds, joint angles and the measured contact force over the last 4 steps, "
           "and outputs 10: how the drone should accelerate and turn, and where the 3 joints should go. A simple inner loop turns those into rotor speeds and joint torques. "
           "HOW it learns: reinforcement learning, trial and error in simulation with the PPO algorithm, 20 million steps, about 3 and a half hours on 30 cores of the Turing cluster. "
           "During training we randomly change the load, the motor speed, the wind and the thrust, so it learns to cope without being told. "
           "RESULT: with motors twice as slow, the NMPC with a disturbance estimate, left, loses control. The learned controller, right, traces the sentence, and finishes 14 of 15 in the test. "
           "The cost: it is less precise, about 1.9 millimetres instead of 1.2, it has no guarantees, and it struggles with the heaviest load.")
steps(sl, 1.45, ["Sensors", "!!Neural network!!", "Rotors and joints"], h=0.7, size=20, x0=2.0, w_total=9.3)
video(sl, "slow", 0.45, 2.4, 12.4, 3.36)
lines(sl, 0.55, 5.95, 12.3, 0.8, ["No model: trained by trial and error with random loads, motor speeds and wind.  Slow motors: NMPC fails, learned controller finishes **14 of 15**."], size=19)

# 17 hybrid -----------------------------------------------------------------------------------------
sl = slide("Step 5b: Keep the Model, Learn Only Its Error",
           "[Sahil] The second learning approach is different, and this is the key difference from the last slide. There, the network replaced the planner. Here the NMPC still plans and flies the drone, with its model, its limits and its guarantees; "
           "a small network only adds a correction to the model: an extra force at the tip, 2 numbers, that the NMPC includes in its prediction. It is trained the same way, by reinforcement learning, rewarded for tracing well. "
           "Same soft wall as before. Left: NMPC with a disturbance estimate, 17.8 millimetres off and pressing hard. Right: with the learned correction, 2.1 millimetres, all 15 sentences, and a light touch. "
           "Why it works: its prediction of the tip depth 0.2 seconds ahead is now off by 0.06 millimetres on average, instead of 7. "
           "A larger correction, 9 numbers that can push on the drone and on every joint, handles even more, as the next table shows. The cost: about 2 milliseconds more computing per step.")
steps(sl, 1.45, ["Sensors", "NMPC + !!learned correction!!", "Rotors and joints"], h=0.7, size=20, x0=2.0, w_total=9.3)
video(sl, "soft", 0.45, 2.4, 12.4, 3.36)
lines(sl, 0.55, 5.95, 12.3, 0.8, ["The model is right again (tip depth error 7 mm → 0.06 mm).  Soft wall: **17.8 mm → 2.1 mm**, all 15 sentences."], size=19)

# 18 table ------------------------------------------------------------------------------------------
sl = slide("All Controllers on the Same Tests",
           "[Sahil] Same tests for every controller. Each cell: tries finished, and the average distance of the tip from the letters. Green: all finished within 3 millimetres. Yellow: some failed, or 3 to 5. Red: none finished, or more than 5. "
           "Load is 100 and 200 grams together, 30 tries; wind is a gust and a steady sine, 30 tries; all at once is a 100 gram load, motors 1.5 times slower and a gust together. "
           "The plain NMPC is the most precise when its model is right, but fails with a load, slow motors and all at once. The disturbance estimate fixes the load. "
           "The learned controller is the only one that copes with slow motors, but loses some heavy-load tries. "
           "The NMPC with the larger learned correction is the most accurate under every disturbance, about 1.2 to 1.4 millimetres, including all at once, but it still fails with motors twice as slow. "
           "Note: in the first run of the no-disturbance test, 2 of the disturbance-estimate tries were stopped by a stuck solver, not by control; the rerun finished 15 of 15.")
cols = ["No disturbance", "Unknown load", "Slower motors", "Softer wall", "Wind", "All at once"]
NN = [15, 30, 15, 15, 30, 15]
rows = [("NMPC", [(15, 1.17), (0, 0), (0, 0), (15, 13.4), (30, 1.36), (0, 0)]),
        ("NMPC + disturbance estimate", [(15, 1.91), (30, 3.41), (0, 0), (15, 17.8), (30, 1.94), (0, 0)]),
        ("Learned controller", [(15, 1.86), (22, 2.47), (14, 2.49), (15, 1.75), (30, 2.28), (8, 2.59)]),
        ("NMPC + learned correction", [(15, 1.79), (30, 3.08), (0, 0), (15, 2.11), (30, 1.81), (0, 0)]),
        ("NMPC + larger learned correction", [(15, 1.33), (30, 1.22), (0, 0), (15, 1.43), (30, 1.35), (15, 1.28)])]
tb = sl.shapes.add_table(6, 7, Inches(0.45), Inches(1.4), Inches(12.45), Inches(4.5)).table
tb.columns[0].width = Inches(3.45)
for j in range(1, 7): tb.columns[j].width = Inches(1.5)


def cell(c, t, fill, size=17, bold=False, color=DK):
    c.fill.solid(); c.fill.fore_color.rgb = fill; c.vertical_anchor = MSO_ANCHOR.MIDDLE; tf = c.text_frame; tf.margin_left = tf.margin_right = Inches(0.05)
    for k, line in enumerate(t.split("\n")):
        p = tf.paragraphs[0] if k == 0 else tf.add_paragraph(); p.alignment = PP_ALIGN.CENTER; r = p.add_run(); r.text = line
        r.font.size = Pt(size if k == 0 else size - 3); r.font.bold = bold or (k == 0 and color == DK and fill != CARD); r.font.name = TNR; r.font.color.rgb = color if k == 0 else GR


cell(tb.cell(0, 0), "", RED)
for j, h in enumerate(cols): cell(tb.cell(0, j + 1), h, RED, size=16, bold=True, color=WHITE)
for i, (name, row) in enumerate(rows):
    cell(tb.cell(i + 1, 0), name, CARD, size=17, bold=True)
    for j, (n, e) in enumerate(row):
        N = NN[j]; cell(tb.cell(i + 1, j + 1), f"0 / {N}\nfailed" if n == 0 else f"{n} / {N}\n{e:.1f} mm", OK if (n == N and e <= 3) else (BAD if n == 0 or e > 5 else MID), size=17)
tb.rows[0].height = Inches(0.6)
for i in range(1, 6): tb.rows[i].height = Inches(0.78)
lines(sl, 0.55, 6.05, 12.3, 0.7, ["Each cell: tries finished · average tip distance from the letters. Green: all finished within 3 mm. Yellow: some failed or 3–5 mm. Red: none finished or over 5 mm."], size=14, color=GR)

# 19 lessons ----------------------------------------------------------------------------------------
sl = slide("What Failed, What We Learned, What Is Next",
           "[Sahil] What failed. Much slower motors still beat every controller that uses a model; the learned controller copes, but not at three times slower. "
           "Pressing harder does not fix a wrong model: force control on the soft wall still missed. And the heaviest objects still fail when a rotor reaches its thrust limit. "
           "What we learned. If the drone knows its load, the load can even help: faster moves, better flights in wind. A correct model gives the most precise control. "
           "And the best use of learning was to correct the model, not to replace it. "
           "Next. Give the NMPC the load mass measured by the arm, which joins our two halves. Teach the learned correction to handle slow motors. And move to hardware, starting with measuring a real servo. Thank you, we are happy to take questions.")
for i, (h, fill, items) in enumerate([("What failed", BAD, ["Much slower motors still beat every model-based controller.", "Pressing harder does not fix a wrong model.", "Heaviest objects: rotors hit their limit."]),
                                      ("What we learned", OK, ["Know the load, and the load can help.", "A correct model is the most precise.", "Learn the model's error; keep the model."]),
                                      ("What is next", CARD, ["Give the NMPC the load measured by the arm.", "Teach the correction to handle slow motors.", "Test on real hardware."])]):
    x = 0.45 + i * 4.18; box(sl, x, 1.5, 3.98, 4.9, fill=fill)
    lines(sl, x + 0.25, 1.7, 3.5, 0.5, [h], size=25, color=DK).text_frame.paragraphs[0].runs[0].font.bold = True
    lines(sl, x + 0.25, 2.5, 3.5, 3.8, items, size=22, gap=22)

OUT = os.path.join(HERE, "2026-11-23_1145_Capstone_Carry_Touch_Write.pptx")
prs.save(OUT)
with open(os.path.join(HERE, "2026-11-23_1145_capstone_speaker_notes.md"), "w") as f:
    f.write("# Capstone talk notes, 2026-11-23 (Ashish + Sahil, 15 min)\n\nSahil: slides 1-4 and 13-19. Ashish: slides 5-12. About 45 s per slide.\n\n")
    for i, (t, n) in enumerate(NOTES, 1): f.write(f"## {i}. {t}\n\n{n}\n\n")
    f.write(open(os.path.join(HERE, "2026-11-23_1145_nmpc_rl_background.md")).read())
print("saved", len(prs.slides), OUT)
