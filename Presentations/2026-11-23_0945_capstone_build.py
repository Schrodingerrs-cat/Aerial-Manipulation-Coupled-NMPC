"""Capstone deck, Mon 2026-11-23 (RBE 594, Ashish + Sahil): v2 of 2026-11-22_0905, rewritten after Sahil's review.
Changes: Ashish's carrying slides added (from his PDF 'Capstone_Carry_Touch_Write_with_arm_damping'; his video stills, not videos);
plain wording, statement titles, numbers inside sentences, general names (no internal run names); the 9-number learned correction (L9W) added to the table.
Writes the .pptx and the speaker notes (.md).  Assets: 2026-11-23_0945_capstone_assets/ (clips from clips.sh)."""
import os, re
HERE = os.path.dirname(os.path.abspath(__file__)); AS = os.path.join(HERE, "2026-11-23_0945_capstone_assets") + "/"
exec(open(os.path.join(HERE, "2026-11-18_1430_capstone_helpers.py")).read().replace("/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck/", AS))
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION

TNR, LGR = "Times New Roman", RGBColor(0xB4, 0xB8, 0xBC)
NOTES = []


def slide(title, notes):
    sl = prs.slides.add_slide(L["Title Only"]); sl.shapes.title.text = title
    for r in sl.shapes.title.text_frame.paragraphs[0].runs: r.font.name = TNR; r.font.size = Pt(34)
    sl.notes_slide.notes_text_frame.text = notes; NOTES.append((title, notes)); return sl


def lines(sl, x, y, w, h, paras, size=21, gap=14, color=DK):
    """Plain paragraphs; **text** = bold red."""
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h)); tf = tb.text_frame; tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    for i, s in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph(); p.space_after = Pt(gap)
        for k, part in enumerate(re.split(r"\*\*", s)):
            if not part: continue
            r = p.add_run(); r.text = part; f = r.font; f.name = TNR; f.size = Pt(size); f.color.rgb = RED if k % 2 else color; f.bold = bool(k % 2)
    return tb


def note(sl, x, y, w, s, size=14): lines(sl, x, y, w, 0.6, [s], size=size, color=GR)


def video(sl, name, x, y, w, h):
    sl.shapes.add_movie(AS + f"v_{name}.mp4", Inches(x), Inches(y), Inches(w), Inches(h), poster_frame_image=AS + f"p_{name}.png", mime_type="video/mp4")


def img(sl, f, x, y, w=None, h=None): sl.shapes.add_picture(AS + f, Inches(x), Inches(y), Inches(w) if w else None, Inches(h) if h else None)


def bars(sl, x, y, w, h, title, cats, vals, colors, fmt, vmax):
    cd = CategoryChartData(); cd.categories = cats; cd.add_series("s", vals)
    ch = sl.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(x), Inches(y), Inches(w), Inches(h), cd).chart
    ch.has_legend = False; ch.has_title = True; ch.chart_title.text_frame.text = title
    t = ch.chart_title.text_frame.paragraphs[0].runs[0].font; t.size = Pt(17); t.bold = True; t.color.rgb = DK; t.name = TNR
    pl = ch.plots[0]; pl.gap_width = 55; pl.has_data_labels = True; dl = pl.data_labels; dl.number_format = fmt; dl.number_format_is_linked = False
    dl.position = XL_LABEL_POSITION.OUTSIDE_END; dl.font.size = Pt(17); dl.font.bold = True; dl.font.color.rgb = DK; dl.font.name = TNR
    for i, c in enumerate(colors):
        pt = pl.series[0].points[i]; pt.format.fill.solid(); pt.format.fill.fore_color.rgb = c
    va = ch.value_axis; va.maximum_scale = vmax; va.minimum_scale = 0; va.visible = False; va.has_major_gridlines = False
    ca = ch.category_axis; ca.tick_labels.font.size = Pt(16); ca.tick_labels.font.name = TNR; ca.tick_labels.font.color.rgb = DK; ca.format.line.color.rgb = LGR


def steps(sl, y, items, h=0.85, x0=0.55, w_total=12.25, size=19):
    n = len(items); gap = 0.4; w = (w_total - gap * (n - 1)) / n
    for i, t in enumerate(items):
        x = x0 + i * (w + gap); box(sl, x, y, w, h, fill=CARD)
        lines(sl, x + 0.1, y + 0.12, w - 0.2, h - 0.15, [t], size=size).text_frame.paragraphs[0].alignment = PP_ALIGN.CENTER
        if i: arrow(sl, x - gap + 0.05, y + h / 2, x - 0.05, y + h / 2)


# 1 title -------------------------------------------------------------------------------------------
sl = prs.slides.add_slide(L["Title Slide"])
sl.shapes.title.text = "Carry, Touch, Write: An Aerial Manipulator That Knows Its Load"
sl.placeholders[1].text = "Load-, wind- and contact-aware control of a drone with an arm\nAshish Sukumar and Sahil Gajera\nRBE 594 Capstone  |  November 23, 2026"
for ph in (sl.shapes.title, sl.placeholders[1]):
    for p in ph.text_frame.paragraphs:
        for r in p.runs: r.font.name = TNR
n1 = "[Sahil] Good morning. We are Ashish and Sahil. Our project is a drone with a robot arm that has to carry things and touch things. Today we show how we made it aware of its load, of the wind, and of the surface it touches, and what worked and what did not."
sl.notes_slide.notes_text_frame.text = n1; NOTES.append(("Title", n1))

# 2 problem -----------------------------------------------------------------------------------------
sl = slide("A Drone With an Arm Gets Pushed Off Course",
           "[Sahil] A drone that only looks is easy. Once it carries or touches something, three things push it off course. "
           "A load makes it heavier and moves its balance point forward. The arm itself pushes the drone every time it moves. And when the pen touches a wall, the wall pushes back. "
           "A normal flight controller ignores all three. Our goal is a drone that carries loads and can write on a wall with the pen within about 2 millimetres of the letters.")
img(sl, "m_write_clean.png", 6.9, 1.55, w=6.0)
lines(sl, 0.55, 1.75, 6.0, 3.6, ["**The load** makes it heavier and pulls it forward.", "**The arm** pushes the drone every time it moves.", "**The wall** pushes back on the pen."], size=24, gap=22)
lines(sl, 0.55, 5.6, 12.3, 0.9, ["Goal: carry loads, and write on a wall with the pen within about **2 mm** of the letters."], size=22)

# 3 plan --------------------------------------------------------------------------------------------
sl = slide("Our Plan: First Carry, Then Touch",
           "[Sahil] We split the project in two halves. Carry: first a load hanging on a string, then weighing a load in the gripper, then carrying a parcel through wind; Ashish leads that part. "
           "Touch: writing on a wall, first with a controller that plans with a model, then adding learning; I lead that part. "
           "Each half has its own test, and every number we show comes from test runs that were not used for tuning.")
lines(sl, 0.55, 1.55, 7.0, 0.5, ["**Carry**: know the load"], size=20); lines(sl, 8.15, 1.55, 4.7, 0.5, ["**Touch**: know the contact"], size=20)
steps(sl, 2.15, ["1. Hanging load", "2. Weigh the load", "3. Carry in wind", "4. Plan with a model", "5. Add learning"], h=1.0, size=19)
lines(sl, 0.55, 3.75, 12.3, 2.6, ["Carry test: pick up a 0.3–0.5 kg parcel in flight and carry it through gusts up to 6 m/s.",
                                  "Touch test: write 5 sentences on a wall under 17 conditions (loads, slow motors, wind, soft or stiff wall), 15 tries each."], size=21, gap=18)

# 4 hanging load ------------------------------------------------------------------------------------
sl = slide("1. A Hanging Load Can Help the Motion",
           "[Sahil] Prof. Rosendo asked whether the swing of a hanging load can help instead of hurt. The drone moves 2 metres with 200 grams on a 60 centimetre string. "
           "Left: a normal drone that ignores the load has to move slowly to avoid a big swing; it is settled after 4.4 seconds. "
           "Right: our planner knows the load. It swings the load forward on purpose and catches it, so both arrive at rest after 1.6 seconds, with less than half the rotor effort. "
           "So a known load can work for us. That raises the next question: how does the drone know its load?")
video(sl, "swing", 0.45, 1.45, 12.4, 3.36)
lines(sl, 0.55, 5.0, 12.3, 1.6, ["Planning with the swing, drone and load are at rest after **1.6 s** instead of **4.4 s**, using **less than half** the rotor effort.",
                                 "The planner swings the load forward on purpose and catches it (2 m move, 200 g on a 60 cm string)."], size=20, gap=10)

# 5 know the load (Ashish) --------------------------------------------------------------------------
sl = slide("2. The Drone Has to Know Its Load",
           "[Ashish] A load in the gripper changes the drone's mass. The controller sets thrust from the mass, so if it does not know the load, it drifts. "
           "In our test flights the path error doubles: 38.7 centimetres without knowing the load, 19.1 when it is told the true mass. "
           "Before, we estimated the mass with a Bayesian filter that watches the whole drone. Wind and drag look like load to it, so it was only 70 percent accurate. "
           "Following Prof. Rosendo's idea, we use the arm itself as a scale, and it is 96 percent accurate, about 6 grams off on a 150 gram load.")
img(sl, "ash_robot.png", 7.0, 1.5, w=5.9)
lines(sl, 0.55, 1.7, 6.2, 4.6, ["Not knowing the load **doubles** the path error: **38.7 cm** instead of **19.1 cm**.",
                                "Our old Bayesian filter watched the whole drone, so wind looked like load: **70 %** accurate.",
                                "Using the arm as a scale: **96 %** accurate, about 6 g off on a 150 g load."], size=22, gap=24)

# 6 arm as scale (Ashish) ---------------------------------------------------------------------------
sl = slide("The Arm Is a Scale",
           "[Ashish] Hold a bag at arm's length: your shoulder feels its weight times the lever arm. The arm's motors measure their torque, and the joint angles and link lengths give the lever arm exactly. "
           "So the mass is the only unknown: torque divided by g times the lever arm. For an arm with several joints, the product of exponentials, with the frame on the drone body, gives the lever arm at every joint, "
           "and one least-squares fit over all joints gives the mass. No filter, no search, nothing learned.")
img(sl, "f_lever.png", 0.4, 1.45, w=5.0); img(sl, "f_poe.png", 5.7, 1.45, w=7.2)
lines(sl, 0.55, 5.35, 12.3, 1.4, ["The motors feel weight × lever arm; angles and lengths give the lever arm, so **m = torque / (g × lever arm)**.",
                                  "Several joints: the product of exponentials gives each lever arm, and one least-squares fit gives the mass."], size=20, gap=10)

# 7 results (Ashish) --------------------------------------------------------------------------------
sl = slide("It Weighs Well, and the Drone Flies Better",
           "[Ashish] On 300 test flights never used for tuning, with motor lag, wind, sensor noise and model errors: our estimate is 96 percent accurate, the Bayesian filter 88 percent even after calibration. "
           "With our estimate the path error is 19.8 centimetres, almost the 19.1 of a drone told the true mass. "
           "[Sahil] On our writing robot the same formula reads 99.4 grams for 100 and 199.6 for 200. And when the rotors are 10 percent weak but there is no load, it still reads zero, where a thrust-based estimate would read 170 grams.")
bars(sl, 0.4, 1.4, 4.4, 4.0, "Mass accuracy, % (higher is better)", ["Ours", "Bayes (calibrated)", "Bayes"], (96.4, 88.0, 69.9), [RED, LGR, LGR], '0.0"%"', 118)
bars(sl, 4.9, 1.4, 4.4, 4.0, "Path error, cm (lower is better)", ["Load unknown", "Ours", "True mass"], (38.7, 19.8, 19.1), [LGR, RED, DK], '0.0', 48)
lines(sl, 9.55, 1.6, 3.35, 4.0, ["On our writing robot: 100 g reads **99.4 g**, 200 g reads **199.6 g**.", "Weak rotors do not fool it: no load reads **0 g** (a thrust-based estimate reads 170 g)."], size=19, gap=16)
note(sl, 0.55, 5.75, 12.3, "300 test flights never used for tuning, with motor lag, wind, sensor noise and model errors.")

# 8 limit (Ashish) ----------------------------------------------------------------------------------
sl = slide("The Same Formula Gives the Arm's Limit",
           "[Ashish] The formula also works backwards: for a given load it gives the torque the shoulder needs. The shoulder motor's maximum is 1 newton-metre, so the heaviest load it can hold is 242 grams, "
           "and 200 grams already uses 91 percent of it. [Sahil] Our writing tests agree: the learned controller, which must handle the load by itself, finishes only 7 of 15 sentences with 200 grams.")
img(sl, "f_torque2.png", 0.4, 1.45, w=7.9)
lines(sl, 8.7, 1.8, 4.2, 4.6, ["The shoulder can hold at most **242 g**.", "At 200 g it already uses **91 %** of its torque.", "Writing tests agree: the learned controller finishes only 7 of 15 sentences with 200 g."], size=21, gap=22)

# 9 carry in wind (Ashish) --------------------------------------------------------------------------
sl = slide("3. Carrying a Parcel Through Gusts",
           "[Ashish] Now a harder carry: pick up a 0.3 to 0.5 kilogram parcel mid-flight and carry it through gusts up to 6 metres per second. That is up to a third more mass all at once, "
           "wind pushing the drone, and the parcel swinging on the arm and pulling the drone over. Our method has three steps. First, weigh the parcel with the arm, as before. "
           "Second, with the mass known, the wind is the only unknown left in the drone's momentum balance, so we can measure it. Third, damp each arm joint for its real inertia, including the servo's gearbox. "
           "This is a different, stronger robot: 1.52 kilograms with 3 to 4 newton-metre servos.")
img(sl, "a_carry.png", 6.75, 1.45, w=6.15)
lines(sl, 0.55, 1.6, 6.0, 2.4, ["A 0.3–0.5 kg parcel adds up to a third more mass at once, gusts up to 6 m/s push the drone, and the parcel swings and pulls it over."], size=20)
lines(sl, 0.55, 3.25, 6.0, 2.6, ["**1.** Weigh the parcel with the arm.", "**2.** Mass known, so measure the wind from the drone's motion.", "**3.** Damp each arm joint for its real inertia."], size=21, gap=12)
note(sl, 0.55, 5.7, 12.3, "Carrying robot: 1.52 kg drone, 4-joint arm on 3–4 N·m servos (stronger than the writing robot's arm).")

# 10 vs published (Ashish) --------------------------------------------------------------------------
sl = slide("Same Flight: a Published Controller Flips, Ours Finishes",
           "[Ashish] The same test flight for both: the parcel is picked up at 5 seconds and the gusts reach 6 metres per second. Left, the published L1 adaptive controller, tuned for this task. "
           "With each gust it tilts further, the parcel swings, a rotor hits its limit and it flips at 14.1 seconds. The tuned INDI and PID controllers flip on this flight too. "
           "Ours, right, stays under about 30 degrees of tilt and finishes. Every controller was tuned the same way, on flights we never used for testing.")
img(sl, "a_l1.png", 0.45, 1.45, w=8.0)
lines(sl, 8.75, 1.6, 4.15, 4.8, ["Published L1 adaptive control flips at **14.1 s**.", "Published INDI and PID flip too (15.8 s, 16.3 s).", "Ours stays under about 30° of tilt and **finishes**.",
                                 "All tuned the same way, on flights not used for testing."], size=20, gap=16)

# 11 carry results (Ashish) -------------------------------------------------------------------------
sl = slide("Ours Finishes More Flights in Stronger Wind",
           "[Ashish] Over 30 heavy-parcel test flights each, the published controllers finish about half; ours finishes 83 percent. A version of ours that is told the true wind and mass also gets 83 percent, "
           "so our estimates are as good as knowing the truth here. With a 0.3 kilogram parcel, ours still finishes at least 90 percent of flights in gusts up to 7.5 metres per second, about twice the others. "
           "The flights that fail have a rotor at its thrust limit.")
bars(sl, 0.4, 1.4, 6.0, 4.1, "Heavy parcels, gusts to 6 m/s: flights finished, %", ["L1", "INDI", "PID", "Ours", "Told truth"], (53, 53, 47, 83, 83), [LGR, LGR, LGR, RED, DK], '0"%"', 105)
bars(sl, 6.6, 1.4, 4.6, 4.1, "0.3 kg: strongest gust, m/s", ["L1", "INDI", "PID", "Ours"], (3.8, 3.0, 4.5, 7.5), [LGR, LGR, LGR, RED], '0.0', 9.5)
lines(sl, 11.35, 1.7, 1.6, 4.0, ["**83 %** vs about 50 %", "**2×** the wind"], size=19, gap=24)
note(sl, 0.55, 5.75, 12.3, "Test flights not used for tuning. Left: 30 flights per bar (0.3–0.5 kg). Right: strongest gust with at least 90 % of 20 flights finished. Failures: a rotor at its thrust limit.")

# 12 armature (Ashish) ------------------------------------------------------------------------------
sl = slide("Why the Arm Can Be Damped: the Servo's Gearbox",
           "[Ashish] Damping the arm first looked impossible: in our old model the arm shook at around 1000 degrees per second and the drone crashed. The reason was the model, not the idea. "
           "A real servo has a gearbox with a ratio of about 350, which adds an inertia of about 0.005 kilogram metres squared, many times the wrist link's. "
           "With that in the model, the same damping keeps the arm shaking at about 20 degrees per second and the drone finishes. Lesson: model the real hardware.")
img(sl, "a_armature.png", 0.45, 1.45, w=8.0)
lines(sl, 8.75, 1.6, 4.15, 4.8, ["A real servo's gearbox adds inertia, many times the wrist link's.", "Our old model left it out: the arm shook at about **1,000 °/s** and the drone crashed.",
                                 "With it modelled, the same damping: about **20 °/s**, and the drone finishes."], size=20, gap=16)

# 13 stronger motors (Ashish) -----------------------------------------------------------------------
sl = slide("Better Than Giving the Others 40 % Stronger Motors",
           "[Ashish] Last check: we gave the published controllers motors 40 percent stronger, without adding their weight, which favours them. They finish 70 to 73 percent of the heavy-parcel flights. "
           "Ours, on the standard motors, still finishes 83 percent. To be honest about the limit: the flights we lose fail at the rotor thrust limit, and even a controller told the true wind and mass loses them.")
img(sl, "a_strong.png", 0.45, 1.45, w=8.0)
lines(sl, 8.75, 1.6, 4.15, 4.8, ["Published controllers with **40 % stronger** motors: 70–73 % of flights finished.", "Ours on standard motors: **83 %**.",
                                 "The flights we lose fail at the rotor thrust limit; even knowing the true wind and mass does not save them."], size=20, gap=16)

# 14 one model (Sahil) ------------------------------------------------------------------------------
sl = slide("4. To Touch, Plan With One Model of Drone and Arm",
           "[Sahil] Now the touch half. Writing needs millimetres, and the arm's motion pushes the drone. Here the arm moves its pen 6 centimetres back and forth twice a second. "
           "Left: separate drone and arm controllers, the usual approach. The drone controller does not expect the arm's push, drifts 25 centimetres and loses control. "
           "Right: one model of drone and arm together. The planner knows each arm swing will push the drone and counters it in advance, so the drone stays within 5 millimetres. "
           "Every 20 milliseconds it predicts the next 0.2 seconds and picks the 4 rotor speeds and 3 joint torques; this is model predictive control. One plan takes about 10 milliseconds.")
video(sl, "arm", 0.45, 1.45, 12.4, 3.6)
lines(sl, 0.55, 5.2, 12.3, 1.5, ["Separate controllers: the arm's push makes the drone drift **25 cm** and lose control. One model of both: within **5 mm**.",
                                 "Every 20 ms the planner predicts the next 0.2 s and chooses 4 rotor speeds and 3 joint torques (model predictive control)."], size=20, gap=10)

# 15 writing test -----------------------------------------------------------------------------------
sl = slide("The Writing Test",
           "[Sahil] This is the test: the planner writes HELLO WPI, shown at double speed. Red is ink, green the letters to follow. The drone flies along the wall while the arm draws. "
           "Letters are 5 centimetres tall, the pen moves at 5 centimetres per second and presses 1 millimetre into the wall, and the sensors are as noisy as a motion-capture system. "
           "With nothing disturbing it, the planner finishes all 15 sentences, 1.2 millimetres from the letters on average. That is our reference.")
video(sl, "write", 0.45, 1.45, 7.4, 3.57)
lines(sl, 8.15, 1.6, 4.75, 4.9, ["Red is ink, green the letters to follow.", "5 cm letters, pen at 5 cm/s, pressing 1 mm into the wall; sensor noise like motion capture.",
                                 "No disturbance: **15 of 15** sentences, pen **1.2 mm** from the letters on average."], size=20, gap=16)

# 16 model wrong ------------------------------------------------------------------------------------
sl = slide("When Its Model Is Wrong, the Planner Fails",
           "[Sahil] The planner is only as good as its model. Here it carries 100 grams it does not know about: it cannot keep the pen on the wall, 0 of 15. "
           "With motors slower than its model expects, it tips over, 0 of 15. A softer wall is subtler: it finishes, but 13 millimetres off the letters, pressing 6 times harder. "
           "We measured why: on the soft wall its model predicted the pen 7 millimetres shallower than it really was. Adding a disturbance estimate fixes the loads, but not the soft wall and not slow motors.")
video(sl, "load", 0.45, 1.45, 7.4, 3.57)
lines(sl, 8.15, 1.5, 4.75, 5.0, ["Unknown 100 g load: **0 of 15**, the pen does not stay on the wall.", "Motors slower than modelled: **0 of 15**, the drone tips over.",
                                 "Softer wall: finishes, but **13 mm** off, pressing 6× harder; its model put the pen 7 mm shallower than it was."], size=20, gap=16)

# 17 learned ----------------------------------------------------------------------------------------
sl = slide("5. A Learned Controller Copes With Slow Motors",
           "[Sahil] The other approach learns instead of modelling. A neural network is trained by trial and error in simulation, reinforcement learning, for 20 million steps, "
           "while we randomly change the load, the motor speed and the wind, so it learns to cope without being told. 100 times a second it reads the errors and speeds and outputs where the drone should accelerate and where the joints should go. "
           "With slow motors, the planner with a disturbance estimate, left, tips over. The learned controller, right, writes the sentence, and finishes 14 of 15 in the test.")
steps(sl, 1.45, ["Errors, speeds, pen force", "Neural network", "Drone acceleration, joint targets", "Rotor speeds, joint torques"], h=0.75, size=17)
video(sl, "slow", 0.45, 2.45, 12.4, 3.36)
lines(sl, 0.55, 5.95, 12.3, 0.8, ["Trained by trial and error with random loads, motor speeds and wind. Slow motors: the planner tips over; the learned controller finishes **14 of 15**."], size=19)

# 18 hybrid -----------------------------------------------------------------------------------------
sl = slide("Best of Both: Learn Only What the Model Gets Wrong",
           "[Sahil] Our hybrid keeps the planner and lets a small network correct only its model, adding a small extra force inside the model. The planning stays model-based. "
           "On the soft wall, the planner with a disturbance estimate, left, is 17.8 millimetres off and pressing hard. With the learned correction, right, 2.1 millimetres, all 15 sentences, and a light touch. "
           "The reason: its prediction of the pen depth is right again, off by less than a tenth of a millimetre instead of 7.")
video(sl, "soft", 0.45, 1.45, 12.4, 3.36)
lines(sl, 0.55, 5.0, 12.3, 1.6, ["A small network adds a correction to the planner's model; the planning itself stays model-based.",
                                 "Soft wall: **17.8 mm → 2.1 mm**, all 15 sentences, because the predicted pen depth is right again (7 mm → under 0.1 mm off)."], size=20, gap=10)

# 19 table ------------------------------------------------------------------------------------------
sl = slide("All Controllers on the Same Tests",
           "[Sahil] Same tests for every controller, 15 tries each. Each cell: sentences finished out of 15, and the average pen distance from the letters in millimetres. "
           "The plain planner is precise when its model is right, but fails with loads, slow motors and the soft wall. The learned controller is the only one that survives slow motors. "
           "The bottom row is a larger learned correction, 9 numbers instead of 2, finished last night: it is the most accurate under every disturbance, about 1.2 to 1.4 millimetres, even with 200 grams and with all problems at once; only the plain planner with nothing disturbing it is slightly better. "
           "It still fails with slow motors, which is what we work on next.")
cols = ["No disturbance", "100 g load", "200 g load", "Slow motors", "Soft wall", "Wind gusts", "All at once"]
rows = [("Model-based planner", [(15, 1.17), (0, 0), (0, 0), (0, 0), (15, 13.4), (15, 1.22), (0, 0)]),
        ("+ disturbance estimate", [(15, 1.91), (15, 2.76), (15, 4.05), (0, 0), (15, 17.8), (15, 1.91), (0, 0)]),
        ("Learned controller", [(15, 1.86), (15, 2.32), (7, 2.79), (14, 2.49), (15, 1.75), (15, 1.90), (8, 2.59)]),
        ("+ learned correction (2 numbers)", [(15, 1.79), (15, 2.39), (15, 3.78), (0, 0), (15, 2.11), (15, 1.78), (0, 0)]),
        ("+ learned correction (9 numbers)", [(15, 1.33), (15, 1.23), (15, 1.20), (0, 0), (15, 1.43), (15, 1.32), (15, 1.28)])]
tb = sl.shapes.add_table(6, 8, Inches(0.45), Inches(1.4), Inches(12.45), Inches(4.5)).table
tb.columns[0].width = Inches(3.25)
for j in range(1, 8): tb.columns[j].width = Inches(1.314)


def cell(c, t, fill, size=17, bold=False, color=DK):
    c.fill.solid(); c.fill.fore_color.rgb = fill; c.vertical_anchor = MSO_ANCHOR.MIDDLE; tf = c.text_frame; tf.margin_left = tf.margin_right = Inches(0.05)
    for k, line in enumerate(t.split("\n")):
        p = tf.paragraphs[0] if k == 0 else tf.add_paragraph(); p.alignment = PP_ALIGN.CENTER; r = p.add_run(); r.text = line
        r.font.size = Pt(size if k == 0 else size - 3); r.font.bold = bold or (k == 0 and color == DK and fill != CARD); r.font.name = TNR; r.font.color.rgb = color if k == 0 else GR


cell(tb.cell(0, 0), "", RED)
for j, h in enumerate(cols): cell(tb.cell(0, j + 1), h, RED, size=15, bold=True, color=WHITE)
for i, (name, row) in enumerate(rows):
    cell(tb.cell(i + 1, 0), name, CARD, size=16, bold=True)
    for j, (n, e) in enumerate(row):
        cell(tb.cell(i + 1, j + 1), "0 / 15\nfailed" if n == 0 else f"{n} / 15\n{e:.1f} mm", OK if (n == 15 and e <= 3) else (BAD if n == 0 or e > 5 else MID), size=16)
tb.rows[0].height = Inches(0.65)
for i in range(1, 6): tb.rows[i].height = Inches(0.77)
lines(sl, 0.55, 6.05, 12.3, 0.7, ["Each cell: sentences finished out of 15 · average pen distance from the letters (mm). Green: all finished within 3 mm; yellow: some failed or 3–5 mm; red: none finished or over 5 mm."], size=14, color=GR)

# 20 lessons ----------------------------------------------------------------------------------------
sl = slide("What Failed, What We Learned, What Is Next",
           "[Sahil] To be honest about failures: no controller survives motors three times slower than expected, and slow motors still defeat every planner-based controller. "
           "The heaviest parcels still crash 17 percent of the time at the rotors' thrust limit, and the planner misses its 20 millisecond deadline on about 3.5 percent of steps. "
           "What we learned: knowing the load matters, for the swing, the parcel and the wind. A correct model gives the most precise control. And learning works best as a correction to the model, not a replacement. "
           "Next, for November 27: feed the measured load mass into the writing planner's model, which joins our two halves; handle slow motors in the learned correction; and measure a real servo's gearbox inertia to move toward hardware. Thank you.")
for i, (h, fill, items) in enumerate([("What failed", BAD, ["Motors 3× slower: every controller", "Slow motors: every planner-based controller", "Heaviest parcels: 17 % still crash at the thrust limit", "Planner misses its 20 ms deadline on 3.5 % of steps"]),
                                      ("What we learned", OK, ["Knowing the load matters: faster moves, 83 % vs ~50 % of flights in gusts", "A correct model is the most precise", "Learning works best as a correction to the model"]),
                                      ("Next (Nov 27)", CARD, ["Feed the measured load mass into the writing planner", "Handle slow motors in the learned correction", "Measure a real servo's gearbox; move to hardware"])]):
    x = 0.45 + i * 4.18; box(sl, x, 1.5, 3.98, 5.0, fill=fill)
    lines(sl, x + 0.22, 1.65, 3.55, 0.5, [h], size=23, color=RED if i != 1 else DK).text_frame.paragraphs[0].runs[0].font.bold = True
    lines(sl, x + 0.22, 2.35, 3.55, 4.0, items, size=19, gap=14)

OUT = os.path.join(HERE, "2026-11-23_0945_Capstone_Carry_Touch_Write.pptx")
prs.save(OUT)
with open(os.path.join(HERE, "2026-11-23_0945_capstone_speaker_notes.md"), "w") as f:
    f.write("# Capstone talk notes, 2026-11-23 (Ashish + Sahil, 15 min)\n\nSahil: slides 1-4 and 14-20. Ashish: slides 5-13. About 45 s per slide.\n\n")
    for i, (t, n) in enumerate(NOTES, 1): f.write(f"## {i}. {t}\n\n{n}\n\n")
print("saved", len(prs.slides), OUT)
