"""Capstone deck 2026-11-23 (RBE 594, Ashish + Sahil, 15 min): one story from motivation to the NMPC / RL / NMPC+RL comparison.
Style (advisor, 2026-11-18): Times New Roman, large text, every number explained, at most two videos per slide, talk script in the notes.
Numbers: lab sensor noise, 15 tries per condition (Code/results/am3_write/bench, Turing), timing/prediction from results/am3_timing/2026-11-21_1831_*.
Assets (template, clips, posters, figures) in 2026-11-22_0905_capstone_assets/; side-by-side clips cut from Media_results with its pair.sh."""
import os
HERE = os.path.dirname(os.path.abspath(__file__))
exec(open(os.path.join(HERE, "2026-11-18_1430_capstone_helpers.py")).read().replace("/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck/", os.path.join(HERE, "2026-11-22_0905_capstone_assets") + "/"))
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION

LGR = RGBColor(0xB4, 0xB8, 0xBC); TNR = "Times New Roman"; PINK = RGBColor(0xFB, 0xEB, 0xED)


def slide(title, notes):
    sl = prs.slides.add_slide(L["Title Only"]); sl.shapes.title.text = title
    for r in sl.shapes.title.text_frame.paragraphs[0].runs: r.font.name = TNR; r.font.size = Pt(36)
    sl.notes_slide.notes_text_frame.text = notes; return sl


def say(sl, x, y, w, h, paras, size=20, **kw): return text(sl, x, y, w, h, paras, size=size, **kw)


def point(sl, x, y, w, h, items, size=20):
    tb = bullets(sl, x, y, w, h, items, size=size)
    for p in tb.text_frame.paragraphs: p.space_after = Pt(10)
    return tb


def num(sl, x, y, w, big, meaning, size=34):
    say(sl, x, y, w, 0.65, [big], size=size, color=RED, bold=True); say(sl, x, y + 0.66, w, 1.1, [meaning], size=16, color=DK)


def video(sl, name, x, y, w, h):
    return sl.shapes.add_movie(D + f"v_{name}.mp4", Inches(x), Inches(y), Inches(w), Inches(h), poster_frame_image=D + f"p_{name}.png", mime_type="video/mp4")


def bars(sl, x, y, w, h, title, cats, vals, colors, fmt, vmax):
    cd = CategoryChartData(); cd.categories = cats; cd.add_series("s", vals)
    ch = sl.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(x), Inches(y), Inches(w), Inches(h), cd).chart
    ch.has_legend = False; ch.has_title = True; ch.chart_title.text_frame.text = title
    t = ch.chart_title.text_frame.paragraphs[0].runs[0].font; t.size = Pt(16); t.bold = True; t.color.rgb = DK; t.name = TNR
    pl = ch.plots[0]; pl.gap_width = 55; pl.has_data_labels = True; dl = pl.data_labels; dl.number_format = fmt; dl.number_format_is_linked = False
    dl.position = XL_LABEL_POSITION.OUTSIDE_END; dl.font.size = Pt(16); dl.font.bold = True; dl.font.color.rgb = DK; dl.font.name = TNR
    for i, c in enumerate(colors):
        pt = pl.series[0].points[i]; pt.format.fill.solid(); pt.format.fill.fore_color.rgb = c
    va = ch.value_axis; va.maximum_scale = vmax; va.minimum_scale = 0; va.visible = False; va.has_major_gridlines = False
    ca = ch.category_axis; ca.tick_labels.font.size = Pt(14); ca.tick_labels.font.name = TNR; ca.tick_labels.font.color.rgb = DK; ca.format.line.color.rgb = LGR


def flow(sl, y, items, h=1.05, x0=0.55, w_total=12.25, fills=None):
    """Boxes left to right joined by arrows; items: (head, sub)."""
    n = len(items); gap = 0.4; w = (w_total - gap * (n - 1)) / n
    for i, (head, sub) in enumerate(items):
        x = x0 + i * (w + gap); box(sl, x, y, w, h, fill=(fills or [CARD] * n)[i])
        say(sl, x + 0.12, y + 0.08, w - 0.24, h - 0.1, [[(head, {"bold": True, "size": 17})], [(sub, {"size": 14, "color": GR})]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
        if i: arrow(sl, x - gap + 0.04, y + h / 2, x - 0.04, y + h / 2)


# 1 ---------------------------------------------------------------- title
sl = prs.slides.add_slide(L["Title Slide"])
sl.shapes.title.text = "A Drone That Carries, Touches and Writes"
sl.placeholders[1].text = "Load- and contact-aware control of an aerial manipulator\nAshish Sukumar and Sahil Gajera\nRBE 594 Capstone  |  November 23, 2026"
for ph in (sl.shapes.title, sl.placeholders[1]):
    for p in ph.text_frame.paragraphs:
        for r in p.runs: r.font.name = TNR
sl.notes_slide.notes_text_frame.text = ("[Sahil] Today we tell one story: a flying robot with an arm that has to carry loads and touch surfaces. "
    "We start with why that is hard, then what we tried for the load, then two ways to control it, a model-based planner and a learned controller, and how they compare when the drone writes on a wall.")

# 2 ---------------------------------------------------------------- motivation
sl = slide("Why: Drones That Carry and Touch",
           "[Sahil] SAY: Drones are good at looking. Many real jobs need touching: inspecting a bridge with a probe, marking a wall, picking up a parcel. "
           "For that we put an arm on the drone. Three things then disturb the flight. First, a load: the drone gets heavier and its balance point moves forward. "
           "Second, the arm's own motion: every time the arm swings, it pushes the drone the other way. Third, contact: the wall pushes back on the pen. "
           "A normal drone controller ignores all three. Our goal is a controller that knows about them, and our test is hard on purpose: write a sentence on a wall, "
           "with the pen within about 2 millimetres of the letters, while the drone flies along the wall.")
pic(sl, "m_write.png", 7.0, 1.6, w=5.9)
say(sl, 0.55, 1.6, 6.2, 0.5, ["Three things push a drone with an arm off course:"], size=20, bold=True)
say(sl, 7.0, 4.95, 5.9, 0.4, ["Our robot writing HELLO WPI on a wall (simulation)"], size=14, color=GR)
for i, (h, s) in enumerate([("A load", "heavier, and the balance point moves forward"), ("The arm moving", "every arm swing pushes the drone the other way"), ("Contact", "the wall pushes back on the pen")]):
    y = 2.3 + i * 1.05; badge(sl, 0.6, y + 0.05, i + 1); say(sl, 1.35, y, 5.4, 0.95, [[(h, {"bold": True, "size": 21})], [(s, {"size": 18, "color": GR})]])
box(sl, 0.55, 5.55, 12.3, 1.0, fill=PINK)
say(sl, 0.8, 5.55, 11.9, 1.0, [[("Goal: ", {"bold": True, "color": RED}), ("a controller that knows the load and the contact. Test: write a sentence on a wall, pen within ~2 mm of the letters.", {})]], size=20, anchor=MSO_ANCHOR.MIDDLE)

# 3 ---------------------------------------------------------------- plan
sl = slide("The Plan: Four Steps, One Test",
           "[Sahil] SAY: This is the road map of the project. Step one: a load hanging on a string. Instead of fighting its swing, can we use it? "
           "Step two: a load in the gripper. Can the drone weigh it with its own arm? That is Ashish's analytical method. "
           "Step three: a model-based controller, a nonlinear model predictive controller, NMPC for short, that plans with a model of the drone, the arm and the wall together. "
           "Step four: learning. A reinforcement-learning controller, and a hybrid where learning only corrects the NMPC's model. "
           "Steps three and four are tested on the same job: writing on a wall under 17 different conditions, 15 tries each.")
flow(sl, 1.75, [("1  Hanging load", "use the swing, don't fight it"), ("2  Weigh the load", "the arm as a scale (analytical)"), ("3  Model-based", "NMPC: plan with a model"), ("4  Learning", "RL, and NMPC + RL")], h=1.5, fills=[CARD, CARD, CARD, CARD])
for i, q in enumerate(["Can the swing help the motion?", "Can the drone know its load?", "Is a coupled model enough?", "What can learning add?"]):
    say(sl, 0.55 + i * 3.16, 3.4, 2.95, 0.8, [q], size=17, color=RED, bold=True, align=PP_ALIGN.CENTER)
box(sl, 0.55, 4.55, 12.3, 1.9, fill=WHITE, line=RED)
say(sl, 0.85, 4.6, 11.8, 1.8, [[("The common test for steps 3 and 4: ", {"bold": True, "color": RED}), ("write 5 sentences that use all 26 letters on a wall", {})],
                              ["17 conditions: loads, slower motors, wind, softer / stiffer / slippery wall, ...  15 tries each, with realistic sensor noise"]], size=19, anchor=MSO_ANCHOR.MIDDLE)

# 4 ---------------------------------------------------------------- swing
sl = slide("Step 1: A Hanging Load Can Help",
           "[Sahil] SAY: Prof. Rosendo asked whether a hanging load's swing can help instead of hurt. The drone flies 2 metres with 200 grams on a 60 centimetre string. "
           "Left: a normal drone that ignores the load. To avoid a big swing it must move slowly, and it is still and settled after 4.35 seconds. "
           "Right: our planner knows the load. It swings the load forward on purpose and catches it, so both arrive at rest together, settled after 1.62 seconds: "
           "about 2.7 times faster. It also needs less than half the rotor effort, measured as total thrust over time, 23 versus 54 newton-seconds. "
           "Settled means the drone stays within 5 centimetres of the goal and the swing stays under 2 degrees. Lesson: if the controller knows the load, the load can work for it.")
video(sl, "swing", 0.45, 1.5, 12.4, 3.49)
say(sl, 0.45, 5.0, 6.2, 0.4, ["Left: normal drone, ignores the load"], size=16, color=GR); say(sl, 6.65, 5.0, 6.2, 0.4, ["Right: plans with the load's swing"], size=16, color=GR)
num(sl, 0.55, 5.45, 4.0, "4.35 s → 1.62 s", "time until drone and load are still at the goal (2 m move, 200 g on 60 cm)", size=30)
num(sl, 4.85, 5.45, 4.0, "54 → 23 N·s", "rotor effort (thrust × time): less than half", size=30)
num(sl, 9.0, 5.45, 3.9, "41° swing", "allowed on purpose: the planner swings it forward and catches it", size=30)

# 5 ---------------------------------------------------------------- load unknown (Ashish)
sl = slide("Step 2: The Drone Must Know Its Load",
           "[Ashish] SAY: Now a load in the gripper. The flight controller sets thrust from the drone's mass. If it does not know the load, it drifts: 38.7 cm from the path on average in our test flights. "
           "If we tell it the true mass, the same flights stay within 19.1 cm, half the error. So how do we find the mass in flight? "
           "Before, we used a Bayesian estimator: it compares rotor push with how the drone accelerates and keeps a running best guess. But wind, drag and a weak rotor look like load to it: "
           "it was only 70 percent accurate. Following Prof. Rosendo's suggestion we now use the arm itself as a scale, and it is 96 percent accurate, about 6 grams off on a 150 gram load.")
box(sl, 0.55, 1.55, 2.9, 2.1); num(sl, 0.75, 1.65, 2.55, "38.7 cm", "average distance from the path when the drone does NOT know the load", size=32)
box(sl, 3.6, 1.55, 2.9, 2.1); num(sl, 3.8, 1.65, 2.55, "19.1 cm", "same flights when it IS told the true mass: half the error", size=32)
pic(sl, "ash_robot.png", 6.85, 1.5, w=6.0)
for x, head, sub, big, fill in [(0.55, "Before: Bayesian filter", "watches the whole drone: wind and drag look like load", ("70 % accurate", "off by ~30 % of the load"), CARD),
                                 (3.6, "Now: analytical", "the arm as a scale (Prof. Rosendo)", ("96 % accurate", "~6 g off on a 150 g load"), PINK)]:
    box(sl, x, 3.85, 2.9, 2.75, fill=fill)
    say(sl, x + 0.18, 3.92, 2.6, 0.5, [head], size=18, bold=True, color=RED if fill == PINK else DK); say(sl, x + 0.18, 4.42, 2.6, 0.9, [sub], size=15, color=GR)
    say(sl, x + 0.18, 5.35, 2.6, 0.55, [big[0]], size=26, bold=True, color=RED); say(sl, x + 0.18, 5.92, 2.6, 0.5, [big[1]], size=15)
say(sl, 6.85, 5.4, 6.0, 1.2, [[("Accuracy", {"bold": True, "color": RED}), (" = how close the estimated mass is to the true mass.", {})]], size=17)

# 6 ---------------------------------------------------------------- idea (Ashish)
sl = slide("The Idea: Use the Arm as a Scale",
           "[Ashish] SAY: Hold a shopping bag at arm's length: your shoulder feels the weight times the lever arm. Our joint motors measure torque, and the lever arm comes from the joint angles and link lengths, "
           "which we know exactly. So mass = torque divided by g times lever arm. For a multi-joint arm we get the geometry from the product of exponentials, with the reference frame on the drone body: "
           "chain the joint rotations to get the gripper position, then the Jacobian gives the torque per kilogram at every joint, and a least-squares fit over all joints gives the mass. "
           "Analytical means it is one formula solved directly: no filter, no search, nothing learned. Our formulas match the simulator's physics to 10 to the minus 8 newton-metres.")
pic(sl, "f_lever.png", 0.4, 1.5, w=5.2)
pic(sl, "f_poe.png", 5.8, 1.5, w=7.1)
box(sl, 0.55, 5.4, 4.9, 1.2, fill=CARD)
say(sl, 0.7, 5.4, 4.6, 1.2, [[("torque = m·g·d   →   m = torque / (g·d)", {"bold": True, "size": 17})]], anchor=MSO_ANCHOR.MIDDLE, align=PP_ALIGN.CENTER)
box(sl, 5.8, 5.4, 7.05, 1.2, fill=PINK)
say(sl, 6.0, 5.4, 6.7, 1.2, [[("Product of exponentials → Jacobian → ", {"size": 18}), ("m = Σ a·τ / Σ a·a", {"bold": True, "color": RED, "size": 20})], [("a = torque per kg at each joint;  τ = measured joint torque", {"size": 15, "color": GR})]], anchor=MSO_ANCHOR.MIDDLE)

# 7 ---------------------------------------------------------------- results (Ashish)
sl = slide("Results: Better Mass, Better Flight",
           "[Ashish] SAY: On 300 test flights never used for tuning, with motor lag, wind, sensor noise and model errors. Left: how close each method's mass is to the true mass. "
           "Ours, after a one-time calibration of the servos, 96.4 percent; the Bayesian filter 88 percent even after calibration, 70 before. Middle: distance from the path. "
           "Not knowing the load, 38.7 cm. Using our estimate, 19.8 cm, almost as good as being told the true mass, 19.1. "
           "[Sahil] And on our wall-writing robot, with exact joint torques, the same formula reads 99.4 grams for a 100 gram load and 199.6 for 200. "
           "With no load but the rotors' thrust 10 percent lower than the model thinks, it reads minus 0.2 grams: the arm does not feel the rotors. A thrust-based reading would say 170 grams.")
bars(sl, 0.4, 1.45, 4.3, 4.1, "Mass accuracy, % (higher is better)", ["Ours", "Bayes (cal.)", "Bayes"], (96.4, 88.0, 69.9), [RED, LGR, LGR], '0.0"%"', 118)
bars(sl, 4.8, 1.45, 4.3, 4.1, "Distance from path, cm (lower is better)", ["No estimate", "Ours", "True mass"], (38.7, 19.8, 19.1), [LGR, RED, DK], '0.0', 48)
box(sl, 9.3, 1.55, 3.55, 4.0, fill=PINK)
say(sl, 9.45, 1.62, 3.3, 3.9, [[("Our writing robot", {"bold": True, "color": RED, "size": 19})], [("100 g load → reads 99.4 g", {"size": 17})], [("200 g load → reads 199.6 g", {"size": 17})],
                             [("No load, thrust 10 % low → reads −0.2 g", {"size": 17})], [("(a thrust-based reading: 170 g)", {"size": 15, "color": GR})]])
say(sl, 0.55, 5.85, 12.3, 0.7, ["Ashish: 300 test flights never used for tuning (motor lag, wind, sensor noise, model errors).  Sahil: exact joint torques, writing robot."], size=15, color=GR)

# 8 ---------------------------------------------------------------- carry limit (Ashish)
sl = slide("Turned Around: How Much Can the Arm Carry?",
           "[Ashish] SAY: The formula also works backwards: for a load it gives the shoulder torque needed, 0.47 newton-metres for the arm plus 2.19 per kilogram. The shoulder motor's maximum is 1.0, "
           "so the heaviest load it can hold is 242 grams, and 200 grams already uses 91 percent, with no margin for the arm swinging. "
           "[Sahil] Our writing tests agree: the learned controller, which has to handle the load itself, finishes only 7 of 15 sentences with 200 grams.")
pic(sl, "f_torque2.png", 0.4, 1.5, w=7.9)
num(sl, 8.7, 1.6, 4.2, "242 g", "heaviest load the shoulder motor can hold (its maximum is 1.0 N·m)")
num(sl, 8.7, 3.25, 4.2, "91 %", "of the shoulder's maximum already used at 200 g: no margin for the arm swinging")
box(sl, 8.7, 4.95, 4.15, 1.5, fill=WHITE, line=RED)
say(sl, 8.9, 4.95, 3.8, 1.5, [[("Writing tests agree: ", {"bold": True, "color": RED}), ("the RL controller finishes only 7 of 15 sentences with 200 g", {})]], size=17, anchor=MSO_ANCHOR.MIDDLE)

# 9 ---------------------------------------------------------------- NMPC why/how
sl = slide("Step 3: Why One Model for Drone + Arm (NMPC)",
           "[Sahil] SAY: Why model-based, and why one model? Here the arm swings its pen 6 cm back and forth twice a second. "
           "Left, B: the usual approach, a drone controller and a separate arm controller, each ignoring the other. Every arm swing pushes the drone, the drone controller does not expect it, "
           "and about a second after the arm starts swinging the drone is 25 centimetres off and loses control. Right, N: our NMPC uses one model of drone and arm together, so it knows each arm swing will push the drone and "
           "counters it in advance: the drone stays within 5 millimetres. How it works: every 20 milliseconds it predicts the next 0.2 seconds with the full model, 23 states including the wall contact, "
           "picks the best 4 rotor speeds and 3 joint torques, applies the first command and repeats. On this PC one solve takes about 10 milliseconds.")
video(sl, "bvsn", 0.45, 1.45, 12.4, 3.49)
say(sl, 0.45, 4.95, 6.2, 0.4, ["Left, B: separate drone and arm controllers"], size=16, color=GR); say(sl, 6.65, 4.95, 6.2, 0.4, ["Right, N: one model of drone + arm (NMPC)"], size=16, color=GR)
num(sl, 0.55, 5.4, 3.6, "250 mm vs 4.9 mm", "worst drone drift while the arm swings ±6 cm, 2× a second (B loses control)", size=26)
flow(sl, 5.45, [("Every 20 ms", "measure the state"), ("Predict 0.2 s", "drone + arm + wall model"), ("Choose", "4 rotor speeds, 3 joint torques"), ("Apply first", "then repeat")], h=1.1, x0=4.45, w_total=8.4)

# 10 --------------------------------------------------------------- writing task + N
sl = slide("The Test: Writing on a Wall",
           "[Sahil] SAY: Here is the NMPC writing HELLO WPI, at double speed. Red is ink, where the pen presses; thin green lines are the letters it should follow. "
           "The drone flies along the wall while the arm draws the letters. Letters are 5 cm tall, the pen moves at 5 cm per second and presses 1 millimetre into the wall. "
           "Sensor noise is realistic, like a motion-capture system: about 0.3 millimetres and 0.1 degrees. "
           "With no disturbance the NMPC finishes all 15 sentences, 1.17 millimetres from the letters on average. That is our reference point.")
video(sl, "nwrite", 0.45, 1.45, 7.6, 4.28)
point(sl, 8.35, 1.5, 4.6, 3.3, ["Red = ink, green = the letters to follow", "Letters 5 cm tall, pen 5 cm/s, presses 1 mm into the wall", "Sensor noise like motion capture: 0.3 mm, 0.1°", "5 sentences × 3 noise seeds = 15 tries per condition"], size=18)
box(sl, 8.35, 4.85, 4.5, 1.6, fill=PINK)
num(sl, 8.55, 4.9, 4.2, "15/15,  1.17 mm", "NMPC, no disturbance: sentences finished, and average pen distance from the letters", size=26)

# 11 --------------------------------------------------------------- where NMPC breaks
sl = slide("Where the NMPC Breaks: When Its Model Is Wrong",
           "[Sahil] SAY: The NMPC is only as good as its model. Here it carries 100 grams it does not know about. Its model says the arm is empty, its predictions are wrong, the drone tilts up to 30 degrees and drifts 12 centimetres, and it cannot keep the pen on the wall: in 11 of 15 tries it loses the wall and never gets back. "
           "With motors slower than the model thinks, it loses control outright, tilting past 40 degrees. A softer wall is subtler: it finishes, but 13.4 millimetres off the letters and pressing about 3 newtons, 6 times more than normal. "
           "We measured why: its model expects a stiff wall, so 0.2 seconds ahead it predicts the pen 6.9 millimetres shallower than it really is, on average. "
           "On the normal wall that prediction error is only 0.24 millimetres. Adding a disturbance observer fixes the load cases, but not the soft wall. "
           "We also tried pressing to a target force instead of a depth: the soft-wall error dropped from 13.4 to 6.3 millimetres, still a failure, because the error is inside the model.")
video(sl, "p100", 0.45, 1.45, 7.6, 4.28)
say(sl, 8.35, 1.45, 4.6, 0.5, ["15 tries each, NMPC (N):"], size=18, bold=True)
for i, (big, small) in enumerate([("100 g load: 0/15", "can't keep the pen on the wall: its model thinks the arm is empty"), ("Slow motors: 0/15", "loses control (tilt > 40°): motors slower than the model expects"),
                                  ("Soft wall: 13.4 mm", "finishes, but off the letters, pressing ~3 N (6× normal)")]):
    say(sl, 8.35, 2.0 + i * 1.1, 4.6, 0.5, [big], size=22, bold=True, color=RED); say(sl, 8.35, 2.48 + i * 1.1, 4.6, 0.6, [small], size=15)
box(sl, 8.35, 5.35, 4.5, 1.25, fill=WHITE, line=RED)
say(sl, 8.5, 5.35, 4.25, 1.25, [[("Why: ", {"bold": True, "color": RED}), ("on the soft wall it predicts the pen 6.9 mm shallower than reality (0.24 mm on the normal wall)", {})]], size=16, anchor=MSO_ANCHOR.MIDDLE)
say(sl, 0.45, 5.85, 7.6, 0.7, ["N with an unknown 100 g load, 1× speed"], size=15, color=GR)

# 12 --------------------------------------------------------------- RL
sl = slide("Step 4: A Learned Controller (Reinforcement Learning)",
           "[Sahil] SAY: The second approach learns instead of modelling. A neural network looks at 300 numbers: the pen and drone errors, speeds, joint angles, the measured pen force, over the last 4 time steps. "
           "It outputs 10 numbers 100 times a second: how the drone should accelerate and turn, and where the 3 joints should go; a simple inner loop turns those into rotor speeds and joint torques. "
           "It is trained by trial and error, PPO, 20 million steps, about 3 and a half hours on 30 cores of Turing, and during training we randomly change the load, the motor speed, the wind and the thrust, "
           "so it learns to cope without being told. Here both carry no load but the motors are slower than expected. Left, the NMPC with an observer: it loses control. "
           "Right, the learned controller: it writes the sentence, and finishes 14 of 15 in the benchmark.")
flow(sl, 1.45, [("300 inputs", "errors, speeds, joints, pen force; last 4 steps"), ("Neural network", "2 layers × 256"), ("10 outputs, 100×/s", "drone acceleration, turn rates, 3 joint targets"), ("Inner loop", "→ rotor speeds, joint torques")], h=1.2)
video(sl, "mslow", 0.45, 2.85, 12.4, 3.49)
say(sl, 0.45, 6.38, 12.4, 0.4, ["Slow motors.  Left: NMPC + observer (loses control).  Right: RL (14/15 in the benchmark).  Trained with random loads, motor speeds, wind, thrust: PPO, 20 M steps, ~3.5 h on 30 cores."], size=14, color=GR)

# 13 --------------------------------------------------------------- NMPC + RL
sl = slide("NMPC + RL: Learn Only What the Model Gets Wrong",
           "[Sahil] SAY: The hybrid keeps the NMPC planner and lets a small network correct only its model: it adds two numbers, an extra push at the pen, inside the NMPC's prediction. "
           "The planning, the constraints and the physics stay model-based. Same soft wall as before. Left: NMPC with an observer, 17.8 millimetres off and pressing hard. "
           "Right: with the learned correction, 2.1 millimetres, all 15 sentences, and a light touch. Why it works: its prediction of the pen depth 0.2 seconds ahead is now off by 0.06 millimetres on average, "
           "instead of 6.9. The cost: about 2 milliseconds more computing per step, and the arm moves more busily.")
video(sl, "soft", 0.45, 1.45, 12.4, 3.49)
num(sl, 0.55, 5.1, 4.0, "17.8 → 2.1 mm", "soft wall, 15 tries: pen distance from the letters, NMPC + observer vs NMPC + RL", size=28)
num(sl, 4.85, 5.1, 4.0, "6.9 → 0.06 mm", "average error of the predicted pen depth 0.2 s ahead: the model is right again", size=28)
num(sl, 9.0, 5.1, 3.9, "2 numbers", "all the network adds: an extra push at the pen inside the NMPC's model", size=28)

# 14 --------------------------------------------------------------- comparison
sl = slide("Comparison: 15 Tries per Condition",
           "[Sahil] SAY: Each cell is 15 tries: how many sentences were finished, and the average pen distance from the letters. Green: all finished and within 3 millimetres. "
           "Yellow: some failed, or 3 to 5 millimetres off. Red: none finished, or more than 5 millimetres off. "
           "The plain NMPC is the most precise when its model is right, 1.17 millimetres, but fails with a load, slow motors and the soft wall. The observer fixes the loads. "
           "The learned controller is the only one that copes with slow motors, but struggles with 200 grams, near the shoulder's limit. "
           "The hybrid handles loads and the soft wall, but not slow motors. The last column is the computing time per step on this PC: the NMPC needs about 10 milliseconds of its 20, the network about 1.")
cases = ["No disturbance", "100 g load", "200 g load", "Slow motors", "Soft wall", "Wind gusts", "Compute / step"]
data = [("NMPC (N)", [(15, 1.17), (0, None), (0, None), (0, None), (15, 13.4), (15, 1.22)], "9.6 ms"),
        ("NMPC + observer", [(15, 1.91), (15, 2.76), (15, 4.05), (0, None), (15, 17.8), (15, 1.91)], "10.9 ms"),
        ("RL", [(15, 1.86), (15, 2.32), (7, 2.79), (14, 2.49), (15, 1.75), (15, 1.90)], "1.1 ms"),
        ("NMPC + RL", [(15, 1.79), (15, 2.39), (15, 3.78), (0, None), (15, 2.11), (15, 1.78)], "12.0 ms")]
tb = sl.shapes.add_table(5, 8, Inches(0.45), Inches(1.45), Inches(12.45), Inches(4.4)).table
tb.columns[0].width = Inches(2.35)
for j in range(1, 8): tb.columns[j].width = Inches(1.44)


def cell(c, lines, fill, sizes=(19, 14), bold0=True, color0=DK):
    c.fill.solid(); c.fill.fore_color.rgb = fill; c.vertical_anchor = MSO_ANCHOR.MIDDLE; tf = c.text_frame; tf.margin_left = tf.margin_right = Inches(0.04)
    for k, t in enumerate(lines):
        p = tf.paragraphs[0] if k == 0 else tf.add_paragraph(); p.alignment = PP_ALIGN.CENTER; r = p.add_run(); r.text = t
        r.font.size = Pt(sizes[min(k, 1)]); r.font.bold = bold0 and k == 0; r.font.name = TNR; r.font.color.rgb = color0 if k == 0 else GR


cell(tb.cell(0, 0), [""], RED)
for j, h in enumerate(cases): cell(tb.cell(0, j + 1), [h], RED, sizes=(15, 14), color0=WHITE)
for i, (k, row, ms) in enumerate(data):
    cell(tb.cell(i + 1, 0), [k], CARD, sizes=(18, 14))
    for j, (n, e) in enumerate(row):
        f = OK if (n == 15 and e <= 3) else (BAD if n == 0 or (e or 0) > 5 else MID)
        cell(tb.cell(i + 1, j + 1), [f"{n} / 15", "failed" if e is None else f"{e:.2f} mm"], f)
    cell(tb.cell(i + 1, 7), [ms], WHITE, sizes=(18, 14))
tb.rows[0].height = Inches(0.7)
for i in range(1, 5): tb.rows[i].height = Inches(0.92)
say(sl, 0.45, 5.95, 12.45, 0.8, [[("Each cell: ", {"bold": True, "color": RED}), ("sentences finished of 15, and average pen distance from the letters.  Green: all finished, ≤ 3 mm.  Yellow: some failed or 3–5 mm.  Red: none finished or > 5 mm.  Compute: time per control step, this PC.", {})]], size=15)

# 15 --------------------------------------------------------------- lessons / next
sl = slide("What Failed, What We Learned, What Is Next",
           "[Sahil] SAY: What failed: nothing survives motors three times slower, or several problems at once. The NMPC misses its 20 millisecond deadline on about 3.5 percent of steps. "
           "And pressing to a target force did not fix the soft wall, because the error is inside the model. What we learned: knowing the load matters, for the swing and for the gripper. "
           "A good model is the most precise. Learning copes with slow motors. And the best use of learning was to correct the model, not replace it. "
           "Next, for November 27: a bigger learned correction for motor lag, which is training now; the learned controllers retrained at the realistic noise level; "
           "and closing the loop between the two halves of this project: feeding Ashish's analytical load mass into the NMPC's model, so it knows the load instead of having to learn it.")
cols = [("What failed", BAD, ["Motors 3× slower, or several problems at once: every controller", "NMPC misses its 20 ms deadline on ~3.5 % of steps", "Pressing to a target force did not fix the soft wall (13.4 → 6.3 mm)"]),
        ("What we learned", OK, ["Knowing the load helps: swing 2.7× faster, mass 96 %", "A correct model is the most precise (1.17 mm)", "Learning copes with slow motors", "Best use of learning: correct the model (17.8 → 2.1 mm)"]),
        ("Next (Nov 27)", PINK, ["Larger learned correction for motor lag (trained, testing now)", "Learned controllers retrained at realistic noise", "Feed the analytical load mass into the NMPC's model"])]
for i, (h, fill, items) in enumerate(cols):
    x = 0.45 + i * 4.18; box(sl, x, 1.5, 3.98, 5.1, fill=fill)
    say(sl, x + 0.2, 1.6, 3.6, 0.5, [h], size=22, bold=True, color=RED if i != 1 else DK); point(sl, x + 0.2, 2.3, 3.6, 4.3, items, size=20)

OUT = os.path.join(HERE, "2026-11-22_0905_Capstone_Carry_Touch_Write.pptx")
prs.save(OUT); print("saved", len(prs.slides), OUT)
