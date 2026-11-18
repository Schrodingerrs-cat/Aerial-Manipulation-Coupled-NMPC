"""Capstone deck v3, 2026-11-18: simpler, larger Times New Roman text, every number explained, at most two videos per slide."""
exec(open("/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck2/helpers3.py").read())
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION

LGR = RGBColor(0xB4, 0xB8, 0xBC); TNR = "Times New Roman"; PINK = RGBColor(0xFB, 0xEB, 0xED)


def slide(title, notes):
    sl = prs.slides.add_slide(L["Title Only"]); sl.shapes.title.text = title
    for r in sl.shapes.title.text_frame.paragraphs[0].runs: r.font.name = TNR; r.font.size = Pt(38)
    sl.notes_slide.notes_text_frame.text = notes; return sl


def say(sl, x, y, w, h, paras, size=20, **kw):
    return text(sl, x, y, w, h, paras, size=size, **kw)


def point(sl, x, y, w, h, items, size=20):
    tb = bullets(sl, x, y, w, h, items, size=size)
    for p in tb.text_frame.paragraphs: p.space_after = Pt(12)
    return tb


def num(sl, x, y, w, big, meaning, size=36):
    """A number with a full-sentence meaning under it."""
    say(sl, x, y, w, 0.7, [big], size=size, color=RED, bold=True); say(sl, x, y + 0.72, w, 1.1, [meaning], size=16, color=DK)


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
    ca = ch.category_axis; ca.tick_labels.font.size = Pt(15); ca.tick_labels.font.name = TNR; ca.tick_labels.font.color.rgb = DK; ca.format.line.color.rgb = LGR


# 1 title
sl = prs.slides.add_slide(L["Title Slide"])
sl.shapes.title.text = "Weighing the Load with the Arm's Own Joints"
sl.placeholders[1].text = "An analytical load-mass method for an aerial manipulator\nAshish Sukumar and Sahil Gajera\nRBE 594 Capstone  |  November 18, 2026"
for ph in (sl.shapes.title, sl.placeholders[1]):
    for p in ph.text_frame.paragraphs:
        for r in p.runs: r.font.name = TNR
sl.notes_slide.notes_text_frame.text = "Today: how our drone can weigh the load in its gripper using only its own arm, why that is better than what we did before, and what it tells us about our wall-writing task."

# 2 problem
sl = slide("The Problem: The Drone Does Not Know Its Load",
           "SAY: Our drone carries an arm with a gripper. When it picks something up, it gets heavier, its balance point moves forward, and the arm's motors have to work harder. The flight controller plans its thrust using the drone's mass. If it does not know the load, it gets the thrust wrong and drifts off its path: 38.7 cm on average in our test flights. If we tell it the true mass, the same flights stay within 19.1 cm. So knowing the load halves the error. The question is how to find the mass in flight.")
pic(sl, "ash_robot.png", 6.6, 1.5, w=6.3)
point(sl, 0.55, 1.6, 5.9, 2.4, ["A load in the gripper makes the drone heavier and pulls it forward",
                                "The controller needs the mass to set the right thrust",
                                "Wrong mass → the drone drifts off its path"], size=20)
box(sl, 0.55, 4.25, 2.9, 2.3); num(sl, 0.75, 4.35, 2.55, "38.7 cm", "average distance from the path when the drone does NOT know the load")
box(sl, 3.6, 4.25, 2.9, 2.3); num(sl, 3.8, 4.35, 2.55, "19.1 cm", "the same flights when it IS told the true mass: half the error")
say(sl, 6.6, 6.25, 6.3, 0.4, ["Our robot: 1.05 kg quadrotor with a 4-joint arm, simulated in MuJoCo"], size=14, color=GR)

# 3 before vs now
sl = slide("Before vs. Now: Two Ways to Find the Load",
           "SAY: Before, we used a Bayesian estimator. It watches the whole drone: how hard the rotors push and how the drone actually accelerates. Any missing push must be the load, and a Kalman filter keeps a running best guess with an uncertainty. The weakness: wind, drag and a slightly weak rotor also change how the drone moves, and the filter mistakes them for load. It read the mass only 70 percent accurately. Now, following Prof. Rosendo's suggestion, we use the arm itself as a scale. We know every link length and every joint angle, so the only unknown is the mass, and one formula gives it. The joint motors feel the load hanging on them but not the wind on the drone body. It reads the mass 96 percent accurately. Accuracy here means how close the estimated mass is to the true mass: 96 percent on a 150 g load is about 6 g off.")
for x, head, sub, items, big, fill in [(0.55, "Before: Bayesian estimate", "watch the whole drone", ["Compares rotor push with how the drone accelerates", "A filter keeps a running best guess", "Wind, drag and rotor errors look like load"], ("70 % accurate", "its mass reading is off by about 30 % of the load"), CARD),
                                       (6.85, "Now: analytical method", "use the arm as a scale (Prof. Rosendo)", ["Link lengths and joint angles are known", "So the mass is the only unknown: one formula", "Joint motors feel the load, not the wind"], ("96 % accurate", "off by only about 4 %: ~6 g on a 150 g load"), PINK)]:
    box(sl, x, 1.55, 5.95, 4.15, fill=fill)
    say(sl, x + 0.3, 1.7, 5.4, 0.5, [head], size=24, bold=True, color=RED if fill == PINK else DK); say(sl, x + 0.3, 2.25, 5.4, 0.4, [sub], size=16, color=GR)
    point(sl, x + 0.3, 2.8, 5.4, 1.9, items, size=19)
    say(sl, x + 0.3, 4.7, 5.4, 0.6, [big[0]], size=32, bold=True, color=RED); say(sl, x + 0.3, 5.25, 5.4, 0.4, [big[1]], size=17)
box(sl, 0.55, 5.9, 12.25, 0.75, fill=WHITE, line=RED)
say(sl, 0.8, 5.9, 11.8, 0.75, [[("Accuracy", {"bold": True, "color": RED}), (" = how close the estimated mass is to the true mass. 96 % on a 150 g load means about 6 g off.", {})]], size=18, anchor=MSO_ANCHOR.MIDDLE)

# 4 idea
sl = slide("The Idea: Use the Arm as a Scale",
           "SAY: Think of holding a shopping bag with your arm stretched out. Your shoulder feels the weight, and it feels more the further out the bag is. The torque at the shoulder equals the weight times the lever arm, the horizontal distance from the shoulder to the bag. Our arm's motors measure their torque, and the lever arm comes straight from the joint angles and link lengths, which we know exactly. So the mass is the measured torque divided by g times the lever arm. That is the whole idea. We call it analytical because it is a formula we solve directly: no filter that slowly converges, no search, nothing learned from data.")
pic(sl, "f_lever.png", 0.4, 1.55, w=6.3)
box(sl, 7.0, 1.6, 5.85, 1.45, fill=CARD)
say(sl, 7.25, 1.62, 5.4, 1.4, [[("torque = m \u00b7 g \u00b7 d", {"bold": True, "size": 28})], [("so   m = torque / (g \u00b7 d)", {"bold": True, "size": 28, "color": RED})]], anchor=MSO_ANCHOR.MIDDLE, align=PP_ALIGN.CENTER)
point(sl, 7.0, 3.15, 5.85, 2.3, ["Like a shoulder holding a bag at arm's length", "Motors measure the torque", "Angles and lengths give the lever arm d exactly"], size=20)
box(sl, 7.0, 5.15, 5.85, 1.35, fill=WHITE, line=RED)
say(sl, 7.25, 5.15, 5.4, 1.35, [[("Analytical", {"bold": True, "color": RED}), (" = solved directly with a formula: no filter, no search, nothing learned.", {})]], size=19, anchor=MSO_ANCHOR.MIDDLE)

# 5 PoE
sl = slide("Finding the Lever Arm: Product of Exponentials",
           "SAY: For a real arm with several joints, the lever arm is not one number, so we need the full geometry. We use the product of exponentials from Modern Robotics. We put the reference frame on the drone body. Each joint is described by its rotation axis, and its rotation is one matrix exponential. Multiplying them in order from the body out to the gripper gives exactly where the gripper is: forward kinematics. Taking the derivative gives the Jacobian, which tells how a force at the gripper turns into torque at every joint. From that we get a, the torque per kilogram of load at each joint, and the mass is a least-squares fit over all joints. We checked it: our formulas match the simulator's own physics to 10 to the minus 8 newton metres, so the method itself is exact.")
pic(sl, "f_poe.png", 0.4, 1.55, w=6.2)
steps = [("Chain the joints", "T = e^[S₁]q₁ ··· e^[S₃]q₃ M", "body frame → each joint's rotation → gripper position"),
         ("Jacobian", "a = Jᵀ · (gravity)", "a = torque per kg of load, at each joint"),
         ("Mass", "m = Σ a·τ / Σ a·a", "best fit over all joints, last 1 s")]
for i, (t, f, b) in enumerate(steps):
    y = 1.55 + i * 1.6; box(sl, 6.85, y, 6.0, 1.42); badge(sl, 7.0, y + 0.15, i + 1)
    say(sl, 7.7, y + 0.12, 5.05, 0.45, [[(t + ":  ", {"bold": True}), (f, {"bold": True, "color": RED})]], size=18); say(sl, 7.7, y + 0.72, 5.0, 0.6, [b], size=16, color=GR)
say(sl, 0.55, 6.35, 12.3, 0.45, [[("Check: ", {"bold": True, "color": RED}), ("our formulas match the simulator's physics to 10⁻⁸ N·m, so the method is exact.", {})]], size=18)

# 6 calibration
sl = slide("Real Motors Need One Calibration",
           "SAY: The formula is exact, but a real servo does not measure torque directly. It measures its motor current, which is about 10 percent off, has an offset, and includes gearbox friction. We modelled those errors in the simulator. The fix is a one-time bench calibration, like zeroing a kitchen scale: clamp the drone level, sweep the arm empty, sweep it again holding a known 0.2 kg weight, and fit the error of each joint. With the same formula, the average error of the mass estimate on 100 to 200 g loads drops from 32 g to 7.6 g.")
point(sl, 0.55, 1.6, 6.0, 2.5, ["A servo reads motor current, not true torque: about 10 % off", "Plus an offset and gearbox friction"], size=20)
say(sl, 0.55, 3.4, 6.0, 0.5, ["Fix once, like zeroing a kitchen scale:"], size=20, bold=True, color=RED)
for i, s in enumerate(["Clamp the drone level", "Move the arm empty", "Repeat with a known 0.2 kg weight", "Fit each joint's error"]):
    badge(sl, 0.6, 4.05 + i * 0.62, i + 1, d=0.45); say(sl, 1.25, 4.1 + i * 0.62, 5.3, 0.45, [s], size=19)
bars(sl, 7.0, 1.5, 5.8, 4.4, "Average error of the mass estimate (grams)", ["Before calibration", "After calibration"], [32.0, 7.6], [LGR, RED], '0.0" g"', 40)
say(sl, 7.0, 6.0, 5.8, 0.6, ["on 100-200 g loads; lower is better"], size=16, color=GR, align=PP_ALIGN.CENTER)

# 7 results
sl = slide("Results: Better Mass, Better Flight",
           "SAY: We tested on 300 flights the method never saw during tuning, in a realistic simulator with motor lag, wind, sensor noise and model errors. Left: how close each method's mass estimate is to the true mass. Ours, calibrated, is 96 percent; our earlier Bayesian estimate was 70 percent, or 88 percent after we also calibrated its thrust. Right: what that means for flight. Each method's estimate drives the controller. With no estimate the drone is 38.7 cm off its path on average; with ours 19.8 cm, only 0.7 cm worse than a controller that is simply told the true mass.")
bars(sl, 0.4, 1.5, 6.2, 4.5, "How close the estimated mass is to the true mass", ["Ours", "Bayesian\n(calibrated)", "Bayesian\n(before)"], [96.4, 88.0, 69.9], [RED, GR, LGR], '0"%"', 115)
bars(sl, 6.75, 1.5, 6.2, 4.5, "Distance from the path, cm (lower is better)", ["No estimate", "Ours", "Told the\ntrue mass"], [38.7, 19.8, 19.1], [LGR, RED, DK], '0.0', 46)
say(sl, 0.55, 6.15, 12.3, 0.6, ["300 test flights never used for tuning; realistic simulator with motor lag, wind, sensor noise and model errors"], size=16, color=GR)

# 8 grab video frame
sl = slide("In Flight: Picking Up and Setting Down a Parcel",
           "SAY: One test flight. The drone flies a zigzag, picks up a 150 g parcel at 5 seconds and sets it down at 14 seconds, and the controller uses our estimate. In the plot, black is the true load. Blue is our analytical estimate: it jumps to the new load about a second after the grab and stays there. Red and tan are the Bayesian estimates: they wander whenever the drone turns hard, because the turn looks like a change of load to them.")
pic(sl, "ash_grab.png", 0.45, 1.5, w=8.9)
say(sl, 0.45, 6.55, 8.9, 0.4, ["Zigzag flight, 150 g parcel picked up at 5 s, set down at 14 s"], size=14, color=GR)
box(sl, 9.6, 1.5, 3.25, 4.95)
point(sl, 9.8, 1.7, 2.9, 4.6, [[("Black", {"bold": True}), (": true load", {})], [("Blue", {"bold": True}), (": ours, on the new load within 1 s", {})], [("Red, tan", {"bold": True}), (": Bayesian, wander in sharp turns", {})]], size=19)

# 9 carry limit
sl = slide("Turned Around: How Much Can the Arm Carry?",
           "SAY: The same formula also works backwards. For a given load, it tells how much torque the shoulder must give: 0.47 newton metres for the arm itself plus 2.19 per kilogram of load. The shoulder motor can give at most 1.0 newton metre, so the heaviest load it can hold is 242 g. At 200 g the shoulder already works at 91 percent, so there is almost no margin left for the arm swinging. Our wall-writing simulations show exactly this: the learned controllers write fine with 180 g and lose control at 200 g, the moment the shoulder runs out of torque.")
pic(sl, "f_torque2.png", 0.4, 1.5, w=7.9)
num(sl, 8.7, 1.6, 4.2, "242 g", "heaviest load the shoulder motor can hold (its maximum is 1.0 N·m)")
num(sl, 8.7, 3.25, 4.2, "91 %", "of the shoulder's maximum already used at 200 g: no margin for the arm swinging")
box(sl, 8.7, 4.95, 4.15, 1.5, fill=WHITE, line=RED)
say(sl, 8.9, 4.95, 3.8, 1.5, [[("Simulation agrees: ", {"bold": True, "color": RED}), ("learned controllers write with 180 g, lose control at 200 g", {})]], size=17, anchor=MSO_ANCHOR.MIDDLE)

# 10 wall writing video: two side by side
sl = slide("Wall Writing with a 200 g Load",
           "SAY: Our second task: the drone writes a sentence on a wall with a pen pad while flying along it. Both runs carry 200 g in the gripper. Left, our model-based controller, an NMPC that plans ahead with a model of the drone and arm, writes the whole sentence. Right, our learned controller, trained with reinforcement learning, writes most of it, then near the end, where the drone has travelled furthest, it drifts off. That is the torque limit from the previous slide.")
sl.shapes.add_movie(D + "v_nd_r21_200.mp4", Inches(0.45), Inches(1.55), Inches(12.4), Inches(3.49), poster_frame_image=D + "p_nd_r21_200.png", mime_type="video/mp4")
for x, t, b in [(0.45, "Left: NMPC (model-based)", "plans ahead with a model of the drone and arm; writes the whole sentence"),
                (6.65, "Right: RL (learned)", "trained by reinforcement learning; writes most of it, drifts off near the end")]:
    say(sl, x, 5.25, 6.0, 0.45, [t], size=20, bold=True, color=RED); say(sl, x, 5.75, 6.0, 0.8, [b], size=17)

# 11 wall writing results
sl = slide("Wall Writing: Results So Far",
           "SAY: Each controller wrote 5 sentences with 3 different sensor-noise seeds, so 15 tries per condition. Each cell shows how many of the 15 sentences were finished, and the average tracing error in millimetres, how far the pen line is from the letter. Our learned controller is now the most accurate, 1.87 mm with no disturbance, and copes with slow motors where the NMPC fails. The NMPC copes with the 200 g load, where the learned controller finishes only 5 of 15. No controller handles everything yet.")
say(sl, 0.55, 1.5, 12.3, 0.5, [[("Each cell: ", {"bold": True}), ("sentences finished out of 15, and average pen error in mm (how far the line is from the letter)", {})]], size=18)
cases = ["No disturbance", "100 g load", "200 g load", "Slow motors", "Wind"]
data = [("NMPC (model-based)", [(15, 2.62), (15, 2.65), (15, 3.05), (0, None), (15, 2.70)]), ("RL (learned)", [(15, 1.87), (15, 2.32), (5, 2.75), (11, 2.47), (15, 2.64)])]
tb = sl.shapes.add_table(3, 6, Inches(0.55), Inches(2.2), Inches(12.25), Inches(2.6)).table
tb.columns[0].width = Inches(2.75)
for j in range(1, 6): tb.columns[j].width = Inches(1.9)
for j, h in enumerate([""] + cases):
    c = tb.cell(0, j); c.text = h; c.fill.solid(); c.fill.fore_color.rgb = RED; p = c.text_frame.paragraphs[0]; p.alignment = PP_ALIGN.CENTER
    if p.runs: f = p.runs[0].font; f.color.rgb = WHITE; f.bold = True; f.size = Pt(18); f.name = TNR
for i, (k, row) in enumerate(data):
    c = tb.cell(i + 1, 0); c.text = k; c.fill.solid(); c.fill.fore_color.rgb = CARD; c.vertical_anchor = MSO_ANCHOR.MIDDLE
    f = c.text_frame.paragraphs[0].runs[0].font; f.bold = True; f.size = Pt(19); f.name = TNR; f.color.rgb = DK
    for j, (n, e) in enumerate(row):
        c = tb.cell(i + 1, j + 1); c.vertical_anchor = MSO_ANCHOR.MIDDLE; c.fill.solid(); c.fill.fore_color.rgb = OK if n == 15 else (BAD if n == 0 else MID)
        tf = c.text_frame; p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER; r = p.add_run(); r.text = f"{n} / 15"; r.font.size = Pt(24); r.font.bold = True; r.font.name = TNR; r.font.color.rgb = DK
        p2 = tf.add_paragraph(); p2.alignment = PP_ALIGN.CENTER; r2 = p2.add_run(); r2.text = "failed" if e is None else f"{e:.2f} mm"; r2.font.size = Pt(17); r2.font.name = TNR; r2.font.color.rgb = GR
tb.rows[0].height = Inches(0.6); tb.rows[1].height = Inches(1.0); tb.rows[2].height = Inches(1.0)
point(sl, 0.55, 5.1, 12.3, 1.6, [[("RL is the most accurate", {"bold": True}), (" (1.87 mm) and handles slow motors", {})],
                                 [("NMPC handles the 200 g load", {"bold": True}), ("; RL hits the shoulder's torque limit there", {})]], size=20)

# 12 next
sl = slide("Next Steps, Toward November 27",
           "SAY: Next, three questions for the final presentation on November 27. One: now that the drone knows its load, how far forward can it safely hold it? We can compute that in closed form from the measured mass. Two: when does the estimate stop being trustworthy, with wind gusts and a slowly drifting torque sensor? Three: is knowing the load better than adapting to it? We will compare against L1 adaptive control, a published controller that adapts without measuring the load. For wall writing, we are building controllers that respect the shoulder's torque limit and training the learned controller on heavier loads. Limits: all of this is in simulation, the calibration needs a known weight once, and a drifting sensor needs recalibrating.")
for i, (t, q) in enumerate([("Payload-aware reach", "How far can it hold a given load and still fly?"), ("Robustness", "When does the estimate stop being trustworthy?"),
                            ("vs. L1 adaptive control", "Is knowing the load better than adapting to it?"), ("Wall writing", "Respect the shoulder's torque limit; train RL on heavier loads")]):
    x = 0.55 + i * 3.1; box(sl, x, 1.6, 2.85, 3.5, fill=PINK if i == 0 else CARD); badge(sl, x + 0.2, 1.8, i + 1)
    say(sl, x + 0.2, 2.5, 2.5, 0.9, [t], size=21, bold=True, color=RED if i == 0 else DK); say(sl, x + 0.2, 3.45, 2.5, 1.5, [q], size=18, color=DK)
say(sl, 0.55, 5.35, 12.3, 0.9, [[("Limits: ", {"bold": True, "color": RED}), ("simulation only; the calibration needs a known weight once per robot; a drifting torque sensor needs recalibrating.", {})]], size=18)
say(sl, 0.55, 6.25, 12.3, 0.5, ["Lynch & Park, Modern Robotics (2017)  ·  Wu et al., L1Quad (2023)  ·  Hanover et al., IEEE RA-L (2022)  ·  Todorov et al., MuJoCo (2012)"], size=13, color=GR)

OUT = "/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck2/2026-11-18_1430_Capstone_Analytical_Load_Mass.pptx"
prs.save(OUT); print("saved", len(prs.slides))
