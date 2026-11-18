"""Combined capstone deck 2026-11-18: Ashish's analytic load-mass results (4-joint flight robot) + Sahil's PoE torque limit and wall-writing update. 8 slides, WPI template."""
exec(open("/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck2/helpers.py").read())
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION

LGR = RGBColor(0xB4, 0xB8, 0xBC)


def bars(sl, x, y, w, h, title, cats, vals, colors, fmt, vmax):
    cd = CategoryChartData(); cd.categories = cats; cd.add_series("s", vals)
    ch = sl.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(x), Inches(y), Inches(w), Inches(h), cd).chart
    ch.has_legend = False; ch.has_title = True; ch.chart_title.text_frame.text = title
    t = ch.chart_title.text_frame.paragraphs[0].runs[0].font; t.size = Pt(14); t.bold = True; t.color.rgb = DK; t.name = "Arial"
    pl = ch.plots[0]; pl.gap_width = 60; pl.has_data_labels = True; dl = pl.data_labels; dl.number_format = fmt; dl.number_format_is_linked = False
    dl.position = XL_LABEL_POSITION.OUTSIDE_END; dl.font.size = Pt(13); dl.font.bold = True; dl.font.color.rgb = DK; dl.font.name = "Arial"
    for i, c in enumerate(colors):
        pt = pl.series[0].points[i]; pt.format.fill.solid(); pt.format.fill.fore_color.rgb = c
    va = ch.value_axis; va.maximum_scale = vmax; va.minimum_scale = 0; va.visible = False; va.has_major_gridlines = False
    ca = ch.category_axis; ca.tick_labels.font.size = Pt(12); ca.tick_labels.font.name = "Arial"; ca.tick_labels.font.color.rgb = DK; ca.format.line.color.rgb = LGR
    return ch


# 1 title
sl = prs.slides.add_slide(L["Title Slide"])
sl.shapes.title.text = "Weighing the Load with the Arm's Own Joints"
sl.placeholders[1].text = "An analytical load-mass method for an aerial manipulator, and the wall-writing update\nAshish Sukumar and Sahil Gajera\nRBE 594 Capstone  |  November 18, 2026"
sl.notes_slide.notes_text_frame.text = "Two parts. First our analytical way to weigh the load the arm carries, and what it does for flight. Then the wall-writing task, where the same model explains where our controllers stop working."

# 2 idea
sl = slide("The Idea: the Arm Can Weigh Its Load",
           "A load in the gripper changes the thrust the drone needs, where its centre of mass is, and the torque every joint must give. Prof. Rosendo's point: we know every link length and every joint angle, and the load sits at the end effector, so its mass is the only unknown. One equation gives it, no Bayesian estimator. Why it matters: a drone that does not know its load misses its path by 38.7 cm on average; told the true mass, 19.1 cm.")
pic(sl, "ash_robot.png", 6.7, 1.45, w=6.2)
text(sl, 0.55, 1.6, 5.9, 0.5, ["A load in the gripper changes:"], size=18, bold=True)
bullets(sl, 0.55, 2.15, 5.9, 1.5, [[("the ", {}), ("thrust", {"bold": True}), (" the drone needs", {})], [("where the ", {}), ("centre of mass", {"bold": True}), (" is", {})], [("the ", {}), ("torque", {"bold": True}), (" each joint must give", {})]], size=16)
box(sl, 0.55, 3.65, 5.9, 1.15, fill=WHITE, line=RED)
text(sl, 0.8, 3.65, 5.45, 1.15, [[("Idea (Prof. Rosendo): ", {"bold": True, "color": RED}), ("link lengths and joint angles are known, so the load's ", {}), ("mass is the only unknown", {"bold": True}), (". One equation gives it.", {})]], size=15, anchor=MSO_ANCHOR.MIDDLE)
for i, (big, lab) in enumerate([("38.7 cm", "path error when the drone does not know the load"), ("19.1 cm", "path error when it is told the true mass")]):
    x = 0.55 + i * 3.05; box(sl, x, 5.05, 2.85, 1.45); stat(sl, x + 0.2, 5.15, 2.5, big, lab, size=30)
text(sl, 6.7, 6.15, 6.2, 0.4, ["1.05 kg quadrotor, 4-joint 0.47 kg arm (MuJoCo); load at the end effector"], size=12, color=GR)

# 3 how
sl = slide("How It Works: Three Steps, One Calibration",
           "Step 1: product of exponentials, every joint a screw axis in the drone body frame, body tilt from the IMU. Step 2: the joint torques carry the arm plus the load, and the load's share is linear in its mass: a_e, the torque per kilogram, is known from the angles, IMU and link lengths. Step 3: least squares over the last second gives m in closed form. Our formulas match the physics engine to 1e-8 N m. Real torque readings are not exact, though: about 10 % scale error, an offset, gearbox friction. One bench calibration with a known 0.2 kg weight fixes that: 79 % to 96 % accurate.")
steps = [("Kinematic chain", "Product of exponentials: each joint a screw axis in the body frame {b}", "T = e^[S₁]q₁ ··· e^[S₄]q₄ M"),
         ("Torque balance", "The load's share of the joint torque is linear in its mass", "τ = τ_arm + m aₑ"),
         ("Mass, closed form", "Least squares over the joints, last 1 s; nothing learned", "m = Σ aₑ·(τ − τ_arm) / Σ aₑ·aₑ")]
for i, (t, b, f) in enumerate(steps):
    x = 0.55 + i * 4.15; box(sl, x, 1.6, 3.8, 2.7); badge(sl, x + 0.2, 1.8, i + 1)
    text(sl, x + 0.9, 1.85, 2.8, 0.5, [t], size=18, bold=True); text(sl, x + 0.2, 2.55, 3.4, 0.8, [b], size=13, color=GR)
    text(sl, x + 0.2, 3.45, 3.4, 0.6, [f], size=15, bold=True, color=RED)
    if i < 2: arrow(sl, x + 3.83, 2.95, x + 4.12, 2.95)
box(sl, 0.55, 4.6, 3.8, 1.9, fill=RED); text(sl, 0.8, 4.75, 3.35, 1.6, [[("Exact", {"bold": True, "size": 22, "color": WHITE})], [("Matches the physics engine to 10⁻⁸ N·m on 200 random states", {"color": WHITE})]], size=14)
box(sl, 4.7, 4.6, 8.1, 1.9)
text(sl, 4.95, 4.72, 7.7, 0.45, [[("But real torque readings are not exact", {"bold": True, "color": RED})]], size=16)
text(sl, 4.95, 5.2, 4.3, 1.2, ["~10 % scale error, offset, gearbox friction, link masses ±5 %"], size=13, color=GR)
text(sl, 4.95, 5.85, 4.3, 0.6, ["Fix: one bench calibration with a known 0.2 kg weight"], size=13, bold=True)
text(sl, 9.5, 4.85, 3.2, 0.7, ["79 % → 96 %"], size=30, bold=True, color=RED); text(sl, 9.5, 5.55, 3.2, 0.8, ["accuracy, same formula, after calibration"], size=13, color=GR)

# 4 results
sl = slide("Results: Accurate Mass, Near-Perfect Flight",
           "Tested in a realistic simulator: rotor lag, thrust error, wind, servo friction, noisy and biased sensors, on 300 flights the method never saw. Left: load-mass accuracy on the same 55 flights for every estimator; the calibrated analytic method reads 96 %, an average error of 7.6 g, ahead of our earlier Bayesian estimate, because joint torques do not feel wind and drag. Right: tracking error when each estimate drives the controller. The analytic estimate halves the error, 38.7 to 19.8 cm, within 0.7 cm of a controller told the true mass.")
bars(sl, 0.4, 1.5, 6.2, 4.0, "Load-mass accuracy (higher is better)", ["Analytic\n(ours)", "Analytic,\nuncalibrated", "Bayesian,\nthrust calib.", "Bayesian,\nbefore"], [96.4, 79.2, 88.0, 69.9], [RED, LGR, GR, LGR], '0.0"%"', 110)
bars(sl, 6.75, 1.5, 6.2, 4.0, "Path error, cm (lower is better)", ["No\nestimate", "Analytic\n(ours)", "Bayesian,\nbefore", "Bayesian,\nthrust calib.", "True mass\n(best case)"], [38.7, 19.8, 20.6, 17.7, 19.1], [LGR, RED, GR, GR, DK], '0.0', 45)
for i, (big, lab) in enumerate([("7.6 g", "average mass error on 0.1-0.2 kg loads"), ("54 of 55", "flights at 90 % accuracy or better"), ("38.7 → 19.8 cm", "path error, no estimate vs ours"), ("0.7 cm", "from a controller told the true mass")]):
    stat(sl, 0.55 + i * 3.15, 5.65, 3.0, big, lab, size=24)

# 5 video frame
sl = slide("In Flight: Grab and Release",
           "A test flight: zigzag path, a 0.15 kg parcel picked up from a pedestal at 5 s and set down at 14 s. The controller flies on the analytic estimate. Black is the true load, blue our analytic estimate: it reaches the new load about a second after the grab and stays there. The Bayesian estimates, red and tan, wander when the drone turns hard.")
pic(sl, "ash_grab.png", 0.5, 1.5, w=8.6)
text(sl, 0.5, 6.4, 8.6, 0.4, ["Test flight, seed 200: 0.15 kg parcel grabbed at 5 s, released at 14 s; the controller flies on the analytic estimate"], size=12, color=GR)
box(sl, 9.45, 1.5, 3.4, 4.85)
text(sl, 9.7, 1.65, 3.0, 0.5, ["What to watch"], size=18, bold=True, color=RED)
bullets(sl, 9.7, 2.25, 3.0, 4.0, [[("Black", {"bold": True}), (": true load", {})], [("Blue", {"bold": True}), (": ours, on the new load ~1 s after the grab", {})],
                                 [("Red / tan", {"bold": True}), (": Bayesian, wander in hard turns", {})], [("Bottom", {"bold": True}), (": each estimate's error, live", {})]], size=14)

# 6 torque limit
sl = slide("How Much Can the Arm Carry?",
           "The same kinematic chain, now on our wall-writing robot, which uses this arm with the wrist fixed. Its Jacobian gives the static shoulder torque a load needs: 0.47 N m for the arm plus 2.19 N m per kilogram at the hand. The servo is rated 1.0 N m, so the limit is 242 g, and at 200 g the shoulder already uses 91 %. The simulation agrees: our learned controllers write with up to 180 g and flip at 200 g, the moment the shoulder saturates. The same model also gives closed-form inverse kinematics, 18 times faster than our numerical solver, and the load mass to about 5 g.")
pic(sl, "f_torque.png", 0.4, 1.5, w=8.4)
stat(sl, 9.25, 1.55, 3.65, "242 g", "most the shoulder can hold: 1.0 N·m servo limit")
stat(sl, 9.25, 2.95, 3.65, "91 %", "of the limit used at 200 g: no margin left")
stat(sl, 9.25, 4.35, 3.65, "18× / ±5 g", "faster closed-form IK; load mass on this robot")
text(sl, 0.5, 6.05, 8.3, 0.5, [[("Simulation agrees: ", {"bold": True, "color": RED}), ("learned controllers write with 180 g and flip at 200 g, when the shoulder saturates", {})]], size=13)

# 7 wall writing
sl = slide("Wall Writing: NMPC vs Learning",
           "The task: write whole sentences on a wall with a pen pad, while the drone flies along the text. We compare our NMPC with a disturbance observer against our reinforcement-learning policy, on 5 sentences times 3 noise seeds per condition, 15 runs per cell, no hand-picked runs. RL is now the most accurate, 1.87 mm with no disturbance, and handles slow motors where the NMPC fails. The NMPC handles the 200 g load where RL still fails 10 of 15, exactly the torque limit from the previous slide. The video shows that 200 g case.")
sl.shapes.add_movie(D + "v_200.mp4", Inches(0.45), Inches(1.5), Inches(6.4), Inches(3.6), poster_frame_image=D + "p_200.png", mime_type="video/mp4")
text(sl, 0.45, 5.15, 6.4, 0.4, ["200 g at the hand. Top: NMPC + observer writes; early RL flips. Bottom: newer RL policies"], size=11, color=GR)
cases = ["no disturbance", "100 g load", "200 g load", "slow motors", "wind"]
data = [("NMPC + observer", [(15, 2.62), (15, 2.65), (15, 3.05), (0, None), (15, 2.70)]), ("RL (latest)", [(15, 1.87), (15, 2.32), (5, 2.75), (11, 2.47), (15, 2.64)])]
tb = sl.shapes.add_table(6, 3, Inches(7.15), Inches(1.5), Inches(5.7), Inches(3.9)).table
tb.columns[0].width = Inches(1.9); tb.columns[1].width = Inches(1.9); tb.columns[2].width = Inches(1.9)
for j, h in enumerate(["sentences done / 15", "NMPC + observer", "RL (latest)"]):
    c = tb.cell(0, j); c.text = h; c.fill.solid(); c.fill.fore_color.rgb = RED; f = c.text_frame.paragraphs[0].runs[0].font; f.color.rgb = WHITE; f.bold = True; f.size = Pt(12); f.name = "Arial"
for i, cs in enumerate(cases):
    c = tb.cell(i + 1, 0); c.text = cs; c.fill.solid(); c.fill.fore_color.rgb = CARD; c.vertical_anchor = MSO_ANCHOR.MIDDLE
    f = c.text_frame.paragraphs[0].runs[0].font; f.bold = True; f.size = Pt(13); f.name = "Arial"; f.color.rgb = DK
    for j, (_, row) in enumerate(data):
        n, e = row[i]; c = tb.cell(i + 1, j + 1); c.vertical_anchor = MSO_ANCHOR.MIDDLE; c.fill.solid(); c.fill.fore_color.rgb = OK if n == 15 else (BAD if n == 0 else MID)
        p = c.text_frame.paragraphs[0]; p.alignment = PP_ALIGN.CENTER; r = p.add_run(); r.text = f"{n}" + ("" if e is None else f"   {e:.2f} mm"); r.font.size = Pt(14); r.font.name = "Arial"; r.font.color.rgb = DK; r.font.bold = n == 15
for r in range(6): tb.rows[r].height = Inches(0.62)
for i, (big, lab) in enumerate([("1.87 mm", "RL tracing error, best so far"), ("200 g", "NMPC writes it; RL 5/15 (torque limit)"), ("slow motors", "RL writes 11/15; NMPC fails")]):
    stat(sl, 0.55 + i * 4.2, 5.65, 3.9, big, lab, size=24)

# 8 next
sl = slide("Next Steps: Final Presentation on November 27",
           "Our research topic is payload-aware aerial manipulation: the drone can weigh its load, so how should it use that mass to carry heavier loads more safely? Four threads to the final presentation. Payload-aware reach: from the measured mass, compute in closed form how far forward the load can be held. Robustness: wind and torque-sensor drift. Against the state of the art: L1 adaptive control on the same flights. And for wall writing: put the torque limit into the controllers and finish the RL run training on heavier loads and slower motors. Limits: everything is simulation, the calibration needs a known weight once, and a drifting torque sensor needs recalibrating.")
cards = [("Payload-aware reach", "How far can the arm reach with a given load and still fly? Closed form from the measured mass."),
         ("Robustness", "When does the estimate stop being trustworthy? Wind gusts and torque-sensor drift."),
         ("vs. state of the art", "Is knowing the load better than adapting to it? L1 adaptive control, same test flights."),
         ("Wall writing", "Controllers that respect the 1.0 N·m limit; RL now training on heavier loads and slower motors.")]
for i, (t, b) in enumerate(cards):
    x = 0.55 + i * 3.1; box(sl, x, 1.6, 2.85, 3.35, fill=CARD if i else RGBColor(0xFB, 0xEB, 0xED)); badge(sl, x + 0.2, 1.8, i + 1)
    text(sl, x + 0.2, 2.5, 2.5, 0.7, [t], size=17, bold=True, color=RED if i == 0 else DK); text(sl, x + 0.2, 3.25, 2.5, 1.6, [b], size=13, color=GR)
text(sl, 0.55, 5.2, 12.2, 0.5, [[("Limits: ", {"bold": True, "color": RED}), ("simulation only; calibration needs a known weight once per robot; a drifting torque sensor needs recalibrating.", {})]], size=14)
text(sl, 0.55, 5.75, 12.2, 0.9, ["Lynch & Park, Modern Robotics, 2017 (product of exponentials)  ·  Kane & Levinson, Dynamics, 1985  ·  Wu et al., L1Quad, arXiv 2302.07208, 2023  ·  Hanover et al., adaptive NMPC for quadrotors, IEEE RA-L 2022  ·  Ruggiero et al., Aerial manipulation review, IEEE RA-L 2018  ·  Todorov et al., MuJoCo, IROS 2012"], size=10, color=GR)

OUT = "/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck2/2026-11-18_1330_Capstone_Combined_Ashish_Sahil.pptx"
prs.save(OUT); print("saved", len(prs.slides))
