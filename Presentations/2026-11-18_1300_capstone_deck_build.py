"""Capstone deck 2026-11-18 on the WPI weekly-progress template (layouts, logo, colours kept; old slides removed)."""
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.oxml.ns import qn
import copy

D = "/tmp/claude-1000/-home-pinaka-arm-optimal/583a49ff-8efa-4056-bfc4-a95dbad4b876/scratchpad/deck/"
RED, DK, GR, CARD, WHITE = RGBColor(0xAB, 0x16, 0x2B), RGBColor(0x27, 0x26, 0x26), RGBColor(0x6D, 0x6D, 0x6D), RGBColor(0xF2, 0xF2, 0xF2), RGBColor(0xFF, 0xFF, 0xFF)
OK, BAD, MID = RGBColor(0xDD, 0xEE, 0xDE), RGBColor(0xF6, 0xD9, 0xDD), RGBColor(0xFB, 0xEB, 0xC8)

prs = Presentation(D + "wp.pptx")
L = {l.name: l for l in prs.slide_layouts}
ids = prs.slides._sldIdLst
for s in list(ids):
    prs.part.drop_rel(s.get(qn("r:id"))); ids.remove(s)


def text(sl, x, y, w, h, paras, size=14, color=DK, bold=False, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, name=None):
    """paras: list of str or list of (str, dict) runs per paragraph."""
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h)); tf = tb.text_frame; tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0; tf.vertical_anchor = anchor
    if name: tb.name = name
    for i, p in enumerate(paras):
        para = tf.paragraphs[0] if i == 0 else tf.add_paragraph(); para.alignment = align; para.space_after = Pt(6)
        for r in (p if isinstance(p, list) else [(p, {})]):
            t, o = r if isinstance(r, tuple) else (r, {})
            run = para.add_run(); run.text = t; f = run.font; f.name = "Arial"; f.size = Pt(o.get("size", size)); f.bold = o.get("bold", bold)
            f.italic = o.get("italic", False); f.color.rgb = o.get("color", color)
            if o.get("sup"): run.font._rPr.set("baseline", "30000")
    return tb


def bullets(sl, x, y, w, h, items, size=15):
    tb = text(sl, x, y, w, h, [[("•  ", {"color": RED, "bold": True}), (t, {})] if isinstance(t, str) else [("•  ", {"color": RED, "bold": True})] + t for t in items], size=size)
    for p in tb.text_frame.paragraphs: p.space_after = Pt(9)
    return tb


def box(sl, x, y, w, h, fill=CARD, line=None, shape=MSO_SHAPE.ROUNDED_RECTANGLE, name=None):
    s = sl.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h)); s.fill.solid(); s.fill.fore_color.rgb = fill
    if line is None: s.line.fill.background()
    else: s.line.color.rgb = line; s.line.width = Pt(1.5)
    s.shadow.inherit = False
    if shape == MSO_SHAPE.ROUNDED_RECTANGLE: s.adjustments[0] = 0.06
    if name: s.name = name
    return s


def badge(sl, x, y, n, d=0.55):
    c = box(sl, x, y, d, d, fill=RED, shape=MSO_SHAPE.OVAL); tf = c.text_frame; tf.margin_left = tf.margin_right = 0
    p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER; r = p.add_run(); r.text = str(n); r.font.size = Pt(18); r.font.bold = True; r.font.color.rgb = WHITE; r.font.name = "Arial"


def arrow(sl, x1, y1, x2, y2, color=RED):
    c = sl.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y1), Inches(x2), Inches(y2)); c.line.color.rgb = color; c.line.width = Pt(2.5)
    ln = c.line._get_or_add_ln(); tail = ln.makeelement(qn("a:tailEnd"), {"type": "triangle", "w": "med", "len": "med"}); ln.append(tail)


def stat(sl, x, y, w, big, small, size=34):
    text(sl, x, y, w, 0.65, [big], size=size, color=RED, bold=True); text(sl, x, y + 0.68, w, 0.7, [small], size=13, color=GR)


def slide(title, notes):
    sl = prs.slides.add_slide(L["Title Only"]); sl.shapes.title.text = title; sl.notes_slide.notes_text_frame.text = notes; return sl


def pic(sl, f, x, y, w=None, h=None):
    return sl.shapes.add_picture(D + f, Inches(x), Inches(y), Inches(w) if w else None, Inches(h) if h else None)


# 1 title
sl = prs.slides.add_slide(L["Title Slide"])
sl.shapes.title.text = "Capstone Project Experience:\nAerial Manipulation"
st = sl.placeholders[1]; st.text = "Analytical kinematics, payload mass and the wall-writing update\nAshish Sukumar and Sahil Gajera\nRBE 594 Capstone  |  November 18, 2026"
sl.notes_slide.notes_text_frame.text = "Two parts today. First the analytical work from last meeting: closed-form kinematics with the product of exponentials, and the payload mass that falls out of it with no Bayesian estimator. Then where the wall-writing task stands, and how the analytical model explains where the controllers break."

# 2 story
sl = slide("Know the Geometry, and the Rest Follows",
           "The thread of the talk. Because we know every joint angle and every link length, the kinematics is closed form. With the closed-form Jacobian, the payload mass is one formula. And with the same Jacobian, the torque the shoulder needs tells us the heaviest load the arm can carry, which is exactly where our controllers start to fail.")
cards = [("Analytical kinematics", "Product of exponentials, space frame on the drone body", "18×", "faster IK than the numerical solver, and exact"),
         ("Payload mass", "One formula from joint torques and the PoE Jacobian", "± 5 g", "for 100 g and 200 g loads, in 0.2 s"),
         ("Where it breaks", "Same Jacobian gives the shoulder torque a load needs", "242 g", "servo limit, matching the sim failures at 200 g"),
         ("Wall-writing task", "NMPC vs RL writing sentences on a wall in contact", "1.87 mm", "best tracing error so far (RL policy R21)")]
for i, (t, wt, big, lab) in enumerate(cards):
    x = 0.55 + i * 3.12; box(sl, x, 1.65, 2.75, 4.75, name=f"Card {i + 1}"); badge(sl, x + 0.2, 1.9, i + 1)
    text(sl, x + 0.2, 2.65, 2.35, 0.75, [t], size=17, bold=True); text(sl, x + 0.2, 3.4, 2.35, 1.2, [wt], size=13, color=GR)
    text(sl, x + 0.2, 4.7, 2.35, 0.6, [big], size=28, color=RED, bold=True); text(sl, x + 0.2, 5.35, 2.35, 0.9, [lab], size=12, color=GR)
    if i < 3: arrow(sl, x + 2.78, 4.0, x + 3.09, 4.0)

# 3 robot and task
sl = slide("The Robot and the Task",
           "Our platform: a quadrotor with a three-joint arm under it, a fixed wrist and a soft pen pad. The task is to write whole sentences on a wall: the pad presses on the wall while the drone flies along the text. This clip is our best RL policy writing HELLO WPI, at real speed, with realistic sensor noise.")
sl.shapes.add_movie(D + "v_r21.mp4", Inches(0.5), Inches(1.55), Inches(7.6), Inches(4.275), poster_frame_image=D + "p_r21.png", mime_type="video/mp4")
text(sl, 0.5, 5.9, 7.6, 0.4, ["RL policy R21 writing “HELLO WPI”, real speed, realistic sensor noise"], size=12, color=GR)
stat(sl, 8.6, 1.55, 4.3, "3 joints", "yaw + two pitch joints; wrist fixed at 20°")
stat(sl, 8.6, 3.0, 4.3, "5 cm letters", "pad pressing about 0.3 N, drone moving along the text")
bullets(sl, 8.6, 4.5, 4.3, 2.0, ["Lengths known: 5, 16, 14 cm links", "Angles known: joint encoders", "So the geometry needs no estimation"], size=14)

# 4 PoE
sl = slide("Analytical Kinematics: Product of Exponentials",
           "We put the space frame {s} on the drone body. Each joint is a screw axis S_i = (omega_i, -omega_i x p_i): joint 1 turns about the body z axis, joints 2 and 3 about y. M is the hand pose with all joints at zero. Forward kinematics is the product of three matrix exponentials times M. The pad sits at a fixed offset in the hand frame.")
pic(sl, "f_poe.png", 0.4, 1.5, w=6.6)
box(sl, 7.35, 1.6, 5.45, 1.35, fill=CARD); text(sl, 7.6, 1.75, 5.0, 1.1, [[("T", {"bold": True}), ("(q)  =  e", {}), ("[S₁]q₁", {"size": 19, "sup": True}), (" e", {}), ("[S₂]q₂", {"size": 19, "sup": True}), (" e", {}), ("[S₃]q₃", {"size": 19, "sup": True}), ("  M", {"bold": True})],
                                                                             [("pad  =  T(q) · c", {"size": 16, "color": GR})]], size=22, anchor=MSO_ANCHOR.MIDDLE)
bullets(sl, 7.35, 3.25, 5.45, 3.0, [[("Space frame {s} = drone body", {"bold": True}), (", so the arm needs no world-frame terms", {})],
                                   [("Screw axes ", {"bold": True}), ("Sᵢ = (ωᵢ, −ωᵢ × pᵢ)", {}), (": joint 1 about z, joints 2 and 3 about y", {})],
                                   [("M", {"bold": True}), (" = hand pose at q = 0; wrist fixed at 20°", {})],
                                   [("Same model gives the space Jacobian", {"bold": True}), (", used next for the mass", {})]], size=14)

# 5 IK
sl = slide("Closed-Form Inverse Kinematics",
           "Inverse kinematics in three steps. The yaw joint is atan2 of the target. In the yawed plane the rest is a planar two-link arm, solved with the law of cosines. That gives up to four answers: yaw or yaw plus pi, elbow up or down. We keep the one closest to the current posture. It is exact and 18 times faster than the numerical least-squares solver we used, which also ends up to 0.7 mrad off because it is pulled towards a preferred posture.")
steps = [("Yaw", "q₁ = atan2(y, x)"), ("Planar 2-link", "q₃ from law of cosines,\nq₂ from the triangle"), ("Pick one of 4", "closest to the current\nposture (no jumps)")]
for i, (t, b) in enumerate(steps):
    x = 0.55 + i * 2.75; box(sl, x, 1.75, 2.4, 2.3); badge(sl, x + 0.2, 1.95, i + 1)
    text(sl, x + 0.2, 2.65, 2.0, 0.5, [t], size=17, bold=True); text(sl, x + 0.2, 3.15, 2.05, 0.9, [b], size=13, color=GR)
    if i < 2: arrow(sl, x + 2.43, 2.9, x + 2.72, 2.9)
tb = sl.shapes.add_table(4, 3, Inches(0.55), Inches(4.45), Inches(7.9), Inches(1.8)).table
rows = [("Check (1000 random poses)", "Result", "Meaning"), ("FK vs simulation model", "4 × 10⁻¹⁶ m", "identical"), ("IK round trip", "4 × 10⁻¹⁵ rad", "exact"), ("Jacobian vs finite differences", "1 × 10⁻¹⁰", "exact")]
for r, row in enumerate(rows):
    for c, v in enumerate(row):
        cell = tb.cell(r, c); cell.text = v; p = cell.text_frame.paragraphs[0]; f = p.runs[0].font; f.size = Pt(13); f.name = "Arial"; f.bold = r == 0
        f.color.rgb = WHITE if r == 0 else DK; cell.fill.solid(); cell.fill.fore_color.rgb = RED if r == 0 else (CARD if r % 2 else WHITE)
tb.columns[0].width = Inches(3.6); tb.columns[1].width = Inches(2.3); tb.columns[2].width = Inches(2.0)
stat(sl, 9.2, 1.75, 3.7, "0.07 ms", "closed form, per solve")
stat(sl, 9.2, 3.2, 3.7, "1.17 ms", "numerical solver (ours before)")
stat(sl, 9.2, 4.65, 3.7, "0.7 mrad", "numerical solver's error; ours is exact")

# 6 mass
sl = slide("Payload Mass in One Formula",
           "A load m at the hand pulls on the joints with torque tau = J^T R^T (-m g e_z). With the angles and lengths known, J is exact from the PoE model, so this is linear in m and one least-squares line gives it. The joint torques come from the momentum observer the controller already runs. Results: 99 and 200 g for 100 and 200 g loads, settled within 0.2 s, about 5 g noise. The key test is the right-most group: with no load but 10 % less thrust, the vertical force alone reads a phantom 170 g. The joint torques do not see thrust error, so ours stays at zero.")
box(sl, 0.5, 1.55, 12.3, 1.05, fill=CARD)
text(sl, 0.75, 1.62, 11.8, 0.9, [[("τ", {"bold": True}), ("  =  J", {}), ("ᵀ", {}), (" R", {}), ("ᵀ", {}), (" (−m g e", {}), ("z", {"size": 14}), (")  =  a m", {}), ("        ⇒        ", {"color": RED}),
                                   ("m  =  aᵀτ / aᵀa", {"bold": True, "color": RED}), ("     no Bayesian estimator needed", {"size": 15, "color": GR})]], size=22, anchor=MSO_ANCHOR.MIDDLE)
pic(sl, "f_mass.png", 0.45, 2.8, w=7.9)
stat(sl, 8.9, 2.85, 4.0, "99 g / 200 g", "for 100 g / 200 g loads, settles in 0.2 s")
stat(sl, 8.9, 4.2, 4.0, "0 g vs 170 g", "no load, thrust −10 %: ours vs vertical force only")
text(sl, 8.9, 5.6, 4.0, 0.9, ["Joint torques ignore thrust and wind errors, so the estimate is not fooled"], size=13, color=DK)

# 7 torque limit (connecting slide)
sl = slide("Torque Model Predicts the Breaking Point",
           "Connecting the analytics to the task. The same PoE Jacobian gives the static shoulder torque a load needs: 0.47 N m for the arm itself plus 2.19 N m per kilogram at the hand. The servo is rated 1.0 N m, so the static limit is 242 g. At 200 g the shoulder already needs 91 % of its torque, and any arm swing tips it into saturation. That is exactly what the simulation shows: our RL policies write up to 180 g and flip at 200 g. The NMPC, which plans within torque limits, still writes at 200 g.")
pic(sl, "f_torque.png", 0.45, 1.5, w=8.3)
stat(sl, 9.2, 1.55, 3.7, "0.47 + 2.19 m", "shoulder torque in N m, m in kg (PoE, writing posture)")
stat(sl, 9.2, 3.0, 3.7, "242 g", "static limit at the 1.0 N m servo")
stat(sl, 9.2, 4.45, 3.7, "91 %", "of the limit used at 200 g: no margin for swing")
text(sl, 0.5, 6.0, 8.3, 0.5, [[("Sim agrees: ", {"bold": True, "color": RED}), ("RL writes up to 180 g, flips at 200 g; the shoulder hits 1.0 N m just before the flip", {})]], size=13)

# 8 controllers
sl = slide("Wall Writing: Three Controllers, One Test",
           "Three controllers on the same robot. N is our coupled NMPC with the full arm-drone model. ND adds a disturbance observer to it, so it corrects for unknown loads. RL is a PPO policy that sends commands to a fast inner loop with the same observer. The benchmark: 5 sentences times 3 noise seeds, under 8 conditions: nominal, payloads, slow motors, joint damping, thrust error, gusts and wind. Pass: whole sentence written, tracing error at most 2 mm.")
ctl = [("N", "Coupled NMPC", "full arm + drone model, 50 Hz"), ("ND", "NMPC + observer", "learns unknown forces and torques online"), ("RL", "PPO policy", "trained with random loads, motor lag and wind")]
for i, (k, t, b) in enumerate(ctl):
    x = 0.55 + i * 2.55; box(sl, x, 1.6, 2.3, 2.55)
    text(sl, x + 0.2, 1.8, 1.9, 0.6, [k], size=30, color=RED, bold=True); text(sl, x + 0.2, 2.5, 1.9, 0.5, [t], size=16, bold=True); text(sl, x + 0.2, 3.0, 1.95, 1.0, [b], size=12, color=GR)
box(sl, 8.4, 1.6, 4.4, 2.55, fill=WHITE, line=RED)
text(sl, 8.65, 1.75, 3.95, 2.3, [[("Benchmark", {"bold": True, "color": RED, "size": 17})], "5 sentences × 3 seeds", "8 conditions: payload 100 / 200 g, motor lag ×2, damping ×3, thrust −10 %, gust, wind",
                                  [("Pass: ", {"bold": True}), ("whole sentence, error ≤ 2 mm", {})]], size=13)
cases = ["nominal", "100 g", "200 g", "motor lag", "damping", "thrust −10 %", "gust", "wind"]
for i, c in enumerate(cases):
    x = 0.55 + i * 1.53; box(sl, x, 4.6, 1.38, 0.75, fill=CARD); text(sl, x, 4.6, 1.38, 0.75, [c], size=13, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, bold=True)
text(sl, 0.55, 5.6, 12.2, 0.8, ["Every number on the next slide is from closed-loop simulation with realistic sensor noise: 15 runs per cell, no hand-picked runs."], size=13, color=GR)

# 9 results table
sl = slide("Results: Robust or Accurate, Not Yet Both",
           "Each cell: sentences completed out of 15, then mean tracing error in mm. Green is all 15 done, red is a failure. The plain NMPC is accurate but fails any payload or slow motors. With the observer, ND handles every payload, including 200 g, but not slow motors. Our best RL policy, R21, is the most accurate controller on 6 of 8 conditions and under 2 mm on 4, but completes only 5 of 15 at 200 g and 11 of 15 with slow motors. No controller passes all 8 yet.")
data = {"N: NMPC": [(15, 2.07), (0, None), (0, None), (0, None), (15, 2.31), (15, 2.70), (15, 2.03), (15, 2.23)],
        "ND: NMPC + observer": [(15, 2.62), (15, 2.65), (15, 3.05), (0, None), (15, 2.77), (15, 2.59), (15, 2.66), (15, 2.70)],
        "RL: policy R21": [(15, 1.87), (15, 2.32), (5, 2.75), (11, 2.47), (15, 1.94), (15, 1.97), (15, 1.90), (15, 2.64)]}
tb = sl.shapes.add_table(4, 9, Inches(0.5), Inches(1.6), Inches(12.3), Inches(3.2)).table
tb.columns[0].width = Inches(2.7)
for j in range(1, 9): tb.columns[j].width = Inches(1.2)
for j, h in enumerate(["completed / 15, error mm"] + cases):
    c = tb.cell(0, j); c.text = h; c.fill.solid(); c.fill.fore_color.rgb = RED; f = c.text_frame.paragraphs[0].runs[0].font; f.color.rgb = WHITE; f.bold = True; f.size = Pt(12); f.name = "Arial"
    c.text_frame.paragraphs[0].alignment = PP_ALIGN.CENTER if j else PP_ALIGN.LEFT
for i, (k, row) in enumerate(data.items()):
    c = tb.cell(i + 1, 0); c.text = k; c.fill.solid(); c.fill.fore_color.rgb = CARD; f = c.text_frame.paragraphs[0].runs[0].font; f.bold = True; f.size = Pt(14); f.name = "Arial"; f.color.rgb = DK
    for j, (n, e) in enumerate(row):
        c = tb.cell(i + 1, j + 1); c.vertical_anchor = MSO_ANCHOR.MIDDLE; c.fill.solid(); c.fill.fore_color.rgb = OK if n == 15 else (BAD if n == 0 else MID)
        tf = c.text_frame; p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER; r = p.add_run(); r.text = f"{n}"; r.font.size = Pt(20); r.font.bold = True; r.font.name = "Arial"; r.font.color.rgb = DK
        p2 = tf.add_paragraph(); p2.alignment = PP_ALIGN.CENTER; r2 = p2.add_run(); r2.text = "—" if e is None else f"{e:.2f} mm"; r2.font.size = Pt(12); r2.font.name = "Arial"
        r2.font.color.rgb = RED if (e is not None and e <= 2.0) else GR; r2.font.bold = e is not None and e <= 2.0
for r in range(4): tb.rows[r].height = Inches(0.8)
stat(sl, 0.5, 5.15, 4.0, "4 of 8", "conditions where RL is under the 2 mm line")
stat(sl, 4.7, 5.15, 4.0, "15 / 15 at 200 g", "only the NMPC with observer (ND)")
stat(sl, 8.9, 5.15, 4.0, "0 controllers", "pass all 8 conditions yet")

# 10 video 200 g
sl = slide("Seeing It: 200 g at the Hand",
           "Same sentence, same 200 g load, same noise. Top left, ND writes it all. Top right, an earlier RL policy, R19, flips in under 3 seconds: shoulder torque saturation, as the torque curve predicted. Bottom row, R20 writes it all, and our latest, R21, writes most of it but drifts off at the last letters. The failure is always near the end of the sentence, where the drone has travelled furthest.")
sl.shapes.add_movie(D + "v_200.mp4", Inches(0.5), Inches(1.5), Inches(8.6), Inches(4.84), poster_frame_image=D + "p_200.png", mime_type="video/mp4")
stat(sl, 9.5, 1.55, 3.4, "ND", "writes it all, 2.9 mm")
stat(sl, 9.5, 2.85, 3.4, "R19", "flips at 2.6 s: shoulder saturates")
stat(sl, 9.5, 4.15, 3.4, "R21", "writes 9 of 11 strokes, then drifts")
text(sl, 0.5, 6.4, 8.6, 0.4, ["HELLO WPI, 200 g at the hand, realistic sensor noise; top: ND, R19; bottom: R20, R21"], size=12, color=GR)

# 11 limits and next
sl = slide("Limits and Next Steps",
           "Limits: the mass formula assumes a point load at a known place on the hand; an unknown position needs four parameters, still linear, still closed form; a swinging load is not covered. On the task, no controller passes all 8 conditions. Next: put the analytical mass and torque model inside the controllers so they plan within the servo limit, swap the closed-form IK into the reference generator, and finish the RL run training on wider loads and motor lag, which is on the cluster now.")
for x, t, items in [(0.5, "Limits", ["Mass formula: point load at a known place on the hand", "Unknown load position: 4 parameters, still linear (not built yet)", "Swinging load: not covered", "No controller passes all 8 conditions; RL 5/15 at 200 g, ND 0/15 with slow motors"]),
                    (6.85, "Next", ["Feed the PoE mass and torque model into the controllers: plan within the 1.0 N m servo limit", "Closed-form IK in the reference generator (18× faster, exact)", "Sweep load mass and position to map the limit", "RL run with wider loads and motor lag: training now"])]:
    box(sl, x, 1.6, 5.95, 3.5); text(sl, x + 0.3, 1.8, 5.4, 0.5, [t], size=20, bold=True, color=RED if t == "Limits" else DK)
    bullets(sl, x + 0.3, 2.45, 5.4, 2.6, items, size=15)
box(sl, 0.5, 5.35, 12.3, 0.95, fill=WHITE, line=RED); text(sl, 0.8, 5.35, 11.8, 0.95, [[("Take-away: ", {"bold": True, "color": RED}), ("known angles and lengths give exact kinematics, the load mass, and the torque limit that explains where the controllers fail.", {})]], size=15, anchor=MSO_ANCHOR.MIDDLE)

prs.save(D + "2026-11-18_1300_Capstone_Analytical_Kinematics_and_Writing_Update.pptx"); print("saved", len(prs.slides))
