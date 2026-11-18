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


