#!/usr/bin/env python3
"""Draw docs/images/hub-wiring.svg (hub circuit for hand wiring / the future PCB).

Run from the repository root: python3 docs/images/src/hub_wiring_svg.py
"""
from pathlib import Path

W, H = 980, 640
INK, MUTED, ACCENT = '#1f2933', '#52606d', '#2f6fb3'
F = "font-family='-apple-system,Segoe UI,Helvetica,Arial,sans-serif'"
out = []


def esc(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def text(x, y, s, size=13, weight=400, fill=INK, anchor='start'):
    out.append(f"<text x='{x}' y='{y}' font-size='{size}' font-weight='{weight}' fill='{fill}' text-anchor='{anchor}' {F}>{esc(s)}</text>")


def line(*pts, w=2, color=INK):
    d = ' '.join(f'{x},{y}' for x, y in pts)
    out.append(f"<polyline points='{d}' fill='none' stroke='{color}' stroke-width='{w}' stroke-linejoin='round'/>")


def dot(x, y):
    out.append(f"<circle cx='{x}' cy='{y}' r='3.5' fill='{INK}'/>")


def panel(x, y, w, h, title):
    out.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='12' fill='#f7f9fb' stroke='#c5ccd3' stroke-width='1.5'/>")
    text(x + 14, y + 26, title, 15, 700)


def res_v(x, y1, y2, label, value):
    """Vertical resistor between y1 and y2 (body 34 px)."""
    m = (y1 + y2) / 2
    line((x, y1), (x, m - 17)); line((x, m + 17), (x, y2))
    out.append(f"<rect x='{x-6}' y='{m-17}' width='12' height='34' fill='#fff' stroke='{INK}' stroke-width='2'/>")
    text(x + 12, m - 2, label, 12, 700); text(x + 12, m + 13, value, 12, 400, MUTED)


def res_h(x1, x2, y, label, value):
    m = (x1 + x2) / 2
    line((x1, y), (m - 17, y)); line((m + 17, y), (x2, y))
    out.append(f"<rect x='{m-17}' y='{y-6}' width='34' height='12' fill='#fff' stroke='{INK}' stroke-width='2'/>")
    text(m, y - 12, f'{label} {value}', 12, 400, INK, 'middle')


def cap_v(x, y1, y2, label, value):
    m = (y1 + y2) / 2
    line((x, y1), (x, m - 4)); line((x, m + 4), (x, y2))
    line((x - 12, m - 4), (x + 12, m - 4), w=2.5); line((x - 12, m + 4), (x + 12, m + 4), w=2.5)
    text(x + 16, m - 2, label, 12, 700); text(x + 16, m + 13, value, 12, 400, MUTED)


def gnd(x, y):
    line((x, y), (x, y + 8))
    for i, half in enumerate((11, 7, 3)):
        line((x - half, y + 8 + i * 4), (x + half, y + 8 + i * 4), w=2)


def rail(x, y, label):
    line((x, y), (x, y - 10)); line((x - 10, y - 10), (x + 10, y - 10), w=2.5)
    text(x, y - 16, label, 12, 700, INK, 'middle')


def pin(x, y, label, side='left'):
    out.append(f"<rect x='{x-30 if side == 'left' else x}' y='{y-11}' width='30' height='22' rx='4' fill='{ACCENT}'/>")
    text(x - 15 if side == 'left' else x + 15, y + 4, label, 12, 700, '#fff', 'middle')


out.append(f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {W} {H}' width='{W}' height='{H}'>")
out.append(f"<rect width='{W}' height='{H}' rx='18' fill='#ffffff'/>")
text(W / 2, 34, 'Хаб: схема подключения (XIAO ESP32-C3 / S3)', 20, 700, INK, 'middle')

# ---------------------------------------------------------------- motor driver
panel(20, 52, 450, 370, 'Вибромотор — вывод D2 (активный HIGH)')
X_M = 340  # motor column
rail(X_M, 110, '3V3')
line((X_M, 110), (X_M, 150)); dot(X_M, 150)
# motor + flyback diode in parallel between 150 and 250
line((X_M, 150), (X_M, 172)); out.append(f"<circle cx='{X_M}' cy='196' r='22' fill='#fff' stroke='{INK}' stroke-width='2'/>")
text(X_M, 202, 'M', 16, 700, INK, 'middle'); line((X_M, 218), (X_M, 250)); dot(X_M, 250)
text(X_M - 34, 188, 'M1', 12, 700, INK, 'end'); text(X_M - 34, 203, 'мотор 3 В', 12, 400, MUTED, 'end')
text(X_M - 34, 218, '≤ 100 мА', 12, 400, MUTED, 'end')
XD = X_M + 62
line((X_M, 150), (XD, 150), (XD, 182)); line((XD, 212), (XD, 250), (X_M, 250))
out.append(f"<polygon points='{XD-10},{212} {XD+10},{212} {XD},{192}' fill='#fff' stroke='{INK}' stroke-width='2'/>")
line((XD - 11, 190), (XD + 11, 190), w=2.5); line((XD, 182), (XD, 190))
text(XD + 14, 198, 'D1', 12, 700); text(XD + 14, 213, 'SS14', 11, 400, MUTED)
# MOSFET (drawn as N-channel: drain top, source bottom, gate left)
line((X_M, 250), (X_M, 286))
out.append(f"<line x1='{X_M-8}' y1='286' x2='{X_M-8}' y2='318' stroke='{INK}' stroke-width='3'/>")
line((X_M - 8, 290), (X_M, 290), (X_M, 286)); line((X_M - 8, 314), (X_M, 314), (X_M, 340))
line((X_M - 16, 290), (X_M - 16, 314), w=2.5); line((X_M - 16, 302), (X_M - 70, 302))
out.append(f"<polygon points='{X_M-8},302 {X_M-1},298 {X_M-1},306' fill='{INK}'/>")
line((X_M - 8, 302), (X_M, 302), (X_M, 314))
text(X_M + 10, 300, 'Q1  AO3400A', 12, 700); text(X_M + 10, 315, 'N-MOSFET, SOT-23', 11, 400, MUTED)
gnd(X_M, 340)
# gate network
XG = X_M - 70
dot(XG, 302); res_v(XG, 302, 372, 'R2', '100 кОм'); gnd(XG, 372)
res_h(110, XG, 302, 'R1', '100 Ом'); pin(110, 302, 'D2')
# bulk caps on the motor supply
for i, (lab, val) in enumerate((('C1', '10 мкФ'), ('C2', '100 нФ'))):
    cx = 70 + i * 70
    line((cx, 150), (cx, 160)); cap_v(cx, 160, 214, lab, val); gnd(cx, 214)
line((70, 150), (X_M, 150)); dot(140, 150)
text(36, 404, 'R2 держит мотор выключенным при загрузке, D1 гасит выброс', 12, 400, MUTED)
text(36, 418, 'напряжения при выключении. GPIO к мотору напрямую не подключать.', 12, 400, MUTED)

# ---------------------------------------------------------------- button
panel(490, 52, 220, 175, 'Кнопка — D3')
pin(540, 120, 'D3')
line((540, 120), (590, 120)); out.append(f"<circle cx='594' cy='120' r='4' fill='#fff' stroke='{INK}' stroke-width='2'/>")
out.append(f"<circle cx='640' cy='120' r='4' fill='#fff' stroke='{INK}' stroke-width='2'/>")
line((596, 106), (640, 110), w=2.5); line((618, 108), (618, 96), w=2); line((610, 96), (626, 96), w=2.5)
line((644, 120), (670, 120), (670, 140)); gnd(670, 140)
text(578, 150, 'S1', 12, 700); text(506, 182, 'Подтяжка внутренняя, к GND.', 12, 400, MUTED)
text(506, 198, 'Коротко — статус, 1,5 с — MAC.', 12, 400, MUTED)

# ---------------------------------------------------------------- LED
panel(730, 52, 230, 175, 'Светодиод — D1')
pin(780, 110, 'D1')
res_h(780, 860, 110, 'R5', '1 кОм')
line((860, 110), (900, 110), (900, 126))
out.append(f"<polygon points='890,126 910,126 900,146' fill='#fff' stroke='{INK}' stroke-width='2'/>")
line((889, 147), (911, 147), w=2.5); line((900, 147), (900, 166)); gnd(900, 166)
text(918, 140, 'LED', 12, 700)

# ---------------------------------------------------------------- battery + divider
panel(490, 245, 470, 177, 'Аккумулятор и измерение — A0')
text(506, 290, 'Li-ion 3,7 В', 12, 700); text(506, 305, '1S, с защитой', 12, 400, MUTED)
out.append(f"<rect x='516' y='318' width='40' height='60' rx='5' fill='#fff' stroke='{INK}' stroke-width='2'/>")
text(536, 340, '+', 16, 700, INK, 'middle'); text(536, 372, '−', 16, 700, INK, 'middle')
line((556, 330), (600, 330))
out.append(f"<circle cx='604' cy='330' r='4' fill='#fff' stroke='{INK}' stroke-width='2'/>")
line((607, 328), (640, 316), w=2.5); out.append(f"<circle cx='644' cy='330' r='4' fill='#fff' stroke='{INK}' stroke-width='2'/>")
text(624, 356, 'SW1', 12, 700, INK, 'middle'); text(624, 370, 'выключатель', 11, 400, MUTED, 'middle')
line((648, 330), (720, 330)); dot(720, 330)
line((720, 330), (720, 300), (800, 300)); pin(800, 300, 'BAT+', 'right')
line((536, 378), (536, 392)); gnd(536, 392)
res_v(720, 330, 372, 'R3', '100 кОм'); dot(720, 372)
line((720, 372), (800, 372)); pin(800, 372, 'A0', 'right')
line((720, 372), (720, 380)); res_v(720, 380, 410, '', ''); gnd(720, 410)
text(732, 400, 'R4 100 кОм', 12, 400, INK)
cap_v(880, 372, 404, 'C3', '100 нФ'); line((830, 372), (880, 372)); gnd(880, 404)
text(560, 404, '− на площадку BAT−', 11, 400, MUTED)

# ---------------------------------------------------------------- OLED
panel(20, 440, 940, 180, 'OLED SH1106 1,3″ (I²C, адрес 0x3C)')
labels = (('VCC', '3V3'), ('GND', 'GND'), ('SCL', 'D5'), ('SDA', 'D4'))
for i, (o, p) in enumerate(labels):
    y = 494 + i * 30
    out.append(f"<rect x='60' y='{y-11}' width='54' height='22' rx='4' fill='#111'/>")
    text(87, y + 4, o, 12, 700, '#fff', 'middle'); line((114, y), (300, y)); pin(330, y, p)
text(380, 498, 'Питать от 3V3, не от 5V: при работе от аккумулятора', 12, 400, MUTED)
text(380, 514, 'на выводе 5V нет напряжения.', 12, 400, MUTED)
text(380, 540, 'У многих модулей уже есть подтяжки SDA/SCL 4,7–10 кОм.', 12, 400, MUTED)
text(380, 556, 'Если их нет — поставить 4,7 кОм от SDA и SCL к 3V3.', 12, 400, MUTED)
text(380, 582, 'Общий GND для всех узлов обязателен.', 12, 700, INK)
out.append('</svg>')

Path('docs/images/hub-wiring.svg').write_text('\n'.join(out), encoding='utf-8')
print('docs/images/hub-wiring.svg')
