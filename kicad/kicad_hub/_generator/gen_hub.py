"""Generate the hub schematic (hub.kicad_sch) for the ESP-NOW bite hub, rev A.

Written in KiCad 7 file format so that both KiCad 7 (Linux CI/cloud) and
KiCad 8/9 (macOS) open it. Usage: python3 gen_hub.py <project dir>
KiCad symbol libraries are taken from $KICAD_SYMBOL_DIR or the usual install paths.
"""
import copy, glob, json, math, os, sys, uuid
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'kicad_bite', '_generator'))
from sexp import parse, dump, Q, find, findall  # noqa: E402

OUT = sys.argv[1]
PROJ = 'hub'
U = 2.54
os.makedirs(OUT, exist_ok=True)


def symdir():
    for d in (os.environ.get('KICAD_SYMBOL_DIR'), os.environ.get('KICAD9_SYMBOL_DIR'),
              os.environ.get('KICAD8_SYMBOL_DIR'), os.environ.get('KICAD7_SYMBOL_DIR'),
              '/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols',
              '/usr/share/kicad/symbols'):
        if d and os.path.isdir(d):
            return d
    raise SystemExit('KiCad symbol library not found; set KICAD_SYMBOL_DIR')


SYM = symdir()


def uid():
    return Q(str(uuid.uuid4()))


ROOT = uid()
_libs = {}


def lib_symbol(lib, name):
    if lib not in _libs:
        tree = parse(open(f'{SYM}/{lib}.kicad_sym').read())[0]
        _libs[lib] = {s[1]: s for s in findall(tree, 'symbol')}
    return _libs[lib][name]


def flatten(lib, name):
    s = lib_symbol(lib, name)
    ext = find(s, 'extends')
    if not ext:
        return copy.deepcopy(s)
    parent = flatten(lib, ext[1])
    pname = parent[1]
    child = {p[1]: p for p in findall(s, 'property')}
    new = ['symbol', Q(name)]
    for e in parent[2:]:
        if isinstance(e, list) and e[0] == 'property':
            new.append(copy.deepcopy(child.pop(e[1], e)))
        elif isinstance(e, list) and e[0] == 'symbol':
            sub = copy.deepcopy(e)
            sub[1] = Q(name + sub[1][len(pname):])
            new.append(sub)
        else:
            new.append(copy.deepcopy(e))
    for p in child.values():
        new.append(copy.deepcopy(p))
    return new


def eff(size=1.27, hide=False, justify=None, bold=False):
    font = ['font', ['size', size, size]]
    if bold:
        font.append('bold')
    e = ['effects', font]
    if justify:
        e.append(['justify'] + justify.split())
    if hide:
        e.append('hide')
    return e


def pin(ptype, x, y, ang, name, num, length=5.08):
    return ['pin', ptype, 'line', ['at', x, y, ang], ['length', length],
            ['name', Q(name), eff()], ['number', Q(num), eff()]]


def custom_symbol(name, ref, value, rect, pins, desc, datasheet=''):
    return ['symbol', Q(name), ['pin_names', ['offset', 1.016]], ['in_bom', 'yes'], ['on_board', 'yes'],
            ['property', Q('Reference'), Q(ref), ['at', rect[0], rect[1] + 1.27, 0], eff(justify='left')],
            ['property', Q('Value'), Q(value), ['at', rect[0], rect[3] - 1.27, 0], eff(justify='left')],
            ['property', Q('Footprint'), Q(''), ['at', 0, 0, 0], eff(hide=True)],
            ['property', Q('Datasheet'), Q(datasheet), ['at', 0, 0, 0], eff(hide=True)],
            ['property', Q('ki_description'), Q(desc), ['at', 0, 0, 0], eff(hide=True)],
            ['symbol', Q(name + '_0_1'),
             ['rectangle', ['start', rect[0], rect[1]], ['end', rect[2], rect[3]],
              ['stroke', ['width', 0.254], ['type', 'default']], ['fill', ['type', 'background']]]],
            ['symbol', Q(name + '_1_1')] + pins]


# XIAO ESP32-C3: same 14-pad module outline and numbering as the sensor's XIAO footprint
CUSTOM = {}
xp = []
for i, (n, nm) in enumerate([('1', 'D0/A0/GPIO2'), ('2', 'D1/GPIO3'), ('3', 'D2/GPIO4'), ('4', 'D3/GPIO5'),
                              ('5', 'D4/SDA/GPIO6'), ('6', 'D5/SCL/GPIO7'), ('7', 'D6/TX/GPIO21')]):
    xp.append(pin('bidirectional', -20.32, 7.62 - 2.54 * i, 0, nm, n))
xp.append(pin('power_in', 20.32, 7.62, 180, '5V', '14'))
xp.append(pin('power_in', 20.32, 5.08, 180, '3V3', '12'))
for i, (n, nm) in enumerate([('11', 'D10/GPIO10'), ('10', 'D9/GPIO9'), ('9', 'D8/GPIO8'), ('8', 'D7/RX/GPIO20')]):
    xp.append(pin('bidirectional', 20.32, -2.54 - 2.54 * i, 180, nm, n))
xp.append(pin('power_in', 0, -17.78, 90, 'GND', '13'))
CUSTOM['XIAO_ESP32C3'] = custom_symbol(
    'XIAO_ESP32C3', 'U', 'XIAO_ESP32C3', (-15.24, 10.16, 15.24, -12.7), xp,
    'Seeed Studio XIAO ESP32-C3 module', 'https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/')


def get_symdef(lib_id):
    lib, name = lib_id.split(':')
    if lib == 'hub':
        return copy.deepcopy(CUSTOM[name])
    return flatten(lib, name)


items, lib_symbols, parts, pwr_n = [], {}, {}, [0]


def mm(v):
    return round(v * U, 4)


def sym_pins(sd):
    res = {}
    for sub in findall(sd, 'symbol'):
        for p in findall(sub, 'pin'):
            at = find(p, 'at')
            res.setdefault(find(p, 'number')[1], (float(at[1]), float(at[2]), float(at[3])))
    return res


class Part:
    def __init__(self, lib_id, ref, value, x, y, fp=None, desc=None, fields='lib', hide_val=False, mpn=None):
        sd = get_symdef(lib_id)
        if lib_id not in lib_symbols:
            emb = copy.deepcopy(sd)
            emb[1] = Q(lib_id)
            lib_symbols[lib_id] = emb
        self.ref = ref
        X, Y = mm(x), mm(y)
        self.pins = {}
        for num, (px, py, ang) in sym_pins(sd).items():
            a = math.radians(ang)
            self.pins[num] = (round(X + px, 4), round(Y - py, 4), -round(math.cos(a)), round(math.sin(a)))
        props = {p[1]: p for p in findall(sd, 'property')}
        power = ref.startswith('#')
        inst = ['symbol', ['lib_id', Q(lib_id)], ['at', X, Y, 0], ['unit', 1],
                ['in_bom', 'no' if power else 'yes'], ['on_board', 'no' if power else 'yes'], ['dnp', 'no'],
                ['uuid', uid()]]
        vals = {'Reference': ref, 'Value': value}
        if fp is not None:
            vals['Footprint'] = fp
        if desc is not None:
            vals['Description'] = desc
        if mpn:
            vals['MPN'] = mpn
        for key in ('Reference', 'Value', 'Footprint', 'Datasheet', 'Description', 'MPN'):
            lp = props.get(key)
            if key not in vals and not lp:
                continue
            txt = vals.get(key, lp[2] if lp else '')
            hidden = key not in ('Reference', 'Value') or (key == 'Value' and hide_val) or (power and key == 'Reference')
            if lp:
                at = find(lp, 'at')
                px, py = float(at[1]), float(at[2])
            else:
                px, py = 0, 0
            e = eff(hide=hidden)
            if not hidden and fields == 'right':
                px, py, e = 2.54, (1.27 if key == 'Reference' else -1.27), eff(justify='left')
            elif not hidden and isinstance(fields, tuple):
                px, py = fields[0] if key == 'Reference' else fields[1]
                e = eff(justify='left')
            elif not hidden and lp:
                e = copy.deepcopy(find(lp, 'effects'))
            inst.append(['property', Q(key), Q(txt), ['at', round(X + px, 4), round(Y - py, 4), 0], e])
        for num in self.pins:
            inst.append(['pin', Q(num), ['uuid', uid()]])
        inst.append(['instances', ['project', Q(PROJ), ['path', Q('/' + ROOT), ['reference', Q(ref)], ['unit', 1]]]])
        items.append(inst)
        parts[ref] = self

    def p(self, num):
        return self.pins[num]


def wire(*pts):
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        items.append(['wire', ['pts', ['xy', round(x1, 4), round(y1, 4)], ['xy', round(x2, 4), round(y2, 4)]],
                      ['stroke', ['width', 0], ['type', 'default']], ['uuid', uid()]])


def label(x, y, net, d):
    ang = {(-1, 0): 180, (1, 0): 0, (0, -1): 90, (0, 1): 270}[d]
    just = 'right bottom' if ang in (180, 270) else 'left bottom'
    items.append(['label', Q(net), ['at', round(x, 4), round(y, 4), ang], ['fields_autoplaced'],
                  eff(justify=just), ['uuid', uid()]])


def lbl(pr, net, stub=1.0):
    x, y, dx, dy = pr
    ex, ey = x + dx * stub * U, y + dy * stub * U
    wire((x, y), (ex, ey))
    label(ex, ey, net, (dx, dy))


def power(kind, x, y):
    pwr_n[0] += 1
    ref = ('#FLG' if kind == 'PWR_FLAG' else '#PWR') + f'{pwr_n[0]:02d}'
    Part('power:' + kind, ref, kind, x / U, y / U)


def gnd(pr, stub=1.0):
    x, y, dx, dy = pr
    if (dx, dy) == (0, 1):
        wire((x, y), (x, y + stub * U))
        power('GND', x, y + stub * U)
    else:
        # side/top pin: GND symbol right at the stub end, turned to face away from the pin,
        # so the ground never lands on a neighbouring pin's stub
        mx, my = x + dx * stub * U, y + dy * stub * U
        wire((x, y), (mx, my))
        power('GND', mx, my)
        items[-1][2][3] = {(-1, 0): 270, (1, 0): 90, (0, -1): 180}[(dx, dy)]


def flag(net, x, y):
    power('PWR_FLAG', mm(x), mm(y))
    wire((mm(x), mm(y)), (mm(x), mm(y) + U))
    if net == 'GND':
        power('GND', mm(x), mm(y) + U)
    else:
        label(mm(x), mm(y) + U, net, (1, 0))


def nc(pr):
    items.append(['no_connect', ['at', pr[0], pr[1]], ['uuid', uid()]])


def text(x, y, s, size=1.27, bold=False):
    items.append(['text', Q(s), ['at', mm(x), mm(y), 0], eff(size, justify='left top', bold=bold), ['uuid', uid()]])


def frame(x1, y1, x2, y2, title):
    items.append(['rectangle', ['start', mm(x1), mm(y1)], ['end', mm(x2), mm(y2)],
                  ['stroke', ['width', 0], ['type', 'dash']], ['fill', ['type', 'none']], ['uuid', uid()]])
    text(x1 + 0.5, y1 + 0.5, title, 1.8, bold=True)


def two(lib_id, ref, val, x, y, top, bot, fp, mpn=None):
    pr = Part(lib_id, ref, val, x, y, fp=fp, fields='right', mpn=mpn)
    for num, net in (('1', top), ('2', bot)):
        if net == 'GND':
            gnd(pr.p(num))
        elif net:
            lbl(pr.p(num), net)
    return pr


R = 'Resistor_SMD:R_0603_1608Metric'
C = 'Capacitor_SMD:C_0603_1608Metric'
C8 = 'Capacitor_SMD:C_0805_2012Metric'
LEDFP = 'LED_SMD:LED_0603_1608Metric'
SOT = 'Package_TO_SOT_SMD:SOT-23'

# ============================================================ 1. USB-C charging input
frame(4, 4, 60, 40, '1. Вход зарядки USB-C (только питание)')
J1 = Part('Connector:USB_C_Receptacle_PowerOnly_6P', 'J1', 'USB-C 6P', 14, 20,
          fp='Connector_USB:USB_C_Receptacle_GCT_USB4125-xx-x-0190_6P_TopMnt_Horizontal',
          mpn='GCT USB4125-GF-A-0190', desc='Charging-only USB-C receptacle, bottom edge under a rubber plug')
lbl(J1.p('A9'), 'VBUS_IN')
lbl(J1.p('A5'), 'CC1')
lbl(J1.p('B5'), 'CC2')
gnd(J1.p('A12'))
gnd(J1.p('S1'))
two('Device:R', 'R1', '5.1k', 32, 24, 'CC1', 'GND', R)
two('Device:R', 'R2', '5.1k', 37, 24, 'CC2', 'GND', R)
two('Device:Polyfuse', 'F1', 'PTC 1.1A', 44, 14, 'VBUS_IN', 'VBUS_5V', 'Fuse:Fuse_1206_3216Metric',
    mpn='Bourns MF-NSMF110-2')
VD1 = Part('Diode:SMF5V0A', 'VD1', 'SMF5V0A', 52, 24, fp='Diode_SMD:D_SMF', fields='right')
lbl(VD1.p('1'), 'VBUS_5V')
gnd(VD1.p('2'))
flag('VBUS_5V', 50, 32)
text(5, 34, 'CC1/CC2 через 5,1 кОм на GND: любой USB-C блок отдаёт 5 В.\n'
            'Данные не подключены: прошивка только через USB-C самой XIAO.\n'
            'VD1 и F1 защищают от переполюсовки и перенапряжения.')

# ============================================================ 2. charger
frame(62, 4, 132, 54, '2. Зарядка BQ24074: 600 мА, power-path')
U1 = Part('Battery_Management:BQ24074RGT', 'U1', 'BQ24074RGT', 96, 26,
          fp='Package_DFN_QFN:VQFN-16-1EP_3x3mm_P0.5mm_EP1.6x1.6mm', mpn='TI BQ24074RGTR')
lbl(U1.p('13'), 'VBUS_5V')
lbl(U1.p('10'), 'SYS')
lbl(U1.p('11'), 'SYS')
lbl(U1.p('2'), 'VBAT')
lbl(U1.p('3'), 'VBAT')
lbl(U1.p('1'), 'TS')
gnd(U1.p('4'))
gnd(U1.p('6'))
lbl(U1.p('5'), 'SYS')
lbl(U1.p('12'), 'ILIM')
lbl(U1.p('16'), 'ISET')
nc(U1.p('14'))
lbl(U1.p('15'), 'SYSOFF')
lbl(U1.p('7'), 'PGOOD_N')
lbl(U1.p('9'), 'CHG_N')
gnd(U1.p('8'))
gnd(U1.p('17'), stub=2.0)
two('Device:R', 'R3', '1.5k', 66, 42, 'ISET', 'GND', R)
two('Device:R', 'R4', '1.5k', 71, 42, 'ILIM', 'GND', R)
two('Device:R', 'R5', '10k', 76, 42, 'TS', 'GND', R)
two('Device:R', 'R6', '0', 81, 42, 'SYSOFF', 'GND', R)
two('Device:C', 'C1', '1uF', 112, 14, 'VBUS_5V', 'GND', C8)
two('Device:C', 'C2', '4.7uF', 118, 14, 'SYS', 'GND', C8)
two('Device:C', 'C3', '4.7uF', 124, 14, 'VBAT', 'GND', C8)
D1 = Part('Device:LED', 'D1', 'red CHG', 112, 32, fp=LEDFP, fields='right')
lbl(D1.p('1'), 'CHG_N')
lbl(D1.p('2'), 'D1_A')
two('Device:R', 'R8', '1k', 112, 40, 'VBUS_5V', 'D1_A', R)
D2 = Part('Device:LED', 'D2', 'green PG', 124, 32, fp=LEDFP, fields='right')
lbl(D2.p('1'), 'PGOOD_N')
lbl(D2.p('2'), 'D2_A')
two('Device:R', 'R9', '1k', 124, 40, 'VBUS_5V', 'D2_A', R)
text(63, 47, 'R3 ISET: Iзар = 890 / 1,5 кОм ≈ 0,59 А (0,5C для 1200 мА·ч; пересчитать при смене аккумулятора).\n'
             'R4 ILIM: Iвх = 1550 / 1,5 кОм ≈ 1,0 А.  EN2 = 1, EN1 = 0 → вход по ILIM.  CE = 0 → зарядка разрешена.\n'
             'R5: вместо NTC (25 °C). R6: SYSOFF на GND (вкл.). CE и EN1 на GND. TMR свободен = таймер по умолчанию.\n'
             'Коэффициенты и выводы 14/15 сверить с даташитом BQ24074 перед заказом.')

# ============================================================ 3. battery
frame(4, 42, 60, 78, '3. LiPo 1S 1200 мА·ч, выключатель, делитель')
J2 = Part('Connector_Generic:Conn_01x02', 'J2', 'LiPo JST-PH', 10, 52,
          fp='Connector_JST:JST_PH_S2B-PH-SM4-TB_1x02-1MP_P2.00mm_Horizontal', fields='right',
          desc='LiPo 1S with its own protection board (PCM); 1 = +, 2 = -', mpn='JST S2B-PH-SM4-TB')
lbl(J2.p('1'), 'BT_P')
gnd(J2.p('2'))
Q1 = Part('Transistor_FET:AO3401A', 'Q1', 'AO3401A', 26, 52, fp=SOT, fields='right')
lbl(Q1.p('3'), 'BT_P')
lbl(Q1.p('2'), 'VBAT')
gnd(Q1.p('1'))
two('Device:R', 'R10', '100k', 40, 50, 'VBAT', 'VBAT_SENSE', R)
two('Device:R', 'R11', '100k', 46, 50, 'VBAT_SENSE', 'GND', R)
two('Device:C', 'C4', '100nF', 52, 50, 'VBAT_SENSE', 'GND', C)
SW1 = Part('Switch:SW_SPDT', 'SW1', 'PCM12 ON/OFF', 16, 68, fp='Button_Switch_SMD:SW_SPDT_PCM12', fields='right',
           mpn='C&K PCM12SMTR')
lbl(SW1.p('2'), 'SYS')
lbl(SW1.p('1'), 'SYS_SW')
nc(SW1.p('3'))
D3 = Part('Device:D_Schottky', 'D3', 'B5819W', 32, 68, fp='Diode_SMD:D_SOD-123', fields='right')
lbl(D3.p('2'), 'SYS_SW')
lbl(D3.p('1'), '5V_XIAO')
flag('5V_XIAO', 40, 64)
flag('GND', 46, 64)
text(5, 73, 'Q1 — защита от неверной полярности разъёма. Аккумулятор — со своей платой защиты.\n'
            'Делитель до выключателя (~21 мкА в выкл.). D3 не даёт USB XIAO питать BQ24074.')

# ============================================================ 4. MCU
frame(62, 56, 132, 92, '4. XIAO ESP32-C3')
U2 = Part('hub:XIAO_ESP32C3', 'U2', 'XIAO ESP32-C3', 96, 72, fp='hub:XIAO_THT',
          fields=((-15.24, 12.7), (-15.24, -15.24)), mpn='Seeed 113991054')
for num, net in (('1', 'VBAT_SENSE'), ('2', 'LED'), ('3', 'MOTOR'), ('4', 'BTN'), ('5', 'SDA'), ('6', 'SCL'),
                 ('14', '5V_XIAO'), ('12', '3V3')):
    lbl(U2.p(num), net)
for num in ('7', '8', '9', '10', '11'):
    nc(U2.p(num))
gnd(U2.p('13'))
two('Device:C', 'C5', '22uF', 122, 66, '3V3', 'GND', C8)
two('Device:C', 'C6', '100nF', 128, 66, '3V3', 'GND', C)
flag('3V3', 122, 76)
text(63, 86, 'Выводы как в прошивке ESP_HUB: LED D1, мотор D2, кнопка D3, OLED D4/D5, батарея A0.\n'
             'USB-C XIAO — прошивка (верхний торец, под заглушкой). D6–D10 свободны.')

# ============================================================ 5. motor
frame(4, 80, 60, 116, '5. Вибромотор (ERM 10 мм, 3 В)')
J3 = Part('Connector_Generic:Conn_01x02', 'J3', 'MOTOR', 10, 90, fp='hub:WIRE_2P', fields='right',
          desc='Wire pads to the coin vibration motor glued to the back cover')
lbl(J3.p('1'), '3V3')
lbl(J3.p('2'), 'MOT_N')
D4 = Part('Device:D_Schottky', 'D4', 'B5819W', 22, 90, fp='Diode_SMD:D_SOD-123', fields='right')
lbl(D4.p('1'), '3V3')
lbl(D4.p('2'), 'MOT_N')
Q2 = Part('Transistor_FET:AO3400A', 'Q2', 'AO3400A', 44, 92, fp=SOT, fields='right')
lbl(Q2.p('3'), 'MOT_N')
lbl(Q2.p('1'), 'Q2_G')
gnd(Q2.p('2'))
two('Device:R', 'R12', '100', 30, 102, 'MOTOR', 'Q2_G', R)
two('Device:R', 'R13', '100k', 36, 102, 'Q2_G', 'GND', R)
two('Device:C', 'C7', '10uF', 44, 104, '3V3', 'GND', C8)
two('Device:C', 'C8', '100nF', 50, 104, '3V3', 'GND', C)
text(5, 111, 'D2 = HIGH → вибрация. R13 держит мотор выключенным при загрузке.')

# ============================================================ 6. UI: button, LED, OLED
frame(62, 94, 132, 116, '6. Кнопка, светодиод, OLED SH1106 1,3″')
SW2 = Part('Switch:SW_Push', 'SW2', 'TL3342', 68, 100, fp='Button_Switch_SMD:SW_SPST_TL3342', fields='right',
           mpn='E-Switch TL3342F160QG')
lbl(SW2.p('1'), 'BTN')
gnd(SW2.p('2'))
two('Device:C', 'C9', '100nF', 74, 104, 'BTN', 'GND', C)
two('Device:R', 'R14', '1k', 82, 102, 'LED', 'D5_A', R)
D5 = Part('Device:LED', 'D5', 'blue', 88, 106, fp=LEDFP, fields='right')
lbl(D5.p('2'), 'D5_A')
gnd(D5.p('1'))
J4 = Part('Connector_Generic:Conn_01x04', 'J4', 'OLED', 110, 103, fp='Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical',
          fields='right', desc='SH1106 1.3in I2C module: 1 GND, 2 VCC, 3 SCL, 4 SDA (check your module!)')
gnd(J4.p('1'))
lbl(J4.p('2'), '3V3')
lbl(J4.p('3'), 'SCL')
lbl(J4.p('4'), 'SDA')
two('Device:R', 'R15', '4.7k', 122, 102, '3V3', 'SCL', R)
two('Device:R', 'R16', '4.7k', 127, 102, '3V3', 'SDA', R)
text(63, 112, 'Кнопка к GND, подтяжка в ESP32. Порядок выводов OLED сверить с модулем (бывает VCC/GND наоборот).')

text(136, 6, 'ХАБ ПОКЛЁВКИ — схема rev A (2026-11-25)\n\n'
             'Корпус «бипер» с клипсой на ремень / подставкой, 75 × 47 × 24 мм (с клипсой 30).\n'
             'Плата 68 × 40 мм, OLED на гребёнке над платой, LiPo за платой.\n\n'
             'ПРИМЕЧАНИЯ\n'
             '1. Питание: USB-C J1 → BQ24074 → SYS → SW1 → D3 → 5V XIAO →\n'
             '   стабилизатор XIAO 3,3 В → OLED, мотор, светодиод.\n'
             '2. Батарейные площадки XIAO не используются.\n'
             '3. Не подключать USB XIAO и J1 одновременно, пока не проверено.\n'
             '4. Ток хаба не измерен; 1200 мА·ч ≈ 14–18 ч (оценка).\n'
             '5. Концепт: перед заказом сверить номиналы с даташитами.', 1.5)

sch = ['kicad_sch', ['version', 20230121], ['generator', 'eeschema'], ['uuid', ROOT], ['paper', Q('A2')],
       ['title_block', ['title', Q('Хаб поклёвки на XIAO ESP32-C3 — rev A')], ['date', Q('2026-11-25')],
        ['rev', Q('A')], ['comment', 1, Q('Концепт: перед изготовлением сверить с даташитами')]],
       ['lib_symbols'] + list(lib_symbols.values())]
sch += items
sch += [['sheet_instances', ['path', Q('/'), ['page', Q('1')]]]]
open(f'{OUT}/{PROJ}.kicad_sch', 'w').write(dump(sch) + '\n')

lib = ['kicad_symbol_lib', ['version', 20220914], ['generator', 'kicad_symbol_editor']]
lib += [copy.deepcopy(s) for s in CUSTOM.values()]
open(f'{OUT}/hub.kicad_sym', 'w').write(dump(lib) + '\n')
open(f'{OUT}/sym-lib-table', 'w').write(
    '(sym_lib_table\n  (lib (name "hub")(type "KiCad")(uri "${KIPRJMOD}/hub.kicad_sym")(options "")(descr "Hub symbols"))\n)\n')
open(f'{OUT}/fp-lib-table', 'w').write(
    '(fp_lib_table\n  (lib (name "hub")(type "KiCad")(uri "${KIPRJMOD}/hub.pretty")(options "")(descr "Hub footprints"))\n)\n')
if not os.path.exists(f'{OUT}/{PROJ}.kicad_pro'):
    open(f'{OUT}/{PROJ}.kicad_pro', 'w').write(json.dumps({"meta": {"filename": f"{PROJ}.kicad_pro", "version": 1}}, indent=2))
print('parts:', len([p for p in parts if not p.startswith('#')]), 'items:', len(items))
