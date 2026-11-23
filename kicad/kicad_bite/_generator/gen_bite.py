"""Generate KiCad project: bite sensor, schematic v4.1."""
import copy, os, sys, uuid, json, math
sys.path.insert(0, os.path.dirname(__file__))
from sexp import parse, dump, Q, find, findall

KI = '/Applications/KiCad/KiCad.app/Contents/SharedSupport'
OUT = sys.argv[1]
PROJ = 'bite_sensor'
os.makedirs(OUT, exist_ok=True)
U = 2.54


def uid():
    return Q(str(uuid.uuid4()))


ROOT = uid()

# ------------------------------------------------------------------ library access
_libcache = {}


def lib_block(lib, name):
    if lib not in _libcache:
        _libcache[lib] = open(f'{KI}/symbols/{lib}.kicad_sym').read()
    t = _libcache[lib]
    i = t.index(f'\n\t(symbol "{name}"\n') + 1
    j = t.find('\n\t(symbol "', i + 5)
    if j < 0:
        j = t.rindex('\n)')
    return parse(t[i:j])[0]


def flatten(lib, name):
    s = lib_block(lib, name)
    ext = find(s, 'extends')
    if not ext:
        return copy.deepcopy(s)
    parent = flatten(lib, ext[1])
    pname = parent[1]
    child_props = {p[1]: p for p in findall(s, 'property')}
    new = ['symbol', Q(name)]
    for e in parent[2:]:
        if isinstance(e, list) and e[0] == 'property':
            new.append(copy.deepcopy(child_props.pop(e[1], e)))
        elif isinstance(e, list) and e[0] == 'symbol':
            sub = copy.deepcopy(e)
            sub[1] = Q(name + sub[1][len(pname):])
            new.append(sub)
        else:
            new.append(copy.deepcopy(e))
    for p in child_props.values():
        new.insert(2, copy.deepcopy(p))
    return new


def eff(size=1.27, hide=False, justify=None, bold=False):
    font = ['font', ['size', size, size]]
    if bold:
        font.append(['bold', 'yes'])
    e = ['effects', font]
    if justify:
        e.append(['justify'] + justify.split())
    if hide:
        e.append(['hide', 'yes'])
    return e


def pin(ptype, x, y, ang, name, num, length=5.08):
    return ['pin', ptype, 'line', ['at', x, y, ang], ['length', length],
            ['name', Q(name), eff()], ['number', Q(num), eff()]]


def custom_symbol(name, ref, value, rect, pins, ref_at, val_at, desc, datasheet=''):
    return ['symbol', Q(name), ['pin_names', ['offset', 1.016]],
            ['exclude_from_sim', 'no'], ['in_bom', 'yes'], ['on_board', 'yes'],
            ['property', Q('Reference'), Q(ref), ['at', ref_at[0], ref_at[1], 0], eff()],
            ['property', Q('Value'), Q(value), ['at', val_at[0], val_at[1], 0], eff()],
            ['property', Q('Footprint'), Q(''), ['at', 0, 0, 0], eff(hide=True)],
            ['property', Q('Datasheet'), Q(datasheet), ['at', 0, 0, 0], eff(hide=True)],
            ['property', Q('Description'), Q(desc), ['at', 0, 0, 0], eff(hide=True)],
            ['symbol', Q(name + '_0_1'),
             ['rectangle', ['start', rect[0], rect[1]], ['end', rect[2], rect[3]],
              ['stroke', ['width', 0.254], ['type', 'default']], ['fill', ['type', 'background']]]],
            ['symbol', Q(name + '_1_1')] + pins,
            ['embedded_fonts', 'no']]


CUSTOM = {}
xp = []
L_ = [('1', 'D0/GPIO1/A0'), ('2', 'D1/GPIO2'), ('3', 'D2/GPIO3'), ('4', 'D3/GPIO4'),
      ('5', 'D4/GPIO5/SDA'), ('6', 'D5/GPIO6/SCL'), ('7', 'D6/GPIO43/TX')]
for i, (n, nm) in enumerate(L_):
    xp.append(pin('bidirectional', -20.32, 12.7 - 2.54 * i, 0, nm, n))
xp.append(pin('power_in', 20.32, 12.7, 180, '5V', '14'))
xp.append(pin('power_in', 20.32, 10.16, 180, '3V3', '12'))
R_ = [('11', 'D10/GPIO9'), ('10', 'D9/GPIO8'), ('9', 'D8/GPIO7'), ('8', 'D7/GPIO44/RX')]
for i, (n, nm) in enumerate(R_):
    xp.append(pin('bidirectional', 20.32, 2.54 - 2.54 * i, 180, nm, n))
xp.append(pin('power_in', 0, -22.86, 90, 'GND', '13'))
CUSTOM['XIAO_ESP32S3'] = custom_symbol(
    'XIAO_ESP32S3', 'U', 'XIAO_ESP32S3', (-15.24, 15.24, 15.24, -17.78), xp,
    (0, 16.51), (0, -25.4), 'Seeed Studio XIAO ESP32-S3 module',
    'https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/')
CUSTOM['FS8205A'] = custom_symbol(
    'FS8205A', 'U', 'FS8205A', (-7.62, 5.08, 7.62, -5.08),
    [pin('passive', -12.7, 2.54, 0, 'S1', '2'), pin('passive', -12.7, 2.54, 0, 'S1', '3'),
     pin('input', -12.7, -2.54, 0, 'G1', '4'), pin('input', 12.7, -2.54, 180, 'G2', '5'),
     pin('passive', 12.7, 2.54, 180, 'S2', '6'), pin('passive', 12.7, 2.54, 180, 'S2', '7'),
     pin('passive', 0, 10.16, 270, 'D', '1'), pin('passive', 0, 10.16, 270, 'D', '8')],
    (-7.62, 11.43), (0, -6.35), 'Dual N-MOSFET, common drain (TSSOP-8), for 1S protection')


def get_symdef(lib_id):
    lib, name = lib_id.split(':')
    if lib == 'bs':
        return copy.deepcopy(CUSTOM[name])
    return flatten(lib, name)


# ------------------------------------------------------------------ schematic model
items = []
lib_symbols = {}
parts = {}
pwr_n = [0]


def mm(v):
    return round(v * U, 4)


def sym_pins(sd):
    res = {}
    for sub in findall(sd, 'symbol'):
        for p in findall(sub, 'pin'):
            at = find(p, 'at')
            num = find(p, 'number')[1]
            res.setdefault(num, (float(at[1]), float(at[2]), float(at[3])))
    return res


class Part:
    def __init__(self, lib_id, ref, value, x, y, fp=None, desc=None, fields='lib', hide_ref=False, hide_val=False):
        sd = get_symdef(lib_id)
        if lib_id not in lib_symbols:
            emb = copy.deepcopy(sd)
            emb[1] = Q(lib_id)
            lib_symbols[lib_id] = emb
        self.ref, self.x, self.y = ref, x, y
        X, Y = mm(x), mm(y)
        self.pins = {}
        for num, (px, py, ang) in sym_pins(sd).items():
            ax, ay = X + px, Y - py
            a = math.radians(ang)
            dx, dy = -round(math.cos(a)), round(math.sin(a))
            self.pins[num] = (round(ax, 4), round(ay, 4), dx, dy)
        props = {p[1]: p for p in findall(sd, 'property')}
        inst = ['symbol', ['lib_id', Q(lib_id)], ['at', X, Y, 0], ['unit', 1],
                ['exclude_from_sim', 'no'], ['in_bom', 'no' if ref.startswith('#') else 'yes'],
                ['on_board', 'no' if ref.startswith('#') else 'yes'], ['dnp', 'no'], ['uuid', uid()]]
        vals = {'Reference': ref, 'Value': value}
        if fp is not None:
            vals['Footprint'] = fp
        if desc is not None:
            vals['Description'] = desc
        for key in ('Reference', 'Value', 'Footprint', 'Datasheet', 'Description'):
            lp = props.get(key)
            text = vals.get(key, lp[2] if lp else '')
            if lp:
                at = find(lp, 'at')
                px, py = float(at[1]), float(at[2])
                e = copy.deepcopy(find(lp, 'effects'))
                hid = find(lp, 'hide')
                if (hid and hid[1] == 'yes') or key in ('Footprint', 'Datasheet', 'Description'):
                    if not find(e, 'hide'):
                        e.append(['hide', 'yes'])
            else:
                px, py, e = 0, 0, eff(hide=True)
            ang = 0
            if isinstance(fields, tuple) and key in ('Reference', 'Value'):
                px, py = fields[0] if key == 'Reference' else fields[1]
                e = eff(justify='left')
            if fields == 'left' and key in ('Reference', 'Value'):
                px, py = -2.54, (1.27 if key == 'Reference' else -1.27)
                e = eff(justify='right')
            if fields == 'right' and key in ('Reference', 'Value'):
                px, py = 2.54, (1.27 if key == 'Reference' else -1.27)
                e = eff(justify='left')
            if (key == 'Reference' and hide_ref) or (key == 'Value' and hide_val):
                e = eff(hide=True)
            inst.append(['property', Q(key), Q(text), ['at', round(X + px, 4), round(Y - py, 4), ang], e])
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


def junction(x, y):
    items.append(['junction', ['at', round(x, 4), round(y, 4)], ['diameter', 0], ['color', 0, 0, 0, 0], ['uuid', uid()]])


def label(x, y, net, direction):
    ang = 180 if direction == (-1, 0) else 0
    just = 'right bottom' if ang == 180 else 'left bottom'
    items.append(['label', Q(net), ['at', round(x, 4), round(y, 4), ang], ['fields_autoplaced', 'yes'],
                  eff(justify=just), ['uuid', uid()]])


def lbl(pinref, net, stub=1.0):
    x, y, dx, dy = pinref
    ex, ey = x + dx * stub * U, y + dy * stub * U
    wire((x, y), (ex, ey))
    label(ex, ey, net, (dx, dy))
    return ex, ey


def power(kind, x, y, rot=0):
    pwr_n[0] += 1
    lib_id = 'power:' + kind
    ref = ('#FLG' if kind == 'PWR_FLAG' else '#PWR') + f'{pwr_n[0]:02d}'
    Part(lib_id, ref, kind, x / U, y / U, hide_ref=True, hide_val=bool(rot))
    if rot:
        items[-1][2][3] = rot


def gnd_at(x, y):
    power('GND', x, y)


def gnd(pinref, stub=1.0):
    x, y, dx, dy = pinref
    if (dx, dy) == (0, 1):
        ex, ey = x, y + stub * U
        wire((x, y), (ex, ey))
    else:
        mx, my = x + dx * stub * U, y + dy * stub * U
        ex, ey = mx, my + U
        wire((x, y), (mx, my), (ex, ey))
    gnd_at(ex, ey)


def nc(pinref):
    x, y = pinref[0], pinref[1]
    items.append(['no_connect', ['at', x, y], ['uuid', uid()]])


def text(x, y, s, size=1.5, bold=False):
    items.append(['text', Q(s), ['exclude_from_sim', 'no'], ['at', mm(x), mm(y), 0],
                  eff(size, justify='left top', bold=bold), ['uuid', uid()]])


def frame(x1, y1, x2, y2, title):
    items.append(['rectangle', ['start', mm(x1), mm(y1)], ['end', mm(x2), mm(y2)],
                  ['stroke', ['width', 0], ['type', 'dash']], ['fill', ['type', 'none']], ['uuid', uid()]])
    text(x1 + 0.5, y1 + 0.5, title, 2.0, bold=True)


def two(lib_id, ref, val, x, y, top, bot, fp, fields='right'):
    """vertical 2-pin part; top/bot = net name or 'GND'"""
    pr = Part(lib_id, ref, val, x, y, fp=fp, fields=fields)
    for num, net in (('1', top), ('2', bot)):
        if net == 'GND':
            gnd(pr.p(num))
        elif net:
            lbl(pr.p(num), net)
    return pr


RFP = 'Resistor_SMD:R_0603_1608Metric'
CFP = 'Capacitor_SMD:C_0603_1608Metric'
C8FP = 'Capacitor_SMD:C_0805_2012Metric'

def gnd_bus(pinrefs, bx):
    """connect several left-side pins to GND through a vertical bus at x=bx (mm)"""
    ys = sorted(p[1] for p in pinrefs)
    for (x, y, dx, dy) in pinrefs:
        wire((x, y), (bx, y))
    for a_, b_ in zip(ys, ys[1:]):
        wire((bx, a_), (bx, b_))
    for y in ys[1:-1]:
        junction(bx, y)
    junction(bx, ys[0])
    wire((bx, ys[0]), (bx - 2 * U, ys[0]), (bx - 2 * U, ys[0] + U))
    gnd_at(bx - 2 * U, ys[0] + U)


# =================================================================== BLOCK 1: USB-C + charger
frame(5, 5, 58, 45, '1. Магнитный разъём зарядки и зарядное устройство BQ24074 (1 А)')
J1 = Part('Connector_Generic:Conn_01x02', 'J1', 'MAG_CHG 2P', 14, 16,
          fp='bs:CHG_WIRE_SPLIT', fields='right',
          desc='Wire pads to the 2-pin magnetic charging connector potted in the enclosure cap')
lbl(J1.p('1'), 'VBUS_IN')
gnd(J1.p('2'))
two('Device:Polyfuse', 'F1', 'PTC 1.1A', 26, 12, 'VBUS_IN', 'VBUS_5V', 'Fuse:Fuse_1206_3216Metric')
VD1 = Part('Diode:SMF5V0A', 'VD1', 'SMF5V0A', 35, 11)
lbl(VD1.p('1'), 'VBUS_5V')
gnd(VD1.p('2'))
power('PWR_FLAG', mm(54), mm(19))
wire((mm(54), mm(19)), (mm(54), mm(20)))
label(mm(54), mm(20), 'VBUS_5V', (-1, 0))

U6 = Part('Battery_Management:BQ24074RGT', 'U6', 'BQ24074RGT', 26, 32)
lbl(U6.p('13'), 'VBUS_5V')
gnd(U6.p('8'))
gnd_bus([U6.p('4'), U6.p('6')], U6.p('4')[0] - 3 * U)
lbl(U6.p('5'), 'SYS')
lbl(U6.p('16'), 'ISET')
lbl(U6.p('12'), 'ILIM')
nc(U6.p('14'))
nc(U6.p('15'))
lbl(U6.p('10'), 'SYS')
lbl(U6.p('2'), 'VBAT')
lbl(U6.p('1'), 'NTC')
nc(U6.p('7'))
lbl(U6.p('9'), 'CHG_N')
two('Device:R', 'R15', '910', 9, 37, 'ISET', 'GND', RFP)
two('Device:R', 'R16', '1.1k', 15, 37, 'ILIM', 'GND', RFP)
two('Device:C', 'C12', '1uF', 41, 27, 'VBUS_5V', 'GND', CFP)
two('Device:C', 'C13', '4.7uF', 47, 27, 'SYS', 'GND', C8FP)
two('Device:C', 'C14', '4.7uF', 53, 27, 'VBAT', 'GND', C8FP)
LED3 = Part('Device:LED', 'LED3', 'orange CHG', 46, 39, fp='LED_SMD:LED_0603_1608Metric')
lbl(LED3.p('1'), 'CHG_N')
lbl(LED3.p('2'), 'LED3_A')
two('Device:R', 'R17', '1.5k', 54, 39, 'VBUS_5V', 'LED3_A', RFP)
text(6, 42.5, 'R15: Iзар = 890/R15 ≈ 1.0 А.  R16: Iвх = 1550/R16 ≈ 1.4 А.  EN2=1, EN1=0 → режим ILIM.\n'
              'TMR и ITERM не подключены = значения по умолчанию (сверить с даташитом BQ24074).', 1.2)

# =================================================================== BLOCK 2: battery + protection
frame(5, 47, 58, 78, '2. Аккумулятор 18650 и защита (DW01A + FS8205A)')
J2 = Part('Connector_Generic:Conn_01x04', 'J2', 'JST-PH 4P', 12, 57,
          fp='Connector_JST:JST_PH_B4B-PH-SM4-TB_1x04-1MP_P2.00mm_Vertical', fields='lib',
          desc='JST-PH 4-pin vertical SMD on the bottom side at the cap end: 18650 cell + NTC cable (PHR-4)')
lbl(J2.p('1'), 'BT_P')
lbl(J2.p('2'), 'B_MINUS')
lbl(J2.p('3'), 'NTC')
gnd(J2.p('4'))
Q1 = Part('Transistor_FET:AO3401A', 'Q1', 'AO3401A', 26, 56)
lbl(Q1.p('3'), 'BT_P')
lbl(Q1.p('2'), 'VBAT')
gnd(Q1.p('1'))
U7 = Part('Battery_Management:DW01A', 'U7', 'DW01A', 42, 58)
lbl(U7.p('5'), 'DW_VCC')
lbl(U7.p('6'), 'B_MINUS')
lbl(U7.p('2'), 'DW_CS')
nc(U7.p('4'))
lbl(U7.p('1'), 'DW_OD', stub=2)
lbl(U7.p('3'), 'DW_OC')
two('Device:R', 'R13', '100', 33, 52, 'VBAT', 'DW_VCC', RFP)
two('Device:C', 'C11', '100nF', 52, 57, 'DW_VCC', 'B_MINUS', CFP)
two('Device:R', 'R14', '1k', 52, 65, 'DW_CS', 'GND', RFP)
for fx, net in ((47, 'DW_VCC'), (53, 'B_MINUS')):
    power('PWR_FLAG', mm(fx), mm(51))
    wire((mm(fx), mm(51)), (mm(fx), mm(52)))
    label(mm(fx), mm(52), net, (1, 0))
U8 = Part('bs:FS8205A', 'U8', 'FS8205A', 26, 70, fp='Package_SO:TSSOP-8_4.4x3mm_P0.65mm')
lbl(U8.p('2'), 'B_MINUS')
lbl(U8.p('4'), 'DW_OD')
sx, sy = U8.p('6')[:2]
wire((sx, sy), (sx + 5 * U, sy), (sx + 5 * U, sy + 2 * U))
gnd_at(sx + 5 * U, sy + 2 * U)
lbl(U8.p('5'), 'DW_OC')
nc(U8.p('1'))
text(6, 64, 'J2 — кабель ячейки (PHR-4): 1 = B+, 2 = B−,\n3–4 = NTC 10k, приклеен к ячейке.\nДля хранения J2 отсоединяют.\nQ1 — защита от переполюсовки.\nB− только через U8; GND = P− (S2 у U8).', 1.2)

# =================================================================== BLOCK E: IMU
frame(5, 80, 58, 102, '5. IMU LSM6DSO (I²C, адрес 0x6A)')
U4 = Part('Sensor_Motion:LSM6DS3', 'U4', 'LSM6DSO', 33, 91, fields=((-15.24, 19.05), (-15.24, 16.51)))
vx, vy = U4.p('5')[:2]
wx, wy = U4.p('8')[:2]
wire((vx, vy), (vx, vy - U), (vx, vy - 2 * U))
wire((wx, wy), (wx, wy - U), (vx, vy - U))
junction(vx, vy - U)
label(vx, vy - 2 * U, '3V3_SYS', (0, -1))
pts = [U4.p('1'), U4.p('2'), U4.p('3')]
bx = pts[0][0] - 2 * U
for (x, y, dx, dy) in pts:
    wire((x, y), (bx, y))
wire((bx, pts[0][1]), (bx, pts[1][1]))
wire((bx, pts[1][1]), (bx, pts[2][1]))
junction(bx, pts[1][1])
wire((bx, pts[2][1]), (bx - 2 * U, pts[2][1]))
junction(bx, pts[2][1])
wire((bx - 2 * U, pts[2][1]), (bx - 2 * U, pts[2][1] + U))
gnd_at(bx - 2 * U, pts[2][1] + U)
lbl(U4.p('14'), 'SDA')
lbl(U4.p('13'), 'SCL')
lbl(U4.p('12'), '3V3_SYS')
lbl(U4.p('4'), 'IMU_INT1')
nc(U4.p('9'))
nc(U4.p('10'))
nc(U4.p('11'))
gnd(U4.p('6'))
two('Device:C', 'C1', '100nF', 8, 87, '3V3_SYS', 'GND', CFP)
two('Device:C', 'C2', '100nF', 14, 87, '3V3_SYS', 'GND', CFP)
two('Device:C', 'C3', '10uF', 11, 96, '3V3_SYS', 'GND', C8FP)
two('Device:R', 'R1', '4.7k', 47, 89, '3V3_SYS', 'SDA', RFP)
two('Device:R', 'R2', '4.7k', 53, 89, '3V3_SYS', 'SCL', RFP)

# =================================================================== BLOCK C: buck-boost
frame(60, 5, 117, 36, '3. Стабилизатор 3.3 В (buck-boost)')
two('Device:C', 'C4', '10uF', 66, 20, 'SYS', 'GND', C8FP)
U3 = Part('Regulator_Switching:TPS63001', 'U3', 'TPS63001', 97, 20, fp='Package_SON:Texas_DRC0010J')
lbl(U3.p('5'), 'SYS')
lbl(U3.p('8'), 'SYS')
lbl(U3.p('6'), 'SYS')
x, y, dx, dy = U3.p('7')
wire((x, y), (x - 2 * U, y))
power('GND', x - 2 * U, y, rot=270)
L1 = Part('Device:L', 'L1', '1.5uH', 89, 22, fp='Inductor_SMD:L_Bourns-SRN4018', fields='left')
l1x, l1y = U3.p('4')[:2]
l2x, l2y = U3.p('2')[:2]
wire((l1x, l1y), (mm(89), l1y), L1.p('1')[:2])
wire((l2x, l2y), (mm(89), l2y), L1.p('2')[:2])
lbl(U3.p('1'), '3V3_SYS')
lbl(U3.p('10'), '3V3_SYS')
gx, gy = U3.p('9')[:2]
px, py = U3.p('3')[:2]
wire((gx, gy), (gx, gy + U))
wire((px, py), (px, gy + U))
wire((gx, gy + U), (px, gy + U))
gnd_at(px, gy + U)
two('Device:C', 'C5', '22uF', 107, 20, '3V3_SYS', 'GND', C8FP)
two('Device:C', 'C6', '22uF', 113, 20, '3V3_SYS', 'GND', C8FP)
text(61, 29.5, 'TPS63001: фикс. 3.3 В, FB → VOUT. EN=0 → выход отключён (load disconnect).\n'
               'PS/SYNC=0 → энергосбережение. L1: Isat ≥ 2.5 А. Силовую петлю делать короткой.\n'
               'EN = SYS: выключателя нет, «выкл» = deep sleep ESP32-S3 (корпус герметичный).', 1.2)

# =================================================================== BLOCK D: MCU
frame(60, 38, 117, 74, '4. Микроконтроллер XIAO ESP32-S3')
U5 = Part('bs:XIAO_ESP32S3', 'U5', 'XIAO ESP32-S3', 88, 54, fp='bs:XIAO_ESP32S3_THT', fields=((-15.24, 19.05), (-15.24, 16.51)))
for num, net in (('1', 'VBAT_SENSE'), ('2', 'BTN'), ('4', 'IMU_INT1'), ('5', 'SDA'), ('6', 'SCL'),
                 ('12', '3V3_SYS'), ('11', 'BUZZ_EN'), ('10', 'LED2'), ('9', 'LED1')):
    lbl(U5.p(num), net)
for num in ('3', '7', '8', '14'):
    nc(U5.p(num))
gnd(U5.p('13'))
two('Device:C', 'C7', '100uF', 108, 56, '3V3_SYS', 'GND', 'Capacitor_SMD:C_1206_3216Metric')
power('PWR_FLAG', mm(114), mm(60))
wire((mm(114), mm(60)), (mm(114), mm(61)))
gnd_at(mm(114), mm(61))
text(61, 69, 'USB-C XIAO — только прошивка (под крышкой корпуса). D2 (strap), D6/D7 (UART0), 5V — не подключать.\nVBAT_SENSE — только ADC1 (D0). C7 — у вывода 3V3 XIAO.', 1.2)

# =================================================================== BLOCK H: buzzer
frame(60, 76, 117, 102, '9. Зуммер (пассивный SMD, ШИМ ~4 кГц)')
BZ1 = Part('Device:Buzzer', 'BZ1', 'CMT-8504-100-SMT', 70, 85, fp='Buzzer_Beeper:MagneticBuzzer_CUI_CMT-8504-100-SMT')
lbl(BZ1.p('1'), '3V3_SYS')
lbl(BZ1.p('2'), 'BZ_N')
VD2 = Part('Diode:1N4148W', 'VD2', '1N4148W', 82, 83)
lbl(VD2.p('1'), '3V3_SYS')
lbl(VD2.p('2'), 'BZ_N')
two('Device:C', 'C10', '10uF', 92, 86, '3V3_SYS', 'GND', C8FP)
two('Device:R', 'R6', '100', 99, 84, 'BUZZ_EN', 'Q2_G', RFP)
two('Device:R', 'R7', '100k', 104, 91, 'Q2_G', 'GND', RFP)
Q2 = Part('Transistor_FET:AO3400A', 'Q2', 'AO3400A', 111, 87)
lbl(Q2.p('1'), 'Q2_G')
lbl(Q2.p('3'), 'BZ_N')
gnd(Q2.p('2'))
text(61, 97, 'Звук формирует ШИМ (LEDC) на D10. VD2 гасит выброс катушки.\nНазначение площадок 1–4 BZ1 сверить с даташитом CUI.', 1.2)

# =================================================================== right column
frame(120, 5, 160, 24, '6. Кнопка (толкатель с O-кольцом сверху корпуса)')
two('Device:R', 'R3', '10k', 126, 13, '3V3_SYS', 'BTN', RFP)
two('Device:C', 'C8', '100nF', 136, 16, 'BTN', 'GND', CFP)
SW2 = Part('Switch:SW_Push', 'SW2', 'TL3342', 150, 14, fp='Button_Switch_SMD:SW_SPST_TL3342')
lbl(SW2.p('1'), 'BTN')
gnd(SW2.p('2'))

frame(120, 26, 160, 48, '7. Измерение VBAT')
two('Device:R', 'R8', '100k', 126, 33, 'VBAT', 'VBAT_SENSE', RFP)
two('Device:R', 'R9', '100k', 126, 42, 'VBAT_SENSE', 'GND', RFP)
two('Device:C', 'C9', '100nF', 138, 40, 'VBAT_SENSE', 'GND', CFP)
text(145, 32, 'VBAT/2 =\n1.5–2.1 В\n→ ADC1 (D0)', 1.2)

frame(120, 50, 160, 72, '8. Светодиоды (от GPIO)')
two('Device:R', 'R4', '680', 125, 57, 'LED1', 'LED1_A', RFP)
LED1 = Part('Device:LED', 'LED1', 'red', 131, 66, fp='LED_SMD:LED_0603_1608Metric')
lbl(LED1.p('2'), 'LED1_A')
gnd(LED1.p('1'))
two('Device:R', 'R5', '100*', 144, 57, 'LED2', 'LED2_A', RFP)
LED2 = Part('Device:LED', 'LED2', 'green', 150, 66, fp='LED_SMD:LED_0603_1608Metric')
lbl(LED2.p('2'), 'LED2_A')
gnd(LED2.p('1'))
text(121, 69.5, '* R5 подобрать по Vf зелёного LED (~3 В)', 1.2)

text(120, 76, 'ДАТЧИК ПОКЛЁВКИ — схема v4.1 (герметичный корпус)\n'
              'Корпус-пенал ~84 × 29 × 32 мм, плата 73 × 24 мм над ячейкой 18650.\n\n'
              'ПРИМЕЧАНИЯ:\n'
              '1. U4: символ LSM6DS3 (распиновка = LSM6DSO);\n'
              '   выв. 10–11 не подключены — сверить с даташитом.\n'
              '2. J1 — провода к магнитному разъёму в крышке. Без зарядки\n'
              '   на контактах нет напряжения (BQ24074 не пускает ток обратно).\n'
              '   Переполюсовка: VD1 открывается, F1 размыкает цепь.\n'
              '3. Выключателя нет: deep sleep (~70 мкА), пробуждение\n'
              '   по BTN (D1) и INT1 (D3). Deep sleep при VBAT < 3.3 В.\n'
              '4. Прошивка: OTA; аварийно — USB-C XIAO под крышкой.', 1.3)

# ------------------------------------------------------------------ write files
sch = ['kicad_sch', ['version', 20250114], ['generator', Q('eeschema')], ['generator_version', Q('9.0')],
       ['uuid', ROOT], ['paper', Q('A3')],
       ['title_block', ['title', Q('Датчик поклёвки на XIAO ESP32-S3 — схема v4.1')], ['date', Q('2026-11-22')],
        ['rev', Q('v4.1')], ['comment', 1, Q('Концепт: перед изготовлением сверить с даташитами')]],
       ['lib_symbols'] + list(lib_symbols.values())]
sch += items
sch += [['sheet_instances', ['path', Q('/'), ['page', Q('1')]]], ['embedded_fonts', 'no']]
open(f'{OUT}/{PROJ}.kicad_sch', 'w').write(dump(sch) + '\n')

lib = ['kicad_symbol_lib', ['version', 20251024], ['generator', Q('claude')], ['generator_version', Q('10.0')]]
lib += [copy.deepcopy(s) for s in CUSTOM.values()]
open(f'{OUT}/bs.kicad_sym', 'w').write(dump(lib) + '\n')
open(f'{OUT}/sym-lib-table', 'w').write(
    '(sym_lib_table\n  (version 7)\n  (lib (name "bs")(type "KiCad")(uri "${KIPRJMOD}/bs.kicad_sym")(options "")(descr "Project symbols"))\n)\n')
open(f'{OUT}/fp-lib-table', 'w').write(
    '(fp_lib_table\n  (version 7)\n  (lib (name "bs")(type "KiCad")(uri "${KIPRJMOD}/bs.pretty")(options "")(descr "Project footprints"))\n)\n')
if not os.path.exists(f'{OUT}/{PROJ}.kicad_pro'):
  open(f'{OUT}/{PROJ}.kicad_pro', 'w').write(json.dumps({"meta": {"filename": f"{PROJ}.kicad_pro", "version": 3}}, indent=2))
print('parts:', len([p for p in parts if not p.startswith('#')]), 'items:', len(items))
