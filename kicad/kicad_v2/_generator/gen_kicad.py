"""Generate KiCad project for schematic v2."""
import copy, os, sys, uuid, json, math
sys.path.insert(0, os.path.dirname(__file__))
from sexp import parse, dump, Q, find, findall

KI = '/Applications/KiCad/KiCad.app/Contents/SharedSupport'
OUT = sys.argv[1]
PROJ = 'device_v2'
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
CUSTOM['BQ24074_MODULE'] = custom_symbol(
    'BQ24074_MODULE', 'U', 'BQ24074_module', (-10.16, 5.08, 10.16, -7.62),
    [pin('power_in', -15.24, 2.54, 0, 'IN', '1'), pin('passive', -15.24, -2.54, 0, 'TS', '2'),
     pin('power_out', 15.24, 2.54, 180, 'OUT', '3'), pin('bidirectional', 15.24, -2.54, 180, 'BAT', '4'),
     pin('power_in', 0, -12.7, 90, 'GND', '5')],
    (0, 6.35), (0, -13.97), 'BQ24074 Li-ion charger module with power path (e.g. Adafruit 4755)')
CUSTOM['PROT_1S'] = custom_symbol(
    'PROT_1S', 'U', 'Protection_1S', (-7.62, 5.08, 7.62, -5.08),
    [pin('passive', -12.7, 2.54, 0, 'B+', '1'), pin('passive', -12.7, -2.54, 0, 'B-', '2'),
     pin('passive', 12.7, 2.54, 180, 'P+', '3'), pin('passive', 12.7, -2.54, 180, 'P-', '4')],
    (0, 6.35), (0, -6.35), '1S Li-ion protection module (DW01A + FS8205A), low-side switching')


def get_symdef(lib_id):
    lib, name = lib_id.split(':')
    if lib == 'v2':
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

# =================================================================== BLOCK A: USB-C
frame(5, 5, 58, 35, '1. Вход USB-C (только заряд)')
J1 = Part('Connector:USB_C_Receptacle_PowerOnly_6P', 'J1', 'USB_C_PowerOnly', 13, 20,
          fp='Connector_USB:USB_C_Receptacle_GCT_USB4125-xx-x_6P_TopMnt_Horizontal')
lbl(J1.p('A9'), 'VBUS_IN')
lbl(J1.p('A5'), 'CC1')
lbl(J1.p('B5'), 'CC2')
gnd(J1.p('A12'))
gnd(J1.p('SH'))
two('Device:Polyfuse', 'F1', 'PTC 1.1A', 30, 15, 'VBUS_IN', 'VBUS_5V', 'Fuse:Fuse_1206_3216Metric')
VD1 = Part('Diode:SMF5V0A', 'VD1', 'SMF5V0A', 44, 14)
lbl(VD1.p('1'), 'VBUS_5V')
gnd(VD1.p('2'))
two('Device:R', 'R10', '5.1k', 40, 24, 'CC1', 'GND', RFP)
two('Device:R', 'R11', '5.1k', 48, 24, 'CC2', 'GND', RFP)
power('PWR_FLAG', mm(54), mm(19))
wire((mm(54), mm(19)), (mm(54), mm(20)))
label(mm(54), mm(20), 'VBUS_5V', (-1, 0))
text(6, 31, 'CC1/CC2 через 5.1 кОм на GND — иначе зарядник USB-C не подаст 5 В. D+/D− не используются.', 1.3)

# =================================================================== BLOCK B: battery + charger
frame(5, 37, 58, 74, '2. Аккумулятор, защита, заряд')
BT1 = Part('Device:Battery_Cell', 'BT1', '18650 Li-ion', 10, 48,
           fp='Battery:BatteryHolder_Keystone_1042_1x18650', fields='right')
lbl(BT1.p('1'), 'BT_P')
lbl(BT1.p('2'), 'B_MINUS')
Q1 = Part('Transistor_FET:AO3401A', 'Q1', 'AO3401A', 24, 48)
lbl(Q1.p('3'), 'BT_P')
lbl(Q1.p('2'), 'B_PLUS')
gnd(Q1.p('1'))
U1 = Part('v2:PROT_1S', 'U1', 'Protection_1S', 44, 48, fp='v2:PROT_1S_HDR4')
lbl(U1.p('1'), 'B_PLUS')
lbl(U1.p('2'), 'B_MINUS')
lbl(U1.p('3'), 'VBAT')
gnd(U1.p('4'))
text(6, 55, 'Q1 — защита от переполюсовки 18650.  U1: DW01A + FS8205A.\nB− идёт ТОЛЬКО на модуль защиты; системная GND = P−.', 1.3)
U2 = Part('v2:BQ24074_MODULE', 'U2', 'BQ24074 module', 24, 65, fp='v2:BQ24074_MODULE_HDR5')
lbl(U2.p('1'), 'VBUS_5V')
lbl(U2.p('2'), 'NTC')
lbl(U2.p('3'), 'SYS')
lbl(U2.p('4'), 'VBAT')
gnd(U2.p('5'))
two('Device:Thermistor_NTC', 'RT1', 'NTC 10k', 44, 64, 'NTC', 'GND', 'v2:NTC_WIRE_2P')
text(40, 69.5, 'RT1 — на корпусе 18650.\nU2: ISET → 1 А; EN2=1, EN1=0.', 1.2)

# =================================================================== BLOCK E: IMU
frame(5, 76, 58, 100, '5. IMU LSM6DSO (I²C, адрес 0x6A)')
U4 = Part('Sensor_Motion:LSM6DS3', 'U4', 'LSM6DSO', 33, 88, fields=((-15.24, 19.05), (-15.24, 16.51)))
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
two('Device:C', 'C1', '100nF', 8, 85, '3V3_SYS', 'GND', CFP)
two('Device:C', 'C2', '100nF', 14, 85, '3V3_SYS', 'GND', CFP)
two('Device:C', 'C3', '10uF', 11, 94, '3V3_SYS', 'GND', C8FP)
two('Device:R', 'R1', '4.7k', 47, 86, '3V3_SYS', 'SDA', RFP)
two('Device:R', 'R2', '4.7k', 53, 86, '3V3_SYS', 'SCL', RFP)

# =================================================================== BLOCK C: buck-boost
frame(60, 5, 117, 36, '3. Стабилизатор 3.3 В (buck-boost)')
two('Device:C', 'C4', '10uF', 66, 20, 'SYS', 'GND', C8FP)
SW1 = Part('Switch:SW_SPST', 'SW1', 'ON/OFF', 74, 14, fp='Button_Switch_SMD:SW_SPDT_PCM12')
lbl(SW1.p('1'), 'SYS')
lbl(SW1.p('2'), 'EN')
two('Device:R', 'R12', '1M', 76, 23, 'EN', 'GND', RFP)
U3 = Part('Regulator_Switching:TPS63001', 'U3', 'TPS63001', 97, 20, fp='Package_SON:Texas_DRC0010J')
lbl(U3.p('5'), 'SYS')
lbl(U3.p('8'), 'SYS')
lbl(U3.p('6'), 'EN')
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
               'SW1 коммутирует только EN (сигнальный ток), R12 держит EN в 0.', 1.2)

# =================================================================== BLOCK D: MCU
frame(60, 38, 117, 74, '4. Микроконтроллер XIAO ESP32-S3')
U5 = Part('v2:XIAO_ESP32S3', 'U5', 'XIAO ESP32-S3', 88, 54, fp='v2:XIAO_ESP32S3_THT', fields=((-15.24, 19.05), (-15.24, 16.51)))
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
text(61, 69, 'D2 (strap), D6/D7 (UART0), 5V — не подключать. VBAT_SENSE — только ADC1 (D0).\nC7 — у вывода 3V3 XIAO, для пиков тока Wi-Fi.', 1.2)

# =================================================================== BLOCK H: buzzer
frame(60, 76, 117, 100, '9. Зуммер')
BZ1 = Part('Device:Buzzer', 'BZ1', 'active 3.3V', 70, 85, fp='Buzzer_Beeper:Buzzer_12x9.5RM7.6')
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
text(61, 96.5, 'VD2 гасит выброс катушки зуммера; C10 — локальный фильтр 3V3.', 1.2)

# =================================================================== right column
frame(120, 5, 160, 24, '6. Кнопка')
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

# =================================================================== notes
text(5, 102, 'ПРИМЕЧАНИЯ (концепт v2):\n'
             '1. Распиновку модулей U1, U2, U5 сверить с их документацией.\n'
             '2. U4: символ LSM6DS3 (распиновка совпадает с LSM6DSO); выв. 10–11 не подключены — сверить с даташитом.\n'
             '3. Прошивка через USB-C XIAO: SW1 = OFF, иначе на 3V3 два источника (LDO XIAO и U3).\n'
             '4. Прошивка: deep sleep при VBAT < 3.3 В; пробуждение по INT1 (D3) и BTN (D1).', 1.3)


# ------------------------------------------------------------------ write files
sch = ['kicad_sch', ['version', 20250114], ['generator', Q('eeschema')], ['generator_version', Q('9.0')],
       ['uuid', ROOT], ['paper', Q('A3')],
       ['title_block', ['title', Q('Устройство на XIAO ESP32-S3 — схема v2')], ['date', Q('2026-11-21')],
        ['rev', Q('v2')], ['comment', 1, Q('Концепт: перед изготовлением сверить с даташитами')]],
       ['lib_symbols'] + list(lib_symbols.values())]
sch += items
sch += [['sheet_instances', ['path', Q('/'), ['page', Q('1')]]], ['embedded_fonts', 'no']]
open(f'{OUT}/{PROJ}.kicad_sch', 'w').write(dump(sch) + '\n')

lib = ['kicad_symbol_lib', ['version', 20251024], ['generator', Q('claude')], ['generator_version', Q('10.0')]]
lib += [copy.deepcopy(s) for s in CUSTOM.values()]
open(f'{OUT}/v2.kicad_sym', 'w').write(dump(lib) + '\n')
open(f'{OUT}/sym-lib-table', 'w').write(
    '(sym_lib_table\n  (version 7)\n  (lib (name "v2")(type "KiCad")(uri "${KIPRJMOD}/v2.kicad_sym")(options "")(descr "Project symbols"))\n)\n')
open(f'{OUT}/fp-lib-table', 'w').write(
    '(fp_lib_table\n  (version 7)\n  (lib (name "v2")(type "KiCad")(uri "${KIPRJMOD}/v2.pretty")(options "")(descr "Project footprints"))\n)\n')
open(f'{OUT}/{PROJ}.kicad_pro', 'w').write(json.dumps({"meta": {"filename": f"{PROJ}.kicad_pro", "version": 3}}, indent=2))
print('parts:', len([p for p in parts if not p.startswith('#')]), 'items:', len(items))
