"""Build bite_sensor.kicad_pcb: footprints, placement, nets, outline. Run with KiCad's python."""
import os, sys, re, subprocess
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sexp import parse, find, findall

PRJ = sys.argv[1]
STAGE = sys.argv[2] if len(sys.argv) > 2 else 'place'
KF = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints'
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
PCB = f'{PRJ}/bite_sensor.kicad_pcb'
mm = pcbnew.FromMM


def V(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


# ------------------------------------------------------------------ custom footprints
def kmod(name, descr, pads, silk=(), crtyd=None, labels=(), ref_at=(0, -5), val_at=(0, 5), extra='', pad=1.7, drill=1.0):
    o = [f'(footprint "{name}"', '  (version 20240108)', '  (generator "claude")', '  (layer "F.Cu")',
         f'  (descr "{descr}")', '  (attr through_hole)',
         f'  (property "Reference" "REF**" (at {ref_at[0]} {ref_at[1]} 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
         f'  (property "Value" "{name}" (at {val_at[0]} {val_at[1]} 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
    for (x1, y1, x2, y2) in silk:
        o.append(f'  (fp_rect (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.12) (type solid)) (fill no) (layer "F.SilkS"))')
        o.append(f'  (fp_rect (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.1) (type solid)) (fill no) (layer "F.Fab"))')
    for (x1, y1, x2, y2) in ([crtyd] if crtyd and not isinstance(crtyd[0], tuple) else (crtyd or [])):
        o.append(f'  (fp_rect (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))')
    for (t, x, y, just) in labels:
        j = f' (justify {just})' if just else ''
        o.append(f'  (fp_text user "{t}" (at {x} {y} 0) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12)){j}))')
    o.append(extra)
    for (num, x, y) in pads:
        shape = 'rect' if num == '1' else 'circle'
        o.append(f'  (pad "{num}" thru_hole {shape} (at {x} {y}) (size {pad} {pad}) (drill {drill}) (layers "*.Cu" "*.Mask"))')
    o.append(')')
    return '\n'.join(o) + '\n'


def write_footprints():
    d = f'{PRJ}/bs.pretty'
    os.makedirs(d, exist_ok=True)
    # XIAO ESP32-S3: 21 x 17.8 mm, two rows of 7 at 15.24 mm, USB at top
    names = ['D0', 'D1', 'D2', 'D3', 'D4', 'D5', 'D6', 'D7', 'D8', 'D9', 'D10', '3V3', 'GND', '5V']
    pads, labels = [], []
    for i in range(7):
        pads.append((str(i + 1), -7.62, -7.62 + 2.54 * i))
        labels.append((names[i], -6.3, -7.62 + 2.54 * i, 'left'))
    for i in range(7):
        n = 14 - i
        pads.append((str(n), 7.62, -7.62 + 2.54 * i))
        labels.append((names[n - 1], 6.3, -7.62 + 2.54 * i, 'right'))
    labels.append(('USB', 0, -9.3, None))
    open(f'{d}/XIAO_ESP32S3_THT.kicad_mod', 'w').write(kmod(
        'XIAO_ESP32S3_THT', 'Seeed XIAO ESP32-S3, through-hole pads (verify against Seeed footprint)', pads,
        silk=[(-8.9, -10.5, 8.9, 10.5)], crtyd=(-9.2, -11.6, 9.2, 10.8), labels=labels,
        ref_at=(0, 12), val_at=(0, 13.5)))
    # 18650 wire pads (to the enclosure spring contacts)
    pads = [('1', -1.6, 0), ('2', 1.6, 0)]
    open(f'{d}/BATT_WIRE_2P.kicad_mod', 'w').write(kmod(
        'BATT_WIRE_2P', 'Wire pads for the 18650 cell (+ / -), wires to the enclosure spring contacts', pads,
        silk=[(-3.0, -1.3, 3.0, 1.3)], crtyd=(-3.25, -1.55, 3.25, 1.55),
        labels=[('+', -1.6, -2.1, None), ('-', 1.6, -2.1, None)], ref_at=(0, 2.6), val_at=(0, 4)))
    # NTC on wires
    pads = [('1', -1.27, 0), ('2', 1.27, 0)]
    open(f'{d}/NTC_WIRE_2P.kicad_mod', 'w').write(kmod(
        'NTC_WIRE_2P', 'Wire pads for NTC thermistor glued to the 18650 cell', pads,
        silk=[(-2.6, -1.3, 2.6, 1.3)], crtyd=(-2.85, -1.55, 2.85, 1.55), labels=[('NTC', 0, -2.2, None)],
        ref_at=(0, 2.6), val_at=(0, 4)))
    # magnetic charging connector (potted in the enclosure cap): two wire pads in the
    # free edge strips left and right of the XIAO; 1 = VBUS_IN (+), 2 = GND (-)
    pads = [('1', 0, -10.45), ('2', 0, 10.45)]
    open(f'{d}/CHG_WIRE_SPLIT.kicad_mod', 'w').write(kmod(
        'CHG_WIRE_SPLIT', 'Wire pads to the 2-pin magnetic charging connector (26 AWG, 1 A)', pads,
        crtyd=[(-1.0, -11.2, 1.0, -9.7), (-1.0, 9.7, 1.0, 11.2)],
        labels=[('+', 1.5, -10.45, None)], ref_at=(2.6, -7.5), val_at=(0, 0),
        pad=1.5, drill=0.8))


# ------------------------------------------------------------------ schematic + netlist
def read_schematic():
    sch = parse(open(f'{PRJ}/bite_sensor.kicad_sch').read())[0]
    res = {}
    for s in findall(sch, 'symbol'):
        if not find(s, 'lib_id'):
            continue
        props = {p[1]: p[2] for p in findall(s, 'property')}
        ref = props.get('Reference', '')
        if ref.startswith('#'):
            continue
        res[ref] = dict(uuid=find(s, 'uuid')[1], fp=props.get('Footprint', ''), value=props.get('Value', ''),
                        datasheet=props.get('Datasheet', ''), desc=props.get('Description', ''))
    return res


def read_netlist():
    tmp = os.path.join(os.environ.get('TMPDIR', '/tmp'), 'bite_pcb_net.net')
    subprocess.run([KC, 'sch', 'export', 'netlist', '-o', tmp, f'{PRJ}/bite_sensor.kicad_sch'],
                   capture_output=True)
    n = parse(open(tmp).read())[0]
    pinnet = {}
    for net in findall(find(n, 'nets'), 'net'):
        name = find(net, 'name')[1]
        for x in findall(net, 'node'):
            pinnet[(find(x, 'ref')[1], find(x, 'pin')[1])] = name
    return pinnet


# ------------------------------------------------------------------ placement (mm, deg)
# v4.1: board +3 mm at the cap end (x 97…170) for the JST-PH cell connector J2 on the bottom
BX, BY, BW, BH = 97, 100, 73, 24
PLACE = {
    # left end: XIAO (USB-C to the left end), strips above/below it
    'U5': (107.3, 112.0, 90), 'C7': (103.5, 101.4, 0),
    'C9': (101.0, 122.6, 0), 'R8': (104.0, 122.6, 0), 'R9': (107.0, 122.6, 0),
    'R3': (113.0, 122.6, 0), 'C8': (116.0, 122.6, 0),
    'J1': (98.4, 112.0, 0),    # wire pads of the magnetic charging connector (cap end)
    # JST-PH 4P cell + NTC connector, BOTTOM side, body at the board edge, plug mates from below
    'J2': (101.7, 112.0, 270, 'B'),
    # buck-boost (EN = SYS, no power switch)
    'U3': (128.4, 111.5, 0), 'L1': (123.4, 111.5, 90), 'C5': (125.2, 107.6, 0), 'C6': (128.8, 107.6, 0),
    'C4': (126.0, 115.4, 0),
    # top UI at the closed right end: button under the O-ring pusher, LEDs beside it, buzzer
    'SW2': (166.0, 112.0, 90), 'LED1': (166.0, 105.5, 90), 'LED2': (166.0, 118.5, 90),
    'R4': (162.6, 105.6, 90), 'R5': (162.6, 118.4, 90),
    'BZ1': (155.4, 105.4, 0),
    # IMU
    'U4': (137.0, 111.2, 0), 'C1': (133.8, 110.2, 90), 'C2': (140.2, 110.2, 90), 'C3': (133.8, 113.6, 90),
    'R1': (140.2, 113.6, 90), 'R2': (142.2, 112.0, 90),
    # buzzer driver
    'Q2': (145.8, 108.8, 0), 'R6': (145.6, 112.2, 0), 'R7': (148.8, 111.6, 90), 'VD2': (148.9, 107.6, 90),
    'C10': (151.4, 112.0, 0),
    # battery, protection
    'Q1': (134.2, 121.6, 0), 'U8': (137.4, 117.4, 0), 'U7': (143.4, 117.6, 0),
    'R13': (146.8, 116.6, 90), 'R14': (148.6, 116.6, 90), 'C11': (150.4, 116.6, 90),
    # charger (right end)
    'U6': (155.6, 118.2, 0), 'C12': (157.6, 114.6, 0), 'C13': (152.2, 114.8, 90), 'C14': (151.4, 120.4, 90),
    'R15': (155.0, 122.8, 0), 'R16': (158.2, 122.8, 0), 'LED3': (159.8, 118.8, 90), 'R17': (160.4, 115.0, 90),
    'F1': (165.0, 102.0, 0), 'VD1': (164.6, 121.9, 0),
}


def load_fp(fpid):
    lib, name = fpid.split(':')
    path = f'{PRJ}/bs.pretty' if lib == 'bs' else f'{KF}/{lib}.pretty'
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        raise RuntimeError(f'footprint not found: {fpid}')
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def build():
    write_footprints()
    syms = read_schematic()
    pinnet = read_netlist()
    board = pcbnew.NewBoard(PCB)
    ds = board.GetDesignSettings()
    dnc = ds.m_NetSettings.GetDefaultNetclass()
    dnc.SetClearance(mm(0.15))
    dnc.SetTrackWidth(mm(0.25))
    dnc.SetViaDiameter(mm(0.6))
    dnc.SetViaDrill(mm(0.3))
    ds.m_MinClearance = mm(0.15)
    ds.SetBoardThickness(mm(1.0))
    nets = {}
    for name in sorted(set(pinnet.values())):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni
    missing = []
    for ref, info in syms.items():
        if ref not in PLACE:
            missing.append(ref)
            continue
        fp = load_fp(info['fp'])
        board.Add(fp)
        fp.SetReference(ref)
        fp.SetValue(info['value'])
        try:
            fp.GetField(pcbnew.FIELD_T_DATASHEET).SetText(info['datasheet'])
            fp.GetField(pcbnew.FIELD_T_DESCRIPTION).SetText(info['desc'])
        except Exception as ex:
            print('field set failed', ex)
        x, y, rot = PLACE[ref][:3]
        fp.SetPosition(V(x, y))
        if len(PLACE[ref]) > 3 and PLACE[ref][3] == 'B':
            try:
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            except (AttributeError, TypeError):
                fp.Flip(fp.GetPosition(), False)
        fp.SetOrientationDegrees(rot)
        fp.SetPath(pcbnew.KIID_PATH('/' + info['uuid']))
        rt = fp.Reference()
        rt.SetTextSize(V(0.8, 0.8))
        rt.SetTextThickness(mm(0.12))
        rt.SetLayer(pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab)
        try:
            fp.SetSheetfile('bite_sensor.kicad_sch')
            fp.SetSheetname('/')
        except Exception:
            pass
        for pad in fp.Pads():
            key = (ref, pad.GetNumber())
            if key in pinnet:
                pad.SetNet(nets[pinnet[key]])
    if missing:
        print('NOT PLACED:', missing)
    gnd = nets['GND']
    # thermal vias under U3 exposed pad
    u3 = board.FindFootprintByReference('U3')
    c = u3.GetPosition()
    for dx in (-0.35, 0.35):
        for dy in (-0.6, 0.6):
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(c.x + mm(dx), c.y + mm(dy)))
            v.SetWidth(mm(0.5))
            v.SetDrill(mm(0.25))
            v.SetNet(gnd)
            v.SetLocked(True)
            board.Add(v)
    edge = pcbnew.PCB_SHAPE(board)
    edge.SetShape(pcbnew.SHAPE_T_RECT)
    edge.SetStart(V(BX, BY))
    edge.SetEnd(V(BX + BW, BY + BH))
    edge.SetLayer(pcbnew.Edge_Cuts)
    edge.SetWidth(mm(0.1))
    board.Add(edge)
    t = pcbnew.PCB_TEXT(board)
    t.SetText('BITE SENSOR v4.1')
    t.SetPosition(V(BX + 50, BY + 12))
    t.SetLayer(pcbnew.B_SilkS)
    t.SetMirrored(True)
    board.Add(t)
    t = pcbnew.PCB_TEXT(board)
    t.SetText('J2: 1 B+  2 B-  3-4 NTC')
    t.SetTextSize(V(0.8, 0.8))
    t.SetTextThickness(mm(0.12))
    t.SetPosition(V(BX + 15.0, BY + 12.0))   # between the XIAO pin rows
    t.SetLayer(pcbnew.B_SilkS)
    t.SetMirrored(True)
    board.Add(t)
    pcbnew.SaveBoard(PCB, board)
    print('saved', PCB, 'footprints:', len(board.GetFootprints()), 'nets:', len(nets))


POWER_HI = ['/VBUS_IN', '/VBUS_5V', '/VBAT', '/BT_P', '/B_PLUS', '/B_MINUS']
POWER = ['/SYS', '/3V3_SYS', 'GND', 'Net-(U3-L1)', 'Net-(U3-L2)', '/BZ_N']


def export_dsn():
    board = pcbnew.LoadBoard(PCB)
    dsn = f'{PRJ}/_route/bite_sensor.dsn'
    os.makedirs(os.path.dirname(dsn), exist_ok=True)
    ok = pcbnew.ExportSpecctraDSN(board, dsn)
    print('dsn export', ok)
    txt = open(dsn).read()
    if '(class Power' in txt:
        print('netclasses exported by KiCad; no patch needed')
        return
    m = re.search(r'\(class kicad_default(.*?)\(circuit', txt, re.S)
    body = m.group(1)

    def q(n):
        return f'"{n}"' if re.search(r'[\s()"/-]', n) or not n else n

    for n in POWER_HI + POWER:
        body = re.sub(r'(?<![\w"/-])' + re.escape(q(n)) + r'(?![\w"/-])', ' ', body)
    new_default = '(class kicad_default' + body + '(circuit'
    via = re.search(r'\(use_via [^)]+\)', txt[m.start():]).group(0)
    extra = ''
    for cname, lst, w in (('power_hi', POWER_HI, 800), ('power', POWER, 300)):
        extra += (f'(class {cname} ' + ' '.join(q(n) for n in lst) +
                  f'\n      (circuit\n        {via}\n      )\n      (rule\n        (width {w})\n        (clearance 150)\n      )\n    )\n    ')
    txt = txt[:m.start()] + extra + new_default + txt[m.end():]
    open(dsn, 'w').write(txt)
    print('classes patched')


def import_ses():
    board = pcbnew.LoadBoard(PCB)
    ses = f'{PRJ}/_route/bite_sensor.ses'
    ok = pcbnew.ImportSpecctraSES(board, ses)
    print('ses import', ok, 'tracks:', len(board.GetTracks()))
    gnd = board.FindNet('GND')
    for t in board.GetTracks():
        if t.GetClass() == 'PCB_TRACK' and t.GetWidth() < mm(0.15):
            t.SetWidth(mm(0.15))
    for ref in ('U5', 'J1', 'J2'):
        board.FindFootprintByReference(ref).SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(gnd)
        z.SetLocalClearance(mm(0.3))
        z.SetMinThickness(mm(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        z.SetIsFilled(False)
        ol = z.Outline()
        ol.NewOutline()
        for (x, y) in ((BX + 0.5, BY + 0.5), (BX + BW - 0.5, BY + 0.5), (BX + BW - 0.5, BY + BH - 0.5), (BX + 0.5, BY + BH - 0.5)):
            ol.Append(mm(x), mm(y))
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    print('zones filled, saved')


if STAGE == 'place':
    build()
elif STAGE == 'dsn':
    export_dsn()
elif STAGE == 'ses':
    import_ses()
