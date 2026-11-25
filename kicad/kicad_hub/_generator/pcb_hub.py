"""Build hub.kicad_pcb (rev A): footprints, placement, nets, outline, routing hand-off, zones, DRC.

Run with KiCad's python (KiCad 7 on Linux or KiCad 8/9 on macOS):
    python3 pcb_hub.py <project dir> place|dsn|ses|drc
"""
import os, re, subprocess, sys
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'kicad_bite', '_generator'))
from sexp import parse, find, findall  # noqa: E402

PRJ = os.path.abspath(sys.argv[1])
STAGE = sys.argv[2] if len(sys.argv) > 2 else 'place'
PCB = f'{PRJ}/hub.kicad_pcb'
mm = pcbnew.FromMM


def first_dir(*cands):
    for d in cands:
        if d and os.path.isdir(d):
            return d
    raise SystemExit('KiCad footprint library not found; set KICAD_FOOTPRINT_DIR')


KF = first_dir(os.environ.get('KICAD_FOOTPRINT_DIR'), os.environ.get('KICAD9_FOOTPRINT_DIR'),
               os.environ.get('KICAD7_FOOTPRINT_DIR'),
               '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints', '/usr/share/kicad/footprints')
KC = os.environ.get('KICAD_CLI') or ('/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
                                     if os.path.exists('/Applications/KiCad/KiCad.app') else 'kicad-cli')


def V(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


# ------------------------------------------------------------------ board geometry (mm)
BX, BY, BW, BH = 100.0, 100.0, 68.0, 40.0          # board 68 x 40, origin top-left
OLED = (104.0, 102.4, 139.4, 135.9)                # 1.3" SH1106 module outline above the board
HOLES = [(102.2, 102.2), (102.2, 137.8), (143.3, 102.6), (141.2, 137.6)]


# ------------------------------------------------------------------ custom footprints
def kmod(name, descr, pads, fab=(), crtyd=None, labels=(), ref_at=(0, -3), pad=1.7, drill=1.0, attr='through_hole'):
    o = [f'(footprint "{name}"', '  (version 20221018)', '  (generator pcbnew)', '  (layer "F.Cu")',
         f'  (descr "{descr}")', f'  (attr {attr})',
         f'  (fp_text reference "REF**" (at {ref_at[0]} {ref_at[1]}) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))',
         f'  (fp_text value "{name}" (at 0 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))']
    for (x1, y1, x2, y2, layer) in fab:
        o.append(f'  (fp_rect (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.12) (type solid)) (fill none) (layer "{layer}"))')
    if crtyd:
        x1, y1, x2, y2 = crtyd
        o.append(f'  (fp_rect (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))')
    for (t, x, y, just) in labels:
        j = f' (justify {just})' if just else ''
        o.append(f'  (fp_text user "{t}" (at {x} {y}) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12)){j}))')
    for (num, x, y) in pads:
        shape = 'rect' if num == '1' else 'circle'
        o.append(f'  (pad "{num}" thru_hole {shape} (at {x} {y}) (size {pad} {pad}) (drill {drill}) (layers "*.Cu" "*.Mask"))')
    o.append(')')
    return '\n'.join(o) + '\n'


def write_footprints():
    d = f'{PRJ}/hub.pretty'
    os.makedirs(d, exist_ok=True)
    # XIAO ESP32-C3 (21 x 17.5 mm): two rows of 7 at 15.24 mm, USB-C at -y. Same pad map as the sensor.
    names = ['D0', 'D1', 'D2', 'D3', 'D4', 'D5', 'D6', 'D7', 'D8', 'D9', 'D10', '3V3', 'GND', '5V']
    pads, labels = [], []
    for i in range(7):
        pads.append((str(i + 1), -7.62, -7.62 + 2.54 * i))
        labels.append((names[i], -6.2, -7.62 + 2.54 * i, 'left'))
        n = 14 - i
        pads.append((str(n), 7.62, -7.62 + 2.54 * i))
        labels.append((names[n - 1], 6.2, -7.62 + 2.54 * i, 'right'))
    labels.append(('USB', 0, -9.0, None))
    open(f'{d}/XIAO_THT.kicad_mod', 'w').write(kmod(
        'XIAO_THT', 'Seeed XIAO ESP32-C3/S3, 14 through-hole pads (castellations soldered flat or on pins)', pads,
        fab=[(-8.75, -10.5, 8.75, 10.5, 'F.SilkS'), (-8.75, -10.5, 8.75, 10.5, 'F.Fab')],
        crtyd=(-9.0, -11.8, 9.0, 10.8), labels=labels, ref_at=(0, 11.6)))
    # two wire pads for the coin motor (wires go to the back cover)
    open(f'{d}/WIRE_2P.kicad_mod', 'w').write(kmod(
        'WIRE_2P', 'Two wire pads, 2.54 mm, for the vibration motor (+ = 3V3)', [('1', 0, -1.27), ('2', 0, 1.27)],
        fab=[(-1.3, -2.6, 1.3, 2.6, 'F.SilkS')], crtyd=(-1.4, -2.7, 1.4, 2.7),
        labels=[('M+', 2.2, -1.27, None)], ref_at=(0, -3.6), pad=1.6, drill=0.9))
    # SH1106 1.3" module: 4-pin 2.54 header on its top edge; outline on F.Fab only (low parts may sit under it)
    pads = [(str(i + 1), -3.81 + 2.54 * i, 0) for i in range(4)]
    w, h, hy = OLED[2] - OLED[0], OLED[3] - OLED[1], 1.7
    labels = [(t, -3.81 + 2.54 * i, 2.0, None) for i, t in enumerate(('G', 'V', 'C', 'D'))]
    open(f'{d}/OLED_SH1106_13_HDR.kicad_mod', 'w').write(kmod(
        'OLED_SH1106_13_HDR', 'SH1106 1.3in I2C OLED module on a 4-pin header: 1 GND 2 VCC 3 SCL 4 SDA (verify module)',
        pads, fab=[(-w / 2, -hy, w / 2, h - hy, 'F.Fab'), (-5.2, -1.3, 5.2, 1.3, 'F.SilkS')],
        crtyd=(-5.4, -1.5, 5.4, 1.5), labels=labels, ref_at=(7.0, 0), pad=1.7, drill=1.0))


# ------------------------------------------------------------------ schematic + netlist
def read_schematic():
    sch = parse(open(f'{PRJ}/hub.kicad_sch').read())[0]
    res = {}
    for s in findall(sch, 'symbol'):
        if not find(s, 'lib_id'):
            continue
        props = {p[1]: p[2] for p in findall(s, 'property')}
        ref = props.get('Reference', '')
        if ref.startswith('#'):
            continue
        res[ref] = dict(uuid=find(s, 'uuid')[1], fp=props.get('Footprint', ''), value=props.get('Value', ''))
    return res


def read_netlist():
    tmp = os.path.join(os.environ.get('TMPDIR', '/tmp'), 'hub_pcb_net.net')
    subprocess.run([KC, 'sch', 'export', 'netlist', '-o', tmp, f'{PRJ}/hub.kicad_sch'], capture_output=True)
    n = parse(open(tmp).read())[0]
    pinnet = {}
    for net in findall(find(n, 'nets'), 'net'):
        name = find(net, 'name')[1]
        for x in findall(net, 'node'):
            pinnet[(find(x, 'ref')[1], find(x, 'pin')[1])] = name
    return pinnet


# ------------------------------------------------------------------ placement (x, y, rot[, 'B' = bottom])
PLACE = {
    # MCU at the right, USB-C (flashing) at the top edge
    'U2': (156.0, 111.6, 0), 'C5': (166.2, 113.4, 90), 'C6': (166.2, 109.6, 90),
    # OLED header on the module's top edge, I2C pull-ups beside it
    'J4': (121.7, 104.1, 0), 'R15': (129.0, 104.0, 90), 'R16': (131.2, 104.0, 90),
    # charging input at the bottom edge, status LEDs next to it (light pipe)
    'J1': (150.0, 137.2, 0), 'R1': (145.0, 131.2, 90), 'R2': (147.0, 131.2, 90),
    'D1': (140.9, 125.8, 90), 'R8': (140.9, 129.0, 90), 'D2': (142.9, 125.8, 90), 'R9': (142.9, 129.0, 90),
    'F1': (135.4, 133.6, 0), 'VD1': (130.6, 133.6, 0),
    # charger under the OLED (low parts only)
    'U1': (127.0, 126.0, 0),
    'R3': (124.6, 122.0, 90), 'R6': (127.2, 121.6, 90), 'C1': (130.3, 121.8, 90),   # top: ISET, SYSOFF, IN
    'R4': (131.0, 124.6, 0), 'C2': (131.2, 127.0, 0),                              # right: ILIM, OUT
    'R5': (123.0, 124.6, 180), 'C3': (122.8, 126.9, 180),                          # left: TS, BAT
    # battery: protection FET, divider; LiPo connector on the bottom side between the XIAO rows
    'Q1': (113.0, 127.0, 0), 'R10': (107.6, 121.4, 0), 'R11': (107.6, 124.0, 0), 'C4': (107.6, 126.6, 0),
    'J2': (158.0, 106.0, 180, 'B'),
    # power switch on the bottom edge next to the USB-C (one rubber flap covers both); D3 at the XIAO 5V pad
    'SW1': (161.5, 137.8, 0), 'D3': (166.2, 105.6, 90),
    'SW2': (152.0, 128.6, 0), 'C9': (152.0, 124.6, 0), 'D5': (159.4, 128.6, 0), 'R14': (159.4, 125.0, 0),
    # motor driver under the OLED, motor wire pads in the strip between OLED and XIAO
    'Q2': (113.0, 113.0, 0), 'R12': (108.6, 113.0, 90), 'R13': (110.6, 116.6, 0), 'D4': (118.0, 112.6, 90),
    'C7': (113.6, 108.6, 0), 'C8': (118.0, 108.4, 0), 'J3': (143.3, 118.0, 0),
}


def load_fp(fpid):
    lib, name = fpid.split(':')
    path = f'{PRJ}/hub.pretty' if lib == 'hub' else f'{KF}/{lib}.pretty'
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        raise RuntimeError(f'footprint not found: {fpid}')
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def flip(fp):
    try:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    except (AttributeError, TypeError):
        fp.Flip(fp.GetPosition(), False)


def text(board, s, x, y, size=1.0, layer=None, mirrored=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetTextSize(V(size, size))
    t.SetTextThickness(mm(size * 0.15))
    t.SetPosition(V(x, y))
    t.SetLayer(layer if layer is not None else pcbnew.F_SilkS)
    t.SetMirrored(mirrored)
    board.Add(t)


def build():
    write_footprints()
    syms = read_schematic()
    pinnet = read_netlist()
    board = pcbnew.NewBoard(PCB)
    ds = board.GetDesignSettings()
    dnc = ds.m_NetSettings.m_DefaultNetClass if hasattr(ds.m_NetSettings, 'm_DefaultNetClass') \
        else ds.m_NetSettings.GetDefaultNetclass()
    dnc.SetClearance(mm(0.15))
    dnc.SetTrackWidth(mm(0.3))
    dnc.SetViaDiameter(mm(0.6))
    dnc.SetViaDrill(mm(0.3))
    ds.m_MinClearance = mm(0.15)
    ds.m_CopperEdgeClearance = mm(0.3)
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
        fpid = info['fp']
        if ref == 'J4':
            fpid = 'hub:OLED_SH1106_13_HDR'
        fp = load_fp(fpid)
        board.Add(fp)
        fp.SetReference(ref)
        fp.SetValue(info['value'])
        x, y, rot = PLACE[ref][:3]
        fp.SetPosition(V(x, y))
        if len(PLACE[ref]) > 3 and PLACE[ref][3] == 'B':
            flip(fp)
        fp.SetOrientationDegrees(rot)
        fp.SetPath(pcbnew.KIID_PATH('/' + info['uuid']))
        rt = fp.Reference()
        rt.SetTextSize(V(0.7, 0.7))
        rt.SetTextThickness(mm(0.1))
        rt.SetLayer(pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab)
        for pad in fp.Pads():
            key = (ref, pad.GetNumber())
            if key in pinnet:
                pad.SetNet(nets[pinnet[key]])
    if missing:
        print('NOT PLACED:', missing)
    for i, (hx, hy) in enumerate(HOLES):
        h = pcbnew.FootprintLoad(f'{KF}/MountingHole.pretty', 'MountingHole_2.2mm_M2')
        h.SetFPID(pcbnew.LIB_ID('MountingHole', 'MountingHole_2.2mm_M2'))
        h.SetReference(f'H{i + 1}')
        h.SetPosition(V(hx, hy))
        try:
            h.SetBoardOnly(True)
            h.SetExcludedFromBOM(True)
        except AttributeError:
            pass
        board.Add(h)
    # rounded-rectangle outline, r = 2 mm
    r = 2.0
    x1, y1, x2, y2 = BX, BY, BX + BW, BY + BH
    for (a, b) in (((x1 + r, y1), (x2 - r, y1)), ((x2, y1 + r), (x2, y2 - r)),
                   ((x2 - r, y2), (x1 + r, y2)), ((x1, y2 - r), (x1, y1 + r))):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(V(*a))
        s.SetEnd(V(*b))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        board.Add(s)
    k = r * (1 - 0.7071068)
    for (sx, sy, mx, my, ex, ey) in ((x1, y1 + r, x1 + k, y1 + k, x1 + r, y1),
                                     (x2 - r, y1, x2 - k, y1 + k, x2, y1 + r),
                                     (x2, y2 - r, x2 - k, y2 - k, x2 - r, y2),
                                     (x1 + r, y2, x1 + k, y2 - k, x1, y2 - r)):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(V(sx, sy), V(mx, my), V(ex, ey))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        board.Add(s)
    # 0.45 mm track/via keep-out strips along the edges, so Freerouting keeps the 0.3 mm edge clearance
    e = 0.45
    for (ax, ay, bx, by) in ((x1, y1, x2, y1 + e), (x1, y2 - e, x2, y2), (x1, y1, x1 + e, y2), (x2 - e, y1, x2, y2)):
        ring = pcbnew.ZONE(board)
        ring.SetIsRuleArea(True)
        ring.SetDoNotAllowTracks(True)
        ring.SetDoNotAllowVias(True)
        ring.SetDoNotAllowPads(False)
        ring.SetDoNotAllowFootprints(False)
        ring.SetDoNotAllowCopperPour(False)
        ls = pcbnew.LSET()
        ls.AddLayer(pcbnew.F_Cu)
        ls.AddLayer(pcbnew.B_Cu)
        ring.SetLayerSet(ls)
        ol = ring.Outline()
        ol.NewOutline()
        for (x, y) in ((ax, ay), (bx, ay), (bx, by), (ax, by)):
            ol.Append(mm(x), mm(y))
        board.Add(ring)
    # OLED module outline for assembly (F.Fab) and silk text
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_RECT)
    s.SetStart(V(OLED[0], OLED[1]))
    s.SetEnd(V(OLED[2], OLED[3]))
    s.SetLayer(pcbnew.F_Fab)
    s.SetWidth(mm(0.1))
    board.Add(s)
    text(board, 'OLED 1.3" above, parts < 1.3 mm', 121.7, 120.0, 0.8, pcbnew.F_Fab)
    text(board, 'BITE HUB rev A', 121.0, 118.0, 1.2, pcbnew.B_SilkS, True)
    text(board, 'LiPo 1+ 2-', 156.0, 112.0, 0.8, pcbnew.B_SilkS, True)
    text(board, 'ON', 166.6, 135.4, 0.8)
    pcbnew.SaveBoard(PCB, board)
    print('saved', PCB, 'footprints:', len(board.GetFootprints()), 'nets:', len(nets))


POWER_HI = ['/VBUS_IN', '/VBUS_5V', '/VBAT', '/BT_P', '/SYS', '/SYS_SW', '/5V_XIAO']
POWER = ['/3V3', 'GND', '/MOT_N']


def export_dsn():
    board = pcbnew.LoadBoard(PCB)
    dsn = f'{PRJ}/_route/hub.dsn'
    os.makedirs(os.path.dirname(dsn), exist_ok=True)
    ok = pcbnew.ExportSpecctraDSN(board, dsn)
    print('dsn export', ok)
    txt = open(dsn).read()
    # one net class for everything, 0.3 mm: wide enough for the 1 A charge path, narrow enough
    # to leave the 0.5 mm-pitch BQ24074 pads (separate power classes made Freerouting drop nets)
    txt = re.sub(r'\(width 250\)', '(width 300)', txt)
    open(dsn, 'w').write(txt)
    print('track width 0.3 mm')


def import_ses_tracks(board, path):
    """Minimal Specctra SES reader (KiCad 7's python cannot import SES without the editor).
    Duplicate segments and dead-end stubs that Freerouting leaves are dropped before tracks are created."""
    pads = [(p, p.GetNetname()) for f in board.GetFootprints() for p in f.Pads()]
    if len(board.GetTracks()):
        raise SystemExit('board already has tracks: run the "place" stage first')
    ses = parse(open(path).read())[0]
    routes = find(ses, 'routes')
    res = find(routes, 'resolution')
    scale = {'um': 1e-3, 'mm': 1.0, 'mil': 0.0254}[res[1]] / float(res[2])   # DSN units -> mm
    layers = {'F.Cu': pcbnew.F_Cu, 'B.Cu': pcbnew.B_Cu}
    segs, vias = {}, []
    for net in findall(find(routes, 'network_out'), 'net'):
        name = str(net[1])
        for w in findall(net, 'wire'):
            pth = find(w, 'path')
            layer, width = pth[1], float(pth[2]) * scale
            pts = [(round(float(pth[i]) * scale, 4), round(-float(pth[i + 1]) * scale, 4)) for i in range(3, len(pth) - 1, 2)]
            for a, b in zip(pts, pts[1:]):
                if a != b:
                    segs[(layer, name, min(a, b), max(a, b))] = width
        for v in findall(net, 'via'):
            m = re.search(r'_(\d+):(\d+)_um', str(v[1]))
            vias.append((name, round(float(v[2]) * scale, 4), round(-float(v[3]) * scale, 4),
                         int(m.group(1)) / 1000 if m else 0.6, int(m.group(2)) / 1000 if m else 0.3))
    layer_id = {'F.Cu': pcbnew.F_Cu, 'B.Cu': pcbnew.B_Cu}

    def anchored(layer, name, pt, key):
        for (l2, n2, a, b) in segs:
            if (l2, n2, a, b) != key and l2 == layer and n2 == name and pt in (a, b):
                return True
        if any(n == name and abs(x - pt[0]) < 0.15 and abs(y - pt[1]) < 0.15 for (n, x, y, _, _) in vias):
            return True
        return any(n == name and p.IsOnLayer(layer_id[layer]) and p.HitTest(V(*pt)) for (p, n) in pads)

    removed = True
    while removed:
        removed = False
        for key in list(segs):
            layer, name, a, b = key
            if not (anchored(layer, name, a, key) and anchored(layer, name, b, key)):
                del segs[key]
                removed = True
    for (layer, name, a, b), width in segs.items():
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(V(*a))
        t.SetEnd(V(*b))
        t.SetWidth(mm(max(width, 0.2)))
        t.SetLayer(layers[layer])
        t.SetNet(board.FindNet(name))
        board.Add(t)
    for (name, x, y, d, drill) in vias:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(V(x, y))
        via.SetWidth(mm(d))
        via.SetDrill(mm(drill))
        via.SetNet(board.FindNet(name))
        board.Add(via)
    return len(segs), len(vias)


def import_ses():
    board = pcbnew.LoadBoard(PCB)
    n_t, n_v = import_ses_tracks(board, f'{PRJ}/_route/hub.ses')
    print('ses import: tracks', n_t, 'vias', n_v)
    gnd = board.FindNet('GND')
    for ref in ('U2', 'J3', 'J4'):
        f = board.FindFootprintByReference(ref)
        if f:
            f.SetZoneConnection(pcbnew.ZONE_CONNECTION_THERMAL) if hasattr(f, 'SetZoneConnection') \
                else f.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_THERMAL)
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


def drc():
    board = pcbnew.LoadBoard(PCB)
    out = f'{PRJ}/docs/drc_report.txt'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    ok = pcbnew.WriteDRCReport(board, out, pcbnew.EDA_UNITS_MILLIMETRES, True)
    print('drc report', ok, out)


{'place': build, 'dsn': export_dsn, 'ses': import_ses, 'drc': drc}[STAGE]()
