"""Print a grouped BOM (CSV) from hub.kicad_sch. Usage: python3 bom.py <project dir>"""
import csv, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'kicad_bite', '_generator'))
from sexp import parse, find, findall  # noqa: E402

sch = parse(open(os.path.join(sys.argv[1], 'hub.kicad_sch')).read())[0]
groups = {}
for s in findall(sch, 'symbol'):
    if not find(s, 'lib_id'):
        continue
    p = {x[1]: x[2] for x in findall(s, 'property')}
    if p.get('Reference', '#').startswith('#'):
        continue
    key = (p.get('Value', ''), p.get('Footprint', '').split(':')[-1], p.get('MPN', ''))
    groups.setdefault(key, []).append(p['Reference'])


def order(ref):
    head = ''.join(c for c in ref if c.isalpha())
    return head, int(''.join(c for c in ref if c.isdigit()) or 0)


w = csv.writer(sys.stdout)
w.writerow(['Qty', 'References', 'Value', 'Footprint', 'MPN / note'])
for (val, fp, mpn), refs in sorted(groups.items(), key=lambda kv: order(sorted(kv[1], key=order)[0])):
    refs.sort(key=order)
    w.writerow([len(refs), ' '.join(refs), val, fp, mpn])
