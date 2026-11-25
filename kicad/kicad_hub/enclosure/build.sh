#!/usr/bin/env bash
# Пересборка STL и картинок корпуса хаба + проверка посадки платы. Запуск: bash build.sh
set -e
cd "$(dirname "$0")"
mkdir -p stl img
for p in shell cover clip pusher flap plug_top gasket; do
  openscad -D "part=\"$p\"" -o "stl/$p.stl" hub_enclosure.scad 2>&1 | grep -E "ERROR" || true
  echo "stl/$p.stl"
done
r=$(openscad -D 'part="clash"' -o "${TMPDIR:-/tmp}/_hub_clash.stl" hub_enclosure.scad 2>&1 || true)
python3 - "${TMPDIR:-/tmp}/_hub_clash.stl" <<'PY'
import re, sys
try:
    z = [float(m) for m in re.findall(r'vertex\s+\S+\s+\S+\s+(\S+)', open(sys.argv[1]).read())]
except FileNotFoundError:
    z = []
print('clash: OK (оболочка касается платы только плоскостями бобышек)' if not z or max(z) - min(z) < 1e-6
      else f'clash: ЕСТЬ ПЕРЕСЕЧЕНИЕ, z {min(z):.2f}..{max(z):.2f}')
PY
X() { xvfb-run -a openscad --colorscheme=Tomorrow --imgsize=1400,1000 "$@" hub_enclosure.scad 2>&1 | grep -E "ERROR" || true; }
X -D 'part="assembly"' --camera=34,20,12,55,0,200,190 -o img/assembly.png
X -D 'part="exploded"' --camera=34,20,20,60,0,210,260 -o img/exploded.png
X -D 'part="back"' --camera=34,20,0,125,0,25,190 -o img/back.png
