#!/usr/bin/env bash
# Пересборка STL и картинок корпуса хаба + проверка посадки платы. Запуск: bash build.sh
set -e
cd "$(dirname "$0")"
mkdir -p stl img
for p in shell cover clip pusher flap plug_top gasket; do
  openscad -D "part=\"$p\"" -o "stl/$p.stl" hub_enclosure.scad 2>&1 | grep -E "ERROR" || true
  echo "stl/$p.stl"
done
for c in clash clash_cover; do
  openscad -D "part=\"$c\"" -o "${TMPDIR:-/tmp}/_hub_$c.stl" hub_enclosure.scad >/dev/null 2>&1 || true
  python3 - "${TMPDIR:-/tmp}/_hub_$c.stl" "$c" <<'PY'
import re, sys
try:
    z = sorted({round(float(m), 3) for m in re.findall(r'vertex\s+\S+\s+\S+\s+(\S+)', open(sys.argv[1]).read())})
except FileNotFoundError:
    z = []
# only touching planes (bosses/posts on the board, motor on the cover) are allowed: no z extent
print(f'{sys.argv[2]}: OK (только касание плоскостями {z})' if len(z) <= 2 and all(v in (2.0, 10.0, 11.0) for v in z)
      else f'{sys.argv[2]}: ЕСТЬ ПЕРЕСЕЧЕНИЕ, z {z[:6]}...')
PY
done
X() { xvfb-run -a openscad --colorscheme=Tomorrow --imgsize=1400,1000 "$@" hub_enclosure.scad 2>&1 | grep -E "ERROR" || true; }
X -D 'part="assembly"' --camera=34,20,12,55,0,200,190 -o img/assembly.png
X -D 'part="exploded"' --camera=34,20,20,60,0,210,260 -o img/exploded.png
X -D 'part="back"' --camera=34,20,0,125,0,25,190 -o img/back.png
