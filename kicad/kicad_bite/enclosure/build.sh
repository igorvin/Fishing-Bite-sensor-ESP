#!/bin/zsh
# Пересборка STL корпуса v2 + проверки посадки. Запуск: zsh build.sh
cd "$(dirname "$0")"
O() { openscad --backend=manifold "$@"; }
mkdir -p stl img
rm -f stl/tub.stl stl/lid_print.stl
for p in body_print cap_print pusher_print retainer_print; do
  O -D "part=\"$p\"" -o "stl/${p%_print}.stl" bite_enclosure.scad 2>&1 | grep -E "ERROR" ; echo "stl/${p%_print}.stl"
done
for p in clash_pusher clash_ret clash_slide_pusher; do
  r=$(O -D "part=\"$p\"" -o "${TMPDIR:-/tmp}/_c.stl" bite_enclosure.scad 2>&1)
  echo "$r" | grep -q "empty" && echo "$p: OK (пересечений нет)" || echo "$p: ЕСТЬ ПЕРЕСЕЧЕНИЕ"
done
