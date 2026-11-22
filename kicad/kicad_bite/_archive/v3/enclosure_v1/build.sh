#!/bin/sh
# Пересобрать STL и проверить посадку платы/ячейки после правки параметров
cd "$(dirname "$0")"
for p in tub lid_print; do openscad --backend=manifold -D "part=\"$p\"" -o "stl/$p.stl" bite_enclosure.scad; done
for p in clash_lid clash_lid_cell; do
  openscad --backend=manifold -D "part=\"$p\"" -o "_fit/$p.stl" bite_enclosure.scad 2>&1 | grep -q "empty" && echo "$p: OK (пересечений нет)" || echo "$p: ЕСТЬ ПЕРЕСЕЧЕНИЕ -> _fit/$p.stl"
done
