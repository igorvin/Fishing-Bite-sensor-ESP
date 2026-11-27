#!/usr/bin/env bash
# Hub board pipeline: schematic -> netlist -> placement -> DSN -> Freerouting -> SES + zones -> DRC -> fab files.
# Linux (KiCad 7 from Ubuntu):  bash kicad/kicad_hub/_generator/run_hub.sh
# macOS (KiCad 8/9 app):        KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \
#                               KICAD_CLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli bash .../run_hub.sh
# FREEROUTING=<path to freerouting-2.x.jar> (default: $TMPDIR/freerouting.jar, downloaded if missing).
set -euo pipefail
S="$(cd "$(dirname "$0")" && pwd)"
PRJ="$(dirname "$S")"
KPY="${KPY:-/usr/bin/python3}"
KC="${KICAD_CLI:-kicad-cli}"
export KICAD_CLI="$KC"
if [ -d /usr/share/kicad/footprints ]; then export KICAD7_FOOTPRINT_DIR=/usr/share/kicad/footprints KICAD7_SYMBOL_DIR=/usr/share/kicad/symbols; fi
JAR="${FREEROUTING:-${TMPDIR:-/tmp}/freerouting.jar}"
[ -f "$JAR" ] || curl -sfL -o "$JAR" https://github.com/freerouting/freerouting/releases/download/v2.1.0/freerouting-2.1.0.jar
cd "$PRJ"
Q() { grep -vE "Fontconfig|^$" || true; }

python3 "$S/gen_hub.py" "$PRJ"
"$KC" sch export pdf -o hub_schematic.pdf hub.kicad_sch 2>&1 | Q
# Freerouting is not deterministic: route again (max 5 tries) until DRC finds no unconnected pads
for try in 1 2 3 4 5; do
  "$KPY" "$S/pcb_hub.py" "$PRJ" place 2>&1 | grep -v swig | Q
  "$KPY" "$S/pcb_hub.py" "$PRJ" dsn 2>&1 | grep -v swig | Q
  rm -f _route/hub.ses
  (cd _route && java -Djava.awt.headless=true -jar "$JAR" --gui.enabled=false -de hub.dsn -do hub.ses -mp 60 -mt 1 > fr.log 2>&1)
  grep -oE "Auto-router pass #[0-9]+ .*score of [0-9.]+ \([^)]*\)" _route/fr.log | tail -1 || true
  "$KPY" "$S/pcb_hub.py" "$PRJ" ses 2>&1 | grep -v swig | Q
  "$KPY" "$S/pcb_hub.py" "$PRJ" drc 2>&1 | grep -v swig | Q
  grep -E "Found" docs/drc_report.txt || true
  grep -q "Found 0 unconnected pads" docs/drc_report.txt && break
  echo "route try $try left unconnected pads, routing again"
done

rm -rf fab/gerber fab/hub_gerbers.zip; mkdir -p fab/gerber
"$KC" pcb export gerbers -l F.Cu,B.Cu,F.Mask,B.Mask,F.Silkscreen,B.Silkscreen,F.Paste,B.Paste,Edge.Cuts -o fab/gerber/ hub.kicad_pcb 2>&1 | Q >/dev/null
"$KC" pcb export drill -o fab/gerber/ hub.kicad_pcb 2>&1 | Q >/dev/null
(cd fab/gerber && zip -q ../hub_gerbers.zip ./*)
"$KC" pcb export pos --side both --format csv --units mm -o fab/hub-pos.csv hub.kicad_pcb 2>&1 | Q >/dev/null
"$KPY" "$S/bom.py" "$PRJ" > fab/hub-bom.csv
"$KC" pcb export pdf -l F.Cu,F.Silkscreen,F.Fab,Edge.Cuts -o docs/hub_pcb_top.pdf hub.kicad_pcb 2>&1 | Q >/dev/null
"$KC" pcb export pdf -l B.Cu,B.Silkscreen,B.Fab,Edge.Cuts -m -o docs/hub_pcb_bottom.pdf hub.kicad_pcb 2>&1 | Q >/dev/null
"$KC" pcb export step --subst-models -f -o fab/hub.step hub.kicad_pcb 2>&1 | Q >/dev/null || true
echo done
