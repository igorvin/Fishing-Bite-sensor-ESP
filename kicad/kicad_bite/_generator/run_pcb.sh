#!/bin/zsh
# Full pipeline: schematic -> ERC -> placement -> DSN -> Freerouting -> SES + zones -> DRC -> fab/docs.
# Run from anywhere:  zsh kicad_bite/_generator/run_pcb.sh
set -e
S="$(cd "$(dirname "$0")" && pwd)"
PRJ="$(dirname "$S")"
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
JAVA=$(brew --prefix openjdk)/bin/java
JAR="$PRJ/../kicad_v2/_tools/freerouting/freerouting-2.5.0.jar"
T="${TMPDIR:-/tmp}"
cd "$PRJ"
Q() { grep -vE "Fontconfig|^$" || true; }

# 1. schematic
python3 $S/gen_bite.py "$PRJ"
$KC sch upgrade bite_sensor.kicad_sch 2>&1 | Q >/dev/null
$KC sch export pdf -o bite_sensor_schematic.pdf bite_sensor.kicad_sch >/dev/null 2>&1

# 2. board
$KPY $S/pcb_bite.py "$PRJ" place 2>&1 | grep -E "saved|NOT|rror" || true
$KC sch erc -o docs/erc_report.txt bite_sensor.kicad_sch 2>&1 | grep -i violations || true
$KPY $S/pcb_bite.py "$PRJ" dsn 2>&1 | grep -E "dsn export" || true
rm -f _route/bite_sensor.ses
(cd _route && $JAVA -Djava.awt.headless=true -jar "$JAR" --gui.enabled=false -de bite_sensor.dsn -do bite_sensor.ses -mp 40 -mt 0 > fr.log 2>&1)
grep -E "Auto-routing stage completed" _route/fr.log | sed 's/.*completed:/routing:/' | cut -c1-140
$KPY $S/pcb_bite.py "$PRJ" ses 2>&1 | grep -E "ses import|zones" || true
$KC pcb drc --schematic-parity -o docs/drc_report.txt bite_sensor.kicad_pcb 2>&1 | grep -iE "Found" || true

# 3. fabrication + docs
rm -rf fab/gerber fab/bite_sensor_gerbers.zip; mkdir -p fab/gerber
$KC pcb export gerbers -l F.Cu,B.Cu,F.Mask,B.Mask,F.Silkscreen,B.Silkscreen,F.Paste,B.Paste,Edge.Cuts -o fab/gerber/ bite_sensor.kicad_pcb >/dev/null 2>&1
$KC pcb export drill -o fab/gerber/ bite_sensor.kicad_pcb >/dev/null 2>&1
(cd fab/gerber && zip -q ../bite_sensor_gerbers.zip *)
$KC pcb export pos --side front --format csv --units mm -o fab/bite_sensor-pos.csv bite_sensor.kicad_pcb >/dev/null 2>&1
$KC sch export bom -o fab/bite_sensor-bom.csv --fields 'Reference,Value,Footprint,${QUANTITY}' --group-by 'Value,Footprint' bite_sensor.kicad_sch >/dev/null 2>&1
$KC pcb export pdf --mode-multipage -l F.Cu,B.Cu,F.Silkscreen,F.Fab --cl Edge.Cuts --scale 0 -o docs/bite_sensor_pcb_layers.pdf bite_sensor.kicad_pcb >/dev/null 2>&1
$KC pcb render -o docs/bite_sensor_3d_top.png -w 1600 -h 700 --side top --zoom 1.8 --quality high --background opaque bite_sensor.kicad_pcb >/dev/null 2>&1
$KC pcb render -o docs/bite_sensor_3d_bottom.png -w 1600 -h 700 --side bottom --zoom 1.8 --quality high --background opaque bite_sensor.kicad_pcb >/dev/null 2>&1
$KC pcb render -o docs/bite_sensor_3d_iso.png -w 1600 -h 900 --side top --rotate '-45,0,30' --zoom 1.4 --quality high --background opaque bite_sensor.kicad_pcb >/dev/null 2>&1
# board model for the enclosure fit check (origin = board centre)
$KC pcb export stl -f --user-origin 135x112mm -o enclosure/_fit/pcba.stl bite_sensor.kicad_pcb >/dev/null 2>&1
echo "done"
