#!/usr/bin/env python3
"""Record the sensor's full-rate CSV to a file, with Enter-key marks for simulated bites.

Sends "csv on" to the sensor, keeps only the CSV rows in --out, prints the
sensor's own messages on screen, and sends "csv off" at the end (time limit
or Ctrl+C). Press Enter during the recording to mark a moment (for example a
simulated bite); marks go to <out>.marks.csv with the sensor time in ms.

Needs pyserial, which ships with PlatformIO:
  ~/.platformio/penv/bin/python tools/capture_csv.py --port /dev/cu.usbmodemXXXX \
      --seconds 120 --out tests/fixtures/field/wind.csv
"""
import argparse
import os
import select
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing: run this with PlatformIO's Python, "
             "~/.platformio/penv/bin/python tools/capture_csv.py ...")

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument('--port', required=True, help='serial port of the SENSOR, e.g. /dev/cu.usbmodem1142301')
parser.add_argument('--seconds', type=float, default=120, help='recording length (default 120 s)')
parser.add_argument('--out', required=True, help='CSV file to write')
parser.add_argument('--baud', type=int, default=115200)
args = parser.parse_args()

out_dir = os.path.dirname(os.path.abspath(args.out))
os.makedirs(out_dir, exist_ok=True)
marks_path = os.path.splitext(args.out)[0] + '.marks.csv'

# The ESP32 USB serial resets the chip when RTS is asserted while DTR is released.
# pyserial applies DTR before RTS on open, and the OS asserts both, so releasing DTR
# first (the 2026-11-24 recordings) produced exactly that state and rebooted the
# sensor. Keep DTR asserted while RTS is released, then release DTR.
port = serial.Serial()
port.port, port.baudrate, port.timeout = args.port, args.baud, 0.05
port.dtr = True
port.rts = False
port.open()
try:
    port.dtr = False
except OSError:
    pass  # ports without modem lines (e.g. virtual) ignore DTR

rows = 0
marks = 0
last_ms = None
last_state = '?'
buffer = b''
interactive = sys.stdin.isatty()


def is_csv(line):
    return line.startswith('ms,') or (line[:1].isdigit() and line.count(',') >= 10)


with open(args.out, 'w', encoding='utf-8') as out, open(marks_path, 'w', encoding='utf-8') as mark_file:
    mark_file.write('mark,sensor_ms,row,wall_time\n')
    port.reset_input_buffer()
    port.write(b'csv on\n')
    print(f'Recording {args.seconds:.0f} s to {args.out}.')
    print('>>> Press Enter in THIS window at each simulated bite (a "mark N" line confirms it). Ctrl+C stops early.')
    if not interactive:
        print('(stdin is not a terminal: marks are disabled)')
    started = time.monotonic()
    shown = started
    try:
        while time.monotonic() - started < args.seconds:
            chunk = port.read(4096)
            if chunk:
                buffer += chunk
                *lines, buffer = buffer.split(b'\n')
                for raw in lines:
                    line = raw.decode('utf-8', 'replace').strip('\r')
                    if is_csv(line):
                        if line.startswith('ms,') and rows:
                            continue
                        out.write(line + '\n')
                        if not line.startswith('ms,'):
                            rows += 1
                            fields = line.split(',')
                            last_ms, last_state = fields[0], fields[8]
                    elif line:
                        print(line)
            if interactive and select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                marks += 1
                mark_file.write(f'{marks},{last_ms},{rows},{time.strftime("%H:%M:%S")}\n')
                mark_file.flush()
                print(f'  mark {marks} at sensor {last_ms} ms')
            now = time.monotonic()
            if now - shown >= 5:
                shown = now
                print(f'  {now - started:5.0f} s  rows {rows}  state {last_state}  marks {marks}')
    except KeyboardInterrupt:
        print('\nStopped early.')
    finally:
        port.write(b'csv off\n')
        time.sleep(0.3)
        port.close()

print(f'Saved {rows} rows to {args.out}' + (f' and {marks} marks to {marks_path}' if marks else ''))
if rows == 0:
    print('No CSV rows arrived: check the port is the SENSOR (not the hub) and the firmware has the "csv on" command.')
