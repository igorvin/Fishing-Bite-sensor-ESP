#!/usr/bin/env python3
"""Package the verified local bench builds; never flash or publish."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parent.parent
parser = argparse.ArgumentParser()
parser.add_argument('--build-dir', type=Path, default=root / '.pio' / 'build')
args = parser.parse_args()
build_root = args.build_dir.resolve()
commit = subprocess.check_output(['git', 'rev-parse', '--short=12', 'HEAD'], cwd=root, text=True).strip()
out = root / '.pio' / 'releases' / ('sensor-bench-' + commit)
out.mkdir(parents=True, exist_ok=True)
manifest = {'commit': commit, 'platform': 'platformio/espressif32@8.0.0', 'arduino_esp32': '3.3.12', 'hardware_tested': False, 'field_tested': False, 'build_config': 'platformio.ini', 'builds': {}}
for board in ['xiao_esp32c3', 'seeed_xiao_esp32s3']:
    src = build_root / board
    if not (src / 'firmware.bin').is_file():
        raise SystemExit('Missing verified firmware for ' + board)
    if (src / 'firmware.bin').read_bytes()[:1] != b'\xe9':
        raise SystemExit('Not an Espressif image: ' + board)
    dst = out / board
    dst.mkdir(exist_ok=True)
    records = []
    for name in ['firmware.bin', 'firmware.elf', 'bootloader.bin', 'partitions.bin', 'boot_app0.bin', 'firmware.factory.bin', 'flasher_args.json']:
        file = src / name
        if file.is_file():
            target = dst / name
            shutil.copy2(file, target)
            records.append({'file': name, 'bytes': target.stat().st_size, 'sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
    manifest['builds'][board] = records
(root / 'docs' / 'bench-testing.md').is_file() or (_ for _ in ()).throw(SystemExit('Missing bench instructions'))
shutil.copy2(root / 'docs' / 'bench-testing.md', out / 'BENCH-TESTING.md')
shutil.copy2(root / 'platformio.ini', out / 'platformio.ini')
(out / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
checksums = []
for board, records in manifest['builds'].items():
    checksums.extend(item['sha256'] + '  ' + board + '/' + item['file'] for item in records)
(out / 'SHA256SUMS').write_text('\n'.join(checksums) + '\n')
(out / 'README.txt').write_text('Sensor bench release\nCommit: ' + commit + '\n\nC3: current photographed XIAO ESP32-C3 + LSM6DS3 bench.\nS3: planned Seeed XIAO ESP32-S3 sensor. Do not mix board images.\n\nNo device was flashed; no hardware or fishing effectiveness is claimed.\nUse the source project PlatformIO upload instructions in BENCH-TESTING.md for a first flash.\nAn application-only firmware.bin is not a complete first-flash image.\nPreserve config; do not erase flash or upload a filesystem without a specific reason.\n\nThe current hub keeps the original 22-byte BitePacket protocol.\n')
print(out)
