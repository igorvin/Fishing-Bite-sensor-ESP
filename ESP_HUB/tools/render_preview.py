#!/usr/bin/env python3
"""Generate actual128x64 production-renderer frames and nearest-neighbour PNGs.

No ESP, Adafruit library emulation, system fonts or Python packages are needed.
Output defaults to ignored .pio/hub-preview. All radio/name values are fixtures.
"""
from pathlib import Path
import argparse
import os
import struct
import subprocess
import tempfile
import zlib
root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=root / '.pio/hub-preview')
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix='hub-render-') as tmp:
    binary = Path(tmp) / 'render'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-I', str(root/'include'), str(root/'tools/render_preview.cpp'), '-o', str(binary)], check=True)
    subprocess.run([str(binary), str(args.output)], check=True)
def png(path, width, height, pixels):
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind+data))
    rows = b''.join(b'\0'+bytes(pixels[y*width:(y+1)*width]) for y in range(height))
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB',width,height,8,0,0,0,0)) + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND',b''))
frames=[]
for file in sorted(args.output.glob('*.pbm')):
    raw = file.read_bytes().split(b'\n',2)[2]
    assert len(raw)==1024
    pixels=[0 if raw[y*16+x//8]&(0x80>>(x%8)) else 255 for y in range(64) for x in range(128)]
    png(file.with_suffix('.png'),128,64,pixels)
    large=[pixels[(y//4)*128+x//4] for y in range(256) for x in range(512)]
    png(file.with_name(file.stem+'-4x.png'),512,256,large)
    frames.append((file.stem,pixels))
# Contact sheet uses source pixels only; filenames are listed in terminal/docs.
width=512; height=256*len(frames)+16*(len(frames)-1)
sheet=[24]*(width*height)
for i,(_,p) in enumerate(frames):
    offset=i*272
    for y in range(256):
        sheet[(offset+y)*width:(offset+y+1)*width]=[p[(y//4)*128+x//4] for x in range(width)]
png(args.output/'contact-sheet.png',width,height,sheet)
print('Actual renderer frames:', ', '.join(name for name,_ in frames))
print(args.output/'contact-sheet.png')
