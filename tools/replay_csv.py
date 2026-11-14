#!/usr/bin/env python3
"""Replay timestamped XYZ through the exact portable firmware core.

This is an offline experiment with explicit fixed configuration, not a claim of
bit-for-bit reconstruction of UI changes or hardware/radio timing in the capture.
"""
import argparse, os, pathlib, subprocess, tempfile
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('csv',type=pathlib.Path)
parser.add_argument('--mode',choices=['dynamic','legacy'],default='dynamic')
parser.add_argument('--threshold',type=float,help='g; default dynamic=.02, legacy=.15')
parser.add_argument('--arm-delay',type=int,default=3000,help='ms from first sample; includes 2s quiet calibration, then 1.5s settling')
args=parser.parse_args()
if args.arm_delay<0:parser.error('arm-delay must be nonnegative')
threshold=args.threshold if args.threshold is not None else (.02 if args.mode=='dynamic' else .15)
root=pathlib.Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='bite-replay-') as tmp:
    exe=pathlib.Path(tmp)/'replay'
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror','-I'+str(root/'include'),str(root/'tools/replay.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe),str(args.csv),args.mode,str(threshold),str(args.arm_delay)],check=True)
