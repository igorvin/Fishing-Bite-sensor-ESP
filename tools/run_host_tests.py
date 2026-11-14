#!/usr/bin/env python3
"""Compile the same portable core used by firmware, then run regression tests."""
import pathlib, subprocess, tempfile, os
root=pathlib.Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='bite-tests-') as tmp:
    exe=pathlib.Path(tmp)/'core-tests'
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror','-I'+str(root/'include'),str(root/'tests/test_core.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
