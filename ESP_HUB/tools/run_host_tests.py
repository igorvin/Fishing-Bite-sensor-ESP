#!/usr/bin/env python3
"""Compile and run the production hub-core/renderer regressions, no packages."""
from pathlib import Path
import os
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="fishing-hub-tests-") as tmp:
    binary = Path(tmp) / "hub-tests"
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-misleading-indentation", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-I", str(root / "include"), str(root / "tests/test_hub.cpp"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
