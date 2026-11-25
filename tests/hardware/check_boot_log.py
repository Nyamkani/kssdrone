#!/usr/bin/env python3
"""Offline smoke check for a manually captured, motor-disabled bench boot log."""
import argparse
import re
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("log", type=Path)
parser.add_argument("--min-loop-reports", type=int, default=3)
args = parser.parse_args()
data = args.log.read_text(errors="replace")
required = ["Board initialization successful", "KSS_DRONE FW Version"]
missing = [x for x in required if x not in data]
bad = re.findall(r"(?im)^.*(?:Guru Meditation|assert failed|watchdog got triggered|Board initialization failed|Drone initialization failed).*$", data)
loops = re.findall(r"KSS loop state=(\d+) hz=(\d+)", data)
print(f"Boot markers missing: {missing}; crash markers: {len(bad)}; loop reports: {len(loops)}")
if missing or bad or len(loops) < args.min_loop_reports:
    raise SystemExit(1)
print("PASS: boot-log smoke check only; not HIL or flight validation")
