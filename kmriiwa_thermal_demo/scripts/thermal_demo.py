#!/usr/bin/env python3
from pathlib import Path
import sys

source = Path(__file__).resolve().parents[1] / 'src'
if source.is_dir():
    sys.path.insert(0, str(source))

from kmriiwa_thermal_demo.cli import main

if __name__ == '__main__':
    raise SystemExit(main())
