import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]  # ductsim直下
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
