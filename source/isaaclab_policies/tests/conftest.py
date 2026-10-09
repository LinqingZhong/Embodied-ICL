import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "source/isaaclab_policies"))
sys.path.insert(0, str(ROOT / "scripts/demos/web_control"))
