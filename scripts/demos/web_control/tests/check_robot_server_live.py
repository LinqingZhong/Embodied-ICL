"""Opt-in real simulation check: moves Cobot front-arm joint1 by 0.04 rad and back."""

import argparse
import io
import sys
from pathlib import Path

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_client import RobotClient, RobotError

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--url", default="http://127.0.0.1:8083")
args = parser.parse_args()
r = RobotClient(args.url)
o = r.get_observation()
assert {"scene", "head", "fl_wrist", "fr_wrist"} <= o.images.keys()
print("CAMERAS", {k: Image.open(io.BytesIO(v)).size for k, v in o.images.items()})
s = r.get_joint_state()
names = ["fl_joint1", "fr_joint1"]
ids = [s["joint_names"].index(n) for n in names]
q = [s["positions"][i] for i in ids]
rows = [[q[0] + 0.04 * i / 10, q[1] - 0.04 * i / 10] for i in range(11)]
t = r.execute_joint_chunk(names, rows, 12)
a = r.wait(t.id, timeout=60)
print("RESULT", a.details)
assert a.status == "completed" and a.details["executed_steps"] == 132
assert all(abs(x) < 0.08 for x in a.details["errors"])
t = r.execute_joint_chunk(names, [q], 24)
assert r.wait(t.id).status == "completed"
try:
    r._request("/api/command", {"action": "joint", "joint": 0, "value": 0})
except RobotError as e:
    assert e.status == 400
else:
    raise AssertionError("Observe mode accepted manual movement")
print("LIVE_PASS")
