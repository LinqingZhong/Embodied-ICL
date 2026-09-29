"""Opt-in live chunk check for every configured robot; moves arm joint1 then returns."""

import argparse
import io
import json
import sys
from pathlib import Path

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot import get_profile
from robot_client import RobotClient, RobotError

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--robot", required=True)
parser.add_argument("--url", default="http://127.0.0.1:8083")
parser.add_argument("--output", required=True)
args = parser.parse_args()
profile = get_profile(args.robot)
r = RobotClient(args.url)
obs = r.get_observation()
expected = {c["name"] for c in profile["cameras"]}
assert set(obs.images) == expected
sizes = {name: Image.open(io.BytesIO(data)).size for name, data in obs.images.items()}
state = r.get_joint_state()
assert len(state["positions"]) == len(state["joint_names"]) == len(state["velocities"])
names = [j["name"] for j in profile["joints"] if j["name"].endswith("joint1")]
assert names, "No joint1 selected"
ids = [state["joint_names"].index(n) for n in names]
original = [state["positions"][i] for i in ids]
goals = []
for i, q in zip(ids, original):
    lo, hi = state["limits"][i]
    delta = 0.04 if q + 0.04 < hi else -0.04
    assert lo < q + delta < hi
    goals.append(q + delta)
# The caller constructs the trajectory. Last row holds long enough to measure tracking.
rows = [[q + (g - q) * step / 10 for q, g in zip(original, goals)] for step in range(11)]
task = r.execute_joint_chunk(names, rows, 12)
result = r.wait(task.id, timeout=90)
assert result.status == "completed"
assert result.details["executed_steps"] == 132
assert abs(result.details["finished_sim_time"] - result.details["submitted_sim_time"] - 1.1) < 0.05
hold = r.execute_joint_chunk(names, [goals], 120)
settled = r.wait(hold.id, timeout=90)
assert settled.status == "completed"
errors = settled.details["errors"]
assert all(abs(e) < 0.02 for e in errors), errors
movement = [actual - q for actual, q in zip(settled.details["actual_positions"], original)]
assert all(move * (goal - q) > 0 and abs(move) > 0.02 for move, goal, q in zip(movement, goals, original)), movement
back = r.execute_joint_chunk(names, [original], 120)
returned = r.wait(back.id, timeout=90)
assert returned.status == "completed"
assert all(abs(e) < 0.02 for e in returned.details["errors"])
try:
    r._request("/api/command", {"action": "joint", "joint": ids[0], "value": original[0]})
except RobotError as exc:
    assert exc.status == 400
else:
    raise AssertionError("Observe mode accepted manual movement")
report = dict(
    robot=args.robot,
    cameras=sizes,
    joint_names=names,
    chunk=result.details,
    settled=settled.details,
    movement=movement,
    returned=returned.details,
    observation_sequence=obs.sequence,
    status="passed",
)
Path(args.output).write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
print(json.dumps(report, ensure_ascii=False), flush=True)
