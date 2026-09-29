"""Read observations and submit a small client-generated joint trajectory."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_client import RobotClient

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--url", default="http://127.0.0.1:8080")
parser.add_argument("--joint", help="Exact controllable joint name; omitted means read-only")
parser.add_argument("--delta", type=float, default=0.05, help="rad for revolute joints; m for prismatic joints")
parser.add_argument("--output", default="output/robot_observation")
args = parser.parse_args()
robot = RobotClient(args.url)
obs = robot.get_observation()
output = Path(args.output)
output.mkdir(parents=True, exist_ok=True)
for name, data in obs.images.items():
    # Use an index for filenames rather than treating server-provided camera names as paths.
    index = list(obs.images).index(name)
    (output / f"camera_{index}.jpg").write_bytes(data)
    print(f"camera_{index}.jpg: {name}")
state = robot.get_joint_state()
print("Simulation time:", state["sim_time"])
print("Controllable joints:", state["controllable_joint_names"])
if args.joint:
    if args.joint not in state["controllable_joint_names"]:
        parser.error("Choose a controllable joint from the printed list")
    index = state["joint_names"].index(args.joint)
    start = state["positions"][index]
    # Interpolation is explicitly a CALLER policy; RobotServer executes these rows unchanged.
    rows = [[start + args.delta * i / 20] for i in range(21)]
    task = robot.execute_joint_chunk([args.joint], rows, steps_per_action=12)
    print("Submitted:", task.id)
    result = robot.wait(task.id, timeout=120)
    print(result.details)
