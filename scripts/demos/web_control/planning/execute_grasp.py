"""Execute a cuRobo plan with physical gripping and success-checked HDF5 collection."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from planning.grasp_profiles import validate_plan
from robot import get_profile
from robot_client import RobotClient

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--plan", required=True)
parser.add_argument("--output", required=True)
parser.add_argument("--url", default="http://127.0.0.1:8083")
args = parser.parse_args()
r = RobotClient(args.url, timeout=90)
plan = json.loads(Path(args.plan).read_text())
assert plan["planner"] == "curobo"
fixture = plan["fixture"]
arm = fixture["arm"]
names = r.get_arms()[arm]["joint_names"]
profile = get_profile(fixture["robot"])
gripper = next(g for g in profile["grippers"] if g["name"] == ("gripper" if fixture["robot"] == "x5" else arm))
fk_validation = validate_plan(plan, r.fk)
state = r.get_joint_state()
initial = state["object_states"]["/World/GraspCube"]
start = plan["segments"][0]["positions"][0]
actual = [state["positions"][state["joint_names"].index(n)] for n in names]
if max(abs(a - b) for a, b in zip(actual, start)) > 0.03:
    raise ValueError("Robot differs from plan start; reset the dedicated scene before executing")
if np.linalg.norm(np.array(initial[:3]) - fixture["cube_position"]) > 0.01:
    raise ValueError("Object differs from planned scene; reset before executing")
e = r.start_recording("抓起桌上的红色方块，稳定抬起，然后放回桌面")
measurements = []
success = False
reason = "execution_failed"


def wait_task(task):
    result = r.wait(task.id, timeout=180)
    if result.status != "completed":
        raise RuntimeError(result.details)


def hold(seconds):
    state = r.get_joint_state()
    q = [state["targets"][state["joint_names"].index(n)] for n in names]
    wait_task(r.execute_joint_chunk(names, [q], round(seconds * 120)))


def fingers(value):
    state = r.get_joint_state()
    command = gripper["close" if value == 0 else "open"]
    finger_names = list(command)
    values = [
        max(
            state["limits"][state["joint_names"].index(n)][0],
            min(state["limits"][state["joint_names"].index(n)][1], command[n]),
        )
        for n in finger_names
    ]
    wait_task(r.execute_joint_chunk(finger_names, [values], 180))


try:
    hold(1)
    for segment in plan["segments"]:
        if abs(segment["dt"] - 1 / 120) > 1e-6:
            raise ValueError("Plan timestep must be 1/120 s")
        rows = segment["positions"]
        # API chunks have a bounded length; normal planning segments fit one request.
        if len(rows) > 2048:
            raise ValueError("Trajectory exceeds single-chunk limit")
        wait_task(r.execute_joint_chunk(names, rows, 1))
        hold(0.5)
        measurements.append(dict(stage=segment["name"], state=r.get_joint_state()))
        if segment["name"] == "grasp":
            fingers(0)
        if segment["name"] == "lift":
            heights = []
            for _ in range(4):
                hold(0.5)
                state = r.get_joint_state()["object_states"]["/World/GraspCube"]
                heights.append(state[2])
            success = min(heights) > initial[2] + 0.10 and max(heights) - min(heights) < 0.02
            measurements.append(dict(stage="held", heights=heights, success=success))
            if not success:
                raise RuntimeError(f"Object did not remain lifted: {heights}")
        if segment["name"] == "place":
            fingers(0.04)
    hold(1)
    final_cube = r.get_joint_state()["object_states"]["/World/GraspCube"]
    success = success and abs(final_cube[2] - initial[2]) < 0.02
    reason = "physical_lift_and_place_verified" if success else "placement_failed"
except Exception as exc:
    success = False
    reason = str(exc)
    raise
finally:
    result = r.stop_recording(e["id"], success, reason)
    report = dict(
        recording=result,
        success=success,
        robot=fixture["robot"],
        arm=arm,
        measurements=measurements,
        fk_validation=fk_validation,
        plan=args.plan,
    )
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
