import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinematics import transform
from planning.grasp_profiles import grasp_spec


@pytest.mark.parametrize(
    "robot,arm",
    [("x5", "arm"), ("x5_dual", "left"), ("x5_dual", "right"), ("cobot_magic", "fl"), ("cobot_magic", "fr")],
)
def test_grasp_frames_and_contact(robot, arm):
    s = grasp_spec(robot, arm)
    base = transform(s["base_pose"][:3], s["base_pose"][3:])
    grasp = next(t["pose"] for t in s["targets"] if t["name"] == "grasp")
    contact = base @ transform(grasp[:3], grasp[3:]) @ np.r_[s["contact_offset"], 1]
    assert contact[:3] == pytest.approx(s["cube_position"], abs=1e-6)
    assert s["cube_position"][2] - s["table_position"][2] == pytest.approx(0.05 + s["cube_size"][2] / 2)
    assert len(s["start"]) == 6


def test_unsupported_arm_is_rejected():
    with pytest.raises(ValueError):
        grasp_spec("cobot_magic", "bad")


def test_validate_plan_rejects_orientation_mismatch():
    from planning.grasp_profiles import validate_plan

    s = grasp_spec("x5")
    plan = dict(
        planner="curobo",
        fixture=s,
        segments=[dict(name=t["name"], positions=[[0.0] * 6], dt=1 / 120) for t in s["targets"]],
    )
    with pytest.raises(ValueError, match="rotation"):
        validate_plan(plan, lambda arm, q: dict(position=s["targets"][0]["pose"][:3], quaternion=[1, 0, 0, 0]))
