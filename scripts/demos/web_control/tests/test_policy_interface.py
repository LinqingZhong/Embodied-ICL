import json
import sys
import urllib.error
import urllib.request
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot import get_profile
from robot_client import Observation, RobotClient, RobotError
from robot_server import RobotServer
from server import WebServer


def configured():
    robot = RobotServer(physics_dt=0.01)
    robot.configure(
        ["arm", "finger", "passive"],
        [(-1, 1), (0, 0.04), (-float("inf"), float("inf"))],
        [0, 0.02, 0],
        [0],
        [1],
    )
    return robot


def configured_profile(name):
    profile = get_profile(name)
    names = [joint["name"] for joint in profile["joints"]]
    names += [name for gripper in profile["grippers"] for name in gripper["open"]]
    # USD order need not match profile order, and includes passive joints.
    names = ["passive", *reversed(names)]
    robot = RobotServer(physics_dt=0.01)
    robot.configure_profile(
        names,
        [(-float("inf"), float("inf"))] + [(-3, 3)] * (len(names) - 1),
        [0] * len(names),
        profile,
    )
    return robot, profile


@pytest.fixture
def http_robot(tmp_path):
    robot = configured()
    web = WebServer(robot, port=0, output_dir=tmp_path)
    web.start()
    try:
        yield robot, RobotClient(web.url)
    finally:
        web.stop()


def test_robot_spec_has_exact_metadata_and_legacy_units():
    robot = configured()
    spec = robot.robot_spec()
    assert spec == {
        "name": "Franka Panda",
        "joint_names": ["arm", "finger", "passive"],
        "controllable_joint_names": ["arm", "finger"],
        "limits": [[-1, 1], [0, 0.04], [None, None]],
        "joint_units": {"arm": "rad", "finger": "m"},
        "grippers": [],
        "arms": {},
        "cameras": ["scene", "wrist"],
        "physics_dt": 0.01,
    }
    json.dumps(spec, allow_nan=False)


@pytest.mark.parametrize("name", ["cobot_magic", "a2d"])
def test_robot_spec_preserves_native_profile_metadata(name):
    robot, profile = configured_profile(name)
    spec = robot.robot_spec()
    assert spec["name"] == profile["label"]
    assert spec["joint_names"] == robot.names
    assert spec["controllable_joint_names"] == [joint["name"] for joint in profile["joints"]] + [
        name for gripper in profile["grippers"] for name in gripper["open"]
    ]
    assert spec["limits"][0] == [None, None]
    assert spec["limits"][1:] == robot.limits[1:]
    assert spec["joint_units"] == {
        **{joint["name"]: joint["unit"] for joint in profile["joints"]},
        **{name: gripper["unit"] for gripper in profile["grippers"] for name in gripper["open"]},
    }
    assert spec["grippers"] == [
        {key: gripper[key] for key in ("name", "label", "unit", "open", "close")} for gripper in profile["grippers"]
    ]
    assert spec["cameras"] == [camera["name"] for camera in profile["cameras"]]
    if name == "cobot_magic":
        assert spec["grippers"][0]["open"] == {"fl_joint7": 0.04, "fl_joint8": -0.04}
        assert spec["joint_units"]["camera_stand_1_joint"] == "m"
    else:
        assert spec["joint_units"]["joint_lift_body"] == "m"
        assert spec["joint_units"]["left_hand_joint1"] == "rad"
        assert spec["grippers"][1]["close"]["right_hand_joint1"] == 0.2
    json.dumps(spec, allow_nan=False)


def test_robot_spec_copies_nested_metadata_and_arm_definitions():
    import numpy as np
    from kinematics import ArmKinematics

    robot, _ = configured_profile("cobot_magic")
    robot.kinematics = {
        "fl": ArmKinematics(
            {"offset": [0, 0, 0]},
            ["fl_joint1"],
            [(np.eye(4), np.array([0, 0, 1]), np.eye(4))],
            [[-3, 3]],
        )
    }
    expected_arms = robot.get_arms()
    spec = robot.robot_spec()
    assert spec["arms"] == expected_arms
    spec["arms"]["fl"]["offset"][0] = 99
    spec["arms"]["fl"]["joint_names"][0] = "changed"
    spec["joint_names"][0] = "changed"
    spec["limits"][1][0] = 99
    spec["grippers"][0]["open"]["fl_joint7"] = 99
    assert robot.get_arms() == expected_arms
    assert robot.names[0] == "passive"
    assert robot.limits[1][0] == -3
    assert robot.grippers[0]["open"]["fl_joint7"] == 0.04


def test_robot_spec_does_not_invent_missing_profile_units():
    robot, _ = configured_profile("a2d")
    del robot.joint_meta[robot.names.index("joint_lift_body")]["unit"]
    del robot.grippers[0]["unit"]
    spec = robot.robot_spec()
    assert spec["joint_units"]["joint_lift_body"] is None
    assert spec["joint_units"]["left_hand_joint1"] is None
    assert spec["grippers"][0]["unit"] is None


@pytest.mark.parametrize("phase", ["loading", "stopped", "error"])
def test_robot_spec_requires_ready(phase):
    robot = RobotServer()
    robot.set_phase(phase, "Unavailable")
    with pytest.raises(RuntimeError, match="not ready"):
        robot.robot_spec()


def test_robot_spec_http_rejects_not_ready(http_robot):
    robot, client = http_robot
    robot.set_phase("loading", "Unavailable")
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(client.url + "/api/robot/spec")
    assert exc.value.code == 409
    assert json.load(exc.value)["detail"] == "Simulator is not ready"


def test_client_reads_robot_spec_over_http(http_robot):
    robot, client = http_robot
    assert client.get_robot_spec() == robot.robot_spec()
    robot.set_phase("error", "Unavailable")
    with pytest.raises(RobotError) as exc:
        client.get_robot_spec()
    assert exc.value.status == 409


def test_client_reads_status_over_http(http_robot):
    robot, client = http_robot
    assert client.get_status() == robot.status()
    robot.set_phase("loading", "Unavailable")
    assert client.get_status()["phase"] == "loading"


def test_eef_image_snapshot_stays_at_capture_time():
    robot = configured()
    pose = {"position": [1, 2, 3], "quaternion": [1, 0, 0, 0]}
    robot.eef_poses = {"arm": pose}
    images = {"scene": b"first-image"}
    robot.publish(0.01, [0.1, 0.02, 0], [0, 0, 0], images)
    pose["position"][0] = 5
    images["scene"] = b"changed-image"
    robot.applied[0] = 0.2
    robot.publish(0.02, [0.2, 0.02, 0], [1, 0, 0])
    frame = robot.frame()
    assert frame["eef_poses"]["arm"]["position"] == [1, 2, 3]
    assert frame["images"] == {"scene": b"first-image"}
    assert frame["sim_time"] == 0.01
    assert frame["positions"] == [0.1, 0.02, 0]
    assert frame["targets"] == [0, 0.02, 0]
    assert robot.joint_state()["eef_poses"]["arm"]["position"] == [5, 2, 3]
    frame["eef_poses"]["arm"]["position"][0] = 99
    assert robot.frame()["eef_poses"]["arm"]["position"] == [1, 2, 3]


def test_client_observation_includes_same_frame_eef_poses(http_robot):
    robot, client = http_robot
    robot.eef_poses = {"arm": {"position": [1, 2, 3], "quaternion": [1, 0, 0, 0]}}
    robot.publish(0.01, [0.1, 0.02, 0], [0, 0, 0], {"scene": b"first-image"})
    robot.eef_poses["arm"]["position"][0] = 5
    robot.publish(0.02, [0.2, 0.02, 0], [1, 0, 0])
    observation = client.get_observation()
    assert observation.eef_poses == {"arm": {"position": [1, 2, 3], "quaternion": [1, 0, 0, 0]}}
    assert observation.sim_time == 0.01
    assert observation.images == {"scene": b"first-image"}
    assert client.get_joint_state()["eef_poses"]["arm"]["position"] == [5, 2, 3]


def test_observation_remains_compatible_with_legacy_payload(http_robot):
    robot, client = http_robot
    robot.publish(0.01, [0, 0.02, 0], [0, 0, 0], {"scene": b"image"})
    robot.latest.pop("eef_poses", None)
    assert client.get_observation().eef_poses == {}


def test_observation_eef_default_is_independent_and_preserves_positional_construction():
    first = Observation(1, 0, 0, [], [], [], [], {})
    second = Observation(2, 0, 0, [], [], [], [], {})
    assert first.eef_poses == {}
    first.eef_poses["arm"] = {}
    assert second.eef_poses == {}
