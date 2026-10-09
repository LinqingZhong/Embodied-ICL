import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from state import ControlState


def test_profiles_exclude_passive_joints_and_preserve_units():
    from robot import get_profile

    profile = get_profile("a2d")
    names = [j["name"] for j in profile["joints"]]
    assert len(names) == 18
    assert "left_Right_1_Joint" not in names
    assert next(j for j in profile["joints"] if j["name"] == "joint_lift_body")["unit"] == "m"
    assert {c["name"] for c in profile["cameras"]} == {"scene", "head", "left_wrist", "right_wrist"}


def configured():
    state = ControlState()
    profile = {
        "label": "Test dual arm",
        "joints": [{"name": "lift", "group": "Body", "unit": "m", "speed": 0.1, "step": 0.01}],
        "grippers": [
            {
                "name": "left",
                "label": "Left",
                "unit": "rad",
                "speed": 0.5,
                "open": {"left_driver": 0.8},
                "close": {"left_driver": 0.2},
            },
            {
                "name": "right",
                "label": "Right",
                "unit": "rad",
                "speed": 0.5,
                "open": {"right_driver": 0.9},
                "close": {"right_driver": 0.1},
            },
        ],
        "cameras": [{"name": "scene", "label": "Scene"}, {"name": "head", "label": "Head"}],
    }
    state.configure_profile(["lift", "left_driver", "right_driver", "passive"], [(0, 1)] * 4, [0.5] * 4, profile)
    return state, profile


def test_independent_grippers_and_passive_joint_protection():
    state, _ = configured()
    state.submit({"action": "gripper", "gripper": "left", "open": True})
    assert state.status()["grippers"][0]["targets"] == [0.8]
    assert state.status()["grippers"][1]["targets"] == [0.5]
    with pytest.raises(ValueError):
        state.submit({"action": "joint", "joint": 3, "value": 0.2})
    with pytest.raises(ValueError):
        state.submit({"action": "gripper", "open": True})
    with pytest.raises(ValueError):
        state.submit({"action": "gripper", "gripper": "unknown", "open": True})
    tick = state.consume(0.1)
    assert tick["targets"][1] == pytest.approx(0.55)
    assert tick["targets"][3] == 0.5


def test_units_speed_and_reset():
    state, _ = configured()
    state.submit({"action": "joint", "joint": 0, "value": 0.8})
    assert state.consume(0.1)["targets"][0] == pytest.approx(0.51)
    assert state.status()["joints"][0]["unit"] == "m"
    state.submit({"action": "reset"})
    assert state.consume(0.1)["targets"] == [0.5] * 4
    assert state.status()["cameras"][1]["name"] == "head"


def test_missing_joint_and_invalid_gripper_target_rejected():
    _, profile = configured()
    with pytest.raises(ValueError, match="lift"):
        ControlState().configure_profile(["wrong"], [(0, 1)], [0], profile)
    profile["grippers"][0]["open"]["left_driver"] = 2
    with pytest.raises(ValueError):
        ControlState().configure_profile(["lift", "left_driver", "right_driver"], [(0, 1)] * 3, [0] * 3, profile)


def test_continuous_joint_requires_finite_panel_range():
    _, profile = configured()
    names = ["lift", "left_driver", "right_driver"]
    limits = [(-float("inf"), float("inf")), (0, 1), (0, 1)]
    with pytest.raises(ValueError, match="Finite"):
        ControlState().configure_profile(names, limits, [0.5] * 3, profile)
    profile["joints"][0]["limits"] = [0, 1]
    state = ControlState()
    state.configure_profile(names, limits, [0.5] * 3, profile)
    assert state.status()["joints"][0]["max"] == 1


def test_float32_gripper_limit_roundoff_is_clamped():
    from robot import get_profile

    profile = get_profile("franka")
    names = [j["name"] for j in profile["joints"]] + ["panda_finger_joint1", "panda_finger_joint2"]
    state = ControlState()
    state.configure_profile(names, [(-3, 3)] * 7 + [(0, 0.0399999991)] * 2, [0] * 9, profile)
    state.submit({"action": "gripper", "open": True})
    assert state.status()["grippers"][0]["targets"] == [0.0399999991] * 2


def test_gripper_maps_by_name_even_when_close_order_differs():
    profile = {
        "label": "test",
        "joints": [],
        "cameras": [],
        "grippers": [
            {
                "name": "hand",
                "label": "hand",
                "unit": "rad",
                "speed": 1,
                "open": {"a": 0.8, "b": 0.9},
                "close": {"b": 0.2, "a": 0.1},
            }
        ],
    }
    state = ControlState()
    state.configure_profile(["a", "b"], [(0, 1)] * 2, [0] * 2, profile)
    state.submit({"action": "gripper", "open": False})
    assert state.status()["grippers"][0]["targets"] == [0.1, 0.2]


def test_x5_profile_has_six_arm_joints_and_two_fingers():
    from robot import get_profile

    profile = get_profile("x5")
    assert [j["name"] for j in profile["joints"]] == [f"joint{i}" for i in range(1, 7)]
    assert profile["grippers"][0]["open"] == {"joint7": 0.044, "joint8": 0.044}
    assert {c["name"] for c in profile["cameras"]} == {"scene", "wrist"}


def test_dual_x5_has_distinct_joint_and_gripper_names():
    from robot import get_profile

    p = get_profile("x5_dual")
    assert len(p["joints"]) == 12
    assert len({j["name"] for j in p["joints"]}) == 12
    assert p["instances"][0]["position"] == [-0.3, -0.45, 0.765]
    names = [f"{side}/joint{i}" for side in ("left", "right") for i in range(1, 9)]
    s = ControlState()
    s.configure_profile(names, [(-3, 3)] * 16, [0] * 16, p)
    s.submit({"action": "joint", "joint": 0, "value": 0.2})
    s.submit({"action": "gripper", "gripper": "right", "open": True})
    t = s.consume(1)["targets"]
    assert t[0] == 0.2 and t[8] == 0
    assert t[6] == 0 and t[14] > 0


def test_cobot_magic_four_arms_and_opposed_fingers():
    from robot import get_profile

    p = get_profile("cobot_magic")
    assert len(p["joints"]) == 26
    assert len(p["grippers"]) == 4
    assert p["grippers"][0]["open"] == {"fl_joint7": 0.04, "fl_joint8": -0.04}
    assert not any(j["name"] in ("left_joint", "right_joint") for j in p["joints"])


def test_robot_observation_cameras_cover_dual_center_and_franka_front():
    from robot import get_profile

    dual = get_profile("x5_dual")
    head = next(c for c in dual["cameras"] if c["name"] == "head")
    assert head["body"] == "base_link" and head["instance"] == "left"
    eye = [a + b for a, b in zip(dual["instances"][0]["position"], head["eye_offset"])]
    assert eye[0] == 0 and eye[2] > 1.2
    assert {c["name"] for c in dual["cameras"]} >= {"head", "left_wrist", "right_wrist"}
    front = next(c for c in get_profile("franka")["cameras"] if c["name"] == "front")
    assert front["body"] == "panda_link0"
    assert front["eye_offset"][0] > front["target"][0] > 0
