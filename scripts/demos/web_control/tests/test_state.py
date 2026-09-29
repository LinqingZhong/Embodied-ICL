import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from state import ControlState


def ready_state():
    state = ControlState()
    state.configure(["joint", "finger"], [(-1, 1), (0, 0.04)], [0, 0.04], [0], [1])
    return state


def test_rejects_commands_until_ready():
    with pytest.raises(ValueError, match="ready"):
        ControlState().submit({"action": "pause"})


@pytest.mark.parametrize("value", [float("nan"), float("inf"), 2, "0.2", True])
def test_rejects_bad_joint_target(value):
    with pytest.raises(ValueError):
        ready_state().submit({"action": "joint", "joint": 0, "value": value})


def test_rejects_wrong_joint_and_unknown_action():
    state = ready_state()
    for command in (
        {"action": "joint", "joint": 1, "value": 0},
        {"action": "joint", "joint": True, "value": 0},
        {"action": "launch_shell"},
    ):
        with pytest.raises(ValueError):
            state.submit(command)


def test_latest_target_wins_and_applied_target_is_rate_limited():
    state = ready_state()
    for value in [0.1, 0.2, 0.8]:
        state.submit({"action": "joint", "joint": 0, "value": value})
    tick = state.consume(0.1)
    assert tick["targets"][0] == pytest.approx(0.05)  # 0.5 rad/s
    assert state.status()["joints"][0]["target"] == 0.8


def test_pause_step_reset_and_gripper_semantics():
    state = ready_state()
    state.submit({"action": "pause"})
    state.submit({"action": "joint", "joint": 0, "value": 0.5})
    assert not state.consume(0.1)["step"]
    state.submit({"action": "step"})
    assert state.consume(0.1)["step"]
    assert not state.consume(0.1)["step"]
    state.submit({"action": "gripper", "open": False})
    state.submit({"action": "reset"})
    tick = state.consume(0.1)
    assert tick["reset"] and not tick["step"]
    assert tick["targets"] == [0, 0.04]
    assert state.status()["paused"]
    state.submit({"action": "resume"})
    assert state.consume(0.1)["step"]


def test_step_requires_pause_and_queue_is_bounded():
    state = ready_state()
    with pytest.raises(ValueError, match="pause"):
        state.submit({"action": "step"})
    for _ in range(32):
        state.submit({"action": "pause"})
    with pytest.raises(ValueError, match="queue"):
        state.submit({"action": "reset"})


def test_frame_pair_is_immutable_and_has_capture_state():
    state = ready_state()
    state.publish(1.5, [0.2, 0.03], [0.1, 0], {"scene": b"a", "wrist": b"b"})
    frame = state.frame()
    state.publish(2, [0.3, 0.02], [0, 0], {"scene": b"c", "wrist": b"d"})
    assert frame["sim_time"] == 1.5
    assert frame["positions"] == [0.2, 0.03]
    assert frame["images"]["scene"] == b"a"
    assert state.status()["frame_age"] >= 0


def test_reset_preserves_targets_submitted_after_it():
    state = ready_state()
    state.submit({"action": "pause"})
    state.consume(0.1)
    state.submit({"action": "reset"})
    state.submit({"action": "joint", "joint": 0, "value": 0.5})
    state.submit({"action": "gripper", "open": False})
    tick = state.consume(0.1)
    assert tick["reset"] and not tick["step"]
    assert state.status()["joints"][0]["target"] == 0.5
    state.submit({"action": "resume"})
    tick = state.consume(0.1)
    assert tick["targets"][0] > 0
    assert tick["targets"][1] < 0.04
