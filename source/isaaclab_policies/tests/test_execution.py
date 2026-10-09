import copy
import time
from types import SimpleNamespace

import pytest
from isaaclab_policies import (
    BasePolicy,
    EefActionChunk,
    JointActionChunk,
    PolicyRunner,
    RobotAdapter,
    StopAction,
)


def metadata():
    return dict(
        name="test",
        joint_names=["joint", "finger_l", "finger_r", "wheel"],
        controllable_joint_names=["joint", "finger_l", "finger_r"],
        limits=[[-1, 1], [0, 0.04], [-0.04, 0], [None, None]],
        joint_units={"joint": "rad", "finger_l": "m", "finger_r": "m"},
        grippers=[
            dict(
                name="hand", unit="m", open={"finger_l": 0.04, "finger_r": -0.04}, close={"finger_l": 0, "finger_r": 0}
            )
        ],
        arms={"arm": dict(base="base", flange="flange", joint_names=["joint"], quaternion_order="wxyz")},
        cameras=["scene"],
        physics_dt=1 / 120,
    )


def observation(sequence=1, sim_time=0.1):
    return SimpleNamespace(
        sequence=sequence,
        sim_time=sim_time,
        captured_at=time.time(),
        joint_names=metadata()["joint_names"],
        positions=[0, 0.02, -0.02, 0],
        velocities=[0] * 4,
        targets=[0, 0.02, -0.02, 0],
        images={"scene": b"jpeg"},
        eef_poses={"arm": dict(position=[0, 0, 0], quaternion=[1, 0, 0, 0])},
        object_states={"secret": "not a policy input"},
    )


class Client:
    def __init__(self):
        self.calls = []
        self.cancelled = []
        self.frames = [observation()]
        self.wait_error = None

    def get_robot_spec(self):
        return metadata()

    def get_status(self):
        return dict(phase="ready", paused=False, active_task=None)

    def get_observation(self):
        return self.frames.pop(0) if len(self.frames) > 1 else self.frames[0]

    def execute_joint_chunk(self, *args):
        self.calls.append(("joint", args))
        return SimpleNamespace(id="owned", status="running")

    def execute_eef_chunk(self, *args):
        self.calls.append(("eef", args))
        return SimpleNamespace(id="owned", status="running")

    def wait(self, task_id, **kwargs):
        if self.wait_error:
            raise self.wait_error
        return SimpleNamespace(
            id=task_id,
            status="completed",
            details=dict(
                executed_steps=12,
                finished_sim_time=1.0,
                actual_positions=[0.1],
                targets=[0.2],
                errors=[0.1],
            ),
        )

    def cancel(self, task_id):
        self.cancelled.append(task_id)


def test_native_joint_and_eef_dispatch_and_whitelisted_snapshot():
    client = Client()
    robot = RobotAdapter(client)
    spec = robot.get_robot_spec()
    assert spec.joint_units["finger_r"] == "m"
    obs = robot.get_observation()
    assert not hasattr(obs, "object_states")
    action = JointActionChunk(["finger_r", "finger_l"], [[-0.04, 0.04]], 12)
    result = robot.execute(action)
    assert client.calls == [("joint", (["finger_r", "finger_l"], [[-0.04, 0.04]], 12))]
    assert result.action == action and result.executed_steps == 12
    assert result.details["errors"] == [0.1]
    eef = EefActionChunk("arm", [[0.1, 0.2, 0.3]], [[1, 0, 0, 0]], 8)
    robot.execute(eef)
    assert client.calls[-1] == ("eef", ("arm", eef.positions, eef.quaternions, 8))


@pytest.mark.parametrize(
    "action",
    [
        JointActionChunk(["wheel"], [[0]], 1),
        JointActionChunk(["joint"], [[2]], 1),
        JointActionChunk(["joint"], [[True]], 1),
        JointActionChunk(["joint"], [[float("nan")]], 1),
        JointActionChunk(["joint", "joint"], [[0, 0]], 1),
        JointActionChunk(["joint"], [[0, 0]], 1),
        JointActionChunk(["joint"], [[0]], True),
        EefActionChunk("missing", [[0, 0, 0]], [[1, 0, 0, 0]], 1),
        EefActionChunk("arm", [[0, 0, 0]], [[0, 0, 0, 0]], 1),
        EefActionChunk("arm", [[0, 0, 0]], [], 1),
    ],
)
def test_invalid_actions_never_dispatch(action):
    client = Client()
    with pytest.raises(ValueError):
        RobotAdapter(client).execute(action)
    assert client.calls == []


@pytest.mark.parametrize("failure", [TimeoutError("wait expired"), KeyboardInterrupt()])
def test_interrupted_wait_cancels_owned_task_but_ambiguous_submission_is_not_retried(failure):
    client = Client()
    client.wait_error = failure
    action = JointActionChunk(["joint"], [[0.2]], 1)
    with pytest.raises(type(failure)):
        RobotAdapter(client).execute(action)
    assert client.cancelled == ["owned"] and len(client.calls) == 1

    def ambiguous(*args):
        raise TimeoutError("submission may have arrived")

    client.execute_joint_chunk = ambiguous
    client.cancelled.clear()
    with pytest.raises(TimeoutError):
        RobotAdapter(client).execute(action)
    assert client.cancelled == []


class Policy(BasePolicy):
    def __init__(self):
        self.events = []

    def reset(self, instruction, robot):
        self.events.append(("reset", instruction, robot.name))

    def predict(self, obs):
        self.events.append(("predict", obs.sequence))
        return JointActionChunk(["joint"], [[0.2]], 12) if obs.sequence == 1 else StopAction("finished")

    def observe_result(self, result):
        self.events.append(("feedback", result.status))

    def close(self):
        self.events.append(("close",))


def test_runner_waits_for_capture_after_completed_action_and_closes():
    client = Client()
    client.frames = [observation(), observation(), observation(2, 0.5), observation(3, 1.1)]
    policy = Policy()
    results = PolicyRunner(policy, RobotAdapter(client), poll_interval=0.001).run("move", max_steps=3)
    assert policy.events == [
        ("reset", "move", "test"),
        ("predict", 1),
        ("feedback", "completed"),
        ("predict", 3),
        ("feedback", "stopped"),
        ("close",),
    ]
    assert len(client.calls) == 1 and len(results) == 2


def test_stale_capture_timeout_does_not_trigger_second_prediction():
    client, policy = Client(), Policy()
    runner = PolicyRunner(policy, RobotAdapter(client), observation_timeout=0.02, poll_interval=0.001)
    with pytest.raises(TimeoutError, match="observation"):
        runner.run("move", max_steps=2)
    assert len(client.calls) == 1 and policy.events[-1] == ("close",)


def test_max_steps_and_failed_task_close_policy_without_more_motion():
    client, policy = Client(), Policy()
    PolicyRunner(policy, RobotAdapter(client)).run("move", max_steps=1)
    assert len(client.calls) == 1 and policy.events[-1] == ("close",)
    client.wait = lambda *a, **k: SimpleNamespace(
        id="owned",
        status="failed",
        details={
            "executed_steps": 0,
            "finished_sim_time": 0.2,
        },
    )
    with pytest.raises(RuntimeError, match="failed"):
        PolicyRunner(policy, RobotAdapter(client)).run("move", max_steps=2)
    assert policy.events[-2:] == [("feedback", "failed"), ("close",)]


def test_observation_is_independent_and_rejects_mismatched_joint_order():
    client = Client()
    frame = client.frames[0]
    obs = RobotAdapter(client).get_observation()
    frame.eef_poses["arm"]["position"][0] = 999
    assert obs.eef_poses["arm"]["position"][0] == 0
    client.frames = [copy.deepcopy(frame)]
    client.frames[0].joint_names.reverse()
    with pytest.raises(ValueError, match="joint_names"):
        RobotAdapter(client).get_observation()
