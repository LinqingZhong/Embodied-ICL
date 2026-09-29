import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_server import RobotServer


def configured():
    s = RobotServer()
    s.configure(["arm", "finger"], [(-1, 1), (0, 0.04)], [0, 0.02], [0], [1])
    return s


def tick(s):
    t = s.consume(1 / 120)
    s.publish(s.sim_time + (1 / 120 if t["step"] else 0), [0, 0.02], [0, 0])
    return t


def test_exact_targets_and_completion_after_physical_step():
    s = configured()
    task = s.execute_joint_chunk(["arm"], [[0.8], [-0.6]], 2)
    assert [tick(s)["targets"][0] for _ in range(3)] == [0.8, 0.8, -0.6]
    assert s.get_task(task["id"])["status"] == "running"
    t = s.consume(1 / 120)
    assert t["targets"] == [-0.6, 0.02]
    assert s.get_task(task["id"])["status"] == "running"
    s.publish(4 / 120, [0.1, 0.02], [0, 0])
    result = s.get_task(task["id"])
    assert result["status"] == "completed"
    assert result["errors"] == pytest.approx([-0.7])
    assert tick(s)["targets"][0] == -0.6


def test_busy_pause_cancel_reset():
    s = configured()
    task = s.execute_joint_chunk(["arm"], [[0.8], [-0.6]], 2)
    with pytest.raises(RuntimeError):
        s.execute_joint_chunk(["arm"], [[0]], 1)
    with pytest.raises(ValueError):
        s.submit({"action": "joint", "joint": 0, "value": 0.2})
    s.submit({"action": "pause"})
    assert not tick(s)["step"]
    assert s.get_task(task["id"])["executed_steps"] == 0
    s.submit({"action": "step"})
    assert tick(s)["targets"][0] == 0.8
    s.cancel(task["id"])
    assert s.get_task(task["id"])["status"] == "cancelled"
    s.submit({"action": "resume"})
    assert tick(s)["targets"][0] == 0.8
    task = s.execute_joint_chunk(["arm"], [[0.2]], 2)
    s.submit({"action": "reset"})
    tick(s)
    assert s.get_task(task["id"])["status"] == "cancelled"


@pytest.mark.parametrize(
    "names,rows,steps",
    [
        (["bad"], [[0]], 1),
        (["arm"], [[2]], 1),
        (["arm"], [[float("nan")]], 1),
        (["arm"], [[True]], 1),
        (["arm"], [], 1),
        (["arm", "arm"], [[0, 0]], 1),
        (["arm"], [[0, 0]], 1),
        (["arm"], [[0]], 0),
    ],
)
def test_reject_atomically(names, rows, steps):
    s = configured()
    with pytest.raises(ValueError):
        s.execute_joint_chunk(names, rows, steps)
    assert s.active_task is None
    assert s.desired == [0, 0.02]


def test_observation_http_and_client(tmp_path):
    from robot_client import RobotClient, RobotError
    from server import WebServer

    s = configured()
    s.manual_control = False
    web = WebServer(s, port=0, output_dir=tmp_path)
    web.start()
    try:
        client = RobotClient(web.url)
        with pytest.raises(RobotError) as exc:
            client.get_observation()
        assert exc.value.status == 503
        s.publish(0, [0.1, 0.02], [0, 0], {"head": b"jpeg-head", "wrist": b"jpeg-wrist"})
        s.publish(0.01, [0.2, 0.02], [0, 0])
        obs = client.get_observation()
        assert obs.positions[0] == 0.1 and obs.sim_time == 0
        assert obs.images == {"head": b"jpeg-head", "wrist": b"jpeg-wrist"}
        assert client.get_joint_state()["positions"][0] == 0.2
        task = client.execute_joint_chunk(["arm"], [[0.8], [-0.2]], 1)
        with pytest.raises(RobotError) as exc:
            client.execute_joint_chunk(["arm"], [[0]], 1)
        assert exc.value.status == 409
        with pytest.raises(TimeoutError):
            client.wait(task.id, timeout=0.03, poll_interval=0.01)
        assert client.get_task(task.id).status == "running"
        tick(s)
        tick(s)
        result = client.wait(task.id)
        assert result.status == "completed"
        assert result.details["errors"] == [-0.2]
        assert client.cancel(task.id).status == "completed"
        with pytest.raises(RobotError) as exc:
            client.get_task("missing")
        assert exc.value.status == 404
        with pytest.raises(ValueError):
            s.submit({"action": "joint", "joint": 0, "value": 0})
    finally:
        web.stop()


def test_reset_holds_defaults_and_stop_fails_task():
    s = configured()
    task = s.execute_joint_chunk(["arm"], [[0.8]], 10)
    tick(s)
    s.submit({"action": "reset"})
    assert tick(s)["targets"] == [0, 0.02]
    assert s.get_task(task["id"])["status"] == "cancelled"
    task = s.execute_joint_chunk(["arm"], [[0.8]], 10)
    s.set_phase("error", "physics stopped")
    assert s.get_task(task["id"])["status"] == "failed"


def test_unbounded_passive_joint_limits_are_json_safe():
    import json

    s = RobotServer()
    s.configure(["arm", "wheel"], [(-1, 1), (-float("inf"), float("inf"))], [0, 0], [0], [])
    data = s.joint_state()
    assert data["limits"][1] == [None, None]
    json.dumps(data, allow_nan=False)
