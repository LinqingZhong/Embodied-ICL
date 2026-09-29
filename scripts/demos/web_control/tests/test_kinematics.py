import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinematics import ArmKinematics, transform
from robot_server import RobotServer


def arm():
    return ArmKinematics(dict(offset=[1, 0, 0]), ["joint"], [(np.eye(4), np.array([0, 0, 1]), np.eye(4))], [[-2, 2]])


def test_fk_ik_and_unreachable():
    chain = arm()
    p = chain.fk([0.4])
    assert p["position"] == pytest.approx([np.cos(0.4), np.sin(0.4), 0])
    assert chain.ik(p["position"], p["quaternion"], [0])["joint_positions"] == pytest.approx([0.4])
    with pytest.raises(ValueError):
        chain.ik([4, 4, 4], [1, 0, 0, 0], [0])
    with pytest.raises(ValueError):
        transform([0, 0, 0], [0, 0, 0, 0])


def test_eef_atomic_failure_and_exact_joint_targets():
    s = RobotServer()
    s.configure(["joint"], [[-2, 2]], [0], [0], [])
    s.kinematics = {"arm": arm()}
    p = s.fk("arm", [0.4])
    with pytest.raises(ValueError, match="row 1"):
        s.execute_eef_chunk("arm", [p["position"], [4, 4, 4]], [p["quaternion"]] * 2, 1)
    assert s.active_task is None and not s.planning and s.desired == [0]
    task = s.execute_eef_chunk("arm", [p["position"]], [p["quaternion"]], 1)
    assert s.consume(1 / 120)["targets"] == pytest.approx([0.4])
    s.publish(1 / 120, [0.3], [0])
    assert s.get_task(task["id"])["status"] == "completed"


def test_http_fk_ik_and_eef(tmp_path):
    from robot_client import RobotClient, RobotError
    from server import WebServer

    s = RobotServer()
    s.configure(["joint"], [[-2, 2]], [0], [0], [])
    s.kinematics = {"arm": arm()}
    web = WebServer(s, port=0, output_dir=tmp_path)
    web.start()
    try:
        r = RobotClient(web.url)
        assert r.get_arms()["arm"]["quaternion_order"] == "wxyz"
        target = r.fk("arm", [0.3])
        solved = r.ik("arm", target["position"], target["quaternion"])
        assert solved["joint_positions"] == pytest.approx([0.3])
        with pytest.raises(RobotError) as exc:
            r.execute_eef_chunk("arm", [[9, 9, 9]], [[1, 0, 0, 0]], 1)
        assert exc.value.status == 400 and s.active_task is None
        task = r.execute_eef_chunk("arm", [target["position"]], [target["quaternion"]], 1)
        assert s.consume(1 / 120)["targets"] == pytest.approx([0.3])
        s.publish(1 / 120, [0.3], [0])
        assert r.wait(task.id).status == "completed"
    finally:
        web.stop()


def test_planning_reserves_control_and_freezes_manual_ramp():
    s = RobotServer()
    s.configure(["joint"], [[-2, 2]], [0], [0], [])
    s.submit(dict(action="joint", joint=0, value=1))
    s.planning = True
    assert s.consume(1 / 120)["targets"] == [0]
    with pytest.raises(RuntimeError):
        s.execute_joint_chunk(["joint"], [[0.1]], 1)
    with pytest.raises(ValueError):
        s.submit(dict(action="reset"))


def test_default_seed_handles_physics_soft_limit_overshoot():
    s = RobotServer()
    s.configure(["joint"], [[-2, 2]], [0], [0], [])
    s.kinematics = {"arm": arm()}
    s.positions = [2.01]
    target = s.fk("arm", [1.9])
    assert s.ik("arm", target["position"], target["quaternion"])["joint_positions"] == pytest.approx([1.9])
    with pytest.raises(ValueError, match="seed outside"):
        s.ik("arm", target["position"], target["quaternion"], [2.01])
