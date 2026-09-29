import io
import sys
from pathlib import Path

import h5py
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_server import RobotServer


def test_episode_roundtrip(tmp_path):
    s = RobotServer()
    s.configure(["j"], [[-1, 1]], [0], [0], [])
    s.cameras = [{"name": "head", "label": "head"}]
    s.recording_dir = tmp_path
    b = io.BytesIO()
    Image.new("RGB", (8, 8), "red").save(b, format="JPEG")
    ep = s.start_recording("移动机械臂")
    with pytest.raises(ValueError):
        s.submit({"action": "reset"})
    task = s.execute_joint_chunk(["j"], [[0.1]], 4)
    for i in range(4):
        s.consume(1 / 120)
        s.publish((i + 1) / 120, [0.01 * i], [0], {"head": b.getvalue()} if i == 3 else None)
    result = s.stop_recording(ep["id"], True, "verified")
    assert result["error"] is None and result["samples"] == 4 and result["frames"] == 1
    with h5py.File(result["path"]) as f:
        assert f.attrs["complete"] and f.attrs["success"]
        assert f.attrs["instruction"] == "移动机械臂"
        assert f["state/positions"].shape == (4, 1)
        assert f["frames/step"][0] == 4
        assert Image.open(io.BytesIO(f["images/camera_0"][0].tobytes())).size == (8, 8)
        assert len(f["events/json"]) == 2
    assert s.get_task(task["id"])["status"] == "completed"


def test_recording_client_http(tmp_path):
    from robot_client import RobotClient
    from server import WebServer

    s = RobotServer()
    s.configure(["j"], [[-1, 1]], [0], [0], [])
    s.recording_dir = tmp_path
    w = WebServer(s, port=0, output_dir=tmp_path)
    w.start()
    try:
        r = RobotClient(w.url)
        e = r.start_recording("test")
        s.publish(0.01, [0], [0])
        assert r.recording_status()["id"] == e["id"]
        result = r.stop_recording(e["id"], False, "test_done")
        assert result["closed"] and result["error"] is None
    finally:
        w.stop()
