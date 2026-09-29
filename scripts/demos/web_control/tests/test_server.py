import io
import json
import sys
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import WebServer
from state import ControlState


@pytest.fixture
def running(tmp_path):
    state = ControlState()
    state.configure(["joint", "finger"], [(-1, 1), (0, 0.04)], [0, 0.04], [0], [1])
    server = WebServer(state, port=0, output_dir=tmp_path)
    server.start()
    yield state, server
    server.stop()
    assert not server.thread.is_alive()


def request(server, path, payload=None, origin=None):
    headers = {}
    data = None
    if payload is not None:
        data = json.dumps(payload).encode()
        headers["Content-Type"] = "application/json"
    if origin:
        headers["Origin"] = origin
    return urllib.request.urlopen(urllib.request.Request(server.url + path, data=data, headers=headers), timeout=5)


def test_http_status_validation_and_origin(running):
    state, server = running
    assert json.load(request(server, "/api/status"))["phase"] == "ready"
    for payload in ({"action": "joint", "joint": 0, "value": 4}, [], {"action": "bad"}):
        with pytest.raises(urllib.error.HTTPError) as error:
            request(server, "/api/command", payload)
        assert error.value.code == 400
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, "/api/command", {"action": "reset"}, "https://evil.example")
    assert error.value.code == 403
    assert json.load(request(server, "/api/command", {"action": "joint", "joint": 0, "value": 0.2}))["ok"]
    assert state.status()["joints"][0]["target"] == 0.2


def test_unavailable_frames_and_snapshot_download(running):
    state, server = running
    for path in ("/api/camera/scene.jpg", "/api/stream/scene"):
        with pytest.raises(urllib.error.HTTPError) as error:
            request(server, path)
        assert error.value.code == 503
    image = io.BytesIO()
    Image.new("RGB", (8, 8), "red").save(image, format="JPEG")
    state.publish(2.5, [0.1, 0.03], [0, 0], {"scene": image.getvalue(), "wrist": image.getvalue()})
    assert Image.open(io.BytesIO(request(server, "/api/camera/wrist.jpg").read())).size == (8, 8)
    result = json.load(request(server, "/api/snapshot", {}))
    with zipfile.ZipFile(io.BytesIO(request(server, result["download"]).read())) as archive:
        assert set(archive.namelist()) == {"scene.jpg", "wrist.jpg", "metadata.json"}
        metadata = json.loads(archive.read("metadata.json"))
        assert metadata["sim_time"] == 2.5 and metadata["positions"] == [0.1, 0.03]
    with pytest.raises(urllib.error.HTTPError):
        request(server, "/api/download/not-a-snapshot.zip")


def test_port_conflict_is_reported(running, tmp_path):
    state, server = running
    with pytest.raises(OSError):
        WebServer(state, port=server.port, output_dir=tmp_path).start()


def test_shutdown_with_streaming_client(running):
    state, server = running
    state.publish(0, [0, 0.04], [0, 0], {"scene": b"jpeg", "wrist": b"jpeg"})
    stream = request(server, "/api/stream/scene")
    assert stream.read(7) == b"--frame"
    server.stop()
    assert not server.thread.is_alive()
    stream.close()


def test_configured_cameras_and_multicamera_snapshot(running):
    state, server = running
    with state.lock:
        state.cameras = [{"name": name, "label": name} for name in ("scene", "head", "left_wrist", "right_wrist")]
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, "/api/camera/head.jpg")
    assert error.value.code == 503
    state.publish(1, [0, 0.04], [0, 0], {c["name"]: b"jpeg" for c in state.cameras})
    assert request(server, "/api/camera/head.jpg").read() == b"jpeg"
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, "/api/camera/wrist.jpg")
    assert error.value.code == 404
    result = json.load(request(server, "/api/snapshot", {}))
    with zipfile.ZipFile(io.BytesIO(request(server, result["download"]).read())) as archive:
        assert set(archive.namelist()) == {
            "scene.jpg",
            "head.jpg",
            "left_wrist.jpg",
            "right_wrist.jpg",
            "metadata.json",
        }
