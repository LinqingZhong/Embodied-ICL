"""Opt-in live check: moves A2D joints and grippers on the specified running panel."""

import argparse
import io
import json
import time
import urllib.request
import zipfile

from PIL import Image

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--url", default="http://127.0.0.1:8083")
args = parser.parse_args()


def request(path, data=None):
    body = None if data is None else json.dumps(data).encode()
    return urllib.request.urlopen(
        urllib.request.Request(args.url + path, data=body, headers={"Content-Type": "application/json"}), timeout=10
    )


def status():
    return json.load(request("/api/status"))


def command(action, **kwargs):
    return json.load(request("/api/command", {"action": action, **kwargs}))


def wait(predicate):
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        current = status()
        if predicate(current):
            return current
        time.sleep(0.15)
    raise AssertionError(current)


initial = wait(lambda s: s["phase"] == "ready" and s["sequence"] > 0)
assert "Cobot" in initial["robot"] and len(initial["joints"]) == 26
assert len(initial["cameras"]) == 4
for camera in initial["cameras"]:
    img = Image.open(io.BytesIO(request(f"/api/camera/{camera['name']}.jpg").read()))
    low, high = img.convert("L").getextrema()
    assert high - low > 20, camera
command("reset")
command("resume")
for name, delta in [
    ("camera_stand_1_joint", 0.01),
    ("camera_stand_2_joint", 0.10),
    ("fl_joint1", 0.10),
    ("bl_joint1", 0.10),
    ("fr_joint1", -0.10),
    ("br_joint1", -0.10),
]:
    joint = next(j for j in status()["joints"] if j["name"] == name)
    target = joint["target"] + delta
    command("joint", joint=joint["index"], value=target)
    result = wait(
        lambda s: (
            abs(next(j for j in s["joints"] if j["name"] == name)["actual"] - target)
            < (0.004 if joint["unit"] == "m" else 0.05)
        )
    )
    print("JOINT_PASS", name, flush=True)
for name in ("fl", "fr", "bl", "br"):
    command("gripper", gripper=name, open=False)
    wait(lambda s: next(g for g in s["grippers"] if g["name"] == name)["actual"][0] < 0.008)
    command("gripper", gripper=name, open=True)
    wait(lambda s: next(g for g in s["grippers"] if g["name"] == name)["actual"][0] > 0.032)
    print("GRIPPER_PASS", name, flush=True)
command("pause")
wait(lambda s: s["paused"])
command("reset")
wait(lambda s: s["sim_time"] == 0)
time.sleep(0.3)
result = json.load(request("/api/snapshot", {}))
with zipfile.ZipFile(io.BytesIO(request(result["download"]).read())) as archive:
    assert set(archive.namelist()) == {"scene.jpg", "head.jpg", "fl_wrist.jpg", "fr_wrist.jpg", "metadata.json"}
print("COBOT_LIVE_PASS", flush=True)
