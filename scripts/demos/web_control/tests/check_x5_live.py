"""Opt-in X5 live smoke test: moves the running demo robot. Start run.py first."""

import argparse
import io
import json
import time
import urllib.request
import zipfile

from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8080")
    args = parser.parse_args()

    def request(path, body=None):
        data = None if body is None else json.dumps(body).encode()
        return urllib.request.urlopen(
            urllib.request.Request(args.url + path, data=data, headers={"Content-Type": "application/json"}), timeout=10
        )

    def status():
        return json.load(request("/api/status"))

    def command(action, **kwargs):
        return json.load(request("/api/command", {"action": action, **kwargs}))

    def wait_for(predicate, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            state = status()
            if predicate(state):
                return state
            time.sleep(0.1)
        raise AssertionError(f"Timed out. Last state: {state}")

    wait_for(lambda s: s["phase"] == "ready" and s["sequence"] > 0)
    sizes = {}
    for name in ("scene", "wrist"):
        image = Image.open(io.BytesIO(request(f"/api/camera/{name}.jpg").read()))
        sizes[name] = image.size
        assert image.convert("L").getextrema()[0] != image.convert("L").getextrema()[1]
    command("pause")
    wait_for(lambda s: s["paused"])
    time.sleep(0.2)
    before = status()["sim_time"]
    time.sleep(0.4)
    assert status()["sim_time"] == before
    command("step")
    after = wait_for(lambda s: s["sim_time"] > before)
    assert abs(after["sim_time"] - before - 1 / 120) < 1e-5
    command("reset")
    wait_for(lambda s: s["sim_time"] == 0 and s["paused"])
    command("joint", joint=0, value=0.3)
    command("resume")
    moved = wait_for(lambda s: abs(s["joints"][0]["actual"] - 0.3) < 0.03)
    command("gripper", open=False)
    wait_for(lambda s: s["grippers"][0]["actual"][0] < 0.005)
    command("gripper", open=True)
    wait_for(lambda s: s["grippers"][0]["actual"][0] > 0.038)
    snapshot = json.load(request("/api/snapshot", {}))
    with zipfile.ZipFile(io.BytesIO(request(snapshot["download"]).read())) as archive:
        assert set(archive.namelist()) == {"scene.jpg", "wrist.jpg", "metadata.json"}
        metadata = json.loads(archive.read("metadata.json"))
        assert metadata["sequence"] > 0 and len(metadata["positions"]) == 8
    command("pause")
    wait_for(lambda s: s["paused"])
    command("reset")
    wait_for(lambda s: s["sim_time"] == 0)
    print(
        json.dumps(
            {
                "result": "PASS",
                "camera_sizes": sizes,
                "joint_0_actual": moved["joints"][0]["actual"],
                "snapshot": snapshot["filename"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
