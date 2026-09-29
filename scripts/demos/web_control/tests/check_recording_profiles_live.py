"""Record and replay the stored joint chunk after reset; compare state traces."""

import argparse
import io
import json
import sys
import time
from pathlib import Path

import h5py
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_client import RobotClient

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--robot", required=True)
parser.add_argument("--output", required=True)
args = parser.parse_args()
r = RobotClient("http://127.0.0.1:8083", timeout=60)


def reset():
    r._request("/api/command", {"action": "reset"})
    time.sleep(0.8)
    s = r.get_joint_state()
    assert s["active_task"] is None
    return s


def record(names, rows):
    e = r.start_recording("录制与重放小幅关节序列验证")
    t = r.execute_joint_chunk(names, rows, 12)
    assert r.wait(t.id, timeout=90).status == "completed"
    result = r.stop_recording(e["id"], False, "replay_validation")
    assert result["error"] is None
    return result["path"]


s = reset()
names = [n for n in s["controllable_joint_names"] if n.endswith("joint1") and "hand" not in n and "finger" not in n]
q = [s["positions"][s["joint_names"].index(n)] for n in names]
rows = [[v + 0.04 * i / 10 for v in q] for i in range(11)]
original = record(names, rows)
with h5py.File(original) as f:
    events = [json.loads(x) for x in f["events/json"].asstr()[:]]
    request = next(e["task"] for e in events if e["type"] == "joint_chunk")
reset()
replayed = record(request["joint_names"], request["rows"])


def inspect(path):
    with h5py.File(path) as f:
        assert f.attrs["complete"]
        ts = f["frames/sim_time"][:]
        assert np.allclose(np.diff(ts), 1 / 30)
        for ds in f["images"].values():
            assert len(ds) == len(ts)
            for raw in ds:
                Image.open(io.BytesIO(raw.tobytes())).load()
        st = f["state/step"][:]
        assert np.all(np.diff(st) == 1)
        assert set(f["frames/step"][:]) <= set(st)
        targets = f["state/targets"][:]
        positions = f["state/positions"][:]
        velocities = f["state/velocities"][:]
        return dict(frames=len(ts), cameras=len(f["images"]), samples=len(st)), targets, positions, velocities


a, at, aq, av = inspect(original)
b, bt, bq, bv = inspect(replayed)
# Align at the first changed target (row 1), independent of HTTP/start scheduling.
joint_names = s["joint_names"]
ids = [joint_names.index(n) for n in names]


def start(targets):
    hits = np.flatnonzero(np.all(np.isclose(targets[:, ids], rows[1], atol=1e-10, rtol=0), axis=1))
    assert len(hits)
    return int(hits[0])


ia, ib = start(at), start(bt)
count = 120
assert len(aq) - ia >= count and len(bq) - ib >= count
assert np.array_equal(at[ia : ia + count, ids], bt[ib : ib + count, ids])
error = np.abs(aq[ia : ia + count] - bq[ib : ib + count])
report = dict(
    robot=args.robot,
    original=original,
    replayed=replayed,
    original_info=a,
    replay_info=b,
    compared_steps=count,
    target_sequence_exact=True,
    state_bitwise_identical=bool(np.array_equal(aq[ia : ia + count], bq[ib : ib + count])),
    max_joint_position_difference=float(error.max()),
    rms_joint_position_difference=float(np.sqrt((error**2).mean())),
    max_joint_velocity_difference=float(np.max(np.abs(av[ia : ia + count] - bv[ib : ib + count]))),
)
Path(args.output).write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report), flush=True)
