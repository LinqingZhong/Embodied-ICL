"""Run a JSON joint/EEF action manifest while recording one HDF5 episode."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_client import RobotClient

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("manifest", help="JSON with instruction and actions list")
parser.add_argument("--url", default="http://127.0.0.1:8080")
args = parser.parse_args()
manifest = json.loads(Path(args.manifest).read_text())
robot = RobotClient(args.url, timeout=90)
episode = robot.start_recording(manifest["instruction"])
reason = "execution_error"
try:
    for action in manifest["actions"]:
        action = dict(action)
        kind = action.pop("type")
        if kind not in ("joint", "eef"):
            raise ValueError("Action type must be joint or eef")
        task = (robot.execute_joint_chunk if kind == "joint" else robot.execute_eef_chunk)(**action)
        result = robot.wait(task.id, timeout=300)
        if result.status != "completed":
            raise RuntimeError(str(result.details))
        status = robot.recording_status()
        if status["error"]:
            raise RuntimeError(status["error"])
    reason = "actions_completed_task_success_unverified"
finally:
    # Execution completion alone is not a semantic task-success label.
    result = robot.stop_recording(episode["id"], success=False, reason=reason)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result["error"]:
        raise RuntimeError(result["error"])
