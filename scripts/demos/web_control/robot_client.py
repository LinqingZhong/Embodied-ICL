"""Remote RobotServer client; standard-library only, no Isaac Sim dependency."""

import base64
import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass


class RobotError(RuntimeError):
    def __init__(self, status, detail):
        super().__init__(f"RobotServer HTTP {status}: {detail}")
        self.status = status
        self.detail = detail


@dataclass(frozen=True)
class Observation:
    sequence: int
    sim_time: float
    captured_at: float
    joint_names: list[str]
    positions: list[float]
    velocities: list[float]
    targets: list[float]
    images: dict[str, bytes]


@dataclass(frozen=True)
class ChunkTask:
    id: str
    status: str
    details: dict


class RobotClient:
    def __init__(self, url="http://127.0.0.1:8080", timeout=10):
        self.url = url.rstrip("/")
        self.timeout = timeout

    def _request(self, path, payload=None, *, timeout=None):
        data = None if payload is None else json.dumps(payload, allow_nan=False).encode()
        req = urllib.request.Request(self.url + path, data=data, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout if timeout is None else timeout) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            try:
                detail = json.load(exc).get("detail", str(exc))
            except (ValueError, AttributeError):
                detail = str(exc)
            raise RobotError(exc.code, detail) from exc

    def get_joint_state(self):
        return self._request("/api/robot/joints")

    def get_observation(self):
        data = self._request("/api/robot/observation")
        return Observation(
            **{
                key: data[key]
                for key in ("sequence", "sim_time", "captured_at", "joint_names", "positions", "velocities", "targets")
            },
            images={name: base64.b64decode(value) for name, value in data["images"].items()},
        )

    @staticmethod
    def _task(data):
        return ChunkTask(data["id"], data["status"], data)

    def execute_joint_chunk(self, joint_names, positions, steps_per_action=1):
        """Send absolute targets [T,N]. No retries: an ambiguous timeout may have accepted the task.

        Inspect get_joint_state()['active_task'] before retrying a timed-out submission.
        """
        if hasattr(positions, "tolist"):
            positions = positions.tolist()
        return self._task(
            self._request(
                "/api/robot/chunks",
                {"joint_names": list(joint_names), "positions": positions, "steps_per_action": steps_per_action},
            )
        )

    def get_task(self, task_id):
        return self._task(self._request(f"/api/robot/chunks/{task_id}"))

    def cancel(self, task_id):
        """Hold last commanded targets; this is not an instantaneous physical stop."""
        return self._task(self._request(f"/api/robot/chunks/{task_id}/cancel", {}))

    def wait(self, task_id, timeout=60, poll_interval=0.05):
        """Wait in wall time. Timeout does not cancel the server task."""
        if timeout <= 0 or poll_interval <= 0:
            raise ValueError("timeout and poll_interval must be positive")
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Chunk {task_id} still running; timeout does not cancel execution")
            task = self._task(self._request(f"/api/robot/chunks/{task_id}", timeout=min(self.timeout, remaining)))
            if task.status != "running":
                return task
            time.sleep(min(poll_interval, max(0, deadline - time.monotonic())))

    def get_arms(self):
        return self._request("/api/robot/arms")

    def fk(self, arm, joint_positions=None):
        return self._request("/api/robot/fk", dict(arm=arm, joint_positions=joint_positions))

    def ik(self, arm, position, quaternion, seed=None):
        return self._request("/api/robot/ik", dict(arm=arm, position=position, quaternion=quaternion, seed=seed))

    def execute_eef_chunk(self, arm, positions, quaternions, steps_per_action=1):
        return self._task(
            self._request(
                "/api/robot/eef/chunks",
                dict(arm=arm, positions=positions, quaternions=quaternions, steps_per_action=steps_per_action),
            )
        )

    def start_recording(self, instruction):
        return self._request("/api/robot/recording/start", {"instruction": instruction})

    def stop_recording(self, episode_id, success=False, reason="user_stop"):
        return self._request(
            "/api/robot/recording/stop", dict(episode_id=episode_id, success=success, reason=reason), timeout=40
        )

    def recording_status(self):
        return self._request("/api/robot/recording")
