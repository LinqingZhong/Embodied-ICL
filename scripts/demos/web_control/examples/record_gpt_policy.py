"""Record real GPT actions with an explicit robot-camera whitelist and public response trace."""

import argparse
import copy
import json
import logging
import signal
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / "scripts/demos/web_control"), str(ROOT / "source/isaaclab_policies")]

from isaaclab_policies import GPTConfig, GPTPolicy, PolicyRunner, RobotAdapter
from robot_client import RobotClient


def save_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")


class CameraClient:
    """Filter both metadata and same-frame images; leave control calls untouched."""

    def __init__(self, client, cameras):
        self.client = client
        self.cameras = list(cameras)

    def get_robot_spec(self):
        spec = copy.deepcopy(self.client.get_robot_spec())
        if (
            not self.cameras
            or len(set(self.cameras)) != len(self.cameras)
            or "scene" in self.cameras
            or not set(self.cameras) <= set(spec["cameras"])
        ):
            raise ValueError("Choose unique existing robot cameras; scene camera is excluded")
        spec["cameras"] = self.cameras.copy()
        return spec

    def get_observation(self):
        obs = copy.deepcopy(self.client.get_observation())
        return replace(obs, images={name: obs.images[name] for name in self.cameras})

    def __getattr__(self, name):
        return getattr(self.client, name)


class RecordedGPTPolicy(GPTPolicy):
    def __init__(self, config, directory):
        super().__init__(config)
        self.directory = directory
        self.turn = 0
        self.results = []

    def event(self, event, **data):
        with (self.directory / "trace.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(
                json.dumps(
                    dict(event=event, wall_time=time.time(), turn=self.turn, **data),
                    ensure_ascii=False,
                    allow_nan=False,
                )
                + "\n"
            )

    def reset(self, instruction, robot):
        super().reset(instruction, robot)
        save_json(self.directory / "robot_spec.json", asdict(robot))
        (self.directory / "instructions.txt").write_text(self._instructions, encoding="utf-8")

    def predict(self, observation):
        self.turn += 1
        folder = self.directory / "observations" / f"{self.turn:03d}"
        folder.mkdir(parents=True)
        snapshot = asdict(observation)
        snapshot.pop("images")
        snapshot["cameras"] = list(observation.images)
        save_json(folder / "state.json", snapshot)
        for index, (name, jpeg) in enumerate(observation.images.items()):
            (folder / f"{index}_{name}.jpg").write_bytes(jpeg)
        self.event(
            "predict_start",
            observation_sequence=observation.sequence,
            observation_sim_time=observation.sim_time,
            cameras=list(observation.images),
            eef_poses=observation.eef_poses,
        )
        action = super().predict(observation)
        self.event("action", action_type=type(action).__name__, action=asdict(action))
        return action

    def _request(self, payload):
        names = [
            part["text"][8:]
            for item in payload["input"]
            if item.get("role") == "user"
            for part in item["content"]
            if part.get("type") == "input_text" and part.get("text", "").startswith("Camera: ")
        ]
        if "scene" in names:
            raise ValueError("Scene image leaked into model request")
        self.event(
            "model_request",
            model=payload["model"],
            endpoint=self.config.endpoint,
            reasoning_effort=self.config.reasoning_effort,
            input_items=len(payload["input"]),
            image_count=len(names),
            cameras=sorted(set(names)),
            tools=[tool["name"] for tool in payload["tools"]],
        )
        start = time.monotonic()
        response = super()._request(payload)
        outputs = [
            copy.deepcopy(item)
            for item in response.get("output", [])
            if item.get("type") in ("function_call", "message")
        ]
        self.event(
            "model_response",
            model=response.get("model", self.config.model),
            status=response.get("status"),
            elapsed_seconds=time.monotonic() - start,
            output=outputs,
            usage=response.get("usage"),
        )
        return response

    def observe_result(self, result):
        self.results.append(result)
        self.event("result", result=asdict(result))
        super().observe_result(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--url", default="http://127.0.0.1:8083")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--cameras", nargs="+", required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--max-steps", type=int, default=40)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "trace.jsonl").exists():
        raise FileExistsError("Use a new attempt directory")
    config = GPTConfig(**json.loads(args.config.read_text()))
    client = RobotClient(args.url, timeout=90)
    robot = RobotAdapter(CameraClient(client, args.cameras), action_timeout=180)
    robot.check_ready()
    spec = robot.get_robot_spec()
    policy = RecordedGPTPolicy(config, args.output)
    policy.event(
        "start",
        instruction=args.instruction,
        model=config.model,
        cameras=args.cameras,
        endpoint=config.endpoint,
        robot=spec.name,
    )
    recording = client.start_recording(args.instruction)
    error = None
    try:
        PolicyRunner(policy, robot, observation_timeout=30).run(args.instruction, args.max_steps)
    except (Exception, KeyboardInterrupt) as exc:
        error = f"{type(exc).__name__}: {exc}"
        policy.event("error", error=error)
        print(error, flush=True)
    finally:
        policy.close()
        policy.event("end", decisions=len(policy.results), error=error)
        time.sleep(3)
        stopped = client.stop_recording(recording["id"], False, error or "pending_independent_evaluation")
        report = dict(
            model=config.model,
            robot=spec.name,
            endpoint=config.endpoint,
            policy_config=asdict(config),
            cameras=args.cameras,
            instruction=args.instruction,
            max_steps=args.max_steps,
            recording=stopped,
            error=error,
            results=[asdict(result) for result in policy.results],
            evaluation={"success": False, "verdict": "尚未独立评估"},
        )
        save_json(args.output / "result.json", report)
        print(json.dumps({"recording": stopped, "error": error, "decisions": len(policy.results)}), flush=True)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    main()
