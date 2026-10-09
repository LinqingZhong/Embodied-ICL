"""GPT Responses policy: multimodal observations, typed actions and execution receipts."""

import base64
import copy
import http.client
import json
import math
import os
import urllib.error
import urllib.request
from collections import deque
from dataclasses import asdict, dataclass, field
from urllib.parse import urlparse

from .base import BasePolicy
from .types import (
    ActionResult,
    EefActionChunk,
    JointActionChunk,
    PolicyAction,
    PolicyObservation,
    RobotSpec,
    StopAction,
    validate_action,
)


@dataclass(frozen=True)
class GPTConfig:
    """Explicit settings override ASTRA_* environment defaults. No credential values."""

    model: str = field(default_factory=lambda: os.environ.get("ASTRA_MODEL") or "gpt-6-astra")
    endpoint: str = field(
        default_factory=lambda: os.environ.get("ASTRA_ENDPOINT") or "https://api.openai.com/v1/responses"
    )
    api_key_env: str | None = None
    reasoning_effort: str = field(default_factory=lambda: os.environ.get("ASTRA_REASONING_EFFORT") or "xhigh")
    max_output_tokens: int | None = 16384
    timeout: float = 240
    max_chunk_size: int = 32
    history_steps: int = 4
    preserve_reasoning: bool = True

    def __post_init__(self):
        parsed = urlparse(self.endpoint)
        if (
            parsed.scheme not in ("http", "https")
            or not parsed.netloc
            or not parsed.path.rstrip("/").endswith("/responses")
            or parsed.username
            or parsed.password
        ):
            raise ValueError("endpoint must be an absolute HTTP(S) /responses URL without credentials")
        if not isinstance(self.model, str) or not self.model.strip():
            raise ValueError("model must be nonempty")
        if self.reasoning_effort not in ("none", "minimal", "low", "medium", "high", "xhigh"):
            raise ValueError("Unsupported reasoning_effort")
        if type(self.timeout) not in (int, float) or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        if type(self.max_chunk_size) is not int or not 1 <= self.max_chunk_size <= 2048:
            raise ValueError("max_chunk_size must be an integer in [1, 2048]")
        if type(self.history_steps) is not int or not 0 <= self.history_steps <= 32:
            raise ValueError("history_steps must be an integer in [0, 32]")
        if type(self.preserve_reasoning) is not bool:
            raise ValueError("preserve_reasoning must be a boolean")
        if self.max_output_tokens is not None and (
            type(self.max_output_tokens) is not int or self.max_output_tokens < 1
        ):
            raise ValueError("max_output_tokens must be a positive integer or null")
        if self.api_key_env is not None and (not isinstance(self.api_key_env, str) or not self.api_key_env.strip()):
            raise ValueError("api_key_env must name an environment variable")


INSTRUCTIONS = """You control a simulated robot using only RGB images and measured robot state.
Follow the task instruction and reobserve after each short action chunk. Use exactly one tool per turn.
Joint targets are ABSOLUTE values in each joint's native unit (rad or m), in the explicit joint_names column order.
Grippers are ordinary named joints: use the supplied open/close maps, including coupled joints and opposite signs.
EEF positions are ABSOLUTE flange positions in the selected arm's BASE frame, in metres; quaternions are unit wxyz.
EEF targets refer to the flange, not the finger contact centre. They are NOT world coordinates or delta poses.
Use images and the supplied base/flange definitions; do not assume calibrated camera extrinsics are available.
Each row is held for steps_per_action * physics_dt simulation seconds. Targets are NOT automatically interpolated.
Generate small, smooth changes and enough intermediate points; only named joints change, others retain their targets.
There is no automatic collision planning. EEF IK convergence and completed chunks do not guarantee physical arrival.
Use measured positions and execution errors to evaluate progress, not only previously commanded targets.
finish ends this policy loop and leaves the robot holding its last targets; it is not a measured success signal.
No world object positions, rewards or success ground truth are available.
Do not claim a grasp from tool completion alone.
"""


class GPTPolicy(BasePolicy):
    def __init__(self, config: GPTConfig | None = None):
        self.config = config or GPTConfig()
        key_name = self.config.api_key_env
        if key_name is None:
            key_name = "ASTRA_API_KEY" if os.environ.get("ASTRA_API_KEY") else "OPENAI_API_KEY"
        self._api_key = os.environ.get(key_name)
        if not self._api_key:
            raise RuntimeError(f"Set {key_name} before creating GPTPolicy")
        self._robot = None
        self._pending = None
        self._history = deque(maxlen=self.config.history_steps)
        self._seen_call_ids = set()
        self._finished = False

    def reset(self, instruction: str, robot: RobotSpec) -> None:
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError("instruction must be nonempty text")
        robot.validate()
        self._robot = copy.deepcopy(robot)
        self._instructions = (
            INSTRUCTIONS
            + "\nTask: "
            + instruction
            + "\nRobotSpec: "
            + json.dumps(
                asdict(robot),
                ensure_ascii=False,
                allow_nan=False,
            )
        )
        self._history.clear()
        self._seen_call_ids.clear()
        self._pending = None
        self._finished = False

    def predict(self, observation: PolicyObservation) -> PolicyAction:
        if self._robot is None or self._finished:
            raise RuntimeError("Call reset before starting a new policy episode")
        if self._pending is not None:
            raise RuntimeError("Execution feedback is required before the next prediction")
        observation.validate(self._robot)
        message = self._observation_message(observation)
        response = self._request(self._build_payload(message))
        action, call_id, outputs = self._parse_response(response)
        self._pending = dict(action=copy.deepcopy(action), call_id=call_id, items=[message, *outputs])
        self._seen_call_ids.add(call_id)
        return action

    def observe_result(self, result: ActionResult) -> None:
        if self._pending is None or result.action != self._pending["action"]:
            raise ValueError("Execution feedback does not match the pending action")
        stopped = isinstance(result.action, StopAction)
        allowed = ("stopped",) if stopped else ("completed", "cancelled", "failed")
        if result.status not in allowed or type(result.executed_steps) is not int or result.executed_steps < 0:
            raise ValueError("Expected a terminal execution result")
        receipt = {
            "task_id": result.task_id,
            "status": result.status,
            "executed_steps": result.executed_steps,
            "finished_sim_time": result.finished_sim_time,
            "feedback": {
                key: result.details[key]
                for key in (
                    "joint_names",
                    "actual_positions",
                    "targets",
                    "errors",
                    "reason",
                )
                if key in result.details
            },
        }
        output = dict(
            type="function_call_output",
            call_id=self._pending["call_id"],
            output=json.dumps(receipt, ensure_ascii=False, allow_nan=False),
        )
        self._history.append([*self._pending["items"], output])
        self._pending = None
        self._finished = stopped or result.status != "completed"

    def close(self) -> None:
        self._history.clear()
        self._pending = None
        self._robot = None
        self._finished = True

    def _observation_message(self, observation: PolicyObservation) -> dict:
        state = {
            key: copy.deepcopy(getattr(observation, key))
            for key in (
                "sequence",
                "sim_time",
                "captured_at",
                "joint_names",
                "positions",
                "velocities",
                "targets",
                "eef_poses",
            )
        }
        content = [{"type": "input_text", "text": json.dumps(state, ensure_ascii=False, allow_nan=False)}]
        for name in self._robot.cameras:
            content.append({"type": "input_text", "text": f"Camera: {name}"})
            content.append(
                {
                    "type": "input_image",
                    "detail": "high",
                    "image_url": "data:image/jpeg;base64," + base64.b64encode(observation.images[name]).decode("ascii"),
                }
            )
        return {"role": "user", "content": content}

    def _build_payload(self, message: dict) -> dict:
        payload = dict(
            model=self.config.model,
            instructions=self._instructions,
            input=[item for turn in self._history for item in turn] + [message],
            tools=self._tools(),
            tool_choice="required",
            parallel_tool_calls=False,
            store=False,
            stream=False,
            reasoning={"effort": self.config.reasoning_effort},
        )
        if self.config.preserve_reasoning:
            payload["include"] = ["reasoning.encrypted_content"]
        if self.config.max_output_tokens is not None:
            payload["max_output_tokens"] = self.config.max_output_tokens
        return payload

    def _tools(self) -> list[dict]:
        steps = {
            "type": "integer",
            "minimum": 1,
            "maximum": 120000,
            "description": "Physics steps per row; duration = steps_per_action * physics_dt.",
        }
        tools = [
            _tool(
                "execute_joint_chunk",
                "Apply absolute native-unit joint targets, including gripper joints.",
                {
                    "joint_names": {
                        "type": "array",
                        "minItems": 1,
                        "items": {"type": "string", "enum": self._robot.controllable_joint_names},
                    },
                    "positions": _rows(self.config.max_chunk_size),
                    "steps_per_action": steps,
                },
            )
        ]
        if self._robot.arms:
            limit = min(self.config.max_chunk_size, 128)
            tools.append(
                _tool(
                    "execute_eef_chunk",
                    "Apply absolute flange poses in one arm's BASE frame (m, wxyz).",
                    {
                        "arm": {"type": "string", "enum": list(self._robot.arms)},
                        "positions": _rows(limit, 3),
                        "quaternions": _rows(limit, 4),
                        "steps_per_action": steps,
                    },
                )
            )
        tools.append(
            _tool(
                "finish",
                "End the policy episode; this does not certify task success.",
                {
                    "reason": {"type": "string", "minLength": 1, "maxLength": 2000},
                },
            )
        )
        return tools

    def _parse_response(self, response: dict) -> tuple[PolicyAction, str, list[dict]]:
        if (
            not isinstance(response, dict)
            or response.get("status") != "completed"
            or response.get("error")
            or response.get("incomplete_details") is not None
        ):
            raise RuntimeError("GPT response did not complete; no action dispatched")
        outputs = response.get("output")
        if not isinstance(outputs, list) or any(not isinstance(item, dict) for item in outputs):
            raise ValueError("Invalid Responses output")
        if any(item.get("status") not in (None, "completed") for item in outputs):
            raise ValueError("Responses output contains an incomplete item; no action dispatched")
        calls = [item for item in outputs if item.get("type") == "function_call"]
        if len(calls) != 1:
            raise ValueError("Expected exactly one action tool call")
        call = calls[0]
        call_id = call.get("call_id")
        if not isinstance(call_id, str) or not call_id.strip() or call_id in self._seen_call_ids:
            raise ValueError("Missing or reused action call_id")
        constructors = {
            "execute_joint_chunk": JointActionChunk,
            "execute_eef_chunk": EefActionChunk,
            "finish": StopAction,
        }
        schemas = {tool["name"]: tool["parameters"] for tool in self._tools()}
        name = call.get("name")
        if not isinstance(name, str) or name not in schemas:
            raise ValueError("Unknown action tool")
        try:
            arguments = json.loads(call["arguments"])
        except (KeyError, TypeError, ValueError):
            raise ValueError("Action tool arguments must be valid JSON") from None
        if not isinstance(arguments, dict) or set(arguments) != set(schemas[name]["required"]):
            raise ValueError("Unexpected action tool fields")
        action = constructors[name](**arguments)
        validate_action(action, self._robot, self.config.max_chunk_size)
        # Function output item IDs are response metadata; call_id pairs the receipt.
        # Some gateways cannot verify their own encrypted reasoning on the next call.
        # Explicitly disabling replay still retains observations, actions and receipts.
        history = copy.deepcopy(
            [item for item in outputs if self.config.preserve_reasoning or item.get("type") != "reasoning"]
        )
        for item in history:
            if item.get("type") == "function_call":
                item.pop("id", None)
                item.pop("status", None)
        return action, call_id, history

    def _request(self, payload: dict) -> dict:
        request = urllib.request.Request(
            self.config.endpoint,
            data=json.dumps(payload, allow_nan=False).encode("utf-8"),
            headers={"Authorization": f"Bearer {self._api_key}", "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.config.timeout) as reply:
                return json.load(reply)
        except urllib.error.HTTPError as exc:
            status = exc.code
            exc.close()
            raise RuntimeError(f"GPT API HTTP {status}; no action dispatched") from None
        except (urllib.error.URLError, http.client.HTTPException, TimeoutError, ConnectionError, OSError):
            raise RuntimeError("GPT API transport failed; no action dispatched") from None
        except (ValueError, UnicodeError):
            raise RuntimeError("GPT API returned invalid JSON; no action dispatched") from None


def _tool(name: str, description: str, properties: dict) -> dict:
    return dict(
        type="function",
        name=name,
        description=description,
        strict=True,
        parameters={
            "type": "object",
            "properties": properties,
            "required": list(properties),
            "additionalProperties": False,
        },
    )


def _rows(limit: int, width: int | None = None) -> dict:
    row = {"type": "array", "items": {"type": "number"}, "minItems": width or 1}
    if width is not None:
        row["maxItems"] = width
    return {"type": "array", "minItems": 1, "maxItems": limit, "items": row}
