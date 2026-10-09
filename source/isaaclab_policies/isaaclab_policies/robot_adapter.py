"""Translate policy data to the existing RobotClient without simulator imports."""

import copy
import math
from contextlib import suppress
from dataclasses import fields
from typing import Protocol

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


class RobotClientProtocol(Protocol):
    """Structural interface satisfied by scripts/demos/web_control/robot_client.py."""

    def get_robot_spec(self) -> dict: ...
    def get_status(self) -> dict: ...
    def get_observation(self): ...
    def execute_joint_chunk(self, joint_names, positions, steps_per_action=1): ...
    def execute_eef_chunk(self, arm, positions, quaternions, steps_per_action=1): ...
    def wait(self, task_id, timeout=60, poll_interval=0.05): ...
    def cancel(self, task_id): ...


class RobotAdapter:
    def __init__(self, client: RobotClientProtocol, *, action_timeout: float = 120):
        if not math.isfinite(action_timeout) or action_timeout <= 0:
            raise ValueError("action_timeout must be positive and finite")
        self.client = client
        self.action_timeout = action_timeout
        self._robot_spec = None

    def get_robot_spec(self) -> RobotSpec:
        """Read native units and limits from the server; never import robot profiles."""
        spec = RobotSpec(**copy.deepcopy(self.client.get_robot_spec()))
        spec.validate()
        self._robot_spec = spec
        return copy.deepcopy(spec)

    def get_observation(self) -> PolicyObservation:
        """Keep only synchronized visual/proprioceptive fields, excluding object truth."""
        spec = self._robot_spec or self.get_robot_spec()
        raw = self.client.get_observation()
        observation = PolicyObservation(
            **{item.name: copy.deepcopy(getattr(raw, item.name)) for item in fields(PolicyObservation)}
        )
        observation.validate(spec)
        return observation

    def check_ready(self) -> None:
        """Require an idle, advancing simulator; no implicit resume or scene reset."""
        status = self.client.get_status()
        if status["phase"] != "ready" or status.get("paused") or status.get("active_task"):
            raise RuntimeError("Robot must be ready, unpaused, and have no active task")

    def execute(self, action: PolicyAction) -> ActionResult:
        """Submit once and return the terminal receipt; cancel only a confirmed task."""
        action = copy.deepcopy(action)
        validate_action(action, self._robot_spec or self.get_robot_spec())
        if isinstance(action, StopAction):
            return ActionResult(action, None, "stopped", 0, None, {"reason": action.reason})
        self.check_ready()
        # Deliberately outside the cancellation block: a timed-out submission has
        # an unknown outcome, so guessing the active task's ownership is unsafe.
        task = self._submit(action)
        try:
            terminal = self.client.wait(task.id, timeout=self.action_timeout)
        except BaseException:
            # Preserve the original failure. Never cancel a different task.
            with suppress(Exception):
                self.client.cancel(task.id)
            raise
        details = copy.deepcopy(terminal.details)
        return ActionResult(
            action,
            terminal.id,
            terminal.status,
            details["executed_steps"],
            details["finished_sim_time"],
            details,
        )

    def _submit(self, action: JointActionChunk | EefActionChunk):
        if isinstance(action, JointActionChunk):
            return self.client.execute_joint_chunk(action.joint_names, action.positions, action.steps_per_action)
        return self.client.execute_eef_chunk(
            action.arm,
            action.positions,
            action.quaternions,
            action.steps_per_action,
        )
