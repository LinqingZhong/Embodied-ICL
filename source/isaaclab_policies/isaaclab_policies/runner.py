"""Sequential observation -> prediction -> execution -> receipt policy loop."""

import logging
import math
import time

from .base import BasePolicy
from .robot_adapter import RobotAdapter
from .types import ActionResult, PolicyObservation, StopAction

logger = logging.getLogger(__name__)


class PolicyRunner:
    def __init__(
        self,
        policy: BasePolicy,
        robot: RobotAdapter,
        *,
        observation_timeout: float = 30,
        poll_interval: float = 0.05,
    ):
        if any(not math.isfinite(v) or v <= 0 for v in (observation_timeout, poll_interval)):
            raise ValueError("Observation timeout and poll interval must be positive and finite")
        self.policy = policy
        self.robot = robot
        self.observation_timeout = observation_timeout
        self.poll_interval = poll_interval

    def run(self, instruction: str, max_steps: int) -> list[ActionResult]:
        """Run at most max_steps predictions (not physics steps), always closing policy."""
        results = []
        try:
            if not isinstance(instruction, str) or not instruction.strip():
                raise ValueError("instruction must be nonempty text")
            if type(max_steps) is not int or max_steps <= 0:
                raise ValueError("max_steps must be a positive integer")
            self.robot.check_ready()
            self.policy.reset(instruction, self.robot.get_robot_spec())
            sequence, min_sim_time = 0, 0.0
            for step in range(max_steps):
                observation = self._wait_for_observation(sequence, min_sim_time)
                logger.info(
                    "Policy decision %d/%d, observation %d: predicting", step + 1, max_steps, observation.sequence
                )
                action = self.policy.predict(observation)
                logger.info("Executing %s", type(action).__name__)
                result = self.robot.execute(action)
                logger.info("Action %s: %s (%d physics steps)", result.task_id, result.status, result.executed_steps)
                results.append(result)
                self.policy.observe_result(result)
                if isinstance(action, StopAction):
                    break
                if result.status != "completed":
                    raise RuntimeError(f"Robot task {result.task_id} {result.status}; policy loop stopped")
                sequence = observation.sequence
                min_sim_time = result.finished_sim_time
            return results
        finally:
            self.policy.close()

    def _wait_for_observation(self, after_sequence: int, min_sim_time: float) -> PolicyObservation:
        deadline = time.monotonic() + self.observation_timeout
        while time.monotonic() < deadline:
            self.robot.check_ready()
            try:
                observation = self.robot.get_observation()
            except Exception as exc:
                if getattr(exc, "status", None) != 503:
                    raise
            else:
                if observation.sequence > after_sequence and observation.sim_time + 1e-8 >= min_sim_time:
                    return observation
            time.sleep(min(self.poll_interval, max(0, deadline - time.monotonic())))
        raise TimeoutError("Timed out waiting for a new observation captured after the last action")
