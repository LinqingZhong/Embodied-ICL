"""Implement one subclass per policy module; model dependencies belong there."""

from abc import ABC, abstractmethod

from .types import ActionResult, PolicyAction, PolicyObservation, RobotSpec


class BasePolicy(ABC):
    @abstractmethod
    def reset(self, instruction: str, robot: RobotSpec) -> None:
        """Start an episode's policy context without resetting or moving the robot."""

    @abstractmethod
    def predict(self, observation: PolicyObservation) -> PolicyAction:
        """Produce an action only. RobotAdapter owns all dispatch and waiting."""

    def observe_result(self, result: ActionResult) -> None:
        """Receive actual execution feedback; stateless policies may ignore it."""

    def close(self) -> None:
        """Release policy resources, also called when inference or execution fails."""
