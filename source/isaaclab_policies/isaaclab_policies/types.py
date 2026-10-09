"""Public policy data contract. Joint units and EEF frames come from RobotSpec."""

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class RobotSpec:
    """Static metadata; limits align with all joint_names, including passive joints."""

    name: str
    joint_names: list[str]
    controllable_joint_names: list[str]
    limits: list[list[float | None]]
    joint_units: dict[str, str]
    grippers: list[dict]
    arms: dict[str, dict]
    cameras: list[str]
    physics_dt: float

    def validate(self) -> None:
        _names(self.joint_names, "joint_names")
        _names(self.controllable_joint_names, "controllable_joint_names")
        _names(self.cameras, "cameras")
        if len(self.limits) != len(self.joint_names):
            raise ValueError("limits must align with joint_names")
        if not set(self.controllable_joint_names) <= set(self.joint_names):
            raise ValueError("Unknown controllable joint")
        if not _finite(self.physics_dt) or self.physics_dt <= 0:
            raise ValueError("physics_dt must be positive and finite")
        for name in self.controllable_joint_names:
            bounds = self.limits[self.joint_names.index(name)]
            if len(bounds) != 2 or not all(_finite(v) for v in bounds) or bounds[0] > bounds[1]:
                raise ValueError(f"Invalid controllable joint limits: {name}")
            if self.joint_units.get(name) not in ("rad", "m"):
                raise ValueError(f"Unknown native joint unit: {name}")


@dataclass(frozen=True)
class PolicyObservation:
    """A single capture: JPEG images, joints and measured flange poses at sim_time."""

    sequence: int
    sim_time: float
    captured_at: float
    joint_names: list[str]
    positions: list[float]
    velocities: list[float]
    targets: list[float]
    images: dict[str, bytes]
    eef_poses: dict[str, dict] = field(default_factory=dict)

    def validate(self, robot: RobotSpec) -> None:
        if type(self.sequence) is not int or self.sequence < 1:
            raise ValueError("Observation requires a positive capture sequence")
        if not _finite(self.sim_time) or self.sim_time < 0 or not _finite(self.captured_at):
            raise ValueError("Invalid observation timestamp")
        if self.joint_names != robot.joint_names:
            raise ValueError("Observation joint_names do not match RobotSpec")
        for name in ("positions", "velocities", "targets"):
            _vector(getattr(self, name), len(robot.joint_names), name)
        if set(self.images) != set(robot.cameras):
            raise ValueError("Observation cameras do not match RobotSpec")
        if any(not isinstance(data, bytes) or not data for data in self.images.values()):
            raise ValueError("Observation requires nonempty JPEG bytes")
        if set(self.eef_poses) != set(robot.arms):
            raise ValueError("Missing same-frame eef_poses; restart the updated simulation server")
        for pose in self.eef_poses.values():
            _vector(pose["position"], 3, "EEF position")
            _quaternion(pose["quaternion"])


@dataclass(frozen=True)
class JointActionChunk:
    """Absolute native-unit targets [T,N], held steps_per_action physics steps each."""

    joint_names: list[str]
    positions: list[list[float]]
    steps_per_action: int = 1


@dataclass(frozen=True)
class EefActionChunk:
    """Absolute flange poses in the selected arm base frame, metres and wxyz."""

    arm: str
    positions: list[list[float]]
    quaternions: list[list[float]]
    steps_per_action: int = 1


@dataclass(frozen=True)
class StopAction:
    """Stop inference; the reason is a policy statement, not a measured success."""

    reason: str


PolicyAction = JointActionChunk | EefActionChunk | StopAction


@dataclass(frozen=True)
class ActionResult:
    """Execution receipt. completed means elapsed physics steps, not convergence."""

    action: PolicyAction
    task_id: str | None
    status: str
    executed_steps: int
    finished_sim_time: float | None
    details: dict = field(default_factory=dict)


def validate_action(action: PolicyAction, robot: RobotSpec, max_chunk_size: int = 2048) -> None:
    """Validate the shared boundary before any model or local policy can dispatch."""
    if isinstance(action, StopAction):
        if not isinstance(action.reason, str) or not action.reason.strip() or len(action.reason) > 2000:
            raise ValueError("Stop reason must be nonempty text, at most 2000 characters")
        return
    if not isinstance(action, (JointActionChunk, EefActionChunk)):
        raise ValueError("Expected JointActionChunk, EefActionChunk or StopAction")
    if type(action.steps_per_action) is not int or not 1 <= action.steps_per_action <= 120000:
        raise ValueError("steps_per_action must be an integer in [1, 120000]")
    limit = min(max_chunk_size, 128 if isinstance(action, EefActionChunk) else 2048)
    if not isinstance(action.positions, list) or not 1 <= len(action.positions) <= limit:
        raise ValueError(f"positions must contain 1 to {limit} rows")
    if isinstance(action, JointActionChunk):
        _names(action.joint_names, "joint_names")
        if not set(action.joint_names) <= set(robot.controllable_joint_names):
            raise ValueError("Unknown or passive joint")
        bounds = [robot.limits[robot.joint_names.index(name)] for name in action.joint_names]
        for row in action.positions:
            _vector(row, len(action.joint_names), "Joint position")
            for name, value, (low, high) in zip(action.joint_names, row, bounds):
                if not low <= value <= high:
                    raise ValueError(f"Target outside limits: {name}")
    else:
        if not isinstance(action.arm, str) or action.arm not in robot.arms:
            raise ValueError("Unknown arm or EEF control unsupported")
        if not isinstance(action.quaternions, list) or len(action.quaternions) != len(action.positions):
            raise ValueError("Expected matching positions and quaternions")
        for position, quaternion in zip(action.positions, action.quaternions):
            _vector(position, 3, "EEF position")
            _quaternion(quaternion)


def _finite(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _names(names, label) -> None:
    if not isinstance(names, list) or not names or any(not isinstance(n, str) or not n for n in names):
        raise ValueError(f"{label} must contain nonempty names")
    if len(names) != len(set(names)):
        raise ValueError(f"Duplicate {label}")


def _vector(values, size, label) -> None:
    if not isinstance(values, list) or len(values) != size or not all(_finite(v) for v in values):
        raise ValueError(f"{label} requires {size} finite numbers")


def _quaternion(values) -> None:
    _vector(values, 4, "Quaternion")
    if abs(math.sqrt(sum(v * v for v in values)) - 1) > 0.001:
        raise ValueError("Quaternion must be unit length, ordered wxyz")
