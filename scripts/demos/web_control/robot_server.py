"""Simulator-independent joint chunk execution and coherent robot observations."""

import copy
import math
import time
import uuid
from collections import OrderedDict

from eef_control import EefControl
from recording import RecordingControl
from state import ControlState


class RobotServer(RecordingControl, EefControl, ControlState):
    """Own control state; only the simulation thread calls consume/publish.

    A chunk row is a position setpoint, not a teleport or a convergence promise.
    publish acknowledges the physical step that followed consume.
    """

    def __init__(self, *, manual_control=True, physics_dt=1 / 120):
        super().__init__()
        self.manual_control = manual_control
        self.physics_dt = physics_dt
        self.recorder = None
        self.recording_dir = "output/datasets"
        self.recording_metadata = {}
        self.physics_step = 0
        self.kinematics = {}
        self.eef_poses = {}
        self.object_states = {}
        self.planning = False
        self.tasks = OrderedDict()
        self.active_task = None
        self._pending_step = None

    def execute_joint_chunk(self, joint_names, positions, steps_per_action=1):
        with self.lock:
            if self.phase != "ready":
                raise RuntimeError("Simulator is not ready")
            if self.active_task is not None or self.planning:
                raise RuntimeError("Robot is busy; cancel or wait for the current chunk")
            if self.actions:
                raise RuntimeError("Pending simulator commands; retry after they are processed")
            if not isinstance(joint_names, list) or not joint_names or any(type(n) is not str for n in joint_names):
                raise ValueError("joint_names must be a nonempty list of names")
            if len(set(joint_names)) != len(joint_names):
                raise ValueError("Duplicate joint names")
            allowed = set(self.arms + self.fingers)
            if any(n not in self.names or self.names.index(n) not in allowed for n in joint_names):
                raise ValueError("Unknown or passive joint")
            if type(steps_per_action) is not int or not 1 <= steps_per_action <= 120000:
                raise ValueError("steps_per_action must be an integer in [1, 120000]")
            if not isinstance(positions, list) or not 1 <= len(positions) <= 2048:
                raise ValueError("positions must contain 1 to 2048 rows")
            ids = [self.names.index(n) for n in joint_names]
            for row in positions:
                if not isinstance(row, list) or len(row) != len(ids):
                    raise ValueError("Each row must match joint_names")
                for i, v in zip(ids, row):
                    if type(v) not in (int, float) or not math.isfinite(v):
                        raise ValueError("Targets must be finite numbers")
                    if not self.limits[i][0] <= v <= self.limits[i][1]:
                        raise ValueError(f"Target outside limits: {self.names[i]}")
            # Freeze any unfinished manual ramp; unspecified joints keep applied setpoints.
            self.desired = self.applied.copy()
            task = dict(
                id=uuid.uuid4().hex,
                status="running",
                joint_names=list(joint_names),
                steps_per_action=steps_per_action,
                total_actions=len(positions),
                total_steps=len(positions) * steps_per_action,
                executed_steps=0,
                submitted_sim_time=self.sim_time,
                ids=ids,
                rows=copy.deepcopy(positions),
            )
            self.record_event({"type": "joint_chunk", "task": copy.deepcopy(task)})
            self.tasks[task["id"]] = task
            self.active_task = task["id"]
            while len(self.tasks) > 64:
                self.tasks.popitem(last=False)
            return self.get_task(task["id"])

    def get_task(self, task_id):
        with self.lock:
            task = self.tasks[task_id]
            result = {k: copy.deepcopy(v) for k, v in task.items() if k not in ("ids", "rows")}
            result["completed_actions"] = task["executed_steps"] // task["steps_per_action"]
            return result

    def _finish(self, task, status):
        task.update(
            status=status,
            finished_sim_time=self.sim_time,
            actual_positions=[self.positions[i] for i in task["ids"]],
            targets=[self.applied[i] for i in task["ids"]],
            errors=[self.applied[i] - self.positions[i] for i in task["ids"]],
        )
        self.record_event({"type": "task_result", "task": self.get_task(task["id"])})
        task.pop("rows", None)
        self.active_task = None

    def cancel(self, task_id):
        with self.lock:
            task = self.tasks[task_id]
            if task["status"] == "running":
                self.desired = self.applied.copy()
                self._finish(task, "cancelled")
            return self.get_task(task_id)

    def submit(self, command):
        with self.lock:
            if self.planning:
                raise ValueError("EEF planning in progress; retry after planning")
            if isinstance(command, dict) and command.get("action") in ("joint", "gripper"):
                if not self.manual_control or self.active_task:
                    raise ValueError("Manual movement disabled in observe mode or while a chunk is running")
            if (
                self.recorder
                and not self.recorder.closed
                and isinstance(command, dict)
                and command.get("action") == "reset"
            ):
                raise ValueError("Stop recording before reset to preserve a monotonic episode")
            super().submit(command)
            self.record_event({"type": "command", "command": copy.deepcopy(command)})

    def consume(self, dt):
        with self.lock:
            if "reset" in self.actions and self.active_task:
                self.cancel(self.active_task)
                self.desired = self.defaults.copy()
            tick = super().consume(dt, interpolate=self.active_task is None and not self.planning)
            self._pending_step = None
            if tick["step"] and self.active_task:
                task = self.tasks[self.active_task]
                row = task["rows"][task["executed_steps"] // task["steps_per_action"]]
                for i, value in zip(task["ids"], row):
                    self.applied[i] = self.desired[i] = float(value)
                tick["targets"] = self.applied.copy()
                self._pending_step = self.active_task
            return tick

    def publish(self, sim_time, positions, velocities, images=None):
        with self.lock:
            advanced = sim_time > self.sim_time
            if advanced:
                self.physics_step += 1
            super().publish(sim_time, positions, velocities, images)
            if self.recorder and (advanced or images is not None):
                self.recorder.submit(
                    "sample",
                    dict(
                        sim_time=sim_time,
                        wall_time=time.time(),
                        step=self.physics_step,
                        positions=list(positions),
                        velocities=list(velocities),
                        targets=self.applied.copy(),
                        images=None if images is None else dict(images),
                        object_states=copy.deepcopy(self.object_states),
                    ),
                )
            if images is not None:
                self.latest["targets"] = self.applied.copy()
                self.latest["eef_poses"] = copy.deepcopy(self.eef_poses)
            if self._pending_step and self._pending_step == self.active_task:
                task = self.tasks[self.active_task]
                task["executed_steps"] += 1
                if task["executed_steps"] == task["total_steps"]:
                    self._finish(task, "completed")
            self._pending_step = None

    def set_phase(self, phase, message):
        with self.lock:
            if phase in ("stopped", "error") and self.active_task:
                self._finish(self.tasks[self.active_task], "failed")
            super().set_phase(phase, message)

    def joint_state(self):
        with self.lock:
            if self.phase != "ready":
                raise RuntimeError("Simulator is not ready")
            return dict(
                object_states=copy.deepcopy(self.object_states),
                eef_poses=copy.deepcopy(self.eef_poses),
                sim_time=self.sim_time,
                joint_names=self.names.copy(),
                positions=self.positions.copy(),
                velocities=self.velocities.copy(),
                targets=self.applied.copy(),
                physics_dt=self.physics_dt,
                controllable_joint_names=[self.names[i] for i in self.arms + self.fingers],
                limits=[[v if math.isfinite(v) else None for v in pair] for pair in self.limits],
                active_task=self.active_task,
            )

    def robot_spec(self):
        """Return native control metadata, with unknown profile units left unset.

        Legacy configure() has no unit metadata: its arms use rad and fingers m.
        configure_profile() units and native gripper mappings take precedence.
        """
        with self.lock:
            if self.phase != "ready":
                raise RuntimeError("Simulator is not ready")
            legacy = not self.joint_meta and not self.grippers
            units = {
                self.names[i]: self.joint_meta.get(i, {}).get("unit", "rad" if legacy else None) for i in self.arms
            }
            units.update({self.names[i]: "m" if legacy else None for i in self.fingers})
            for gripper in self.grippers:
                units.update({self.names[i]: gripper.get("unit") for i in gripper["ids"]})
            return dict(
                name=self.robot_label,
                joint_names=self.names.copy(),
                controllable_joint_names=[self.names[i] for i in self.arms + self.fingers],
                limits=[[v if math.isfinite(v) else None for v in pair] for pair in self.limits],
                joint_units=units,
                grippers=[
                    {key: copy.deepcopy(gripper.get(key)) for key in ("name", "label", "unit", "open", "close")}
                    for gripper in self.grippers
                ],
                arms=copy.deepcopy(self.get_arms()),
                cameras=[camera["name"] for camera in self.cameras],
                physics_dt=self.physics_dt,
            )

    def status(self):
        with self.lock:
            result = super().status()
            result.update(
                manual_control=self.manual_control and not self.planning,
                active_task=self.active_task,
                physics_dt=self.physics_dt,
            )
            return result
