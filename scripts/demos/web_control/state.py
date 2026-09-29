"""Thread-safe commands and immutable camera snapshots; no simulator dependencies."""

import copy
import math
import threading
import time
from collections import deque


class ControlState:
    def __init__(self):
        self.lock = threading.RLock()
        self.phase = "loading"
        self.message = "Loading Isaac Sim and robot assets…"
        self.paused = False
        self.actions = deque()
        self.latest = None
        self.sequence = 0
        self.sim_time = 0.0
        self.names = []
        self.arms = []
        self.fingers = []
        self.robot_label = "Franka Panda"
        self.joint_meta = {}
        self.grippers = []
        self.cameras = [{"name": "scene", "label": "场景相机"}, {"name": "wrist", "label": "腕部相机"}]
        self.speeds = {}

    def configure(self, names, limits, defaults, arms, fingers):
        with self.lock:
            self.names = list(names)
            self.limits = list(limits)
            self.defaults = list(defaults)
            self.desired = list(defaults)
            self.applied = list(defaults)
            self.positions = list(defaults)
            self.velocities = [0.0] * len(names)
            self.arms, self.fingers = list(arms), list(fingers)
            self.phase, self.message = "ready", "Robot ready"

    def configure_profile(self, names, limits, defaults, profile):
        names = list(names)
        limits = [list(pair) for pair in limits]
        meta, speeds, grippers, used = {}, {}, [], set()

        def resolve(name, speed):
            if name not in names:
                raise ValueError(f"Profile joint missing from USD: {name}")
            index = names.index(name)
            if index in used:
                raise ValueError(f"Joint assigned more than once: {name}")
            if not math.isfinite(speed) or speed <= 0:
                raise ValueError(f"Invalid target speed: {name}")
            used.add(index)
            speeds[index] = speed
            return index

        for joint in profile["joints"]:
            index = resolve(joint["name"], joint["speed"])
            low, high = limits[index]
            if "limits" in joint:
                low, high = max(low, joint["limits"][0]), min(high, joint["limits"][1])
            if not all(math.isfinite(v) for v in (low, high)) or low >= high:
                raise ValueError(f"Finite panel limits required: {joint['name']}")
            if not low <= defaults[index] <= high:
                raise ValueError(f"Default outside panel limits: {joint['name']}")
            limits[index] = [low, high]
            meta[index] = {k: joint[k] for k in ("group", "unit", "step")}
        for gripper in profile["grippers"]:
            if set(gripper["open"]) != set(gripper["close"]):
                raise ValueError("Gripper open/close must address the same joints")
            item = copy.deepcopy(gripper)
            item["ids"] = [resolve(name, item["speed"]) for name in item["open"]]
            for name, index in zip(item["open"], item["ids"]):
                for command in ("open", "close"):
                    value = item[command][name]
                    low, high = limits[index]
                    if not math.isfinite(value) or not low - 1e-6 <= value <= high + 1e-6:
                        raise ValueError(f"Gripper target outside limits: {name}")
                    # USD/PhysX limits are float32; tolerate rounding, then clamp to the actual limit.
                    item[command][name] = min(high, max(low, value))
            grippers.append(item)
        fingers = [i for g in grippers for i in g["ids"]]
        with self.lock:
            self.configure(names, limits, defaults, list(meta), fingers)
            self.robot_label = profile["label"]
            self.joint_meta, self.speeds, self.grippers = meta, speeds, grippers
            self.cameras = [{"name": c["name"], "label": c["label"]} for c in profile["cameras"]]

    def set_phase(self, phase, message):
        with self.lock:
            self.phase, self.message = phase, message

    def submit(self, command):
        if not isinstance(command, dict):
            raise ValueError("Command must be an object")
        action = command.get("action")
        fields = {
            "joint": {"action", "joint", "value"},
            "gripper": {"action", "open"},
            "pause": {"action"},
            "resume": {"action"},
            "step": {"action"},
            "reset": {"action"},
        }
        if action == "gripper" and "gripper" in command:
            fields["gripper"] = {"action", "open", "gripper"}
        if not isinstance(action, str) or action not in fields or set(command) != fields[action]:
            raise ValueError("Unknown command or unexpected fields")
        with self.lock:
            if self.phase != "ready":
                raise ValueError("Simulator is not ready")
            if action == "joint":
                index, value = command["joint"], command["value"]
                if type(index) is not int or index not in self.arms:
                    raise ValueError("Invalid arm joint")
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError("Joint target must be a finite number")
                low, high = self.limits[index]
                if not low <= value <= high:
                    raise ValueError("Joint target exceeds limits")
                self.desired[index] = float(value)
                return
            if action == "gripper":
                if type(command["open"]) is not bool:
                    raise ValueError("Gripper open must be boolean")
                if self.grippers:
                    name = command.get("gripper")
                    if name is None and len(self.grippers) == 1:
                        name = self.grippers[0]["name"]
                    selected = next((g for g in self.grippers if g["name"] == name), None)
                    if selected is None:
                        raise ValueError("Choose a valid gripper")
                    targets = selected["open" if command["open"] else "close"]
                    for index in selected["ids"]:
                        self.desired[index] = targets[self.names[index]]
                else:
                    if "gripper" in command:
                        raise ValueError("Unknown gripper")
                    for index in self.fingers:
                        self.desired[index] = self.limits[index][1 if command["open"] else 0]
                return
            paused = self.paused
            for pending in self.actions:
                if pending in ("pause", "resume"):
                    paused = pending == "pause"
            if action == "step" and not paused:
                raise ValueError("Single step requires pause")
            if len(self.actions) >= 32:
                raise ValueError("Command queue is full")
            if action == "reset":
                self.desired = self.defaults.copy()
            self.actions.append(action)

    def consume(self, dt, *, interpolate=True):
        with self.lock:
            reset, steps = False, 0
            # Leave additional single-step requests for subsequent physical ticks.
            while self.actions:
                action = self.actions[0]
                if action == "step" and steps:
                    break
                self.actions.popleft()
                if action == "pause":
                    self.paused = True
                elif action == "resume":
                    self.paused = False
                elif action == "step":
                    steps = 1
                elif action == "reset":
                    reset, steps = True, 0
                    self.applied = self.defaults.copy()
            stepping = not self.paused or bool(steps)
            if stepping and interpolate:
                for index in range(len(self.applied)):
                    speed = self.speeds.get(index, 0.04 if index in self.fingers else 0.5)
                    delta = self.desired[index] - self.applied[index]
                    self.applied[index] += max(-speed * dt, min(speed * dt, delta))
            return {"step": stepping, "reset": reset, "targets": self.applied.copy()}

    def publish(self, sim_time, positions, velocities, images=None):
        with self.lock:
            self.sim_time = sim_time
            self.positions, self.velocities = list(positions), list(velocities)
            if images is not None:
                self.sequence += 1
                self.latest = {
                    "sequence": self.sequence,
                    "sim_time": sim_time,
                    "captured_at": time.time(),
                    "monotonic": time.monotonic(),
                    "positions": list(positions),
                    "velocities": list(velocities),
                    "targets": self.desired.copy(),
                    "joint_names": self.names.copy(),
                    "robot": self.robot_label,
                    "cameras": copy.deepcopy(self.cameras),
                    "images": dict(images),
                }

    def frame(self):
        with self.lock:
            return copy.deepcopy(self.latest)

    def status(self):
        with self.lock:
            return {
                "robot": self.robot_label,
                "cameras": copy.deepcopy(self.cameras),
                "grippers": [
                    {
                        "name": g["name"],
                        "label": g["label"],
                        "unit": g["unit"],
                        "actual": [self.positions[i] for i in g["ids"]],
                        "targets": [self.desired[i] for i in g["ids"]],
                    }
                    for g in self.grippers
                ],
                "phase": self.phase,
                "message": self.message,
                "paused": self.paused,
                "sim_time": self.sim_time,
                "sequence": self.sequence,
                "frame_age": None if self.latest is None else time.monotonic() - self.latest["monotonic"],
                "joints": [
                    {
                        **self.joint_meta.get(i, {"group": "机械臂", "unit": "rad", "step": 0.05}),
                        "index": i,
                        "name": self.names[i],
                        "min": self.limits[i][0],
                        "max": self.limits[i][1],
                        "actual": self.positions[i],
                        "target": self.desired[i],
                        "applied": self.applied[i],
                    }
                    for i in self.arms
                ],
                "gripper_width": (
                    sum(self.positions[i] for i in self.fingers)
                    if self.names
                    and (not self.grippers or (len(self.grippers) == 1 and self.grippers[0].get("width_sum")))
                    else None
                ),
            }
