"""Validate scene manifests without importing Isaac Sim (metres, wxyz quaternions)."""

import json
import math
import re
from pathlib import Path


def vector(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label}: expected {length} numbers")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f"{label}: expected finite numbers")
    return value


def load_scene_config(path, defaults=None):
    path = Path(path).expanduser().resolve()
    data = json.loads(path.read_text())
    allowed = {"assets", "ground_z", "robot_position", "scene_eye", "scene_target", "wrist_target"}
    if not isinstance(data, dict) or set(data) - allowed:
        raise ValueError(f"Scene must be an object with keys: {sorted(allowed)}")
    data = {**(defaults or {}), **data}
    scene = {"ground_z": data.get("ground_z", 0)}
    vector([scene["ground_z"]], 1, "ground_z")
    for key, default in {
        "robot_position": [0, 0, 0],
        "scene_eye": [1.6, 1.4, 1.2],
        "scene_target": [0.25, 0, 0.35],
        "wrist_target": [0.55, 0, 0.04],
    }.items():
        scene[key] = vector(data.get(key, default), 3, key)
    if scene["scene_eye"] == scene["scene_target"]:
        raise ValueError("scene_eye and scene_target must differ")
    assets = data.get("assets", [])
    if not isinstance(assets, list):
        raise ValueError("assets must be a list")
    scene["assets"] = []
    names = set()
    for entry in assets:
        allowed = {"name", "usd_path", "mode", "position", "orientation", "scale"}
        if not isinstance(entry, dict) or set(entry) - allowed:
            raise ValueError(f"Asset must be an object with keys: {sorted(allowed)}")
        name = entry.get("name", "")
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
            raise ValueError("Asset name must be a valid USD identifier")
        if name in names:
            raise ValueError(f"Duplicate asset name: {name}")
        names.add(name)
        source = entry.get("usd_path")
        if not isinstance(source, str) or not source.strip():
            raise ValueError(f"{name}: usd_path is required")
        if not source.startswith(("isaac://", "https://", "http://", "omniverse://")):
            local = Path(source).expanduser()
            local = (path.parent / local).resolve()
            if not local.is_file():
                raise ValueError(f"{name}: USD file not found: {local}")
            source = str(local)
        mode = entry.get("mode", "visual")
        if mode not in ("visual", "static", "dynamic"):
            raise ValueError(f"{name}: mode must be visual, static or dynamic")
        asset = {"name": name, "usd_path": source, "mode": mode}
        for key, default in {"position": [0, 0, 0], "orientation": [1, 0, 0, 0], "scale": [1, 1, 1]}.items():
            asset[key] = vector(entry.get(key, default), len(default), f"{name}.{key}")
        norm = math.hypot(*asset["orientation"])
        if norm < 1e-8:
            raise ValueError(f"{name}: orientation cannot be zero")
        asset["orientation"] = [v / norm for v in asset["orientation"]]
        if min(asset["scale"]) <= 0:
            raise ValueError(f"{name}: scale must be positive")
        scene["assets"].append(asset)
    return scene
