"""Two independent X5 articulations using RoboDojo dual_x5.yml base poses."""

import copy

from .x5 import PROFILE as SINGLE

PROFILE = copy.deepcopy(SINGLE)
PROFILE.update(label="方舟 ARX X5 双臂", default_scene="scenes/x5_dual_table.json")
PROFILE["instances"] = [
    {"name": "left", "position": [-0.3, -0.45, 0.765], "orientation": [0.70710678, 0, 0, 0.70710678]},
    {"name": "right", "position": [0.3, -0.45, 0.765], "orientation": [0.70710678, 0, 0, 0.70710678]},
]
PROFILE["joints"] = []
PROFILE["grippers"] = []
PROFILE["cameras"] = [{"name": "scene", "label": "场景相机"}]
for side, label in [("left", "左"), ("right", "右")]:
    for joint in SINGLE["joints"]:
        PROFILE["joints"].append({**joint, "name": f"{side}/{joint['name']}", "group": f"{label}臂"})
    gripper = copy.deepcopy(SINGLE["grippers"][0])
    gripper.update(name=side, label=f"{label}夹爪")
    for action in ("open", "close"):
        gripper[action] = {f"{side}/{name}": value for name, value in gripper[action].items()}
    PROFILE["grippers"].append(gripper)
    PROFILE["cameras"].append(
        {
            "name": f"{side}_wrist",
            "label": f"{label}腕相机",
            "instance": side,
            "body": "link6",
            "eye_offset": [0.08, 0.08, 0.08],
        }
    )
