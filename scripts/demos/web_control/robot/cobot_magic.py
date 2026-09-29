"""Mobile ALOHA/Cobot Magic model: four arms, fixed Tracer2 base."""

PROFILE = {
    "label": "AgileX Cobot Magic",
    "asset_module": "isaaclab_assets.robots.agilex",
    "asset_config": "COBOT_MAGIC_CFG",
    "default_scene": "scenes/cobot_magic.json",
    "joints": [
        {"name": "camera_stand_1_joint", "group": "相机支架", "unit": "m", "speed": 0.03, "step": 0.01},
        {"name": "camera_stand_2_joint", "group": "相机支架", "unit": "rad", "speed": 0.2, "step": 0.05},
    ],
    "grippers": [],
    "cameras": [
        {"name": "scene", "label": "场景相机"},
        {"name": "head", "label": "顶部相机", "body": "camera_stand_2_Link", "eye_offset": [0.12, 0, 0.05]},
    ],
}
for prefix, label in [("fl", "前左"), ("fr", "前右"), ("bl", "后左"), ("br", "后右")]:
    PROFILE["joints"] += [
        {"name": f"{prefix}_joint{i}", "group": label + "臂", "unit": "rad", "speed": 0.3, "step": 0.05}
        for i in range(1, 7)
    ]
    PROFILE["grippers"].append(
        {
            "name": prefix,
            "label": label + "夹爪",
            "unit": "m",
            "speed": 0.03,
            "open": {f"{prefix}_joint7": 0.04, f"{prefix}_joint8": -0.04},
            "close": {f"{prefix}_joint7": 0.0, f"{prefix}_joint8": 0.0},
        }
    )
    if prefix in ("fl", "fr"):
        PROFILE["cameras"].append(
            {
                "name": prefix + "_wrist",
                "label": label + "腕部相机",
                "body": prefix + "_link6",
                "eye_offset": [0.08, 0, 0.08],
            }
        )
