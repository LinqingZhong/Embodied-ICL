"""ARX X5 single-arm browser profile. Asset configuration loads after AppLauncher."""

PROFILE = {
    "label": "方舟 ARX X5",
    "asset_module": "isaaclab_assets.robots.arx",
    "asset_config": "ARX_X5_CFG",
    "default_scene": "scenes/table_bottle.json",
    "joints": [
        {"name": f"joint{i}", "group": "机械臂", "unit": "rad", "speed": 0.3, "step": 0.05} for i in range(1, 7)
    ],
    "grippers": [
        {
            "name": "gripper",
            "label": "夹爪",
            "unit": "m",
            "speed": 0.03,
            "open": {"joint7": 0.044, "joint8": 0.044},
            "close": {"joint7": 0.0, "joint8": 0.0},
        }
    ],
    "cameras": [
        {"name": "scene", "label": "场景相机"},
        {"name": "wrist", "label": "腕部相机", "body": "link6", "eye_offset": [0.08, -0.08, 0.08]},
    ],
    "scene_defaults": {},
}
