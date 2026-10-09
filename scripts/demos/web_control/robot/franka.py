"""Franka Panda panel configuration. Imported without starting Isaac Sim."""

PROFILE = {
    "label": "Franka Panda",
    "asset_module": "isaaclab_assets.robots.franka",
    "asset_config": "FRANKA_PANDA_CFG",
    "joints": [
        {"name": f"panda_joint{i}", "group": "机械臂", "unit": "rad", "speed": 0.5, "step": 0.05} for i in range(1, 8)
    ],
    "grippers": [
        {
            "name": "gripper",
            "label": "夹爪",
            "unit": "m",
            "speed": 0.04,
            "width_sum": True,
            "open": {"panda_finger_joint1": 0.04, "panda_finger_joint2": 0.04},
            "close": {"panda_finger_joint1": 0.0, "panda_finger_joint2": 0.0},
        }
    ],
    "cameras": [
        {"name": "scene", "label": "场景相机"},
        {"name": "wrist", "label": "腕部相机", "body": "panda_hand", "eye_offset": [0, -0.1, 0.04]},
        {
            "name": "front",
            "label": "正面观察相机",
            "body": "panda_link0",
            "eye_offset": [1.35, 0.35, 0.85],
            "target": [0.35, 0, 0.35],
        },
    ],
    "scene_defaults": {},
}
