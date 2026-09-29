"""Agibot A2D: fixed base, 18 direct controls, two coupled grippers, four cameras."""

PROFILE = {
    "label": "智元 Agibot A2D",
    "asset_module": "isaaclab_assets.robots.agibot",
    "asset_config": "AGIBOT_A2D_CFG",
    "fix_root_link": True,
    "default_scene": "scenes/a2d_table_bottle.json",
    "scene_defaults": {"ground_z": 0, "robot_position": [-0.6, 0, 0]},
    "joints": [
        {"name": "joint_lift_body", "group": "躯干", "unit": "m", "speed": 0.03, "step": 0.01},
        {"name": "joint_body_pitch", "group": "躯干", "unit": "rad", "speed": 0.2, "step": 0.05},
        # USD head yaw is continuous; the panel intentionally exposes only +/- 90 degrees.
        {
            "name": "joint_head_yaw",
            "group": "头部",
            "unit": "rad",
            "speed": 0.3,
            "step": 0.05,
            "limits": [-1.5707963, 1.5707963],
        },
        {"name": "joint_head_pitch", "group": "头部", "unit": "rad", "speed": 0.3, "step": 0.05},
    ]
    + [
        {"name": f"{side}_arm_joint{i}", "group": label, "unit": "rad", "speed": 0.3, "step": 0.05}
        for side, label in [("left", "左臂"), ("right", "右臂")]
        for i in range(1, 8)
    ],
    "grippers": [
        {
            "name": side,
            "label": label,
            "unit": "rad",
            "speed": 0.5,
            "open": {
                f"{side}_{joint}": 0.994 for joint in ["hand_joint1", "Right_Support_Joint", "Left_Support_Joint"]
            },
            "close": {
                f"{side}_{joint}": close for joint in ["hand_joint1", "Right_Support_Joint", "Left_Support_Joint"]
            },
        }
        for side, label, close in [("left", "左夹爪", 0.0), ("right", "右夹爪", 0.2)]
    ],
    "cameras": [
        {"name": "scene", "label": "场景相机"},
        {"name": "head", "label": "头部相机", "body": "link_pitch_head", "eye_offset": [0.15, 0, 0.05]},
        {"name": "left_wrist", "label": "左腕相机", "body": "left_base_link", "eye_offset": [0.08, -0.08, 0.06]},
        {"name": "right_wrist", "label": "右腕相机", "body": "right_base_link", "eye_offset": [0.08, 0.08, 0.06]},
    ],
}
