"""Deterministic grasp fixtures; all planner poses are in the selected arm base."""

import numpy as np
from scipy.spatial.transform import Rotation


def grasp_spec(robot, arm=None):
    if robot not in ("x5", "x5_dual", "cobot_magic"):
        raise ValueError("Unsupported grasp robot")
    arm = arm or {"x5": "arm", "x5_dual": "left", "cobot_magic": "fl"}[robot]
    allowed = {"x5": ["arm"], "x5_dual": ["left", "right"], "cobot_magic": ["fl", "fr"]}
    if arm not in allowed[robot]:
        raise ValueError(f"Unsupported grasp arm {robot}/{arm}")
    x5 = robot.startswith("x5")
    base = [0.0, 0.0, 0.0]
    rotation = Rotation.identity()
    if robot == "x5_dual":
        base = [-0.3 if arm == "left" else 0.3, -0.45, 0.765]
        rotation = Rotation.from_euler("z", np.pi / 2)
    elif robot == "cobot_magic":
        base = [-0.36125, 0.3 if arm == "fl" else -0.3, 0.925]
    table_z = 0.0 if x5 else -0.15
    offset = [0.14, 0, 0] if x5 else [0, 0, 0.11]
    quat = [2**-0.5, 0, 2**-0.5, 0] if x5 else [0, 1, 0, 0]
    cube_size = [0.04, 0.04, 0.04]
    contact = np.array([0.30, 0, table_z + cube_size[2] / 2])
    grasp = contact - Rotation.from_quat(np.array(quat)[[1, 2, 3, 0]]).apply(offset)
    targets = []
    for name, dz in [("approach", 0.10), ("grasp", 0), ("lift", 0.16), ("place", 0), ("retreat", 0.10)]:
        targets.append(dict(name=name, pose=[*(grasp + [0, 0, dz]), *quat]))
    bq = rotation.as_quat()[[3, 0, 1, 2]].tolist()
    return dict(
        robot=robot,
        arm=arm,
        base_pose=[*base, *bq],
        start=[0] * 6 if x5 else [0, 1, -1.2, 0, 0, 0],
        targets=targets,
        contact_offset=offset,
        cube_size=cube_size,
        cube_position=(rotation.apply(contact) + base).tolist(),
        table_position=(rotation.apply([0.38, 0, table_z - 0.05]) + base).tolist(),
        table_orientation=bq,
        table_dims=[0.5, 0.44, 0.1],
        robot_position=[-0.6, 0, 0.15] if not x5 else [0, 0, 0],
        ground_z=-0.8 if robot == "x5" else 0.0,
    )


def validate_plan(plan, fk):
    """Reject malformed trajectories or a planner/simulator flange mismatch before recording."""
    if plan.get("planner") != "curobo":
        raise ValueError("Expected a cuRobo plan")
    fixture = plan["fixture"]
    expected = grasp_spec(fixture["robot"], fixture["arm"])
    if fixture != expected:
        raise ValueError("Plan fixture differs from current grasp configuration; regenerate the plan")
    segments = plan["segments"]
    if [s["name"] for s in segments] != [t["name"] for t in fixture["targets"]]:
        raise ValueError("Expected approach/grasp/lift/place/retreat stages")
    errors = []
    for target, segment in zip(fixture["targets"], segments):
        rows = np.asarray(segment["positions"], dtype=float)
        if rows.ndim != 2 or rows.shape[1] != 6 or not 1 <= len(rows) <= 2048 or not np.isfinite(rows).all():
            raise ValueError("Invalid six-joint trajectory")
        if abs(segment["dt"] - 1 / 120) > 1e-6:
            raise ValueError("Plan timestep must be 1/120 s")
        actual = fk(fixture["arm"], rows[-1].tolist())
        pe = float(np.linalg.norm(np.array(actual["position"]) - target["pose"][:3]))
        qa = Rotation.from_quat(np.array(actual["quaternion"])[[1, 2, 3, 0]])
        qt = Rotation.from_quat(np.array(target["pose"][3:])[[1, 2, 3, 0]])
        re = float((qt.inv() * qa).magnitude())
        if pe > 1e-4 or re > 1e-3:
            raise ValueError(f"Planner/RobotServer FK mismatch: position={pe} m rotation={re} rad")
        errors.append(dict(stage=target["name"], position_error=pe, rotation_error=re))
    return errors
