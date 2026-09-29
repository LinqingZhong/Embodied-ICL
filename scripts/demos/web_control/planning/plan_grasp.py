"""Offline cuRobo worker for the native X5 and Cobot grasp fixtures."""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from grasp_profiles import grasp_spec
from scipy.spatial.transform import Rotation


def robot_config(spec):
    from curobo.util_file import load_yaml

    root = Path(__file__).resolve().parents[4] / "source/isaaclab_assets/data/Robots"
    x5 = spec["robot"].startswith("x5")
    directory = root / ("ARX/X5" if x5 else "AgileX/CobotMagic")
    prefix = "" if x5 else spec["arm"] + "_"
    urdf = directory / ("X5A.urdf" if x5 else "cobot_magic.urdf")
    names = [prefix + f"joint{i}" for i in range(1, 7)]
    if x5:
        original = load_yaml(str(directory / "curobo_tmp.yml"))["robot_cfg"]["kinematics"]
        spheres = original["collision_spheres"]
        ignore = original["self_collision_ignore"]
    else:
        # Capsules sampled along each link's outgoing joint offset. Approximation,
        # not a certified mesh enclosure; demo only, see documentation.
        tree = ET.parse(urdf).getroot()
        spheres = {}
        for i in range(7):
            link = prefix + ("base_link" if i == 0 else f"link{i}")
            endpoint = np.fromstring(tree.find(f"joint[@name='{prefix}joint{i + 1}']/origin").get("xyz"), sep=" ")
            if i == 6:
                # Palm ends at z=.067; joint7 origin belongs to the sliding finger, not palm.
                endpoint = np.array([0.0, 0.0, 0.065])
            spheres[link] = [
                dict(center=(endpoint * t).tolist(), radius=0.035)
                for t in np.linspace(0, 1, max(2, int(np.linalg.norm(endpoint) / 0.025)))
            ]
        links = list(spheres)
        ignore = {a: links[max(0, i - 2) : i] + links[i + 1 : i + 3] for i, a in enumerate(links)}
    kin = dict(
        urdf_path=str(urdf),
        asset_root_path=str(directory),
        base_link=prefix + "base_link",
        ee_link=prefix + "link6",
        collision_link_names=list(spheres),
        collision_spheres=spheres,
        self_collision_ignore=ignore,
        self_collision_buffer={k: 0.0 for k in spheres},
        collision_sphere_buffer=0.0,
        use_global_cumul=True,
        cspace=dict(
            joint_names=names,
            retract_config=spec["start"],
            null_space_weight=[1.0] * 6,
            cspace_distance_weight=[1.0] * 6,
            max_acceleration=4.0,
            max_jerk=50.0,
        ),
    )
    return dict(kinematics=kin)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, choices=["x5", "x5_dual", "cobot_magic"])
    parser.add_argument("--arm")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    spec = grasp_spec(args.robot, args.arm)
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

    tensor = TensorDeviceType()
    rotation = Rotation.from_quat(np.array(spec["base_pose"][3:])[[1, 2, 3, 0]])
    table = rotation.inv().apply(np.array(spec["table_position"]) - spec["base_pose"][:3]).tolist()
    world = {"cuboid": {"table": {"dims": spec["table_dims"], "pose": [*table, 1, 0, 0, 0]}}}
    cfg = MotionGenConfig.load_from_robot_config(
        robot_config(spec),
        world,
        tensor,
        interpolation_dt=1 / 120,
        num_trajopt_seeds=2,
        num_graph_seeds=2,
        num_ik_seeds=32,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
        use_cuda_graph=False,
        interpolation_steps=3000,
    )
    planner = MotionGen(cfg)
    q = spec["start"]
    segments = []
    for target in spec["targets"]:
        result = planner.plan_single(
            JointState.from_position(tensor.to_device([q])),
            Pose.from_list(target["pose"], tensor),
            MotionGenPlanConfig(max_attempts=8, timeout=40, time_dilation_factor=0.35),
        )
        if not result.success.item():
            raise RuntimeError(f"Planning failed {target['name']}: {result.status}")
        rows = result.get_interpolated_plan().position.cpu().tolist()
        q = rows[-1]
        segments.append(dict(name=target["name"], positions=rows, dt=float(result.interpolation_dt)))
        print(target["name"], len(rows), q, flush=True)
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dict(planner="curobo", fixture=spec, segments=segments), indent=2))


if __name__ == "__main__":
    main()
