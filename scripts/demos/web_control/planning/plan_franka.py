"""Offline cuRobo planning worker. No Isaac Sim imports; exits to release GPU memory."""

import argparse
import json
from pathlib import Path

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--request", required=True)
parser.add_argument("--output", required=True)
args = parser.parse_args()
request = json.loads(Path(args.request).read_text())
tensor_args = TensorDeviceType()
robot = load_yaml(join_path(get_robot_configs_path(), "franka.yml"))["robot_cfg"]
# Match the flange convention used by RobotServer, not panda_hand/TCP.
robot["kinematics"]["ee_link"] = "panda_link8"
world = {"cuboid": {"table": {"dims": [0.7, 0.8, 0.1], "pose": [0.55, 0, 0.15, 1, 0, 0, 0]}}}
config = MotionGenConfig.load_from_robot_config(
    robot,
    world,
    tensor_args,
    interpolation_dt=1 / 120,
    num_trajopt_seeds=2,
    num_graph_seeds=2,
    num_ik_seeds=16,
    collision_checker_type=CollisionCheckerType.PRIMITIVE,
    use_cuda_graph=False,
    interpolation_steps=3000,
)
planner = MotionGen(config)
q = request["start"]
segments = []
for target in request["targets"]:
    start = JointState.from_position(tensor_args.to_device([q]))
    result = planner.plan_single(
        start,
        Pose.from_list(target["pose"], tensor_args),
        MotionGenPlanConfig(max_attempts=5, timeout=20, time_dilation_factor=0.35),
    )
    if not result.success.item():
        raise RuntimeError(f"Planning failed at {target['name']}: {result.status}")
    trajectory = result.get_interpolated_plan()
    positions = trajectory.position.cpu().tolist()
    q = positions[-1]
    segments.append(
        dict(
            name=target["name"],
            positions=positions,
            dt=float(result.interpolation_dt),
            planner_status=str(result.status),
        )
    )
Path(args.output).write_text(json.dumps(dict(planner="curobo", segments=segments), indent=2))
print("PLAN_SAVED", args.output, flush=True)
