#!/usr/bin/env python3
"""Configurable robot browser panel. Run with isaaclab.sh -p ... --headless --enable_cameras."""

import argparse
import importlib
import io
import math
import signal
import time
import traceback
from pathlib import Path

from robot import PROFILES, get_profile
from robot_server import RobotServer
from scene_config import load_scene_config
from server import WebServer

from isaaclab.app import AppLauncher


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("Must be positive")
    return number


def spawn_robots(profile, scene, control_quality="legacy", robot_name="", grasp_demo=False):
    from isaaclab.assets import Articulation

    asset_module = importlib.import_module(profile["asset_module"])
    robots = {}
    for instance in profile.get("instances", [{"name": "robot"}]):
        name = instance["name"]
        prim = f"/World/Robots/{name}" if "instances" in profile else "/World/Robot"
        robot_cfg = getattr(asset_module, profile["asset_config"]).replace(prim_path=prim)
        if profile.get("fix_root_link"):
            robot_cfg.spawn.articulation_props.fix_root_link = True
        if "position" in instance:
            # Scene robot_position is a shared translation for multi-robot profiles.
            offset = scene["robot_position"] if scene else [0, 0, 0]
            robot_cfg.init_state.pos = tuple(a + b for a, b in zip(instance["position"], offset))
            robot_cfg.init_state.rot = tuple(instance["orientation"])
        elif scene:
            robot_cfg.init_state.pos = tuple(scene["robot_position"])
        if control_quality == "compensated":
            from simulation_quality import configure_drives

            robot_cfg = configure_drives(robot_cfg, robot_name)
        if grasp_demo and robot_name == "franka":
            robot_cfg.actuators["panda_hand"].stiffness = 800.0
            robot_cfg.actuators["panda_hand"].damping = 40.0
            robot_cfg.actuators["panda_hand"].effort_limit_sim = 20.0
        robots[name] = Articulation(robot_cfg)
    return robots


def mount_cameras(profile, scene, robots, cameras, sim):
    import torch

    from pxr import Gf

    import isaaclab.sim as sim_utils
    from isaaclab.utils.math import create_rotation_matrix_from_view, quat_from_matrix, subtract_frame_transforms

    # Each world view is converted to a local camera transform under its moving link.
    for camera_profile in profile["cameras"]:
        body = camera_profile.get("body")
        if not body:
            continue
        robot = robots[camera_profile.get("instance", "robot")]
        ids, _ = robot.find_bodies(body.split("/")[-1])
        if len(ids) != 1:
            raise ValueError(f"Expected one camera body: {body}")
        eye = robot.data.body_pos_w[:, ids[0]].clone()
        eye += torch.tensor([camera_profile["eye_offset"]], device=sim.device)
        target = camera_profile.get("target", scene["wrist_target"] if scene else [0.55, 0, 0.04])
        orientation = quat_from_matrix(
            create_rotation_matrix_from_view(eye, torch.tensor([target], device=sim.device), "Z", device=sim.device)
        )
        local_pos, local_rot = subtract_frame_transforms(
            robot.data.body_pos_w[:, ids[0]], robot.data.body_quat_w[:, ids[0]], eye, orientation
        )
        # Author local poses using the physics parent pose, not potentially stale USD world transforms.
        camera_prim = sim_utils.get_current_stage().GetPrimAtPath(cameras[camera_profile["name"]].cfg.prim_path)
        pos = camera_profile.get("mount_pos", local_pos[0].cpu().tolist())
        rot = camera_profile.get("mount_rot", local_rot[0].cpu().tolist())
        camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*pos))
        camera_prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(rot[0], Gf.Vec3d(*rot[1:])))


def joint_names(robots, profile):
    return [
        f"{name}/{joint}" if "instances" in profile else joint
        for name, robot in robots.items()
        for joint in robot.joint_names
    ]


def validate_args(parser, args):
    if not args.enable_cameras:
        parser.error("--enable_cameras is required for this panel")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.width > 1920 or args.height > 1080 or args.camera_fps > 30:
        parser.error("Use at most 1920x1080 and 30 camera FPS")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and nonnegative")


def capture_due(clock, sim_time, previous, interval, wall_time, next_wall):
    if clock == "simulation":
        return sim_time - previous >= interval - 1e-8
    return wall_time >= next_wall


def close_recording(state):
    if state.recorder and not state.recorder.closed:
        state.stop_recording(state.recorder.id, False, "simulation_stopped")


def spawn_scene_objects(args, scene, sim_utils):
    objects = []
    if args.grasp_demo:
        from grasp_scene import spawn_grasp_scene

        objects = spawn_grasp_scene(args.robot, args.grasp_arm)
    elif scene:
        from scene_assets import spawn_assets

        objects = spawn_assets(scene)
    else:
        # Static colored reference objects make both viewpoint and motion visible.
        for name, position, color in [
            ("Coral", (0.55, 0.18, 0.045), (0.9, 0.25, 0.16)),
            ("Teal", (0.55, -0.18, 0.045), (0.12, 0.7, 0.55)),
            ("Blue", (0.7, 0, 0.045), (0.15, 0.35, 0.9)),
        ]:
            cube = sim_utils.CuboidCfg(
                size=(0.09, 0.09, 0.09), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color)
            )
            cube.func(f"/World/{name}", cube, translation=position)
    return objects


def configure_grasp_scene(args, scene):
    if args.grasp_demo and args.robot != "franka":
        from planning.grasp_profiles import grasp_spec

        fixture = grasp_spec(args.robot, args.grasp_arm)
        target = fixture["cube_position"]
        scene = dict(
            assets=[],
            ground_z=fixture["ground_z"],
            robot_position=fixture["robot_position"],
            scene_target=target,
            wrist_target=target,
            scene_eye=[target[0] + 1.1, target[1] - 1.2, target[2] + 0.9],
        )

    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--grasp-demo", action="store_true", help="Collision table and dynamic grasp cube")
    parser.add_argument("--grasp-arm", help="Selected grasp arm (arm, left, right, fl, fr)")
    parser.add_argument("--robot", choices=sorted(PROFILES), default="franka", help="Robot profile in robot/")
    parser.add_argument("--scene-config", help="JSON manifest of USD assets; replaces the default cubes")
    parser.add_argument(
        "--mode",
        choices=("web", "observe"),
        default="web",
        help="observe: view simulation and use RobotClient; disable manual movement",
    )
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--control-quality", choices=("legacy", "compensated"), default="compensated")
    parser.add_argument("--visual-quality", choices=("legacy", "studio"), default="studio")
    parser.add_argument("--hdri", help="Optional local/remote HDR environment texture")
    parser.add_argument("--jpeg-quality", type=int, choices=range(1, 101), default=95)
    parser.add_argument("--width", type=positive_int, default=480)
    parser.add_argument("--height", type=positive_int, default=360)
    parser.add_argument("--camera-fps", type=positive_int, default=10)
    parser.add_argument("--capture-clock", choices=("wall", "simulation"), default="wall")
    parser.add_argument("--recording-dir", default="output/datasets")
    parser.add_argument("--output-dir", default="output/web_control")
    parser.add_argument(
        "--duration", type=float, default=0, help="Stop after N wall seconds once ready; 0 is unlimited"
    )
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    validate_args(parser, args)

    if args.grasp_demo and args.robot not in ("franka", "x5", "x5_dual", "cobot_magic"):
        parser.error("Unsupported grasp-demo robot")
    profile = get_profile(args.robot)
    if args.grasp_demo and args.robot == "franka":
        profile["cameras"][1].update(mount_pos=[0.08, 0, 0.02], mount_rot=[0.3686461768, 0, 0.9295697910, 0])
    if not args.scene_config and profile.get("default_scene"):
        args.scene_config = str(Path(__file__).parent / profile["default_scene"])
    scene = None
    if args.scene_config:
        try:
            scene = load_scene_config(args.scene_config, defaults=profile.get("scene_defaults"))
        except (OSError, ValueError) as exc:
            parser.error(f"--scene-config: {exc}")

    scene = configure_grasp_scene(args, scene)

    state = RobotServer(manual_control=args.mode == "web")
    state.recording_dir = args.recording_dir
    state.recording_metadata = {"robot_profile": args.robot, "scene": scene, "settings": vars(args)}
    state.robot_label = profile["label"]
    state.cameras = [{"name": c["name"], "label": c["label"]} for c in profile["cameras"]]
    web = WebServer(state, args.port, args.output_dir)
    web.start()
    print(f"WEB_CONTROL_URL={web.url}", flush=True)
    print(f"ROBOT_SERVER_URL={web.url}", flush=True)
    print("Open through an SSH tunnel. Loading real simulation assets…", flush=True)
    app = sim = None
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    try:
        launcher = AppLauncher(args)
        app = launcher.app
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)

        import torch
        from PIL import Image

        import isaaclab.sim as sim_utils
        from isaaclab.sensors import Camera, CameraCfg

        dt = 1.0 / 120.0
        sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(dt=dt, device=args.device, render=sim_utils.RenderCfg(antialiasing_mode="DLAA"))
        )
        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/Ground", ground, translation=(0, 0, scene["ground_z"] if scene else 0))
        from simulation_quality import setup_visuals

        setup_visuals(sim_utils, args.visual_quality, args.hdri)
        robots = spawn_robots(profile, scene, args.control_quality, args.robot, args.grasp_demo)

        def combined(field):
            return torch.cat([getattr(r.data, field) for r in robots.values()], dim=1)

        objects = spawn_scene_objects(args, scene, sim_utils)
        cameras = {}
        for camera_profile in profile["cameras"]:
            name = camera_profile["name"]
            body = camera_profile.get("body")
            camera_robot = robots[camera_profile.get("instance", "robot")] if body else None
            parent = f"{camera_robot.cfg.prim_path}/{body}" if body else "/World"
            if body and not sim_utils.get_current_stage().GetPrimAtPath(parent).IsValid():
                raise ValueError(f"Camera parent not found: {parent}")
            prim = f"{parent}/{name}_camera"
            cameras[name] = Camera(
                CameraCfg(
                    prim_path=prim,
                    update_period=0,
                    width=args.width,
                    height=args.height,
                    data_types=["rgb"],
                    spawn=sim_utils.PinholeCameraCfg(
                        focal_length=camera_profile.get("focal_length", 18),
                        horizontal_aperture=20.955,
                        clipping_range=(0.02, 20),
                    ),
                )
            )
        sim.reset()
        # Apply the configured pose before deriving camera mounts from world views.
        for robot in robots.values():
            robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
            robot.set_joint_position_target(robot.data.default_joint_pos)
            from simulation_quality import apply_feedforward

            apply_feedforward(robot, args.control_quality)
            robot.write_data_to_sim()
        sim.step(render=False)
        for robot in robots.values():
            robot.update(dt)
        cameras["scene"].set_world_poses_from_view(
            torch.tensor([scene["scene_eye"] if scene else [1.6, 1.4, 1.2]], device=sim.device),
            torch.tensor([scene["scene_target"] if scene else [0.25, 0, 0.35]], device=sim.device),
        )
        mount_cameras(profile, scene, robots, cameras, sim)
        names = joint_names(robots, profile)
        defaults = combined("default_joint_pos")[0].cpu().tolist()
        limits = combined("soft_joint_pos_limits")[0].cpu().tolist()
        state.configure_profile(names, limits, defaults, profile)
        from kinematics import load_kinematics

        state.kinematics = load_kinematics(args.robot, sim_utils.get_current_stage(), robots, names, limits)
        # Compare chain FK with actual physics poses in each independent arm base.
        import numpy as np
        from kinematics import transform

        from isaaclab.utils.math import subtract_frame_transforms

        for arm, chain in state.kinematics.items():
            definition = chain.definition
            body_robot = robots[definition["instance"]]
            base_id = body_robot.body_names.index(definition["base"])
            end_id = body_robot.body_names.index(definition["end_body"])
            p, q = subtract_frame_transforms(
                body_robot.data.body_pos_w[:, base_id],
                body_robot.data.body_quat_w[:, base_id],
                body_robot.data.body_pos_w[:, end_id],
                body_robot.data.body_quat_w[:, end_id],
            )
            actual = transform(p[0].cpu().tolist(), q[0].cpu().tolist()) @ chain.tool
            angles = [combined("joint_pos")[0, names.index(n)].item() for n in chain.joint_names]
            error = float(np.max(np.abs(actual - chain.matrix(angles))))
            if error > 0.002:
                raise ValueError(f"FK/physics mismatch for {arm}: {error}")
            print(f"KINEMATICS_VERIFIED arm={arm} max_matrix_error={error:.8g}", flush=True)
        for _ in range(6):
            sim.render()
        print("WEB_CONTROL_READY", flush=True)
        start = time.monotonic()
        next_tick = next_capture = start
        sim_time = 0.0
        capture_interval = 1.0 / args.camera_fps
        capture_time = -capture_interval
        while app.is_running() and not stop:
            now = time.monotonic()
            if args.duration and now - start >= args.duration:
                break
            tick = state.consume(dt)
            if tick["reset"]:
                for robot in robots.values():
                    robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
                    robot.reset()
                for obj in objects:
                    obj.write_root_state_to_sim(obj.data.default_root_state.clone())
                    obj.reset()
                sim_time = 0.0
                capture_time = -capture_interval
                next_capture = 0
            if tick["step"]:
                start_joint = 0
                for robot in robots.values():
                    end_joint = start_joint + robot.num_joints
                    targets = torch.tensor([tick["targets"][start_joint:end_joint]], device=sim.device)
                    robot.set_joint_position_target(targets)
                    apply_feedforward(robot, args.control_quality)
                    robot.write_data_to_sim()
                    start_joint = end_joint
                sim.step(render=False)
                sim_time += dt
                for robot in robots.values():
                    robot.update(dt)
                for obj in objects:
                    obj.update(dt)
            images = None
            if capture_due(args.capture_clock, sim_time, capture_time, capture_interval, now, next_capture):
                sim.render()
                images = {}
                for name, camera in cameras.items():
                    camera.update(capture_interval, force_recompute=True)
                    pixels = camera.data.output["rgb"][0, ..., :3].cpu().numpy()
                    buffer = io.BytesIO()
                    Image.fromarray(pixels).save(buffer, format="JPEG", quality=args.jpeg_quality, subsampling=0)
                    images[name] = buffer.getvalue()
                capture_time = sim_time
                next_capture = time.monotonic() + capture_interval
            # Pair measured flange poses with the same physical state as joint observations.
            measured_eef = {}
            for arm, chain in state.kinematics.items():
                definition = chain.definition
                body_robot = robots[definition["instance"]]
                base_id = body_robot.body_names.index(definition["base"])
                end_id = body_robot.body_names.index(definition["end_body"])
                p, q = subtract_frame_transforms(
                    body_robot.data.body_pos_w[:, base_id],
                    body_robot.data.body_quat_w[:, base_id],
                    body_robot.data.body_pos_w[:, end_id],
                    body_robot.data.body_quat_w[:, end_id],
                )
                from kinematics import pose

                measured_eef[arm] = pose(transform(p[0].cpu().tolist(), q[0].cpu().tolist()) @ chain.tool)
            with state.lock:
                state.eef_poses = measured_eef
                state.object_states = {obj.cfg.prim_path: obj.data.root_state_w[0].cpu().tolist() for obj in objects}
                state.publish(
                    sim_time, combined("joint_pos")[0].cpu().tolist(), combined("joint_vel")[0].cpu().tolist(), images
                )
            # Bound loop speed; do not accumulate catch-up bursts after slow rendering.
            next_tick = max(next_tick + dt, time.monotonic())
            time.sleep(max(0, next_tick - time.monotonic()))
        state.set_phase("stopped", "Simulation stopped")
    except Exception as exc:
        state.set_phase("error", f"{type(exc).__name__}: {exc}")
        traceback.print_exc()
        # Give an already open browser time to display the cause before exit.
        time.sleep(3)
        raise
    finally:
        close_recording(state)
        web.stop()
        if sim is not None:
            sim.clear_all_callbacks()
            sim.clear_instance()
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
