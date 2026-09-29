"""Opt-in realistic gravity-compensated drives and studio lighting for the demo."""


def configure_drives(cfg, robot_name):
    """Keep gravity and torque limits; use stable implicit PD with bounded gravity feedforward."""
    from isaaclab.actuators import ImplicitActuatorCfg

    keys = {
        "franka": ["panda_shoulder", "panda_forearm"],
        "cobot_magic": ["arms"],
        "x5": ["arm"],
        "x5_dual": ["arm"],
    }.get(robot_name, [])
    if not keys:
        return cfg
    cfg.spawn.rigid_props.disable_gravity = False
    cfg.spawn.articulation_props.solver_position_iteration_count = 16
    cfg.spawn.articulation_props.solver_velocity_iteration_count = 4
    for key in keys:
        old = cfg.actuators[key]
        cfg.actuators[key] = ImplicitActuatorCfg(
            joint_names_expr=old.joint_names_expr,
            stiffness=400.0 if robot_name == "franka" else old.stiffness,
            damping=40.0 if robot_name == "franka" else old.damping,
            effort_limit_sim=old.effort_limit_sim,
            armature=0.01 if old.armature is None else old.armature,
        )
        cfg.actuators[key].gravity_compensation = True
    return cfg


def gravity_feedforward(robot):
    """Apply gravity compensation to tagged arm drives; bound feedforward to configured limits."""
    import torch

    efforts = torch.zeros_like(robot.data.joint_pos)
    gravity = robot.root_physx_view.get_gravity_compensation_forces()
    drive_limits = robot.data.joint_effort_limits.clone()
    for actuator in robot.actuators.values():
        if getattr(actuator.cfg, "gravity_compensation", False):
            efforts[:, actuator.joint_indices] = gravity[:, actuator.joint_indices].clamp(
                -actuator.effort_limit, actuator.effort_limit
            )
            # Reserve a conservative torque budget for feedforward so that the
            # implicit PD drive plus feedforward cannot exceed the original limit.
            drive_limits[:, actuator.joint_indices] = (
                actuator.effort_limit - efforts[:, actuator.joint_indices].abs()
            ).clamp(min=0)
    robot.write_joint_effort_limit_to_sim(drive_limits)
    robot.set_joint_effort_target(efforts)


def studio_lights(sim_utils, hdri=None):
    dome = sim_utils.DomeLightCfg(intensity=500, texture_file=hdri)
    dome.func("/World/Light", dome)
    # Large overhead warm key and cooler fill; area lights produce soft shadows.
    for name, position, intensity, radius, temperature in [
        ("Key", (0.5, -1.0, 3.0), 1800, 1.0, 5200),
        ("Fill", (-1.5, 1.0, 2.5), 900, 0.8, 6500),
    ]:
        light = sim_utils.DiskLightCfg(
            intensity=intensity, radius=radius, enable_color_temperature=True, color_temperature=temperature
        )
        light.func(f"/World/{name}", light, translation=position)


def apply_feedforward(robot, quality):
    if quality == "compensated":
        gravity_feedforward(robot)


def setup_visuals(sim_utils, quality, hdri):
    if quality == "studio":
        studio_lights(sim_utils, hdri)
        from pxr import UsdGeom

        stage = sim_utils.get_current_stage()
        # Replace the grid's visual material, preserving its collision plane.
        material = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.28, 0.30, 0.32), roughness=0.8)
        material.func("/World/Ground/StudioMaterial", material)
        from pxr import Usd, UsdShade

        mat = UsdShade.Material(stage.GetPrimAtPath("/World/Ground/StudioMaterial"))
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Ground")):
            if prim.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    else:
        light = sim_utils.DomeLightCfg(intensity=2500)
        light.func("/World/Light", light)
