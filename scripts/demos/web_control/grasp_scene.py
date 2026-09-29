"""Known collision scene for the Franka planned pick demonstration."""


def spawn_grasp_scene(robot="franka", arm=None):
    import isaaclab.sim as sim
    from isaaclab.assets import RigidObject, RigidObjectCfg

    fixture = dict(
        table_dims=[0.7, 0.8, 0.1],
        table_position=[0.55, 0, 0.15],
        table_orientation=[1, 0, 0, 0],
        cube_position=[0.5, 0, 0.22],
    )
    if robot != "franka":
        from planning.grasp_profiles import grasp_spec

        fixture = grasp_spec(robot, arm)
    material = sim.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.8, restitution=0)
    table = sim.CuboidCfg(
        size=tuple(fixture["table_dims"]),
        collision_props=sim.CollisionPropertiesCfg(),
        physics_material=material,
        visual_material=sim.PreviewSurfaceCfg(diffuse_color=(0.22, 0.25, 0.28), roughness=0.7),
    )
    table.func(
        "/World/GraspTable",
        table,
        translation=tuple(fixture["table_position"]),
        orientation=tuple(fixture["table_orientation"]),
    )
    cube = RigidObject(
        RigidObjectCfg(
            prim_path="/World/GraspCube",
            spawn=sim.CuboidCfg(
                size=tuple(fixture.get("cube_size", [0.04, 0.04, 0.04])),
                rigid_props=sim.RigidBodyPropertiesCfg(),
                mass_props=sim.MassPropertiesCfg(mass=0.05),
                collision_props=sim.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0),
                physics_material=material,
                visual_material=sim.PreviewSurfaceCfg(diffuse_color=(0.75, 0.12, 0.045), roughness=0.5),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=tuple(v + (0.002 if i == 2 else 0) for i, v in enumerate(fixture["cube_position"]))
            ),
        )
    )
    return [cube]
