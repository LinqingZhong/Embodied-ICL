"""Fixed-base Cobot Magic using the supplied mobile_aloha_sim-2.0.0 URDF."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from isaaclab_assets import ISAACLAB_ASSETS_DATA_DIR

COBOT_MAGIC_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/AgileX/CobotMagic/usd/cobot_magic.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            fix_root_link=True,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0, 0, 0.15),
        joint_pos={
            "left_joint|right_joint|camera_stand_[12]_joint|[fb][lr]_joint[1456]": 0.0,
            "[fb][lr]_joint2": 1.0,
            "[fb][lr]_joint3": -1.2,
            "[fb][lr]_joint7": 0.04,
            "[fb][lr]_joint8": -0.04,
        },
    ),
    actuators={
        "arms": ImplicitActuatorCfg(
            joint_names_expr=["[fb][lr]_joint[1-6]"], stiffness=400, damping=40, effort_limit_sim=100, armature=0.01
        ),
        "grippers": ImplicitActuatorCfg(
            joint_names_expr=["[fb][lr]_joint[78]"], stiffness=300, damping=20, effort_limit_sim=10
        ),
        "stand": ImplicitActuatorCfg(
            joint_names_expr=["camera_stand_[12]_joint"], stiffness=50000, damping=600, effort_limit_sim=500
        ),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["left_joint", "right_joint"], stiffness=0, damping=10, effort_limit_sim=10
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
