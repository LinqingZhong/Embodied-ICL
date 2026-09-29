"""ARX X5 articulation based on RoboDojo's env/robot_manager/robot_config/x5.py.

Source repository: https://github.com/RoboDojo-Benchmark/RoboDojo
Local asset data is managed by isaaclab_assets. Optional override: ARX_X5_USD.
"""

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from isaaclab_assets import ISAACLAB_ASSETS_DATA_DIR

DEFAULT_USD = Path(ISAACLAB_ASSETS_DATA_DIR) / "Robots/ARX/X5/ARX.usd"
USD_PATH = Path(os.environ.get("ARX_X5_USD", str(DEFAULT_USD))).expanduser().resolve()

ARX_X5_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, max_depenetration_velocity=5.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0, 0, 0),
        joint_pos={**{f"joint{i}": 0.0 for i in range(1, 7)}, "joint7": 0.044, "joint8": 0.044},
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-6]"],
            effort_limit_sim=100.0,
            velocity_limit_sim=5.0,
            stiffness=4400.0,
            damping=40.0,
            armature=0.01,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["joint7", "joint8"],
            effort_limit_sim=100.0,
            stiffness=2300.0,
            damping=100.0,
        ),
    },
)
