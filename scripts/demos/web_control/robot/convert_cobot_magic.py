"""Convert bundled Cobot Magic URDF using the installed Isaac Sim URDF importer."""

from pathlib import Path

from isaaclab.app import AppLauncher

app = AppLauncher(headless=True).app
import omni.kit.commands

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.sim.converters.asset_converter_base import AssetConverterBase

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
from isaacsim.asset.importer.urdf._urdf import acquire_urdf_interface

from isaaclab_assets import ISAACLAB_ASSETS_DATA_DIR


class InstalledUrdfConverter(UrdfConverter):
    """Use installed 2.4.30 rather than triggering a remote 2.4.31 download."""

    def __init__(self, cfg):
        self._urdf_interface = acquire_urdf_interface()
        AssetConverterBase.__init__(self, cfg=cfg)

    def _get_urdf_import_config(self):
        _, config = omni.kit.commands.execute("URDFCreateImportConfig")
        config.set_distance_scale(1.0)
        config.set_make_default_prim(True)
        config.set_create_physics_scene(False)
        config.set_density(0.0)
        config.set_convex_decomp(False)
        config.set_collision_from_visuals(False)
        config.set_merge_fixed_joints(False)
        config.set_fix_base(True)
        config.set_self_collision(False)
        return config


try:
    root = Path(ISAACLAB_ASSETS_DATA_DIR) / "Robots/AgileX/CobotMagic"
    cfg = UrdfConverterCfg(
        asset_path=str(root / "cobot_magic.urdf"),
        usd_dir=str(root / "usd"),
        usd_file_name="cobot_magic.usd",
        fix_base=True,
        merge_fixed_joints=False,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=100, damping=10), target_type="position"
        ),
    )
    usd_path = InstalledUrdfConverter(cfg).usd_path
    from cobot_materials import restore_arm_materials

    from pxr import Usd

    stage = Usd.Stage.Open(usd_path)
    changed = restore_arm_materials(stage)
    stage.GetRootLayer().Save()
    print(f"Restored embedded materials on {len(changed)} arm visuals.", flush=True)
    print("CONVERTED", usd_path, flush=True)
except Exception:
    import traceback

    traceback.print_exc()
    raise
finally:
    app.close()
