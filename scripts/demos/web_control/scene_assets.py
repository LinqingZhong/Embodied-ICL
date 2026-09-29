"""USD spawning; import only after AppLauncher has started."""

from pxr import Usd, UsdPhysics

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


def spawn_assets(scene):
    """Return dynamic bodies for resetting. Static USDs must contain colliders."""
    objects = []
    for asset in scene["assets"]:
        source = asset["usd_path"]
        if source.startswith("isaac://"):
            source = f"{ISAAC_NUCLEUS_DIR}/{source[len('isaac://') :]}"
        mode = asset["mode"]
        cfg = sim_utils.UsdFileCfg(usd_path=source, scale=tuple(asset["scale"]))
        prim_path = f"/World/Assets/{asset['name']}"
        root = cfg.func(prim_path, cfg, translation=tuple(asset["position"]), orientation=tuple(asset["orientation"]))
        # Instance proxies cannot be edited. These are stage-local overrides;
        # source USD files remain unchanged.
        sim_utils.make_uninstanceable(prim_path)
        prims = list(Usd.PrimRange(root))
        bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        colliders = [p for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)]
        if any(p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in prims):
            raise ValueError(f"{asset['name']}: articulated USDs require a robot loader")
        if mode != "visual" and not colliders:
            raise ValueError(
                f"{asset['name']}: {mode} USD requires authored collision geometry; use visual mode or convert it"
            )
        if mode == "dynamic" and len(bodies) != 1:
            raise ValueError(f"{asset['name']}: dynamic USD requires exactly one rigid body; convert with --mass")
        for prim in bodies:
            body = UsdPhysics.RigidBodyAPI(prim)
            body.CreateRigidBodyEnabledAttr(mode != "visual")
            body.CreateKinematicEnabledAttr(mode == "static")
        for prim in colliders:
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(mode != "visual")
        if mode == "dynamic":
            objects.append(
                RigidObject(
                    RigidObjectCfg(
                        prim_path=prim_path,
                        spawn=None,
                        init_state=RigidObjectCfg.InitialStateCfg(
                            pos=tuple(asset["position"]), rot=tuple(asset["orientation"])
                        ),
                    )
                )
            )
        print(f"WEB_CONTROL_ASSET={asset['name']} mode={mode} source={source}", flush=True)
    return objects
