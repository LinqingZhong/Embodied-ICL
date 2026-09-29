"""Opt-in Isaac Sim check of table contact, object reset and instance physics overrides.

Run with isaaclab.sh -p scripts/demos/web_control/tests/check_scene_assets.py --headless.
"""

import argparse
import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scene_config import load_scene_config

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

from scene_assets import spawn_assets

from pxr import Usd, UsdGeom, UsdPhysics

import isaaclab.sim as sim_utils

sim = None
try:
    scene = load_scene_config(Path(__file__).resolve().parents[1] / "scenes/table_bottle.json")
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device=args.device))
    objects = spawn_assets(scene)
    stage = sim_utils.get_current_stage()
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    print(
        "INITIAL_BOTTLE_BOUNDS",
        bounds.ComputeWorldBound(stage.GetPrimAtPath("/World/Assets/Bottle")).ComputeAlignedRange(),
        flush=True,
    )
    visual = copy.deepcopy(scene["assets"][0])
    visual.update(name="VisualTable", mode="visual", position=[3, 0, 0])
    assert spawn_assets({"assets": [visual]}) == []
    colliders = [
        p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Assets/VisualTable")) if p.HasAPI(UsdPhysics.CollisionAPI)
    ]
    assert colliders and all(not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() for p in colliders)
    sim.reset()
    obj = objects[0]
    initial = obj.data.default_root_state.clone()
    for _ in range(240):
        sim.step(render=False)
        obj.update(1 / 120)
    resting = obj.data.root_state_w.clone()
    print("RESTING_BOTTLE_STATE", resting.cpu().tolist(), flush=True)
    assert 0.07 < resting[0, 2].item() < 0.15, "Bottle must rest upright on the tabletop"
    assert resting[0, 7:10].norm().item() < 0.05, "Bottle must settle on the table"
    obj.write_root_state_to_sim(initial)
    obj.reset()
    assert (obj.data.root_state_w - initial).abs().max().item() < 1e-5
    print("SCENE_ASSETS_PASS", flush=True)
finally:
    if sim is not None:
        sim.clear_all_callbacks()
        sim.clear_instance()
    app.close()
