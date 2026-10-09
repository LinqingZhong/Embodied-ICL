"""Check generated local assets without starting Isaac Sim."""

import json
import os
import subprocess
import sys
import sysconfig
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "examples/create_imitation_scene.py"
sys.path.insert(0, str(ROOT))


def usd_environment():
    env = os.environ.copy()
    cache = Path(sysconfig.get_paths()["purelib"]) / "isaacsim/extscache"
    candidates = sorted(cache.glob("omni.usd.libs-*"))
    if candidates:
        library = candidates[-1]
        env["PYTHONPATH"] = os.pathsep.join([str(library), env.get("PYTHONPATH", "")])
        env["LD_LIBRARY_PATH"] = os.pathsep.join(
            [
                str(library / "bin"),
                str(library / "bin/deps"),
                str(Path(sys.prefix) / "lib"),
                env.get("LD_LIBRARY_PATH", ""),
            ]
        )
    available = subprocess.run([sys.executable, "-c", "from pxr import Usd"], env=env, capture_output=True, text=True)
    if available.returncode:
        pytest.skip("OpenUSD or Isaac Sim's standalone USD libraries are required")
    return env


@pytest.fixture
def generated(tmp_path):
    env = usd_environment()
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--output-dir", str(tmp_path)], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return tmp_path, env


def test_manifest_references_local_assets_and_separates_provenance(generated):
    from scene_config import load_scene_config

    directory, _ = generated
    path = directory / "scene.json"
    scene = load_scene_config(path)
    raw = json.loads(path.read_text())
    assert len(scene["assets"]) >= 3
    assert len([asset for asset in scene["assets"] if asset["mode"] == "dynamic"]) == 1
    assert all(not Path(asset["usd_path"]).is_absolute() for asset in raw["assets"])
    assert all(Path(asset["usd_path"]).is_file() for asset in scene["assets"])
    provenance = json.loads((directory / "scene_provenance.json").read_text())
    assert provenance["metric_calibration"] is False
    assert provenance["model_input"] is False


def test_assets_have_collision_and_single_physical_bottle(generated):
    directory, env = generated
    check = r"""
import json, sys
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
directory = Path(sys.argv[1])
scene = json.loads((directory / "scene.json").read_text())
for asset in scene["assets"]:
    stage = Usd.Stage.Open(str(directory / asset["usd_path"]))
    assert stage and stage.GetDefaultPrim()
    assert UsdGeom.GetStageUpAxis(stage) == "Z"
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1
    prims = list(stage.Traverse())
    colliders = [p for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)]
    bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    assert colliders, asset["name"]
    assert all(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() for p in colliders)
    assert all(not p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in prims)
    if asset["mode"] == "dynamic":
        assert len(bodies) == 1
        body = bodies[0]
        assert UsdPhysics.RigidBodyAPI(body).GetRigidBodyEnabledAttr().Get()
        assert not UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get()
        assert 0.01 <= UsdPhysics.MassAPI(body).GetMassAttr().Get() <= 0.2
        assert len(colliders) == 4
        material, _ = UsdShade.MaterialBindingAPI(colliders[0]).ComputeBoundMaterial("physics")
        assert material
        assert UsdPhysics.MaterialAPI(material.GetPrim()).GetStaticFrictionAttr().Get() > 0.5
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(body).ComputeAlignedRange()
        size = bounds.GetSize()
        assert 0.035 <= min(size) <= 0.05, size
        assert 0.15 <= max(size) <= 0.17, size
    else:
        assert not bodies, asset["name"]
print("All local assets have valid physics metadata")
"""
    result = subprocess.run([sys.executable, "-c", check, str(directory)], env=env, text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr
