"""Create local primitive USD assets for the human-video imitation experiment.

Run with an OpenUSD Python installation or Isaac Lab's Python environment::

    python scripts/demos/web_control/examples/create_imitation_scene.py \
        --output-dir output/video_imitation/scene

This is an approximate scene reconstruction, not a calibrated reconstruction.
Generation uses pxr only and does not launch Isaac Sim or access the network.
"""

import argparse
import json
import math
import os
import sys
import sysconfig
from pathlib import Path


def prepare_usd_environment():
    """Use Isaac Sim's installed USD libraries without starting a Kit application."""
    try:
        from pxr import Usd  # noqa: F401

        return
    except ImportError as error:
        if os.environ.get("IMITATION_SCENE_USD_BOOTSTRAPPED"):
            raise RuntimeError("The installed USD libraries could not be loaded") from error
    cache = Path(sysconfig.get_paths()["purelib"]) / "isaacsim/extscache"
    candidates = sorted(cache.glob("omni.usd.libs-*"))
    if not candidates:
        raise RuntimeError("Run with Python containing OpenUSD or an installed Isaac Sim environment")
    library = candidates[-1]
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join([str(library), env.get("PYTHONPATH", "")])
    env["LD_LIBRARY_PATH"] = os.pathsep.join(
        [
            str(library / "bin"),
            str(library / "bin/deps"),
            str(Path(sys.prefix) / "lib"),
            env.get("LD_LIBRARY_PATH", ""),
        ]
    )
    env["IMITATION_SCENE_USD_BOOTSTRAPPED"] = "1"
    os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]], env)


class Asset:
    """One portable USD with a default prim, visual materials and physical contacts."""

    def __init__(self, path, mass=None):
        from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

        self.path = path
        self.stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        self.root = UsdGeom.Xform.Define(self.stage, "/Asset").GetPrim()
        self.stage.SetDefaultPrim(self.root)
        if mass is not None:
            body = UsdPhysics.RigidBodyAPI.Apply(self.root)
            body.CreateRigidBodyEnabledAttr(True)
            body.CreateKinematicEnabledAttr(False)
            UsdPhysics.MassAPI.Apply(self.root).CreateMassAttr(mass)
        material = UsdShade.Material.Define(self.stage, "/Asset/Materials/Contact")
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(1.0)
        physics.CreateDynamicFrictionAttr(0.8)
        physics.CreateRestitutionAttr(0.0)
        self.contact = material

    def material(self, name, color, metallic=0.0, roughness=0.6):
        from pxr import Gf, Sdf, UsdShade

        path = f"/Asset/Materials/{name}"
        material = UsdShade.Material.Define(self.stage, path)
        shader = UsdShade.Shader.Define(self.stage, path + "/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    def shape(self, name, kind, size, position, material, collision=True):
        from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

        path = "/Asset/" + name
        if kind == "box":
            shape = UsdGeom.Cube.Define(self.stage, path)
            shape.CreateSizeAttr(1.0)
        elif kind == "cylinder":
            # Explicit elliptical mesh keeps visual and collision dimensions identical.
            # PhysX cylinders do not support unequal X/Y scales reliably.
            shape = UsdGeom.Mesh.Define(self.stage, path)
            sides = 32
            shape.CreatePointsAttr(
                [
                    Gf.Vec3f(
                        size[0] / 2 * math.cos(i * 2 * math.pi / sides),
                        size[1] / 2 * math.sin(i * 2 * math.pi / sides),
                        z,
                    )
                    for z in (-size[2] / 2, size[2] / 2)
                    for i in range(sides)
                ]
            )
            faces = [list(reversed(range(sides))), list(range(sides, 2 * sides))]
            faces += [[i, (i + 1) % sides, (i + 1) % sides + sides, i + sides] for i in range(sides)]
            shape.CreateFaceVertexCountsAttr([len(face) for face in faces])
            shape.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
            shape.CreateSubdivisionSchemeAttr("none")
            if collision:
                UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr("convexHull")
            size = (1, 1, 1)
        else:
            raise ValueError(f"Unknown shape: {kind}")
        shape.AddTranslateOp().Set(Gf.Vec3d(*position))
        shape.AddScaleOp().Set(Gf.Vec3f(*size))
        prim = shape.GetPrim()
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(material)
        if collision:
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            binding.Bind(self.contact, materialPurpose="physics")
        return shape

    def save(self):
        self.stage.GetRootLayer().Save()


def create_assets(directory):
    """Green tabletop and compound yellow bottle; sizes are simulation assumptions."""
    directory.mkdir(parents=True, exist_ok=True)
    table = Asset(directory / "green_table.usda")
    green = table.material("Green", (0.025, 0.24, 0.14), roughness=0.85)
    table.shape("Top", "box", (1.0, 0.8, 0.08), (0, 0, 0), green)
    table.save()

    bottle = Asset(directory / "mustard_bottle.usda", mass=0.08)
    yellow = bottle.material("Yellow", (0.95, 0.62, 0.025), roughness=0.5)
    dark = bottle.material("Label", (0.13, 0.07, 0.018), roughness=0.8)
    # Root is at the bottom: body 0–0.12 m, shoulders 0.10–0.14 m, cap to 0.16 m.
    bottle.shape("Body", "cylinder", (0.060, 0.045, 0.12), (0, 0, 0.06), yellow)
    bottle.shape("Shoulder", "cylinder", (0.046, 0.038, 0.035), (0, 0, 0.1225), yellow)
    bottle.shape("Neck", "cylinder", (0.023, 0.023, 0.018), (0, 0, 0.144), yellow)
    bottle.shape("Cap", "cylinder", (0.016, 0.016, 0.015), (0, 0, 0.1525), yellow)
    bottle.shape("FrontLabel", "box", (0.032, 0.001, 0.048), (0, -0.0226, 0.064), dark, False)
    bottle.save()

    box = Asset(directory / "rear_box.usda")
    red = box.material("Red", (0.35, 0.045, 0.035), roughness=0.8)
    white = box.material("White", (0.8, 0.8, 0.75), roughness=0.8)
    box.shape("Body", "box", (0.24, 0.10, 0.10), (0, 0, 0.05), red)
    box.shape("Top", "box", (0.24, 0.10, 0.015), (0, 0, 0.1075), white)
    box.save()


def create_scene(output_dir):
    """Generate geometry only; task labels and goal coordinates never enter robot observations."""
    output_dir = Path(output_dir).expanduser().resolve()
    create_assets(output_dir / "assets")
    scene = dict(
        ground_z=0,
        robot_position=[0, 0, 0],
        scene_eye=[1.4, -1.6, 1.7],
        scene_target=[-0.1, 0.0, 0.85],
        wrist_target=[-0.3, -0.08, 0.85],
        assets=[
            dict(name="Table", usd_path="assets/green_table.usda", mode="static", position=[0, 0.0, 0.725]),
            dict(
                name="MustardBottle",
                usd_path="assets/mustard_bottle.usda",
                mode="dynamic",
                position=[-0.3, -0.08, 0.767],
            ),
            dict(name="RearBox", usd_path="assets/rear_box.usda", mode="static", position=[-0.05, 0.30, 0.765]),
        ],
    )
    scene_path = output_dir / "scene.json"
    scene_path.write_text(json.dumps(scene, indent=2) + "\n")
    provenance = dict(
        description="Approximate green tabletop and yellow mustard bottle from user-selected DexMV human video.",
        source_video="output/human_demo_source/dexmv_mustard_original.mp4",
        metric_calibration=False,
        model_input=False,
        table_top_z=0.765,
        bottle_dimensions_m=[0.06, 0.045, 0.16],
        bottle_mass_kg=0.08,
        limitations=[
            "Primitive approximation, not the original calibrated YCB mesh.",
            "Dimensions, positions and physical parameters are simulation choices.",
            "The human video does not show release; no release target or success truth is supplied to GPT.",
        ],
    )
    (output_dir / "scene_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return scene_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("output/video_imitation/scene"))
    args = parser.parse_args()
    prepare_usd_environment()
    print(create_scene(args.output_dir))


if __name__ == "__main__":
    main()
