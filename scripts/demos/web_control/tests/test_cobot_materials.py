"""USD material regression; requires pxr (available in Isaac Sim)."""

import sys
from pathlib import Path

import pytest

pytest.importorskip("pxr")
from pxr import Usd, UsdGeom, UsdShade

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot.cobot_materials import restore_arm_materials


def test_arm_submesh_colors_win_without_changing_body():
    stage = Usd.Stage.CreateInMemory()
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/robot").GetPrim())
    pale = UsdShade.Material.Define(stage, "/robot/Looks/pale")
    dark = UsdShade.Material.Define(stage, "/robot/Looks/dark")
    for name in ("fr_link2", "body_Link"):
        visual = UsdGeom.Xform.Define(stage, f"/robot/{name}/visuals/link2").GetPrim()
        mesh = UsdGeom.Mesh.Define(stage, visual.GetPath().AppendChild("mesh")).GetPrim()
        UsdShade.MaterialBindingAPI.Apply(visual).Bind(pale, UsdShade.Tokens.strongerThanDescendants)
        UsdShade.MaterialBindingAPI.Apply(mesh).Bind(dark)
        assert UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()[0].GetPath() == pale.GetPath()
    assert restore_arm_materials(stage) == ["/robot/fr_link2/visuals/link2"]
    for name, expected in (("fr_link2", dark), ("body_Link", pale)):
        mesh = stage.GetPrimAtPath(f"/robot/{name}/visuals/link2/mesh")
        assert UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()[0].GetPath() == expected.GetPath()
    assert restore_arm_materials(stage) == []
