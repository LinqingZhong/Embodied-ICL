"""Preserve embedded DAE colors under Cobot Magic's URDF material fallbacks."""

import re


def restore_arm_materials(stage):
    """Author root-layer overrides; retain the URDF color only as a fallback.

    The original URDF inconsistently assigns a uniform color to three arms.
    The importer makes that binding stronger than the DAE's submesh materials.
    Only change arm visuals with embedded material bindings, leaving physics,
    body colors and the original source files intact.
    """
    from pxr import Usd, UsdShade

    changed = []
    with Usd.EditContext(stage, stage.GetRootLayer()):
        for link in stage.GetDefaultPrim().GetChildren():
            if not re.fullmatch(r"(?:fl|fr|bl|br)_(?:base_link|link[1-8]|camera_stand)", link.GetName()):
                continue
            visuals = link.GetChild("visuals")
            if not visuals:
                continue
            # Imported visuals are instances; descendants cannot receive overrides
            # until this visual subtree is made editable (physics stays untouched).
            was_instance = visuals.IsInstance()
            if was_instance:
                visuals.SetInstanceable(False)
            count_before = len(changed)
            for visual in visuals.GetChildren():
                binding = visual.GetRelationship("material:binding")
                if not binding or not binding.GetTargets():
                    continue
                if (
                    UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(binding)
                    != UsdShade.Tokens.strongerThanDescendants
                ):
                    continue
                embedded = any(
                    p != visual and p.GetRelationship("material:binding").GetTargets()
                    for p in Usd.PrimRange(visual, Usd.TraverseInstanceProxies())
                    if p.GetRelationship("material:binding")
                )
                if embedded:
                    UsdShade.MaterialBindingAPI.SetMaterialBindingStrength(
                        binding, UsdShade.Tokens.weakerThanDescendants
                    )
                    changed.append(str(visual.GetPath()))
            if was_instance and len(changed) == count_before:
                visuals.ClearInstanceable()
    return changed


if __name__ == "__main__":
    import argparse

    from pxr import Usd

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("usd_path")
    args = parser.parse_args()
    stage = Usd.Stage.Open(args.usd_path)
    changed = restore_arm_materials(stage)
    stage.GetRootLayer().Save()
    print(f"Restored embedded materials on {len(changed)} arm visuals.")
