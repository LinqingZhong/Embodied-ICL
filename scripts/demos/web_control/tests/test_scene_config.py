import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def load(tmp_path, data):
    from scene_config import load_scene_config

    path = tmp_path / "scene.json"
    path.write_text(json.dumps(data))
    return load_scene_config(path)


def test_resolves_local_asset_relative_to_manifest(tmp_path):
    (tmp_path / "cup.usd").touch()
    scene = load(tmp_path, {"assets": [{"name": "Cup", "usd_path": "cup.usd", "mode": "dynamic"}]})
    assert scene["assets"][0]["usd_path"] == str(tmp_path / "cup.usd")
    assert scene["assets"][0]["orientation"] == [1, 0, 0, 0]
    assert scene["ground_z"] == 0


@pytest.mark.parametrize(
    "asset",
    [
        {"name": "../Cup", "usd_path": "isaac://cup.usd"},
        {"name": "Cup", "usd_path": "isaac://cup.usd", "scale": [1, 0, 1]},
        {"name": "Cup", "usd_path": "isaac://cup.usd", "position": [0, float("nan"), 0]},
        {"name": "Cup", "usd_path": "isaac://cup.usd", "orientation": [0, 0, 0, 0]},
        {"name": "Cup", "usd_path": "isaac://cup.usd", "mode": "typo"},
        {"name": "Cup", "usd_path": "missing.usd"},
        {"name": "Cup", "usd_path": "isaac://cup.usd", "postion": [0, 0, 0]},
    ],
)
def test_rejects_invalid_asset(tmp_path, asset):
    with pytest.raises(ValueError):
        load(tmp_path, {"assets": [asset]})


def test_rejects_duplicate_names(tmp_path):
    asset = {"name": "Cup", "usd_path": "isaac://cup.usd"}
    with pytest.raises(ValueError, match="Duplicate"):
        load(tmp_path, {"assets": [asset, asset]})


def test_preset_loads():
    from scene_config import load_scene_config

    scene = load_scene_config(Path(__file__).resolve().parents[1] / "scenes/table_bottle.json")
    assert scene["ground_z"] == -1.05
    assert [a["mode"] for a in scene["assets"]] == ["static", "dynamic"]
