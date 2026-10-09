import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path[:0] = [
    str(Path(__file__).resolve().parents[1] / "examples"),
    str(Path(__file__).resolve().parents[4] / "source/isaaclab_policies"),
]


def test_camera_selection_filters_spec_and_observation_without_mutating_source():
    from record_gpt_policy import CameraClient

    spec = {"cameras": ["scene", "head", "left_wrist", "right_wrist"]}

    @dataclass(frozen=True)
    class Observation:
        images: dict
        positions: list

    obs = Observation(images={name: name.encode() for name in spec["cameras"]}, positions=[0.2])
    client = SimpleNamespace(get_robot_spec=lambda: spec, get_observation=lambda: obs)
    selected = CameraClient(client, ["head", "left_wrist", "right_wrist"])
    assert selected.get_robot_spec()["cameras"] == ["head", "left_wrist", "right_wrist"]
    assert list(selected.get_observation().images) == ["head", "left_wrist", "right_wrist"]
    assert "scene" in obs.images and "scene" in spec["cameras"]
    assert selected.get_observation().positions == [0.2]


@pytest.mark.parametrize("names", [["scene"], [], ["head", "head"], ["missing"]])
def test_reject_bad_camera_selection(names):
    from record_gpt_policy import CameraClient

    client = SimpleNamespace(get_robot_spec=lambda: {"cameras": ["scene", "head"]})
    with pytest.raises(ValueError):
        CameraClient(client, names).get_robot_spec()
