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


@pytest.mark.parametrize("mode", ["frames", "description"])
def test_recording_saves_demo_interpretation_and_counts_reference_images(tmp_path, monkeypatch, mode):
    import json

    from isaaclab_policies import GPTConfig, GPTPolicy, PolicyObservation, RobotSpec
    from isaaclab_policies.demonstration import IMITATION_INSTRUCTION, DemonstrationFrame, VideoDemonstration
    from record_gpt_policy import RecordedGPTPolicy, parse_args

    args = parse_args(
        [
            "--output",
            str(tmp_path),
            "--config",
            "config.json",
            "--cameras",
            "head",
            "--demo-video",
            "demo.mp4",
            "--icl-mode",
            mode,
        ]
    )
    assert args.instruction is None
    assert args.icl_mode == mode
    monkeypatch.setenv("POLICY_TEST_KEY", "placeholder")
    reply = dict(
        status="ready",
        object_description="block",
        observed_task="move it",
        stages=["lift", "place"],
        completion_condition="block rests",
        uncertainties=[],
    )
    monkeypatch.setattr(
        GPTPolicy,
        "_request",
        lambda *_: dict(
            status="completed",
            output=[
                dict(
                    type="function_call",
                    name="interpret_demonstration",
                    arguments=json.dumps(reply),
                    call_id="interpret",
                )
            ],
        ),
    )
    demo = VideoDemonstration(
        (DemonstrationFrame(0, b"\xff\xd8a"), DemonstrationFrame(1, b"\xff\xd8b")), 1.1, "a" * 64, "/demo.mp4"
    )
    policy = RecordedGPTPolicy(GPTConfig(api_key_env="POLICY_TEST_KEY", icl_mode=mode), tmp_path, demonstration=demo)
    spec = RobotSpec(
        name="test",
        joint_names=["j"],
        controllable_joint_names=["j"],
        limits=[[-1, 1]],
        joint_units={"j": "rad"},
        grippers=[],
        arms={},
        cameras=["head"],
        physics_dt=1 / 120,
    )
    policy.reset(IMITATION_INSTRUCTION, spec)
    assert (tmp_path / "demonstration/manifest.json").exists()
    assert json.loads((tmp_path / "demonstration_interpretation.json").read_text()) == reply
    events = [json.loads(s) for s in (tmp_path / "trace.jsonl").read_text().splitlines()]
    request = next(e for e in events if e["event"] == "model_request")
    assert request["image_count"] == 2 and request["demonstration_image_count"] == 2
    assert request["robot_image_count"] == 0 and request["cameras"] == []
    assert request["phase"] == "demonstration_interpretation"
    assert request["icl_mode"] == mode
    assert (tmp_path / "demonstration_instruction.txt").read_text() == policy.demonstration_instruction
    obs = PolicyObservation(
        sequence=1,
        sim_time=1,
        captured_at=1,
        joint_names=["j"],
        positions=[0],
        velocities=[0],
        targets=[0],
        eef_poses={},
        images={"head": b"\xff\xd8robot"},
    )
    policy._request(policy._build_payload(policy._observation_message(obs)))
    events = [json.loads(s) for s in (tmp_path / "trace.jsonl").read_text().splitlines()]
    request = [e for e in events if e["event"] == "model_request"][-1]
    assert request["phase"] == "control"
    assert request["demonstration_image_count"] == (2 if mode == "frames" else 0)
    assert request["robot_image_count"] == 1
