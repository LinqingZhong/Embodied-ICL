"""Offline evidence checks for annotated model recordings; no simulator or model required."""

import io
import json
import shutil
import sys
from pathlib import Path

import h5py
import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))


def test_compact_timeline_keeps_every_action_through_receipt_at_recorded_speed():
    from export_gpt_video import build_timeline

    events = [
        {"event": "action", "turn": 1, "wall_time": 130},
        {"event": "result", "turn": 1, "wall_time": 135},
        {"event": "action", "turn": 2, "wall_time": 210},
        {"event": "result", "turn": 2, "wall_time": 221},
        {"event": "action", "turn": 3, "wall_time": 270},
        {"event": "error", "turn": 3, "wall_time": 277},
    ]
    segments = build_timeline(100, 280, events)
    assert segments[0]["source_start"] == 100
    assert segments[-1]["source_end"] == 280
    assert all(a["source_end"] == b["source_start"] for a, b in zip(segments, segments[1:]))
    for left, right in [(130, 135), (210, 221), (270, 277)]:
        retained = [s for s in segments if s["source_start"] <= left and s["source_end"] >= right]
        assert len(retained) == 1
        segment = retained[0]
        assert segment["kind"] == "recorded"
        assert segment["output_seconds"] == pytest.approx(segment["source_end"] - segment["source_start"])
    assert any(s["kind"] == "shortened_wait" for s in segments)
    assert sum(s["output_seconds"] for s in segments) < 75
    full = build_timeline(100, 280, events, full=True)
    assert sum(s["output_seconds"] for s in full) == 180


def test_model_camera_whitelist_uses_metadata_indices_and_excludes_scene():
    from export_gpt_video import camera_indices

    metadata = {"cameras": ["scene", "right_wrist", "head", "left_wrist"]}
    report = {"cameras": ["head", "left_wrist", "right_wrist"]}
    assert camera_indices(metadata, report) == [2, 3, 1]
    for names in [["scene"], [], ["head", "head"], ["missing"]]:
        with pytest.raises(ValueError):
            camera_indices(metadata, {"cameras": names})
    with pytest.raises(ValueError):
        camera_indices(metadata, {})


def test_public_response_keeps_all_trajectory_rows_and_assistant_text():
    from export_gpt_video import public_response_text

    rows = [[i + 0.123456789, -i, i / 100] for i in range(90)]
    text = public_response_text(
        {
            "output": [
                {"type": "reasoning", "summary": [{"text": "NOT PUBLIC"}]},
                {"type": "function_call", "name": "move_ee", "arguments": json.dumps({"positions": rows})},
                {
                    "type": "message",
                    "content": [{"type": "output_text", "text": "Release after lifting. 抬升后松开。"}],
                },
            ]
        }
    )
    assert "move_ee" in text
    assert "Release after lifting. 抬升后松开。" in text
    assert "NOT PUBLIC" not in text
    for row in rows:
        assert json.dumps(row) in text


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg is required for video integration")
def test_export_writes_paginated_public_answers_verdict_and_camera_manifest(tmp_path):
    from export_gpt_video import export

    times = np.arange(100, 104.1, 0.2)
    episode = tmp_path / "episode.hdf5"
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
    with h5py.File(episode, "w") as file:
        file.attrs["complete"] = True
        file.attrs["metadata_json"] = json.dumps({"cameras": ["scene", "right", "head", "left"], "robot": "Fixture"})
        file.create_dataset("frames/wall_time", data=times)
        file.create_dataset("frames/sim_time", data=times - 100)
        for index, color in enumerate(colors):
            buffer = io.BytesIO()
            Image.new("RGB", (100, 80), color).save(buffer, format="JPEG")
            dataset = file.create_dataset(
                f"images/camera_{index}", (len(times),), dtype=h5py.vlen_dtype(np.dtype("uint8"))
            )
            for frame in range(len(times)):
                dataset[frame] = np.frombuffer(buffer.getvalue(), dtype=np.uint8)
    rows = [[i + 0.123456789, i / 10, -i] for i in range(60)]
    report = {
        "recording": {"path": str(episode)},
        "model": "fixture-model",
        "robot": "Fixture",
        "instruction": "Pick up the cube",
        "cameras": ["head", "left", "right"],
        "results": [],
        "error": "Fixture motion was rejected",
        "evaluation": {"success": False, "verdict": "Not grasped"},
    }
    (tmp_path / "result.json").write_text(json.dumps(report))
    events = [
        {"event": "predict_start", "turn": 1, "wall_time": 100.1},
        {
            "event": "model_response",
            "turn": 1,
            "wall_time": 101,
            "elapsed_seconds": 0.9,
            "output": [{"type": "function_call", "name": "move_ee", "arguments": json.dumps({"positions": rows})}],
        },
        {"event": "action", "turn": 1, "wall_time": 101.1, "action_type": "MoveEE", "action": {"positions": rows}},
        {"event": "result", "turn": 1, "wall_time": 102, "result": {"status": "rejected", "executed_steps": 0}},
        {"event": "error", "turn": 1, "wall_time": 102.5, "error": report["error"]},
        {"event": "end", "turn": 1, "wall_time": 103, "error": report["error"]},
    ]
    (tmp_path / "trace.jsonl").write_text("\n".join(json.dumps(event) for event in events))
    output = tmp_path / "annotated.mp4"
    manifest = export(tmp_path, output, fps=2, page_seconds=0.5, final_seconds=1)
    assert output.stat().st_size > 0
    assert output.with_suffix(".jpg").exists()
    assert output.with_name("annotated_contact.jpg").exists()
    assert output.with_suffix(".json").exists()
    assert manifest["cameras"] == ["head", "left", "right"]
    assert manifest["camera_indices"] == [2, 3, 1]
    assert manifest["evaluation"]["success"] is False
    assert manifest["model_error"] == report["error"]
    response_pages = [page for page in manifest["annotation_pages"] if page["event"] == "model_response"]
    assert len(response_pages) >= 3
    shown = "\n".join(page["text"] for page in response_pages)
    for row in rows:
        assert json.dumps(row) in shown
    assert any(page["event"] == "result" and "rejected" in page["text"] for page in manifest["annotation_pages"])
    assert any(page["event"] == "final" and "Not grasped" in page["text"] for page in manifest["annotation_pages"])
    with Image.open(output.with_suffix(".jpg")) as sample:
        # Left half contains camera tiles: blue head, yellow left, green right; scene red is excluded.
        pixels = np.asarray(sample)[:, :960]
        red = (pixels[:, :, 0] > 200) & (pixels[:, :, 1] < 50) & (pixels[:, :, 2] < 50)
        assert np.count_nonzero(red) == 0
