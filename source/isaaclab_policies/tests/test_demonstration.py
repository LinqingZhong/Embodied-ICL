import json
import shutil
import subprocess

import pytest


def test_sample_video_preserves_order_endpoints_and_provenance(tmp_path):
    from isaaclab_policies.demonstration import load_video_demonstration

    if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
        pytest.skip("ffmpeg and ffprobe required")
    video = tmp_path / "target name must not reach the model.mp4"
    subprocess.run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-f",
            "lavfi",
            "-i",
            "testsrc2=size=160x120:rate=5:duration=2",
            "-c:v",
            "mpeg4",
            str(video),
        ],
        check=True,
    )
    demo = load_video_demonstration(video, max_frames=4, max_side=96)
    assert len(demo.frames) == 4
    assert demo.frames[0].timestamp_seconds == pytest.approx(0)
    assert demo.frames[-1].timestamp_seconds == pytest.approx(1.8)
    assert demo.duration_seconds == pytest.approx(2)
    assert all(f.jpeg.startswith(b"\xff\xd8") for f in demo.frames)
    assert len({f.jpeg for f in demo.frames}) > 1
    import io

    from PIL import Image

    assert all(max(Image.open(io.BytesIO(f.jpeg)).size) <= 96 for f in demo.frames)
    saved = demo.save(tmp_path / "sampled")
    manifest = json.loads(saved.read_text())
    assert len(manifest["source_sha256"]) == 64
    assert [f["timestamp_seconds"] for f in manifest["frames"]] == [f.timestamp_seconds for f in demo.frames]
    assert all(
        (saved.parent / f["file"]).read_bytes() == frame.jpeg for f, frame in zip(manifest["frames"], demo.frames)
    )
    content = json.dumps(demo.message())
    assert video.name not in content
    assert "human demonstration" in content.lower()
    assert content.count("input_image") == 4


@pytest.mark.parametrize("kwargs", [{"max_frames": 1}, {"max_frames": 33}, {"max_side": 0}, {"max_side": 2000}])
def test_invalid_sampling_options_fail_before_decoder(tmp_path, kwargs):
    from isaaclab_policies.demonstration import load_video_demonstration

    with pytest.raises(ValueError):
        load_video_demonstration(tmp_path / "missing.mp4", **kwargs)


def test_missing_video_is_reported(tmp_path):
    from isaaclab_policies.demonstration import load_video_demonstration

    with pytest.raises(FileNotFoundError):
        load_video_demonstration(tmp_path / "missing.mp4")


@pytest.mark.parametrize(
    "changes",
    [
        {"status": "unknown"},
        {"stages": []},
        {"uncertainties": "none"},
        {"observed_task": ""},
        {"completion_condition": None},
        {"extra": "not allowed"},
    ],
)
def test_interpretation_rejects_malformed_structured_output(changes):
    from isaaclab_policies.demonstration import parse_interpretation

    data = dict(
        status="ready",
        object_description="object",
        observed_task="move",
        stages=["move"],
        completion_condition="at destination",
        uncertainties=[],
    )
    data.update(changes)
    response = dict(
        status="completed",
        output=[dict(type="function_call", name="interpret_demonstration", arguments=json.dumps(data))],
    )
    with pytest.raises(ValueError):
        parse_interpretation(response)


def test_unreadable_video_never_produces_reference_frames(tmp_path):
    from isaaclab_policies.demonstration import load_video_demonstration

    if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
        pytest.skip("ffmpeg and ffprobe required")
    path = tmp_path / "broken.mp4"
    path.write_bytes(b"not a video")
    with pytest.raises(ValueError, match="decoding failed"):
        load_video_demonstration(path)
