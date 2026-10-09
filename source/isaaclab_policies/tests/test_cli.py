import os
import subprocess
import sys
from pathlib import Path

from isaaclab_policies import BasePolicy, create_policy

ROOT = Path(__file__).resolve().parents[3]
CLI = ROOT / "scripts/demos/web_control/run_policy.py"


def test_cli_help_needs_no_api_key_or_simulator():
    env = {k: v for k, v in os.environ.items() if k not in ("OPENAI_API_KEY", "ASTRA_API_KEY")}
    result = subprocess.run([sys.executable, str(CLI), "--help"], cwd="/tmp", env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "--policy" in result.stdout and "--dry-run" in result.stdout and "--instruction" in result.stdout


def test_cli_rejects_unknown_config_before_contacting_robot(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text('{"model": "gpt-6-astra", "misspelled_option": 1}')
    result = subprocess.run(
        [sys.executable, str(CLI), "--instruction", "hold", "--config", str(cfg)],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "misspelled_option" in result.stderr


def test_custom_policy_factory_and_core_import_without_model_frameworks():
    assert isinstance(create_policy("test_execution:Policy"), BasePolicy)
    env = {**os.environ, "PYTHONPATH": str(ROOT / "source/isaaclab_policies")}
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys, isaaclab_policies; "
                "assert not {'torch', 'isaaclab', 'isaacsim', 'openai'} & sys.modules.keys()"
            ),
        ],
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_video_task_cli_accepts_no_operation_instruction():
    from run_policy import parse_args

    args = parse_args(["--demo-video", "/tmp/demo.mp4"])
    assert args.instruction is None
    assert args.demo_video == Path("/tmp/demo.mp4")
    assert args.demo_frames == 16


def test_icl_cli_overrides_config_only_when_explicit(tmp_path):
    from run_policy import load_policy_config, parse_args

    config = tmp_path / "config.json"
    config.write_text('{"icl_mode": "description"}')
    base = ["--demo-video", "/tmp/demo.mp4", "--config", str(config)]
    assert load_policy_config(parse_args(base))["icl_mode"] == "description"
    assert load_policy_config(parse_args(base + ["--icl-mode", "frames"]))["icl_mode"] == "frames"
    assert load_policy_config(parse_args(base + ["--icl-mode", "description"]))["icl_mode"] == "description"


def test_icl_cli_rejects_invalid_mode_and_custom_policy():
    import pytest
    from run_policy import parse_args

    with pytest.raises(SystemExit):
        parse_args(["--instruction", "hold", "--icl-mode", "unknown"])
    with pytest.raises(SystemExit):
        parse_args(["--instruction", "hold", "--policy", "other:Policy", "--icl-mode", "description"])


def test_cli_still_requires_text_or_demo_and_rejects_custom_video_policy():
    import pytest
    from run_policy import parse_args

    with pytest.raises(SystemExit):
        parse_args([])
    with pytest.raises(SystemExit):
        parse_args(["--policy", "other:Policy", "--demo-video", "/tmp/demo.mp4"])


def test_task_input_does_not_derive_instructions_from_video_filename(monkeypatch):
    import policy_inputs
    from isaaclab_policies.demonstration import IMITATION_INSTRUCTION
    from policy_inputs import load_task_input
    from run_policy import parse_args

    marker = object()
    monkeypatch.setattr(policy_inputs, "load_video_demonstration", lambda *a, **kw: marker)
    instruction, demo = load_task_input(parse_args(["--demo-video", "/tmp/put_red_cube_in_bowl.mp4"]))
    assert instruction == IMITATION_INSTRUCTION
    assert demo is marker
    assert "red_cube" not in instruction and "bowl" not in instruction
