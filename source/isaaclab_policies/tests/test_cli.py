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
