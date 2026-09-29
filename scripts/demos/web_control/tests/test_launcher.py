"""Launcher path discovery must work after relocation, without starting a GPU process."""

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

LAUNCHER = Path(__file__).resolve().parents[1] / "start_web_control.sh"


@pytest.mark.parametrize("location", ["", "scripts/demos/web_control"])
def test_launcher_finds_root_from_unrelated_working_directory(tmp_path, location):
    repo = tmp_path / "repository with spaces"
    demo = repo / "scripts/demos/web_control"
    demo.mkdir(parents=True)
    (demo / "run.py").touch()
    entry = repo / "isaaclab.sh"
    entry.write_text('#!/usr/bin/env bash\nprintf "ARG:%s\\n" "$PWD" "$@"\n')
    entry.chmod(0o755)
    launcher = repo / location / "start_web_control.sh"
    shutil.copyfile(LAUNCHER, launcher)
    conda = tmp_path / "conda"
    (conda / "etc/profile.d").mkdir(parents=True)
    (conda / "etc/profile.d/conda.sh").write_text('conda() { export CONDA_PREFIX="/test/env"; }\n')
    result = subprocess.run(
        ["bash", str(launcher), "--port", "9001", "--output-dir", "path with spaces"],
        cwd=tmp_path,
        env={**os.environ, "ISAACLAB_CONDA_ROOT": str(conda)},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    args = [line.removeprefix("ARG:") for line in result.stdout.splitlines() if line.startswith("ARG:")]
    assert args == [
        str(repo),
        "-p",
        str(demo / "run.py"),
        "--headless",
        "--enable_cameras",
        "--robot",
        re.search(r'WEB_ROBOT="([^\"]+)"', LAUNCHER.read_text()).group(1),
        "--port",
        "8080",
        "--width",
        "480",
        "--height",
        "360",
        "--camera-fps",
        "3",
        "--output-dir",
        "output/web_control",
        "--port",
        "9001",
        "--output-dir",
        "path with spaces",
    ]


def test_launcher_reports_missing_repository(tmp_path):
    launcher = tmp_path / "start_web_control.sh"
    shutil.copyfile(LAUNCHER, launcher)
    result = subprocess.run(["bash", str(launcher)], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 1
    assert "找不到 Isaac Lab 仓库根目录" in result.stderr


def test_simulation_launcher_forwards_mode_and_user_overrides(tmp_path):
    simulation = LAUNCHER.with_name("start_simulation.sh")
    shutil.copyfile(simulation, tmp_path / simulation.name)
    (tmp_path / LAUNCHER.name).write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
    result = subprocess.run(
        ["bash", str(tmp_path / simulation.name), "--port", "8099"],
        cwd="/tmp",
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    args = result.stdout.splitlines()
    assert args[:2] == ["--mode", "observe"]
    assert args[-2:] == ["--port", "8099"]
