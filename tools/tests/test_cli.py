"""Tests for tagslam_tools CLI."""

from __future__ import annotations

import pytest
from typer.testing import CliRunner

from tagslam_tools.cli import app


@pytest.fixture
def runner() -> CliRunner:
    return CliRunner()


def test_app_help(runner: CliRunner) -> None:
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "COMMAND" in result.stdout


def test_app_version(runner: CliRunner) -> None:
    result = runner.invoke(app, ["--version"])
    assert result.exit_code == 0


def test_camera_help(runner: CliRunner) -> None:
    result = runner.invoke(app, ["camera", "--help"])
    assert result.exit_code == 0


def test_calibrate_help(runner: CliRunner) -> None:
    result = runner.invoke(app, ["calibrate", "--help"])
    assert result.exit_code == 0


def test_visualize_help(runner: CliRunner) -> None:
    result = runner.invoke(app, ["visualize", "--help"])
    assert result.exit_code == 0


def test_launch_help(runner: CliRunner) -> None:
    result = runner.invoke(app, ["launch", "--help"])
    assert result.exit_code == 0


def test_capture_help(runner: CliRunner) -> None:
    result = runner.invoke(app, ["camera", "capture", "--help"])
    assert result.exit_code == 0


def test_publish_help(runner: CliRunner) -> None:
    result = runner.invoke(app, ["camera", "publish", "--help"])
    assert result.exit_code == 0


def test_version(runner: CliRunner) -> None:
    import tagslam_tools

    assert tagslam_tools.__version__ == "0.1.0"
