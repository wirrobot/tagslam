"""Shared pytest fixtures — load config and test image."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import pytest
from yaml import safe_load

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


def _load_yaml(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    with open(path) as f:
        return safe_load(f)  # type: ignore[no-any-return]


def _find_first_body_with_tags(bodies: list[dict[str, Any]]) -> dict[str, Any] | None:
    for body_dict in bodies:
        for body in body_dict.values():
            if isinstance(body, dict) and "tags" in body:
                return body
    return None


@pytest.fixture
def camera_params() -> tuple[float, float, float, float] | None:
    """Return (fx, fy, cx, cy) from config/cameras.yaml."""
    cfg = _load_yaml(PROJECT_ROOT / "config" / "cameras.yaml")
    if cfg is None:
        return None
    for cam in cfg.values():
        if isinstance(cam, dict) and "intrinsics" in cam:
            fx, fy, cx, cy = cam["intrinsics"]
            return (float(fx), float(fy), float(cx), float(cy))
    return None


@pytest.fixture
def tag_sizes() -> dict[int, float] | None:
    """Return {tag_id: size_m} from config/tagslam.yaml."""
    cfg = _load_yaml(PROJECT_ROOT / "config" / "tagslam.yaml")
    if cfg is None:
        return None
    bodies = cfg.get("bodies", [])
    body = _find_first_body_with_tags(bodies)
    if body is None:
        return None
    result: dict[int, float] = {}
    for tag in body["tags"]:
        result[int(tag["id"])] = float(tag["size"])
    return result if result else None


@pytest.fixture
def test_image_path() -> str:
    """Return absolute path to test.jpg."""
    return str(Path(__file__).resolve().parent / "test.jpg")


@pytest.fixture
def config_root() -> str:
    """Return config directory path, overrideable via TAGSLAM_CONFIG_DIR."""
    return os.environ.get("TAGSLAM_CONFIG_DIR", str(PROJECT_ROOT / "config"))
