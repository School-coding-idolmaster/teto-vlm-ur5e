from __future__ import annotations

from typing import Any, Dict


SCENE_BINDING_SNAPSHOT_ID = "snapshot_id"
SCENE_BINDING_SCENE_VERSION = "scene_version"


def find_scene_binding_mismatches(
    result: Dict[str, Any],
    *,
    expected_snapshot_id: str | None = None,
    expected_scene_version: str | None = None,
) -> list[str]:
    mismatches: list[str] = []
    if expected_snapshot_id and result.get(SCENE_BINDING_SNAPSHOT_ID) != expected_snapshot_id:
        mismatches.append(SCENE_BINDING_SNAPSHOT_ID)
    if expected_scene_version and result.get(SCENE_BINDING_SCENE_VERSION) != expected_scene_version:
        mismatches.append(SCENE_BINDING_SCENE_VERSION)
    return mismatches


__all__ = [
    "SCENE_BINDING_SCENE_VERSION",
    "SCENE_BINDING_SNAPSHOT_ID",
    "find_scene_binding_mismatches",
]
