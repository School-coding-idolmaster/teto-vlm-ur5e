from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import yaml
from PIL import Image

from src.vision.snapshot.camera_snapshot import (
    FORMAL_REALSENSE_SOURCES,
    STATUS_PASS,
    evaluate_formal_snapshot_replay,
)


@dataclass(frozen=True)
class RealSenseSnapshotBundleRequest:
    snapshot_id: str
    scene_version: str
    rgb_path: str | Path
    aligned_depth_path: str | Path
    camera_info_path: str | Path
    metadata_path: str | Path
    tf_snapshot_path: str | Path
    output_manifest: str | Path
    source: str = "realsense_replay"
    capture_timestamp: str | None = None
    camera_frame: str = "camera_color_optical_frame"
    frame_id: str | None = None
    notes: str | None = None
    overwrite: bool = False


class SnapshotBundleError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


def build_realsense_snapshot_bundle(
    request: RealSenseSnapshotBundleRequest,
) -> Dict[str, Any]:
    snapshot_id = _required_text(request.snapshot_id, "E_SNAPSHOT_ID_MISSING", "snapshot_id")
    scene_version = _required_text(
        request.scene_version,
        "E_SCENE_VERSION_MISSING",
        "scene_version",
    )
    if request.source not in FORMAL_REALSENSE_SOURCES:
        raise SnapshotBundleError(
            "E_FORMAL_VISUAL_SOURCE_NOT_REALSENSE",
            f"source must be one of {sorted(FORMAL_REALSENSE_SOURCES)}",
        )

    artifacts = {
        "rgb_ref": _required_file(request.rgb_path, "E_RGB_REF_MISSING"),
        "aligned_depth_ref": _required_file(
            request.aligned_depth_path,
            "E_ALIGNED_DEPTH_REF_MISSING",
        ),
        "camera_info_ref": _required_file(
            request.camera_info_path,
            "E_CAMERA_INFO_REF_MISSING",
        ),
        "metadata_ref": _required_file(request.metadata_path, "E_METADATA_REF_MISSING"),
        "tf_snapshot_ref": _required_file(
            request.tf_snapshot_path,
            "E_TF_SNAPSHOT_REF_MISSING",
        ),
    }
    output_manifest = Path(request.output_manifest).expanduser()
    if output_manifest.suffix.lower() not in {".json", ".yaml", ".yml"}:
        raise SnapshotBundleError(
            "E_OUTPUT_MANIFEST_FORMAT_UNSUPPORTED",
            "output manifest must use .json, .yaml, or .yml",
        )
    if output_manifest.exists() and not request.overwrite:
        raise SnapshotBundleError(
            "E_OUTPUT_MANIFEST_EXISTS",
            f"refusing to overwrite {output_manifest}",
        )

    metadata = _load_mapping(artifacts["metadata_ref"], "E_METADATA_INVALID")
    _load_mapping(artifacts["camera_info_ref"], "E_CAMERA_INFO_INVALID")
    _load_mapping(artifacts["tf_snapshot_ref"], "E_TF_SNAPSHOT_INVALID")
    capture_timestamp = _capture_timestamp(request.capture_timestamp, metadata)
    width, height = _matching_rgb_depth_size(
        artifacts["rgb_ref"],
        artifacts["aligned_depth_ref"],
    )

    output_manifest.parent.mkdir(parents=True, exist_ok=True)
    refs = {
        name: _stable_ref(path, output_manifest.parent)
        for name, path in artifacts.items()
    }
    snapshot = {
        "snapshot_id": snapshot_id,
        "scene_version": scene_version,
        "capture_timestamp": capture_timestamp,
        "ttl_ms": 315360000000,
        "source": request.source,
        "frame_id": request.frame_id or request.camera_frame,
        "camera_frame": request.camera_frame,
        "rgb_ref": refs["rgb_ref"],
        "image_ref": refs["rgb_ref"],
        "aligned_depth_ref": refs["aligned_depth_ref"],
        "depth_ref": refs["aligned_depth_ref"],
        "camera_info_ref": refs["camera_info_ref"],
        "metadata_ref": refs["metadata_ref"],
        "tf_snapshot_ref": refs["tf_snapshot_ref"],
        "width": width,
        "height": height,
        "color_encoding": "rgb8",
        "depth_encoding": "uint16_mm",
        "depth_aligned": True,
        "alignment_status": "aligned_rgb_depth",
        "sync_status": "artifact_bundle_manifest",
        "depth_available": True,
        "camera_info_available": True,
        "metadata_available": True,
        "extrinsics_available": True,
        "depth_required": True,
        "live_camera_enabled": False,
        "builder": "teto_realsense_snapshot_bundle_v1",
        "notes": request.notes,
    }
    payload = {"camera_snapshot": snapshot}
    _write_manifest(output_manifest, payload)

    validation = evaluate_formal_snapshot_replay(output_manifest)
    if validation.get("formal_visual_entry_status") != STATUS_PASS:
        output_manifest.unlink(missing_ok=True)
        raise SnapshotBundleError(
            "E_GENERATED_MANIFEST_VALIDATION_FAILED",
            json.dumps(validation.get("blocking_reasons", [])),
        )
    return {
        "status": "PASS",
        "manifest_path": str(output_manifest),
        "assets_copied": False,
        "snapshot": snapshot,
        "formal_validation": validation,
    }


def _required_text(value: Any, code: str, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SnapshotBundleError(code, f"{field} is required")
    return value.strip()


def _required_file(value: str | Path, code: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_file():
        raise SnapshotBundleError(code, f"required artifact not found: {path}")
    return path.resolve()


def _load_mapping(path: Path, code: str) -> Dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle) if path.suffix.lower() == ".json" else yaml.safe_load(handle)
    except Exception as exc:
        raise SnapshotBundleError(code, f"cannot read {path}: {exc}") from exc
    if not isinstance(data, dict) or not data:
        raise SnapshotBundleError(code, f"{path} must contain a non-empty object")
    return data


def _capture_timestamp(explicit: str | None, metadata: Dict[str, Any]) -> str:
    value = explicit
    if not value:
        for key in ("capture_timestamp", "timestamp_utc", "timestamp"):
            candidate = metadata.get(key)
            if isinstance(candidate, str) and candidate:
                value = candidate
                break
    if not value:
        raise SnapshotBundleError(
            "E_CAPTURE_TIMESTAMP_MISSING",
            "provide --capture-timestamp or an ISO timestamp in metadata",
        )
    normalized = value[:-1] + "+00:00" if value.endswith("Z") else value
    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise SnapshotBundleError(
            "E_CAPTURE_TIMESTAMP_INVALID",
            "capture timestamp must be ISO 8601",
        ) from exc
    if parsed.tzinfo is None:
        raise SnapshotBundleError(
            "E_CAPTURE_TIMESTAMP_INVALID",
            "capture timestamp must include a timezone",
        )
    return parsed.isoformat().replace("+00:00", "Z")


def _matching_rgb_depth_size(rgb_path: Path, depth_path: Path) -> tuple[int, int]:
    try:
        with Image.open(rgb_path) as image:
            rgb_size = image.size
    except Exception as exc:
        raise SnapshotBundleError("E_RGB_ARTIFACT_INVALID", str(exc)) from exc
    try:
        with Image.open(depth_path) as image:
            depth_size = image.size
    except Exception as exc:
        raise SnapshotBundleError("E_ALIGNED_DEPTH_ARTIFACT_INVALID", str(exc)) from exc
    if rgb_size != depth_size:
        raise SnapshotBundleError(
            "E_RGB_DEPTH_DIMENSION_MISMATCH",
            f"RGB size {rgb_size} does not match aligned depth size {depth_size}",
        )
    return int(rgb_size[0]), int(rgb_size[1])


def _stable_ref(path: Path, manifest_dir: Path) -> str:
    return Path(os.path.relpath(path, manifest_dir.resolve())).as_posix()


def _write_manifest(path: Path, payload: Dict[str, Any]) -> None:
    if path.suffix.lower() == ".json":
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    else:
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


__all__ = [
    "RealSenseSnapshotBundleRequest",
    "SnapshotBundleError",
    "build_realsense_snapshot_bundle",
]
