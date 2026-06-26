import json
from pathlib import Path

import pytest
import yaml
from PIL import Image

from scripts.build_realsense_snapshot_bundle import build_parser
from src.vision.snapshot.camera_snapshot import evaluate_formal_snapshot_replay
from src.vision.snapshot.realsense_snapshot_builder import (
    RealSenseSnapshotBundleRequest,
    SnapshotBundleError,
    build_realsense_snapshot_bundle,
)


def test_builder_creates_formal_realsense_replay_manifest(tmp_path):
    request = _request(tmp_path)

    result = build_realsense_snapshot_bundle(request)
    manifest = yaml.safe_load(Path(result["manifest_path"]).read_text(encoding="utf-8"))
    snapshot = manifest["camera_snapshot"]

    assert result["status"] == "PASS"
    assert result["assets_copied"] is False
    assert snapshot["source"] == "realsense_replay"
    assert snapshot["depth_aligned"] is True
    assert snapshot["aligned_depth_ref"] == snapshot["depth_ref"]
    assert snapshot["rgb_ref"] == snapshot["image_ref"]
    assert snapshot["tf_snapshot_ref"]


@pytest.mark.parametrize(
    ("field", "error_code"),
    [
        ("rgb_path", "E_RGB_REF_MISSING"),
        ("aligned_depth_path", "E_ALIGNED_DEPTH_REF_MISSING"),
        ("camera_info_path", "E_CAMERA_INFO_REF_MISSING"),
        ("metadata_path", "E_METADATA_REF_MISSING"),
        ("tf_snapshot_path", "E_TF_SNAPSHOT_REF_MISSING"),
    ],
)
def test_builder_fails_closed_when_required_artifact_is_missing(
    tmp_path,
    field,
    error_code,
):
    request = _request(tmp_path)
    values = dict(request.__dict__)
    values[field] = tmp_path / f"missing_{field}"

    with pytest.raises(SnapshotBundleError) as exc_info:
        build_realsense_snapshot_bundle(RealSenseSnapshotBundleRequest(**values))

    assert exc_info.value.code == error_code


def test_builder_requires_capture_timestamp_provenance(tmp_path):
    request = _request(tmp_path)
    metadata_path = Path(request.metadata_path)
    metadata_path.write_text(json.dumps({"device": "contract_fixture"}), encoding="utf-8")
    values = {**request.__dict__, "capture_timestamp": None}

    with pytest.raises(SnapshotBundleError) as exc_info:
        build_realsense_snapshot_bundle(RealSenseSnapshotBundleRequest(**values))

    assert exc_info.value.code == "E_CAPTURE_TIMESTAMP_MISSING"


def test_rgb_only_record_cannot_pass_formal_snapshot_validation(tmp_path):
    manifest_path = tmp_path / "rgb_only_contract_fixture.yaml"
    manifest_path.write_text(
        yaml.safe_dump(
            {
                "camera_snapshot": {
                    "snapshot_id": "rgb_only_contract_fixture",
                    "scene_version": "rgb_only_scene",
                    "capture_timestamp": "2026-06-19T00:00:00Z",
                    "source": "realsense_replay",
                    "frame_id": "camera_color_optical_frame",
                    "camera_frame": "camera_color_optical_frame",
                    "rgb_ref": "rgb.png",
                }
            }
        ),
        encoding="utf-8",
    )

    result = evaluate_formal_snapshot_replay(manifest_path)

    assert result["formal_visual_entry_status"] == "BLOCKED"
    assert "E_DEPTH_REF_MISSING" in result["blocking_reasons"]
    assert "E_CAMERA_INFO_REF_MISSING" in result["blocking_reasons"]
    assert "E_METADATA_REF_MISSING" in result["blocking_reasons"]
    assert "E_TF_SNAPSHOT_REF_MISSING" in result["blocking_reasons"]


def test_generated_manifest_is_accepted_by_formal_validator(tmp_path):
    result = build_realsense_snapshot_bundle(_request(tmp_path))

    validation = evaluate_formal_snapshot_replay(result["manifest_path"])

    assert validation["formal_visual_entry_status"] == "PASS"


def test_builder_cli_parser_exposes_artifact_inputs():
    args = build_parser().parse_args(
        [
            "--snapshot-id",
            "contract_fixture",
            "--scene-version",
            "contract_fixture_scene",
            "--rgb",
            "rgb.png",
            "--aligned-depth",
            "depth.png",
            "--camera-info",
            "camera_info.json",
            "--metadata",
            "metadata.json",
            "--tf-snapshot",
            "tf.json",
            "--output-manifest",
            "snapshot.yaml",
        ]
    )

    assert args.source == "realsense_replay"
    assert args.aligned_depth == "depth.png"
    assert args.tf_snapshot == "tf.json"


def _request(tmp_path) -> RealSenseSnapshotBundleRequest:
    rgb_path = tmp_path / "contract_fixture_rgb.png"
    depth_path = tmp_path / "contract_fixture_aligned_depth.png"
    camera_info_path = tmp_path / "contract_fixture_camera_info.json"
    metadata_path = tmp_path / "contract_fixture_metadata.json"
    tf_path = tmp_path / "contract_fixture_tf_snapshot.json"

    Image.new("RGB", (8, 6), (20, 40, 60)).save(rgb_path)
    Image.new("I;16", (8, 6), 1000).save(depth_path)
    camera_info_path.write_text(
        json.dumps({"camera_info": {"fx": 1.0, "fy": 1.0, "cx": 4.0, "cy": 3.0}}),
        encoding="utf-8",
    )
    metadata_path.write_text(
        json.dumps(
            {
                "fixture_type": "contract_fixture_not_real_capture",
                "capture_timestamp": "2026-06-19T00:00:00Z",
            }
        ),
        encoding="utf-8",
    )
    tf_path.write_text(
        json.dumps(
            {
                "fixture_type": "contract_fixture_not_real_capture",
                "parent_frame": "base_link",
                "child_frame": "camera_color_optical_frame",
                "translation_m": [0.0, 0.0, 0.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        ),
        encoding="utf-8",
    )
    return RealSenseSnapshotBundleRequest(
        snapshot_id="contract_fixture_snapshot",
        scene_version="contract_fixture_scene_v1",
        rgb_path=rgb_path,
        aligned_depth_path=depth_path,
        camera_info_path=camera_info_path,
        metadata_path=metadata_path,
        tf_snapshot_path=tf_path,
        output_manifest=tmp_path / "contract_fixture_snapshot.yaml",
        capture_timestamp="2026-06-19T00:00:00Z",
        notes="Contract test fixture only; not a real D455 capture.",
    )
