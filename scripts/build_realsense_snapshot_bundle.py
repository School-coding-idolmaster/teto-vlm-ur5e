#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.realsense_snapshot_builder import (  # noqa: E402
    RealSenseSnapshotBundleRequest,
    SnapshotBundleError,
    build_realsense_snapshot_bundle,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Build a formal RealSense snapshot manifest from existing RGB-D artifacts. "
            "This does not start a camera, ROS, MoveIt, or a robot."
        )
    )
    parser.add_argument("--snapshot-id", required=True)
    parser.add_argument("--scene-version", required=True)
    parser.add_argument("--rgb", required=True)
    parser.add_argument("--aligned-depth", required=True)
    parser.add_argument("--camera-info", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--tf-snapshot", required=True)
    parser.add_argument("--output-manifest", required=True)
    parser.add_argument(
        "--source",
        choices=["realsense_d455", "realsense_replay"],
        default="realsense_replay",
    )
    parser.add_argument("--capture-timestamp")
    parser.add_argument("--camera-frame", default="camera_color_optical_frame")
    parser.add_argument("--frame-id")
    parser.add_argument("--notes")
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = build_realsense_snapshot_bundle(
            RealSenseSnapshotBundleRequest(
                snapshot_id=args.snapshot_id,
                scene_version=args.scene_version,
                rgb_path=args.rgb,
                aligned_depth_path=args.aligned_depth,
                camera_info_path=args.camera_info,
                metadata_path=args.metadata,
                tf_snapshot_path=args.tf_snapshot,
                output_manifest=args.output_manifest,
                source=args.source,
                capture_timestamp=args.capture_timestamp,
                camera_frame=args.camera_frame,
                frame_id=args.frame_id,
                notes=args.notes,
                overwrite=args.overwrite,
            )
        )
    except SnapshotBundleError as exc:
        print(
            json.dumps(
                {"status": "BLOCKED", "error_code": exc.code, "message": exc.message},
                indent=2,
            ),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
