"""Compatibility shim for the vision snapshot RealSense bundle builder."""

from src.vision.snapshot.realsense_snapshot_builder import (
    RealSenseSnapshotBundleRequest,
    SnapshotBundleError,
    build_realsense_snapshot_bundle,
)

__all__ = [
    "RealSenseSnapshotBundleRequest",
    "SnapshotBundleError",
    "build_realsense_snapshot_bundle",
]
