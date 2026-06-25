# Vision Boundary

This package is reserved for future D455 capture, snapshot replay, camera
source adapters, image preprocessing, and model-safe visual input handling.

Existing files that already carry related responsibilities include:

- `src/camera_source_adapter.py`
- `src/camera_snapshot.py`
- `src/realsense_snapshot_builder.py`
- `src/image_utils.py`
- `src/vlm_infer.py`
- `scripts/build_realsense_snapshot_bundle.py`

H8 intentionally leaves those files in place because current replay, no-live
camera, no-model-call, and no-robot-control guarantees are tested against their
existing import paths.

## Scene / Camera Snapshot Boundary

H11-A1 keeps the scene/camera snapshot implementation in its current locations:

- `src/camera_snapshot.py`: formal visual snapshot contract, validator,
  replay/formal snapshot compatibility helper, and report formatting.
- `src/camera_source_adapter.py`: source-mode adapter from offline/manual,
  live-disabled, replay, or optional one-shot declarations into a snapshot
  contract.
- `src/realsense_snapshot_builder.py`: RealSense artifact bundle builder that
  validates existing files and writes formal snapshot manifests.
- `scripts/build_realsense_snapshot_bundle.py`: CLI entrypoint for the
  RealSense snapshot bundle builder.

These files are shared-safe but real-path/artifact-path sensitive. They are
used by tests and production code through broad root-level imports, so import
migration is postponed.

The possible future package target is `src/vision/snapshot/`, but migration is
not approved yet. Do not create `src/vision/snapshot/`, `src/camera/`, or
`src/scene_snapshot/` during documentation-only boundary work.

This package remains a future boundary. H11-A1 does not change runtime
behavior, startup behavior, real robot behavior, Isaac behavior, import paths,
file locations, or safety semantics.
