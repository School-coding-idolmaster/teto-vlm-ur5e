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

The scene/camera snapshot implementation is being migrated in small stages:

- `src/vision/snapshot/camera_snapshot.py`: formal visual snapshot contract,
  validator, replay/formal snapshot compatibility helper, and report
  formatting.
- `src/camera_snapshot.py`: temporary root compatibility shim.
- `src/vision/snapshot/camera_source_adapter.py`: source-mode adapter from
  offline/manual, live-disabled, replay, or optional one-shot declarations into
  a snapshot contract.
- `src/camera_source_adapter.py`: temporary root compatibility shim.
- `src/vision/snapshot/realsense_snapshot_builder.py`: RealSense artifact
  bundle builder that validates existing files and writes formal snapshot
  manifests.
- `src/realsense_snapshot_builder.py`: temporary root compatibility shim.
- `scripts/build_realsense_snapshot_bundle.py`: CLI entrypoint for the
  RealSense snapshot bundle builder.

These files are shared-safe but real-path/artifact-path sensitive. Focused
tests and the RealSense snapshot bundle CLI have started using package import
paths. Production `src/` imports still use broad root-level compatibility
paths, so production import migration remains postponed.

The future package target is `src/vision/snapshot/`. H11-A5 moved
`camera_snapshot` into that package. H11-A6 moved `camera_source_adapter` into
that package. H11-A7 moved `realsense_snapshot_builder` into that package. All
three snapshot-related implementations now live under `src/vision/snapshot/`,
and production imports have not been migrated yet. Do not create `src/camera/`
or `src/scene_snapshot/`.

This package remains a future boundary. H11-A7 does not change runtime
behavior, startup behavior, real robot behavior, Isaac behavior, import paths,
production callsites, package-root re-export behavior, or safety semantics.
H11-A8-2 migrated only the script/CLI import for
`scripts/build_realsense_snapshot_bundle.py`; future production import
migration should stay staged and compatibility-tested.
