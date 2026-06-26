# Vision Boundary

This package owns D455 capture-boundary contracts, snapshot replay, camera
source adapters, image preprocessing, and model-safe visual input handling.

## Scene / Camera Snapshot Boundary

The scene/camera snapshot implementation is being migrated in small stages:

- `src/vision/snapshot/camera_snapshot.py`: formal visual snapshot contract,
  validator, replay/formal snapshot compatibility helper, and report
  formatting.
- `src/vision/snapshot/camera_source_adapter.py`: source-mode adapter from
  offline/manual, live-disabled, replay, or optional one-shot declarations into
  a snapshot contract.
- `src/vision/snapshot/realsense_snapshot_builder.py`: RealSense artifact
  bundle builder that validates existing files and writes formal snapshot
  manifests.
- `scripts/build_realsense_snapshot_bundle.py`: CLI entrypoint for the
  RealSense snapshot bundle builder.

These files are shared-safe but real-path/artifact-path sensitive. Focused
tests and the RealSense snapshot bundle CLI have started using package import
paths. H11-A8-3 migrated the first production `src/` import batch for geometry
validity and real-scene shadow. H11-A8-4 migrated the second production batch
for perception shadow and simulation runtime. H11-A8-5 migrated the final
production `src/` import group for `src.cli` and evidence export. Root shims
were removed in H11-A9 after readiness scans found no real consumers.

The future package target is `src/vision/snapshot/`. H11-A5 moved
`camera_snapshot` into that package. H11-A6 moved `camera_source_adapter` into
that package. H11-A7 moved `realsense_snapshot_builder` into that package. All
three snapshot-related implementations now live under `src/vision/snapshot/`,
and production imports now use concrete package modules. Do not create
`src/camera/` or `src/scene_snapshot/`.

The H11 migration changed import locations only. It did not change runtime
behavior, startup behavior, real robot behavior, Isaac behavior,
package-root re-export behavior, or safety semantics. H11-A9 removed
`src/camera_snapshot.py`, `src/camera_source_adapter.py`, and
`src/realsense_snapshot_builder.py` after readiness scans confirmed package
imports had replaced root snapshot imports.
