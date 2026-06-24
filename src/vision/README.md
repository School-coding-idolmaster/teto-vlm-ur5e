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

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.
