# Calibration Boundary

This package is reserved for future camera-to-base transform calibration, D455
extrinsics, TF snapshot metadata, and geometry validation support for real
visual-motion work.

Existing files that already carry related responsibilities include:

- `src/camera_snapshot.py`
- `src/realsense_snapshot_builder.py`
- `src/projector/shadow.py`
- `configs/camera_snapshot.example.yaml`
- `examples/camera_source/sample_extrinsics.json`

H8 intentionally leaves those files in place. They are already covered by
snapshot, replay, and projection safety tests, and moving them now would create
import churn without improving runtime behavior.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.
