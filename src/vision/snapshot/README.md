# Vision Snapshot Namespace

This package is a future home for scene/camera snapshot implementation.

No implementation has been migrated here yet. No import compatibility shim
exists here yet, and production code should not import from
`src.vision.snapshot` yet.

Current implementation remains in:

- `src/camera_snapshot.py`
- `src/camera_source_adapter.py`
- `src/realsense_snapshot_builder.py`

The current CLI entrypoint remains in:

- `scripts/build_realsense_snapshot_bundle.py`

This namespace does not re-export those modules, does not import them, and
does not add behavior. Future migration must be done only with a compatibility
plan and focused tests for current snapshot, camera-source adapter, RealSense
bundle builder, and downstream root-level import consumers.
