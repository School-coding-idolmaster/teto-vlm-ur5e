# Vision Snapshot Namespace

This package is a future home for scene/camera snapshot implementation.

H11-A4 added package-side compatibility adapter modules. H11-A5 moves only the
`camera_snapshot` implementation here. Production imports have not migrated
yet.

Current implementation status:

- `src/vision/snapshot/camera_snapshot.py`: current implementation.
- `src/camera_snapshot.py`: temporary root compatibility shim.
- `src/camera_source_adapter.py`: current root implementation, not moved yet.
- `src/realsense_snapshot_builder.py`: current root implementation, not moved
  yet.

The current CLI entrypoint remains in:

- `scripts/build_realsense_snapshot_bundle.py`

Current package-side adapter modules are:

- `src.vision.snapshot.camera_snapshot`
- `src.vision.snapshot.camera_source_adapter`
- `src.vision.snapshot.realsense_snapshot_builder`

The `camera_source_adapter` and `realsense_snapshot_builder` adapters
explicitly import public symbols from the current root modules so future code
can test package paths before implementation migration. The `camera_snapshot`
module now owns its implementation. None of these modules add behavior, mutate
constants, hide errors, or start services.

`src/vision/snapshot/__init__.py` remains conservative and does not re-export
APIs from the package root. Import concrete adapter modules directly.

Future H11-A6 should consider moving `camera_source_adapter.py` next, while
keeping root modules as compatibility shims until production imports are
migrated in a later step.
