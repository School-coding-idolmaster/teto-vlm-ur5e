# Vision Snapshot Namespace

This package is a future home for scene/camera snapshot implementation.

H11-A4 adds package-side compatibility adapter modules. No implementation has
been migrated here yet, and production imports have not migrated yet. The root
modules remain the source of truth.

Current implementation remains in:

- `src/camera_snapshot.py`
- `src/camera_source_adapter.py`
- `src/realsense_snapshot_builder.py`

The current CLI entrypoint remains in:

- `scripts/build_realsense_snapshot_bundle.py`

Current package-side adapter modules are:

- `src.vision.snapshot.camera_snapshot`
- `src.vision.snapshot.camera_source_adapter`
- `src.vision.snapshot.realsense_snapshot_builder`

These adapters explicitly import public symbols from the current root modules
so future code can test the package path before implementation migration. They
do not add behavior, mutate constants, hide errors, or start services.

`src/vision/snapshot/__init__.py` remains conservative and does not re-export
APIs from the package root. Import concrete adapter modules directly.

Future H11-A5 should move implementation into package modules and turn root
modules into compatibility shims only after focused adapter and snapshot tests
pass.
