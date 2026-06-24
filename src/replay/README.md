# Replay Boundary

This package is reserved for future replay manifests, saved scene records,
snapshot replay, semantic replay, and evidence lookup utilities.

Existing files that already carry related responsibilities include:

- `src/robot_task_inspector.py`
- `src/semantic_simulation_bridge.py`
- `src/evidence_exporter.py`
- `src/output_paths.py`
- `src/simulation_runtime.py`
- `src/camera_snapshot.py`

H8 intentionally leaves those files in place because the repository currently
has both legacy robot-task replay and newer snapshot/evidence replay paths.
Moving either path now would risk confusing legacy coverage with current
runtime boundaries.

This package is only a future boundary. H8 does not change runtime behavior,
startup behavior, real robot behavior, Isaac behavior, or safety semantics.
