# H11 Vision Snapshot Import Inventory And Migration Plan

H11-A3 is an inventory and compatibility-planning pass only. It does not move
implementation files, change imports, add re-exports, or change behavior.

Current implementation files:

- `src/vision/snapshot/camera_snapshot.py`
- `src/vision/snapshot/camera_source_adapter.py`
- `src/vision/snapshot/realsense_snapshot_builder.py`

Compatibility shim files:

- `src/camera_snapshot.py`
- `src/camera_source_adapter.py`
- `src/realsense_snapshot_builder.py`

Future package boundary:

- `src/vision/snapshot/`

H11-A4 adds package-side compatibility adapters under this package. H11-A5
moves only the `camera_snapshot` implementation into the package path. H11-A6
moves only the `camera_source_adapter` implementation into the package path.
H11-A7 moves the `realsense_snapshot_builder` implementation into the package
path.
Root modules remain public compatibility paths, and production imports have not
migrated yet. H11-A8-1 migrated focused tests to package import paths. H11-A8-2
migrated script/CLI imports to package import paths while keeping production
`src/` imports on root compatibility shims. H11-A8-3 migrated the first
production `src/` import batch: `src/geometry_validity.py` and
`src/real_scene_shadow_pipeline.py`.

## Import Inventory

| Consumer | Current import | Consumer class | Risk | Notes |
| --- | --- | --- | --- | --- |
| `src/vision/snapshot/camera_source_adapter.py` | `CameraSnapshotRequest`, `evaluate_camera_snapshot_contract` from `src.vision.snapshot.camera_snapshot` | production `src/` | HIGH | Adapter builds and validates nested snapshot contracts. Real-path-sensitive no-live-camera semantics depend on this. Root `src/camera_source_adapter.py` remains a public compatibility shim. |
| `src/vision/snapshot/realsense_snapshot_builder.py` | `FORMAL_REALSENSE_SOURCES`, `STATUS_PASS`, `evaluate_formal_snapshot_replay` from `src.vision.snapshot.camera_snapshot` | production `src/` | HIGH | Builder validates artifact manifests through formal snapshot replay. Artifact-path-sensitive. Root `src/realsense_snapshot_builder.py` remains a public compatibility shim. |
| `src/geometry_validity.py` | `build_camera_snapshot_request`, `evaluate_camera_snapshot_contract` from `src.vision.snapshot.camera_snapshot` | production `src/` | HIGH | Shared geometry path joins snapshot and grounding evidence before projector/planner consumers. H11-A8-3 migrated this first production batch import. |
| `src/real_scene_shadow_pipeline.py` | `build_camera_snapshot_request`, `evaluate_camera_snapshot_contract` from `src.vision.snapshot.camera_snapshot` | production `src/` | HIGH | Replay/formal snapshot evidence feeds semantic shadow gate. H11-A8-3 migrated this first production batch import. |
| `src/perception_shadow_pipeline.py` | `CameraSnapshotRequest`, `build_camera_snapshot_request`, `evaluate_camera_snapshot_contract` from `src.camera_snapshot`; `CameraSourceAdapterRequest`, `evaluate_camera_source_adapter`, `load_camera_source_config` from `src.camera_source_adapter` | production `src/` | HIGH | Orchestrates camera source, camera snapshot, grounding, geometry, projector, and replay-ready perception evidence. |
| `src/simulation_runtime.py` | `build_camera_snapshot_request`, `evaluate_camera_snapshot_contract`; `build_camera_source_adapter_request`, `evaluate_camera_source_adapter` | production `src/` | HIGH | Shared evidence/runtime path used by many offline checks and reports; import churn can affect replay evidence. |
| `src/evidence_exporter.py` | `format_camera_snapshot_report`, `format_camera_source_report` | production `src/` | HIGH | Writes camera snapshot/source evidence artifacts and summaries. Output field semantics must stay stable. |
| `src/cli.py` | `evaluate_formal_snapshot_replay` from `src.camera_snapshot` | script/CLI | MEDIUM | Formal `python3 -m src.cli snapshot-replay` path. Offline, but public user-facing CLI. |
| `scripts/build_realsense_snapshot_bundle.py` | `RealSenseSnapshotBundleRequest`, `SnapshotBundleError`, `build_realsense_snapshot_bundle` from `src.vision.snapshot.realsense_snapshot_builder` | script/CLI | MEDIUM | Protected CLI entrypoint for artifact bundle building. H11-A8-2 migrated this import only; command behavior and parser stay stable. |
| `tests/test_camera_snapshot.py` | camera snapshot constants, request dataclass, builders, evaluators, report formatter | test | LOW | Direct contract behavior coverage. |
| `tests/test_camera_source_adapter.py` | camera source constants, modes, request dataclass, builders, evaluator, report formatter | test | LOW | Direct source adapter behavior coverage. |
| `tests/test_realsense_snapshot_builder.py` | CLI `build_parser`; `evaluate_formal_snapshot_replay`; builder request/error/function | test | LOW | Direct builder and CLI parser coverage. |
| `tests/test_geometry_validity.py` | `CameraSnapshotRequest`, `evaluate_camera_snapshot_contract` | test | LOW | Cross-stage geometry coverage. |
| `tests/test_projector_shadow.py` | `CameraSnapshotRequest`, `evaluate_camera_snapshot_contract` | test | LOW | Downstream projector coverage through geometry evidence. |
| `tests/test_real_scene_shadow_pipeline.py` | `CameraSnapshotRequest`, `evaluate_camera_snapshot_contract` | test | LOW | Real-scene shadow regression coverage. |
| `README.md` | `snapshot-replay`, bundle builder command, camera snapshot evidence names | config/docs reference | LOW | Needs update when import paths or package ownership become current. |
| `TETO_PROJECT_STATE.md` | formal RealSense replay/bundle references | config/docs reference | LOW | Needs update if migration becomes current baseline. |
| `docs/module_guides/vision.md` | current and future import paths | config/docs reference | LOW | Source of migration policy. |
| `src/vision/README.md` | current files and marker-only package | config/docs reference | LOW | Local package guide. |
| `src/vision/snapshot/README.md` | package implementation ownership and root shim policy | config/docs reference | LOW | Must stay aligned with staged migration status. |
| `src/replay/README.md`, `src/calibration/README.md`, `docs/h8_module_boundaries.md` | current root implementation path mentions | config/docs reference | LOW | Historical or future-boundary docs needing coordinated updates. |
| `configs/*.yaml`, `examples/camera_snapshot_example.json` | data contract keys such as `camera_snapshot`, `camera_source_adapter`, `camera_snapshot_config` | config/docs reference | LOW | Data contract names, not Python import paths. Preserve during migration. |

Package implementation modules now import sibling package modules where
appropriate. Downstream production consumers have not migrated yet and still use
root compatibility paths.

## Public API Stability List

These symbols and names must remain stable through any future migration.

### `src.camera_snapshot`

- `CONTRACT_VERSION`
- `CURRENT_CAMERA_SNAPSHOT_VERSION`
- `STATUS_PASS`
- `STATUS_BLOCKED`
- `STATUS_NOT_REQUESTED`
- `FORMAL_REALSENSE_SOURCES`
- `E_CAMERA_SNAPSHOT_INVALID`
- `E_SCENE_VERSION_MISSING`
- `E_CAPTURE_TIMESTAMP_MISSING`
- `E_CAMERA_SNAPSHOT_STALE`
- `E_CAMERA_FRAME_MISSING`
- `E_IMAGE_REF_MISSING`
- `E_DEPTH_REF_MISSING`
- `E_ALIGNED_DEPTH_REQUIRED`
- `E_CAMERA_INFO_REF_MISSING`
- `E_METADATA_REF_MISSING`
- `E_TF_SNAPSHOT_REF_MISSING`
- `E_LIVE_CAMERA_DISABLED`
- `E_ROBOT_COMMAND_NOT_ALLOWED`
- `FORBIDDEN_ROBOT_CONTROL_FIELDS`
- `SNAPSHOT_FIELDS`
- `CameraSnapshotRequest`
- `load_camera_snapshot_config`
- `build_camera_snapshot_request`
- `evaluate_formal_snapshot_replay`
- `evaluate_camera_snapshot_contract`
- `format_camera_snapshot_report`

### `src.camera_source_adapter`

- `CONTRACT_VERSION`
- `CURRENT_CAMERA_SOURCE_VERSION`
- `STATUS_PASS`
- `STATUS_BLOCKED`
- `STATUS_SAFE_DISABLED`
- `STATUS_NOT_REQUESTED`
- `MODE_OFFLINE_FILE`
- `MODE_MANUAL_SNAPSHOT`
- `MODE_LIVE_DISABLED`
- `MODE_REALSENSE_REPLAY`
- `MODE_REALSENSE_ONE_SHOT`
- `E_LIVE_CAMERA_CAPTURE_NOT_ALLOWED`
- `E_CONTINUOUS_CAPTURE_DISABLED`
- `E_CAMERA_BACKEND_UNAVAILABLE`
- `E_IMAGE_REF_MISSING`
- `E_CAMERA_INFO_MISSING`
- `E_CAMERA_FRAME_MISSING`
- `E_CAPTURE_TIMESTAMP_MISSING`
- `E_LIVE_VLM_DISABLED`
- `E_ROBOT_COMMAND_NOT_ALLOWED`
- `E_UNSUPPORTED_SOURCE_MODE`
- `FORBIDDEN_ROBOT_CONTROL_FIELDS`
- `CAMERA_SOURCE_FIELDS`
- `CameraSourceAdapterRequest`
- `load_camera_source_config`
- `build_camera_source_adapter_request`
- `evaluate_camera_source_adapter`
- `format_camera_source_report`

### `src.realsense_snapshot_builder`

- `RealSenseSnapshotBundleRequest`
- `SnapshotBundleError`
- `build_realsense_snapshot_bundle`

### `scripts.build_realsense_snapshot_bundle`

- `build_parser`
- `main`
- CLI options and exit behavior

## Consumer Risk Classification

HIGH risk consumers are production shared paths that participate in replay,
evidence export, geometry/projector handoff, or real-path-sensitive snapshot
semantics:

- `src/vision/snapshot/camera_source_adapter.py`
- `src/vision/snapshot/realsense_snapshot_builder.py`
- `src/geometry_validity.py`
- `src/real_scene_shadow_pipeline.py`
- `src/perception_shadow_pipeline.py`
- `src/simulation_runtime.py`
- `src/evidence_exporter.py`

MEDIUM risk consumers are script or CLI surfaces that are offline but
user-facing or artifact-path-sensitive:

- `src/cli.py`
- `scripts/build_realsense_snapshot_bundle.py`

LOW risk consumers are tests, docs, configs, and examples:

- `tests/test_camera_snapshot.py`
- `tests/test_camera_source_adapter.py`
- `tests/test_realsense_snapshot_builder.py`
- `tests/test_geometry_validity.py`
- `tests/test_projector_shadow.py`
- `tests/test_real_scene_shadow_pipeline.py`
- `README.md`
- `TETO_PROJECT_STATE.md`
- `docs/module_guides/vision.md`
- `src/vision/README.md`
- `src/vision/snapshot/README.md`
- `src/replay/README.md`
- `src/calibration/README.md`
- `docs/h8_module_boundaries.md`
- `configs/*.yaml`
- `examples/camera_snapshot_example.json`

No current consumer should be treated as legacy/debug for migration purposes.
`scripts/build_realsense_snapshot_bundle.py` is a current formal artifact CLI
and must be protected.

## Recommended Compatibility Strategy

Use a dual-path compatibility period with temporary root compatibility shims.

Do not use mechanical import rewrite only as the first migration step. The root
modules are broad public APIs with production shared consumers, direct test
imports, CLI usage, and docs that identify them as current behavior. A
mechanical rewrite would create avoidable churn and makes rollback harder.

Do not do no-migration forever. The marker namespace now exists, and a staged
move can be safe if root imports remain supported until consumers are migrated
and tested.

Recommended strategy:

1. Add package-side compatibility adapters in `src/vision/snapshot/` that
   re-export the current root APIs, with tests proving symbol identity.
2. Move implementation into `src/vision/snapshot/` modules.
3. Keep root modules as temporary compatibility shims that re-export the same
   public symbols from the new package modules.
4. Keep import compatibility tests that verify root symbols are identical to the
   new package symbols during the dual-path period.
5. Migrate production imports in small groups after behavior tests pass.
6. Remove root shims only after docs and all consumers are updated in a later
   cleanup task.

## Proposed Staged Migration Plan

### H11-A4: Package-Side Compatibility Adapters And Tests

Status: complete.

- Add temporary package-side compatibility adapters:
  - `src.vision.snapshot.camera_snapshot`
  - `src.vision.snapshot.camera_source_adapter`
  - `src.vision.snapshot.realsense_snapshot_builder`
- Each adapter should import and re-export only the existing root public API.
- Add tests that import from both current root paths and package paths.
- Assert key symbols are available and identical across root and package
  imports.
- Do not move implementation yet.
- Do not change production imports yet.
- Keep `src.vision.snapshot.__init__` conservative; import concrete package
  modules directly in tests.

### H11-A5: Move Implementation With Root Shims

Status: complete for all three snapshot modules.

- Move implementation files into, one at a time:
  - `src/vision/snapshot/camera_snapshot.py`
  - `src/vision/snapshot/camera_source_adapter.py`
  - `src/vision/snapshot/realsense_snapshot_builder.py`
- Replace moved root files with compatibility shims:
  - `src/camera_snapshot.py`
  - `src/camera_source_adapter.py`
  - `src/realsense_snapshot_builder.py`
- Root shims may re-export only the existing public API and should include a
  short compatibility note.
- Do not change downstream production imports in this step.
- Preserve `scripts/build_realsense_snapshot_bundle.py` behavior and import
  path unless explicitly handled in a later import-migration step.

H11-A5 moved only `camera_snapshot`. H11-A6 moved only
`camera_source_adapter`. H11-A7 moved `realsense_snapshot_builder`.

### H11-A7: Move RealSense Snapshot Builder With Extra CLI Care

Status: complete.

- Consider moving only `src/realsense_snapshot_builder.py` into
  `src/vision/snapshot/realsense_snapshot_builder.py`.
- Keep `src/realsense_snapshot_builder.py` as a temporary compatibility shim.
- Preserve `scripts/build_realsense_snapshot_bundle.py` imports, parser
  behavior, exit codes, file validation behavior, manifest writing behavior,
  and artifact path semantics.
- Run focused builder tests plus CLI syntax/parser coverage before and after
  the move.

### H11-A8: Gradual Import Migration

Status: in progress.

- H11-A8-1 migrated low-risk tests first to prove the package imports work.
- H11-A8-2 migrated medium-risk CLI imports after focused CLI/parser tests pass.
- H11-A8-3 migrated production batch 1:
  - `src/geometry_validity.py`, `src/real_scene_shadow_pipeline.py`.
- Migrate high-risk production consumers in small batches:
  - batch 2: `src/perception_shadow_pipeline.py`, `src/simulation_runtime.py`.
  - batch 3: `src/evidence_exporter.py`, `src/cli.py`.
- Keep root shims until all docs and consumers no longer rely on root imports.
- Defer root shim removal to a separate cleanup task after a full offline test
  run.

## Required Tests And Checks

Before migration:

```bash
git status --short
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_camera_snapshot.py \
  tests/test_camera_source_adapter.py \
  tests/test_realsense_snapshot_builder.py \
  tests/test_geometry_validity.py \
  tests/test_real_scene_shadow_pipeline.py \
  tests/test_projector_shadow.py \
  tests/test_perception_shadow_pipeline.py \
  tests/test_simulation_runtime.py \
  tests/test_cli.py
```

After moving implementation but before changing consumers:

```bash
PYTHONPYCACHEPREFIX=/tmp/teto_codex_compile_pycache .venv_lab/bin/python -m compileall src scripts tests
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_camera_snapshot.py \
  tests/test_camera_source_adapter.py \
  tests/test_realsense_snapshot_builder.py \
  tests/test_geometry_validity.py \
  tests/test_real_scene_shadow_pipeline.py \
  tests/test_projector_shadow.py \
  tests/test_perception_shadow_pipeline.py \
  tests/test_simulation_runtime.py \
  tests/test_cli.py
git diff --check
```

Before root shim removal:

```bash
PYTHONPYCACHEPREFIX=/tmp/teto_codex_compile_pycache .venv_lab/bin/python -m compileall src scripts tests
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q
git diff --check
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

## Documentation Updates Needed During Future Migration

Update these when implementation actually moves:

- `docs/module_guides/vision.md`
- `src/vision/README.md`
- `src/vision/snapshot/README.md`
- `src/replay/README.md`
- `src/calibration/README.md`
- `docs/h8_module_boundaries.md`, only as a historical note/update
- `README.md`
- `TETO_PROJECT_STATE.md`

Do not rename data contract keys such as `camera_snapshot`,
`camera_source_adapter`, `camera_snapshot_config`, `camera_snapshot_result.json`,
or `camera_snapshot_report.md`. Those are evidence/config names, not Python
module paths.

## Protected Surfaces

Future migration must explicitly protect:

- `scripts/build_realsense_snapshot_bundle.py` CLI behavior, arguments, exit
  codes, and artifact-path semantics.
- Canonical launch scripts:
  - `scripts/start_teto_real_full_stack.sh`
  - `scripts/start_teto_isaac_gui_operator.sh`
- Replay/formal snapshot behavior through `evaluate_formal_snapshot_replay`.
- Real-path-sensitive snapshot contract semantics:
  - no live camera capture from contract validation
  - no live VLM/model calls
  - no ROS2, MoveIt, RTDE, Dashboard, URScript, or real robot connection
  - no robot command, trajectory, joint target, `tcp_pose_world`, or automatic
    retry motion generation
- Existing public API names, status values, modes, error codes, report fields,
  and evidence output names.
