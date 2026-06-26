# Vision Module Guide

This guide records the H11 scene/camera snapshot boundary policy for future
Codex, GPT, and human audits. H11-A1 was documentation-only. H11-A2 added a
namespace marker. H11-A4 adds package-side compatibility adapters, but no
implementation files are moved and production imports are not changed. H11-A5
moves only the `camera_snapshot` implementation into the package path while
keeping the root import as a compatibility shim. H11-A6 moves only the
`camera_source_adapter` implementation into the package path while keeping the
root import as a compatibility shim. H11-A7 moves the
`realsense_snapshot_builder` implementation into the package path while
keeping the root import as a compatibility shim. H11-A8-1 migrated focused
tests to package import paths, and H11-A8-2 migrated the RealSense snapshot
bundle CLI import to the package path. H11-A8-3 migrated the first production
`src/` import batch, covering geometry validity and real-scene shadow pipeline.
H11-A8-4 migrated the second production `src/` import batch, covering
perception shadow and simulation runtime. Other production `src/` imports are
not migrated yet.

## Current Responsibility

The vision boundary covers replayable visual evidence and camera-source
declarations before grounding, geometry validity, projection, planning, or
execution consume that evidence.

Current scene/camera snapshot responsibilities are split across these files:

- `src/vision/snapshot/camera_snapshot.py`: formal visual snapshot contract,
  validator, replay/formal snapshot compatibility helper, and report
  formatting.
- `src/camera_snapshot.py`: temporary root compatibility shim for the package
  implementation.
- `src/vision/snapshot/camera_source_adapter.py`: source-mode adapter from offline file,
  manual snapshot, live-disabled, RealSense replay, or optional one-shot
  declarations into a camera snapshot contract.
- `src/camera_source_adapter.py`: temporary root compatibility shim for the
  package implementation.
- `src/vision/snapshot/realsense_snapshot_builder.py`: RealSense artifact
  bundle builder that validates required files, checks RGB/depth image
  dimensions, and writes formal snapshot manifests.
- `src/realsense_snapshot_builder.py`: temporary root compatibility shim for
  the package implementation.
- `scripts/build_realsense_snapshot_bundle.py`: CLI entrypoint for the
  RealSense snapshot bundle builder.

These modules are shared-safe but real-path/artifact-path sensitive. They must
keep no-motion, no-live-VLM, no-real-robot, no-ROS2, and no-MoveIt semantics
unless a future task explicitly authorizes broader behavior.

## Import And Packaging Policy

Current root-level import paths remain public compatibility paths:

- `src.camera_snapshot`
- `src.camera_source_adapter`
- `src.realsense_snapshot_builder`

Script, focused test, geometry validity, real-scene shadow, perception shadow,
and simulation runtime imports have migrated to package paths in H11-A8.
Remaining production `src/` import migration is still staged because these
root-level imports are broad and current public APIs are depended on by
production code. Known remaining production consumers include `src.cli` and
evidence export.

Do not rename, remove, or reinterpret public dataclasses, builder helpers,
evaluators, report formatters, constants, modes, status values, or error codes
without a dedicated migration plan.

## Future Package Target

The possible future package target is:

- `src/vision/snapshot/`

Migration to that package is staged. Current package-side modules are:

- `src.vision.snapshot.camera_snapshot`: current implementation
- `src.vision.snapshot.camera_source_adapter`: current implementation
- `src.vision.snapshot.realsense_snapshot_builder`: current implementation

`src/camera_snapshot.py`, `src/camera_source_adapter.py`, and
`src/realsense_snapshot_builder.py` are now temporary compatibility shims.
`src/vision/snapshot/__init__.py` does not re-export public APIs. Do not create
alternate package roots such as `src/camera/` or `src/scene_snapshot/`. Future
migration should continue in small behavior-preserving steps only after focused
compatibility tests pass.

## Boundary With Neighbor Modules

Vision snapshot code owns:

- image, depth, camera-info, metadata, TF, and extrinsics references
- snapshot identity and `scene_version`
- capture timestamps and TTL checks
- formal RealSense replay source validation
- live camera blocking and source-mode evidence
- artifact-manifest construction from existing files

Calibration owns future camera-to-base transforms, D455 extrinsics policy, and
calibration metadata beyond the snapshot manifest boundary.

Replay owns broader saved-scene and evidence lookup utilities. It may consume
camera snapshots, but it does not own the snapshot contract fields.

Grounding owns target labels, bboxes, pixel centers, grounding confidence, and
semantic rejection evidence.

Geometry validity owns snapshot/grounding identity checks, image-size checks,
bbox and pixel-center validity, depth-availability checks, confidence
thresholds, and TTL freshness in joined evidence.

Projector owns metric projection from accepted geometry evidence into camera or
world points. It does not own live capture, formal snapshot manifests, or
camera-source selection.

Execution, planning, and safety own robot readiness, planning, MoveIt execute
behavior, real or Isaac backends, and safety-critical execution gates.

## Forbidden Dependencies

Vision snapshot boundary work must not import or start:

- Real hardware or UR driver.
- Isaac Sim.
- ROS or MoveIt.
- RealSense live capture.
- Qwen, VLM, LLM, or model runtimes.
- Real or Isaac execution backends.

The RealSense snapshot builder may read existing artifact files and write
formal manifests. That filesystem behavior is artifact-path sensitive, not
permission to capture from a camera or start services.

## Startup Script Protection

The canonical startup commands remain documented in
`docs/current_entrypoints.md`:

```bash
bash scripts/start_teto_real_full_stack.sh
```

```bash
bash scripts/start_teto_isaac_gui_operator.sh \
  --gui --console \
  --isaac-app /home/genlab/isaac-sim/isaac-sim.sh \
  --ur5e-asset outputs/isaac_assets/generated_ur5e/ur5e_clean_no_tool.usd \
  --motion-duration-sec 3.0 \
  --substep-pause-sec 0.35
```

Do not modify these launch scripts, their arguments, default behavior, path
semantics, or operator expectations during vision documentation or package
boundary work. Use `bash -n` only unless a task explicitly permits startup.

## Focused Checks

Recommended focused checks after vision snapshot documentation or
behavior-neutral boundary work:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPYCACHEPREFIX=/tmp/teto_codex_pycache .venv_lab/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_camera_snapshot.py \
  tests/test_camera_source_adapter.py \
  tests/test_realsense_snapshot_builder.py
```

Startup script syntax checks only:

```bash
bash -n scripts/start_teto_real_full_stack.sh
bash -n scripts/start_teto_isaac_gui_operator.sh
```

Always finish with:

```bash
git diff --check
```
