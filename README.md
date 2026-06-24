# TETO VLM-UR5e

The formal visual pipeline is now RealSense D455 native. Runtime and training
inputs must be a RealSense Scene Snapshot or RealSense Snapshot Replay that
keeps RGB, aligned depth, camera information, metadata, TF/timestamps, and
scene identity together.

## Current Goals

This project currently focuses on:

- Startup animation and launcher
- RealSense D455 snapshot and snapshot-replay contracts
- Aligned RGB-D geometry validation and 2D-to-3D projection
- Snapshot identity, camera information, metadata, TF, timestamps, and
  `scene_version`
- Model-safe preprocessing of the RGB frame from an accepted snapshot
- Prompt template management
- Mock local VLM inference interface
- Optional local Qwen2.5-VL inference through the existing Ollama demo path
- `robot_task_json` result inspector / replay viewer for saved JSONL runs
- Simulation object execution reports and evidence exports through Isaac Runtime
- Read-only UR5e articulation readiness, articulation state observation, and
  simulation-only motion precheck contracts
- Full robot-system shadow bridge evidence that merges ROS2 fake-publish,
  MoveIt plan-only, and UR5 read-only state declarations without motion
- Unified segmented real UR5e relative-motion operation with measured
  per-segment safety gates
- Placeholder robot interface

Historical arbitrary-image conversion and RGB-only demos remain temporarily
available as legacy/debug tools during Phase 1 cleanup. They are not formal
visual inputs, do not create RealSense Scene Snapshots, and must not feed robot
runtime or training datasets intended for the RealSense-native pipeline.

Future work can broaden Qwen2.5-VL, LLaVA, InternVL, Isaac Sim execution,
live ROS2 publish, MoveIt2 execution, UR5 controllers, or other local robotics
and VLM components beyond the current gated paths.

## Current Real And Simulation Execution Modes

The default real console is now the unified segmented real operator:

```bash
bash scripts/start_teto_real_full_stack.sh
```

The startup wrapper can start real-lab services, including the UR driver,
`tcp_pose_broadcaster`, Qwen readiness, MoveIt readiness, controller
activation, and the real operator console. It must be run only with a human
operator physically present, lab permission, and the real UR5e workcell in the
expected state. Use status-only mode when you only want inspection:

```bash
bash scripts/start_teto_real_full_stack.sh --status
```

The default real mode is autonomous segmented execution with measured
per-segment gates. Autonomous does not mean ungated, and it is not the old
single command `y` prompt path. Before each real segment can execute, TETO
requires Dashboard state, active controller state, joint state availability,
MoveIt availability, a fresh current TCP pose, D455 snapshot freshness/sync and
newer-than-previous checks, the shared bounded relative-motion contract, and
the cartesian safety gateway. After each segment, TETO verifies the measured
after TCP pose, position error, direction projection, orientation change, and
post-motion verification result.

The shared bounded relative-motion envelope is `0.50 m`; a `0.51 m` request is
expected to block. Long accepted requests are decomposed into bounded substeps
rather than sent as one large motion.

The legacy real-small-motion acceptance path still exists only as an explicit
legacy manual mode:

```bash
bash scripts/start_teto_qwen_real_operator.sh --legacy-manual-console
```

Dry-run, plan-only, Isaac, fake, or synthetic evidence must not be counted as
REAL_PATH success evidence. Protective-stop recovery, unlock, dashboard
recovery actions, and other robot recovery operations are human lab actions;
they must not be performed automatically by an LLM or by TETO.

Isaac remains SIM_ONLY. The Isaac operator records `execution_mode: isaac_sim`
and simulation evidence, rejects real mode, and does not use Dashboard, RTDE
write, MoveIt `ExecuteTrajectory`, or a physical UR5e connection.

Current recommended real entry remains
`bash scripts/start_teto_real_full_stack.sh`, with
`scripts/teto_operator_console.py` as the real console implementation. Current
recommended Isaac entry remains
`bash scripts/start_teto_isaac_gui_operator.sh --gui --console`.
Legacy/debug/historical tools are marked in their file headers and should not
be used as default architecture paths. Dry-run, plan-only, fake, synthetic, or
Isaac evidence from those tools is not REAL_PATH success evidence. The
`outputs/` tree has not been cleaned; artifact retention or deletion needs a
separate audit and user decision.

## TETO v3.0.6: Planner Audit Evidence

TETO v3.0.5 verifies the current TCP pose before relative real motion and
blocks direction parsing mismatches before target generation. TETO v3.0.6 does
not change planner behavior yet, does not switch to Cartesian path planning,
and does not add a hard path-quality blocker.

Instead, v3.0.6 records how MoveIt planned a relative TCP step so strange paths
can be diagnosed offline. The run evidence now reports planner mode, planner
pipeline and planner id, MoveIt goal type, start-state source, target
orientation source, joint deltas, wrist deltas, Cartesian-path usage status,
and path metrics when a planned trajectory is available. This is intended to
explain cases where the target position is reached but the wrist rotates more
than expected.

The planner audit evidence is software-only and mock-testable. It does not
require a UR5 connection, does not launch robot bringup, does not call
ExecuteTrajectory, and does not send a trajectory.

## TETO v3.0.7: Soft Planner Risk Classification

TETO v3.0.7 classifies the v3.0.6 planner audit evidence into
`planner_risk_status=PASS`, `WARN`, `UNKNOWN`, or `NOT_APPLICABLE`. The default
policy is `lab_planner_audit_soft_v1` in `soft_warn` mode with planner-risk
blocking disabled.

The classifier warns about suspicious joint and wrist deltas, suspected joint
wrap, and long joint-path-to-requested-distance ratios. It also records
informational advisory codes for implicit planning-scene start state and
current MoveGroup pose-goal planning where `cartesian_path_used=false`. Those
advisories live in `planner_risk_infos`, not `planner_risk_warnings`, and do
not by themselves mean the plan failed.

This version does not change default MoveIt planning behavior, does not switch
to `compute_cartesian_path`, and does not block execution by default. Risk
classification is evidence-only unless an explicit hard-block policy is enabled
later. The feature is offline-testable with mocks and requires no UR5
connection.

## TETO v3.0.8: Long-Step Decomposition Contract

TETO v3.0.5 verifies the current TCP pose before real relative motion. TETO
v3.0.6 records planner audit evidence, and v3.0.7 classifies that evidence with
soft planner risk status. TETO v3.0.8 adds an offline long-step decomposition
contract for longer relative TCP requests.

When enabled, a long requested displacement can be accepted only as a
decomposed contract: each substep must remain within the configured substep and
single-step safety limits, while the total displacement and final workspace or
session envelope must remain valid. The decomposition evidence records the
distance regime, substep distances and vectors, total-motion checks, workspace
checks, and explicit proof that safety limits were not bypassed.

The 5 cm boundary remains explicit: with decomposition disabled, `0.05 m` can
remain a valid one-shot step when it is within the configured hard safety
limit. With decomposition enabled, the same `0.05 m` request may also be
represented as a contract-only decomposition such as `[0.02, 0.02, 0.01]`.
Requests above the one-shot hard limit are never accepted as one-shot motions;
they can pass only as `decomposed_autoregressive_contract` when all substep,
total, workspace, and session checks pass.

v3.0.8 does not execute multiple real robot motions, does not send
trajectories, and keeps `real_substep_execution_enabled=false` with
`substep_execution_mode=contract_only`. The feature is offline-testable and
requires no UR5 connection.

## TETO v3.0.9: Expanded Motion Permission Envelope

TETO v3.0.9 expands only the decomposed contract preview envelope for longer
relative TCP requests. The one-shot real-motion envelope remains unchanged:
`max_one_shot_distance_m=0.05` by default, and requests above that limit are
still blocked as one-shot motions.

When long-step decomposition is enabled, contract-only previews can be accepted
up to `max_decomposed_total_distance_m=0.20` by default, with each substep
bounded by `max_decomposed_substep_distance_m=0.02`. The evidence reports the
motion permission envelope version, one-shot and decomposed limits, requested
distance, planned execution style, substep count, substep vectors, and
contract-only execution flags.

This does not enable real multi-step execution. Accepted decomposed contracts
keep `planned_execution_style=decomposed_autoregressive_contract`,
`substep_execution_mode=contract_only`,
`real_substep_execution_enabled=false`, `execute_trajectory_called=false`,
`trajectory_sent=false`, and `real_robot_motion_executed=false`. They do not
create a one-shot target pose or MoveIt plan request for the full long motion.

## TETO v3.0.10: Natural-Language Motion Coverage Expansion

TETO v3.0.10 expands natural-language coverage for relative TCP motion
commands while preserving all robot execution safety gates. It recognizes more
human-like English motion phrases, common TCP/tool/end-effector synonyms,
centimeter and millimeter distance expressions, small written numbers, fuzzy
small-step phrases such as "a little" or "slightly", and a small set of common
Chinese relative-motion phrases.

Language coverage does not expand one-shot real execution permission. Parsed
commands still flow through current TCP readiness, direction guard, one-shot
distance limits, long-step decomposition contract checks, planner audit
evidence, planner risk classification, and manual confirmation behavior before
any real execution path. Parser evidence records
`execution_permission_decided_by_parser=false` and
`safety_gate_still_required=true`.

Fuzzy small-step commands infer a conservative default distance
(`default_small_step_m=0.01` by default), set
`distance_source=inferred_default`, and keep `requires_confirmation=true`.
Ambiguous commands such as "move it over there" or object/vision commands such
as "move to the mug" are classified as clarification-required or unsupported
and are not converted into executable robot motion.

An offline coverage report can be generated with:

```bash
python3 scripts/run_motion_language_coverage.py
```

## TETO v3.0.11: LLM-First Motion Semantics Alignment

TETO v3.0.11 changes natural-language motion parsing from an expanding
hand-written phrase parser into an LLM-first semantic alignment path. Qwen is
asked to emit a structured `teto_motion_semantics.v1` JSON object describing
intent status, intent type, semantic direction, distance quality, converted
meters, language, and confidence. TETO then validates that schema, canonicalizes
the semantic direction enum into the robot-safe base-link axis/sign convention,
converts units, generates a relative TCP motion contract, and records replayable
evidence.

The deterministic rule parser remains available only as a fallback when Qwen is
missing, malformed, invalid, or below confidence threshold. A valid
high-confidence Qwen semantic parse is not overridden by raw-text synonym
matching; disagreements with the fallback parser are recorded as
`qwen_fallback_conflict` evidence. Parser evidence now includes
`semantic_alignment_version=teto_v3_0_11_llm_first_motion_semantics_v1`,
Qwen semantic fields, fallback usage, canonicalization source, and the unchanged
safety handoff flags `execution_permission_decided_by_parser=false` and
`safety_gate_still_required=true`.

This release does not expand robot execution permission. Current TCP readiness,
direction guard, one-shot limits, long-step decomposition contract checks,
planner audit evidence, planner risk classification, and manual confirmation
remain downstream safety gates before any real execution path.

## TETO v3.0.12: Offline Long-Distance Autoregressive Motion Planner

TETO v3.0.12 introduces an offline long-distance autoregressive motion
planner. It converts an accepted canonical long-distance decomposed contract
into a per-substep preview plan. Each substep records current/latest verified
TCP readiness, target generation, planner audit and risk placeholders,
verification policy, simulated post-step evidence, and an explicit
continue-or-abort decision.

Targets are generated sequentially from the latest simulated verified TCP
pose. They are not all precomputed from the initial pose. This models the
future observe, generate, plan/audit, execute-or-skip, verify, and continue-or-
abort loop while keeping execution disabled.

The one-shot envelope is unchanged at `max_one_shot_distance_m=0.05` by
default. Real multi-step execution remains disabled. The planner runs only in
`offline_preview` or `contract_only` mode, keeps
`real_substep_execution_enabled=false`, never creates a real MoveIt request,
never calls ExecuteTrajectory, and never sends a trajectory. Qwen or the
fallback normalizer supplies only canonical motion semantics:
`execution_permission_decided_by_parser=false`. The downstream safety gate
remains authoritative.

The standalone historical preview entrypoint was removed in H6 Round 1A.
The underlying planner contract remains covered by tests and by the guarded
real long-motion safety harness.

## TETO v3.0.13: Minimal Guarded Real Vector Long-Motion Test

TETO v3.0.13 extends `relative_cartesian_motion` to support a base-link
Cartesian delta vector `[dx, dy, dz]`. Single-axis commands remain compatible
and are represented internally as vectors. Multi-axis commands and explicit
`--delta-json` input produce `vector_relative` contracts whose requested
distance is the Euclidean vector norm.

Long vector motions are decomposed by straight-line interpolation in TCP
position space. TETO does not execute all X motion followed by Y and Z. For
the intended first lab command, `move forward 30 cm and left 10 cm`, the
vector is `{"x": 0.30, "y": 0.10, "z": 0.0}`, its norm is approximately
`0.316 m`, and a `0.02 m` maximum substep norm produces 16 autoregressive
vector substeps with a nominal delta of
`{"x": 0.01875, "y": 0.00625, "z": 0.0}` each.

The minimal one-command test entrypoint is:

```bash
python3 scripts/safety_harnesses/run_real_long_motion_safety_check.py \
  --delta-json '{"x":0.30,"y":0.10,"z":0.0}' \
  --max-total-distance-m 0.35 \
  --max-substep-distance-m 0.02 \
  --mock-current-tcp-pose \
  '{"position_m":[0.0,0.0,0.5],"orientation_xyzw":[0,0,0,1]}'
```

This defaults to offline preview. Real autoregressive execution requires all
three explicit flags: `--real`, `--enable-real-autoregressive-execution`, and
`--armed-long-motion-test`. Each armed substep reads the current TCP pose,
generates one target from that latest actual pose, plans and executes only that
substep, reads TCP again, verifies vector projection, distance, and orthogonal
drift, and aborts immediately on failure.

The one-shot real-motion limit remains unchanged at `0.05 m`; a long vector
cannot use the one-shot path. The parser still records
`execution_permission_decided_by_parser=false`, and the downstream safety
gate remains authoritative. No scenario harness, batch test framework, YAML
scenario infrastructure, warmup sequence, or auto-reverse behavior was added.

## TETO v3.0.14: Safety Gateway Authority Repair

TETO v3.0.14 repairs the safety authority boundaries identified by the
v3.0.13 code-health audit. The authoritative Cartesian gateway now enforces a
`max_one_shot_distance_m=0.05` default independently of expanded decomposed or
vector preview envelopes. Requests above that limit cannot silently become
one-shot target poses.

Real-motion evidence now preserves whether ExecuteTrajectory was attempted or
a real motion command was sent even when post-motion verification later fails.
Every armed autoregressive vector substep must route through an authoritative
gateway that supplies measured current TCP readiness, safety/execution results,
and post-motion verification evidence. The real long-motion CLI no longer
constructs synthetic confirmation or safety-state values and fails closed when
the authoritative gateway is unavailable.

This safety repair does not add a scenario harness, batch framework, or new
execution permission. Preview behavior and the v3.0.13 vector decomposition
geometry remain unchanged.

## Project Structure

```text
teto_vlm/
├── README.md
├── requirements.txt
├── assets/
│   └── sample_images/
├── data/
│   ├── raw/
│   ├── processed/
│   └── annotations/
├── outputs/
│   ├── logs/
│   ├── image_conversion/
│   └── results/
├── src/
│   ├── cli.py
│   ├── image_utils.py
│   ├── prompt_utils.py
│   ├── vlm_infer.py
│   ├── dataset_utils.py
│   ├── robot_interface.py
│   └── logger.py
├── scripts/
│   ├── start_teto_real_full_stack.sh
│   ├── start_teto_isaac_gui_operator.sh
│   └── check_env.py
└── tests/
    ├── test_image_utils.py
    └── test_prompt_utils.py
```

## Install

```bash
python3 -m pip install -r requirements.txt
```

`torch`, `transformers`, and `accelerate` are optional for future real VLM
inference and are not required for the mock demo. The Qwen backend does not
install or download models; it reuses the local Ollama model name from the
existing `qwen_vl_demo.py.save` script.

## Run

Check the environment:

```bash
python3 scripts/check_env.py
```

Validate the formal RealSense snapshot replay example:

```bash
python3 -m src.cli snapshot-replay \
  --snapshot-manifest configs/camera_snapshot.example.yaml
```

The formal snapshot contract carries at least:

- `snapshot_id`
- `scene_version`
- `rgb_ref`
- `depth_ref`
- `camera_info_ref`
- `metadata_ref`
- `tf_snapshot_ref`
- `capture_timestamp`
- `source: realsense_d455` or `source: realsense_replay`

The example manifest is a schema fixture, not a real D455 capture. TETO now
provides an artifact-to-manifest builder for already captured D455 files:

```bash
python3 scripts/build_realsense_snapshot_bundle.py \
  --snapshot-id d455_snapshot_001 \
  --scene-version lab_scene_001 \
  --rgb /path/to/color.png \
  --aligned-depth /path/to/aligned_depth_u16.png \
  --camera-info /path/to/camera_info.json \
  --metadata /path/to/capture_metadata.json \
  --tf-snapshot /path/to/tf_snapshot.json \
  --capture-timestamp 2026-06-19T00:00:00Z \
  --output-manifest outputs/realsense_snapshot_fixtures/d455_snapshot_001.yaml
```

The builder checks that every required artifact exists, RGB and aligned depth
have matching dimensions, metadata/camera-info/TF files contain structured
objects, and the generated manifest passes `evaluate_formal_snapshot_replay`.
It writes stable references and does not copy or modify the input artifacts.
Missing RGB, aligned depth, camera information, metadata, TF, scene identity,
or timestamp fails closed.

This is not a live D455 capture implementation. It does not start a camera,
ROS, MoveIt, or a robot, and it cannot manufacture a missing TF calibration.
A native capture component must still acquire and verify the RGB/depth pairing,
alignment, device metadata, calibration, TF snapshot, timestamps, and file
provenance before the builder is called.

The old RGB-only launcher/demo entrypoints were removed in H6 Round 1A. Current
formal visual inputs are RealSense snapshot/replay contracts.

## Prompt Types

Prompt templates are defined in `src/prompt_utils.py` and can be used by both
single image recognition and batch image recognition.

Current prompt types:

- `describe_image`: brief image description.
- `locate_objects`: visible objects and approximate image positions.
- `spatial_relationship`: simple spatial relationship description between main
  objects. This is an existing spatial prompt and is equivalent to a basic SRL
  task.
- `spatial_relationship_lite`: SRL prompt for listing main visible objects and
  describing approximate spatial relationships with simple terms.
- `spatial_relationship_json`: SRL prompt that asks for approximate object
  positions and relations in JSON format.
- `robot_instruction_parse`: convert a natural language instruction into a
  simple JSON robot action plan.
- `manipulation_candidate`: existing robot manipulation prompt that asks which
  visible object is suitable for manipulation; it is related to manipulation
  spatial reasoning but is not the full SRL template.
- `manipulation_spatial_analysis`: robot manipulation-oriented spatial prompt
  covering objects, positions, relationships, easiest target, and obstacles.
- `robot_task_json`: controlled JSON intermediate representation for a later
  robot pipeline. It produces structured scene and target analysis only. It is
  not a robot control command, does not execute UR5, and does not generate
  URScript, joint angles, trajectories, or motion plans. It is intended for
  later planner_gateway / semantic_task_server stages to read, while the
  current project only performs structured understanding and safety judgment.
  Humans, animals, and unsafe or unsuitable objects must be marked with
  `manipulation_assessment.candidate=false`.

Inspect the latest saved controlled robot task JSON run without rerunning VLM
inference:

```bash
python3 scripts/inspect_robot_task_json.py
```

Inspect a specific saved run and show per-item details:

```bash
python3 scripts/inspect_robot_task_json.py --run-dir outputs/results/robot_task_json/run_YYYYMMDD_HHMMSS --details
```

Limit detailed output:

```bash
python3 scripts/inspect_robot_task_json.py --details --limit 20
```

`robot_task_json` is designed as a structured handoff for future stages such as
planner_gateway / semantic_task_server. The current TETO project only performs
structured visual understanding, JSON validation, and manipulation safety
assessment. It does not execute UR5, connect to ROS2 / MoveIt, or produce robot
control commands. Humans, living animals, fragile, dangerous, sharp, hot,
liquid-filled, transparent, reflective, unclear, or otherwise unsuitable
objects should not be selected as robot manipulation candidates. When such a
target is detected, `candidate` should be `false`, `difficulty` should be
`unsafe` when appropriate, and `error.code` should be `E_UNSAFE`.

Saved `robot_task_json` runs can still be inspected directly, but the old
interactive visual launcher was removed in H6 Round 1A.

## Historical TETO V1.1.4 legacy RGB-only smoke test

Use the Qwen backend to create a small real-output smoke set without connecting
to ROS2, MoveIt, UR5, or any robot controller. Prepare four image folders or a
single folder containing:

- a normal manipulable object, such as a cup or box
- a person / human image
- a bird / animal image
- an empty scene or scene with no clear manipulation target

Inspect a saved run:

```bash
python3 scripts/inspect_robot_task_json.py --details
```

Check the inspector summary and item details for:

- `parse_success`
- `validation_failed`
- `unsafe_count`
- `rejected_count`
- `validation_errors`

Expected safety behavior: person / human and bird / animal cases should be
rejected as manipulation candidates. If the output clearly names only living
beings and no safe target, the normalized error should prefer `E_UNSAFE`.
Empty or truly unclear scenes may remain `E_NO_TARGET`.

## TETO V1.2.0 2D grounding preparation

`robot_task_json` can now carry rough 2D target grounding fields for future
2D-to-3D work:

- `target.bbox_xyxy`: approximate `[x_min, y_min, x_max, y_max]` target box in
  input-image pixels, or `null`.
- `geometry_2d.pixel_center`: approximate `[cx, cy]` candidate pixel, or `null`.
  If a valid bbox is present and the center is missing, TETO may infer it from
  the bbox.
- `geometry_2d.image_width` and `geometry_2d.image_height`: image dimensions
  filled by the program from the input image when possible.
- `geometry_2d.confidence`: rough 2D localization confidence from `0.0` to
  `1.0`, or `null`.

These fields are visual grounding hints only. They are not 3D coordinates, not
camera/world coordinates, not MoveIt goals, not UR5 commands, and not robot
control instructions. A bbox or pixel center must not be used directly for real
robot execution. Later versions may feed `pixel_center` into a separate
2D-to-3D projector, but V1.2.0 does not read depth images, compute TF, generate
trajectories, or command hardware.

## TETO V1.2.1 smoke report and validation display hardening

After each `robot_task_json` batch run, TETO writes a queryable smoke report in
the run directory:

```text
outputs/results/robot_task_json/run_YYYYMMDD_HHMMSS/smoke_report.md
outputs/results/robot_task_json/run_YYYYMMDD_HHMMSS/smoke_report.json
```

The report summarizes parse, validation, rejection, unsafe, no-target, and 2D
grounding counts, then lists compact per-image details. Inspector output now
distinguishes pre-normalization errors from post-normalization errors, so safety
issues corrected by normalization remain visible for audit without being shown
as unresolved normalized-state failures.

## TETO V1.2.2 real Qwen smoke verified

TETO V1.2.2 fixes smoke report grounding display to use normalized grounding
fields only. Raw model grounding is retained separately as `raw_*` audit data
and does not affect `grounding_count`.

Verified real Qwen smoke run:

```text
outputs/results/robot_task_json/run_20260528_172133
```

Observed summary:

- `total_count`: 4
- `parse_success_count`: 4
- `validation_failed_count`: 0
- `rejected_count`: 3
- `grounding_count`: 1
- `grounding_missing_count`: 3
- `no_target_count`: 3

For `E_NO_TARGET` / `unknown` items, normalized `bbox_xyxy` and `pixel_center`
are `null`, and `grounded` is `false`. The valid `camera` target is counted as
the single grounded item. `post_normalization_errors` is empty for all four
items.

## Historical TETO V1.3.0 legacy semantic image record

Historical RGB-only `robot_task_json` output keeps a compatibility `scene`
record for semantic replay. It is explicitly a
`legacy_rgb_only_record`, not a RealSense Scene Snapshot:

```json
{
  "scene": {
    "scene_version": "run_YYYYMMDD_HHMMSS_item_001",
    "capture_timestamp": "ISO8601 string or unknown",
    "image_path": "input image path",
    "image_width": 640,
    "image_height": 480,
    "record_type": "legacy_rgb_only_record",
    "source": "legacy_semantic_image",
    "is_realsense_scene_snapshot": false,
    "status": "valid"
  }
}
```

The scene image size mirrors `geometry_2d.image_width` and
`geometry_2d.image_height`. `scene.status` is `invalid` for parse failures or
post-normalization validation failures, otherwise `valid`. Candidate targets
also receive `target.target_id="obj_001"` when `candidate=true` and the target
label is known; no-target, unknown, rejected, or unsafe cases use
`target_id="unknown"`.

This compatibility record has no depth, camera information, metadata, TF, or
sensor-backed timestamp. It must not be promoted to the formal RealSense
snapshot pipeline or used as robot visual input.

## TETO V1.4.0 scene cache replay contract preparation

Each `robot_task_json` batch run now writes lightweight scene and replay index
files beside the existing results and smoke report:

```text
outputs/results/robot_task_json/run_YYYYMMDD_HHMMSS/scene_index.json
outputs/results/robot_task_json/run_YYYYMMDD_HHMMSS/replay_index.json
```

`normalized_json.scene` remains a per-record scene snapshot. `scene_index.json`
is the run-level scene cache: it lists scene versions, scene status, target
IDs, labels, candidate flags, error codes, and normalized 2D grounding fields
for quick lookup. It uses normalized `bbox_xyxy`, `pixel_center`, and
`geometry_2d.confidence`; raw model grounding is kept only in the smoke report
audit fields and does not affect grounding counts.

`replay_index.json` is a semantic replay contract for future tooling. It does
not rerun the model and does not execute any robot behavior. It records the
result JSONL path, record index, scene identity, target identity, normalized
grounding hints, replay sample flags, and rejection reason so later tools can
answer why a scene was accepted or rejected.

These indexes are still only software-readable semantic middleware artifacts.
They are not 3D coordinates, MoveIt goals, UR5 commands, URScript, joint
angles, trajectories, or robot control instructions.

## TETO V1.4.1 real-run replay index verification

The saved run inspector now includes read-only `scene_index.json` and
`replay_index.json` inspection. It can summarize scene count, valid/rejected
scene counts, no-target count, normalized grounding count, positive replay
sample count, hard negative sample count, and `rejection_reason` distribution.

It also performs a run-level consistency check between the two index files:
scene and replay record counts, run IDs, and scene version sets are compared
without rerunning the model. Missing index files in older runs are reported as
warnings instead of crashes.

```bash
python3 scripts/inspect_robot_task_json.py --run-dir outputs/results/robot_task_json/run_YYYYMMDD_HHMMSS --indexes
```

This remains semantic middleware inspection only. TETO does not execute robot
behavior, connect to ROS2 / MoveIt / UR5, generate URScript, generate joint
angles, generate trajectories, or send robot control commands.

## TETO V1.5.0 semantic replay helpers

The historical standalone semantic replay entrypoint was removed in H6 Round
1A. The replay helper functions remain test-covered for saved `robot_task_json`
runs. They read existing `replay_index.json` and `results.jsonl` files to
list, filter, inspect, and export replay subsets without rerunning the model.

Filters include `--positive`, `--hard-negative`, `--reason`, `--error-code`,
`--candidate true|false`, and `--grounded true|false`. The statistics view
reports total records, positive samples, hard negatives, rejection reason
counts, error code counts, grounded/ungrounded counts, and candidate /
non-candidate counts.

Exports are JSONL references to the original run, replay record, and result
record. They do not copy images or large files and do not write under
`outputs/` unless that path is explicitly requested. This is intended for hard
negative review, positive sample review, and later data-loop preparation.

Semantic replay is still only semantic middleware. It does not call Qwen, does
not change prompts, does not rerun VLM inference, does not connect to ROS2 /
MoveIt / UR5, and does not generate URScript, joint angles, trajectories, or
robot control commands.

## TETO V1.5.1 semantic replay real-run polish

The semantic replay helpers print run source context in stats output, including
the run directory, replay index path, results JSONL path, and replay `run_id`.
When filters are active, it also reports `filtered_total` and prints the active
filter set so real-run reviews are easier to audit.

`--show N` now uses stable review sections for replay record fields, matching
result record status, normalized scene/target/error fields, and raw audit
fields. Raw bbox and pixel-center fields are displayed only for audit; they do
not affect normalized grounding, positive replay sample, or hard negative
sample decisions.

V1.5.1 remains semantic middleware polish only. It does not implement
planner_gateway, rerun models, call Qwen, change prompts, connect to ROS2 /
MoveIt / UR5, generate URScript, generate joint angles, generate trajectories,
or send robot control commands.

## TETO V1.6.0 planner gateway input contract preparation

`src/planner_gateway_contract.py` defines the first dry-run contract boundary
between TETO semantic results and a future `planner_gateway`. It answers
whether a normalized semantic result is eligible for planner handoff, why it is
rejected when it is not eligible, and what a future planner input skeleton would
look like when the semantic result is eligible.

The eligibility check requires a valid scene, `candidate=true`, `error.code=OK`,
a known `target_id`, a known target label, normalized `bbox_xyxy`, normalized
`pixel_center`, grounded status, positive geometry confidence, and the current
`teto_robot_task.v1` schema. Rejection reason codes include
`E_SCENE_INVALID`, `E_NOT_CANDIDATE`, `E_NO_TARGET`, `E_UNKNOWN_TARGET`,
`E_MISSING_BBOX`, `E_MISSING_PIXEL_CENTER`, `E_NOT_GROUNDED`,
`E_LOW_GEOMETRY_CONFIDENCE`, `E_SCHEMA_UNSUPPORTED`, and
`E_MISSING_NORMALIZED_JSON`.

Eligible records can build a `teto_planner_gateway_input.v1` skeleton with 2D
target fields, confidence fields, dry-run execution policy, and explicit
planner requirements. The skeleton always sets `dry_run_only=true` and
`allow_robot_motion=false`.

The contract explicitly lists missing runtime inputs that future systems must
provide before any real planning could occur:

- `depth_aligned_to_color`
- `camera_point_m`
- `world_point_m`
- `tf_timestamp`
- `scene_ttl_ms`
- `robot_safety_state`

V1.6.0 is still semantic middleware only. It does not implement planner_gateway,
connect to ROS2 / MoveIt / UR5, call Qwen, rerun VLM inference, change prompts,
generate URScript, generate joint angles, generate trajectories, or send robot
control commands.

## TETO V1.6.1 semantic replay planner eligibility display

The semantic replay detail view now includes a planner gateway
eligibility section for the selected replay sample. Reviewers can see whether a
sample is eligible for a future `planner_gateway`, inspect rejection reasons
for rejected samples, and confirm that the planner contract remains dry-run
only.

Eligible samples show a planner input skeleton summary with contract version,
task ID, scene version, intent, target ID, target label, normalized 2D grounding
fields, missing runtime inputs, and execution policy. Rejected samples show
`planner_input: null` along with the eligibility reasons and required missing
fields.

The planner display always reports `dry_run_only=true` and
`allow_robot_motion=false`. It does not add 2D to 3D projection, ROS2, MoveIt,
UR5, URScript, joint angles, trajectories, `tcp_pose_world`, or any robot
control command. TETO remains semantic middleware.

## TETO V1.7.0 2D to 3D projector contract preparation

`src/projector_contract.py` defines a dry-run projector contract for deciding
whether normalized 2D semantic geometry is ready for a future 2D to 3D
projector. It checks candidate status, normalized grounding, `bbox_xyxy`,
`pixel_center`, and geometry confidence, then returns a stable
`teto_projector.v1` result with projector status, missing runtime inputs,
warnings, errors, and `allow_robot_motion=false`.

This release does not implement real projection. `camera_point_m` and
`world_point_m` remain `None`, and projector confidence remains `0.0` until a
future runtime supplies depth, camera info, camera frame, camera extrinsics,
and TF data:

- `depth_sample`
- `camera_info`
- `camera_frame`
- `camera_extrinsics`
- `tf_tree`

`build_projector_input` creates only a dry-run input skeleton containing the
normalized pixel center, bbox, image size, and empty runtime placeholders. It
does not compute depth, camera intrinsics, transforms, world coordinates,
`tcp_pose_world`, joint angles, trajectories, URScript, or robot motion.

V1.7.0 remains semantic middleware plus geometry contract preparation. It does
not call Qwen, rerun models, change prompts, connect to ROS2 / MoveIt / UR5,
or send robot control commands.

## TETO V1.7.1 projector eligibility replay display

The semantic replay detail view now displays projector eligibility beside
planner eligibility for the selected replay sample. The detail view reports
projector status, eligibility, projector confidence, runtime inputs still
missing, errors, warnings, and `allow_robot_motion=false`.

Replay statistics also include projector counts:

- `projector_eligible`
- `projector_rejected`
- `projector_ready`

This is visibility only. It does not perform depth projection, camera
intrinsics computation, TF computation, world coordinate solving,
`tcp_pose_world` generation, joint angle generation, trajectory generation,
URScript generation, or robot motion.

## TETO V1.8.0 execution readiness contract

`src/execution_readiness_contract.py` combines planner gateway eligibility and
projector eligibility into a dry-run execution readiness contract. It answers
whether one normalized semantic result is simultaneously planner-eligible and
projector-eligible, and reports `planner_rejected`, `projector_rejected`, or
`dry_run_ready`.

The contract aggregates planner and projector blocking reasons while keeping
`allow_robot_motion=false` for every status. `build_execution_readiness_input`
creates a dry-run summary with scene, target, 2D grounding, contract versions,
missing runtime inputs, and execution policy.

V1.8.0 is the first merge point for planner and projector layers, but it is
still Semantic Middleware. It does not create MoveIt requests, simulate robot
motion, compute TF, project depth, compute world coordinates, generate
`tcp_pose_world`, generate joint angles, generate trajectories, generate
URScript, or execute a robot.

## TETO V1.8.1 execution readiness replay display

The semantic replay detail view now displays Execution Readiness after
Planner Gateway Eligibility and Projector Eligibility. The replay detail view
shows `dry_run_ready`, `planner_rejected`, or `projector_rejected`, plus ready
state, planner/projector eligibility flags, blocking reasons, warnings, and
`allow_robot_motion=false`.

Replay statistics now include execution readiness counts:

- `execution_ready`
- `execution_rejected`
- `execution_planner_rejected`
- `execution_projector_rejected`
- `execution_dry_run_ready`

This is replay visibility only. It does not add ROS2, MoveIt, Isaac Sim, UR5,
RTDE, TF, depth projection, world coordinates, robot motion, URScript, joint
angles, trajectories, or `tcp_pose_world`.

## TETO V1.9.0 simulation bridge contract

`src/simulation_bridge_contract.py` adds the final semantic preparation layer
before V2.0. It connects Execution Readiness to a simulation task
representation and answers whether a normalized semantic result is ready to be
consumed by a future simulation bridge.

The contract checks that execution readiness is already `true`, that a
`world_point_m` exists, that `scene_version` exists, and that `ttl_ms` exists.
If any requirement is missing, it reports blocking reasons such as
`E_NOT_EXECUTION_READY`, `E_NO_WORLD_POINT`, `E_NO_SCENE_VERSION`, or
`E_NO_TTL`.

When ready, `build_simulation_task` produces only a semantic simulation task:

- `task_type`
- `target_label`
- `target_world_point`
- `scene_version`
- `ttl_ms`

`robot_task_json` results now include `simulation_bridge_result`, and the
inspector reports simulation ready/rejected counts plus a per-item Simulation
Bridge PASS/FAIL detail. This is still V1.x Semantic Middleware. It does not
call Isaac Sim APIs, ROS2, MoveIt, UR5, RTDE, TF, depth projection, robot
motion, URScript, joint angles, trajectories, `moveit_goal`,
`execution_command`, or `tcp_pose_world`.

The removed legacy visual launcher exposed prompt helper keywords for single
image recognition and batch image recognition. Historical shortcuts included:

- `describe`: `describe_image`
- `objects` or `locate`: `locate_objects`
- `spatial` or `relation`: `spatial_relationship`
- `srl`: `spatial_relationship_lite`
- `srl_json` or `json`: `spatial_relationship_json`
- `manipulation` or `grasp`: `manipulation_spatial_analysis`
- `candidate`: `manipulation_candidate`
- `robot_json` or `task_json`: `robot_task_json`

## TETO V2.0.0 First Simulation Execution

TETO V2.0.0 starts the V2 line with the smallest runtime execution step:
`SimulationApp -> World -> world.reset -> simulation_task -> simulation steps
-> simulation_execution_result`.

`src/simulation_runtime.py` provides the runtime boundary:

- `build_simulation_execution_result`
- `run_first_simulation_execution`

The command-line entry point is:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py --dry-run
```

Dry-run and no-Isaac modes do not import Isaac Sim and are intended for normal
pytest and CI-style checks. The real Isaac runtime path delays Isaac imports
until execution time, then uses:

- `from isaacsim import SimulationApp`
- `World()`
- `world.reset()`
- a small number of `world.step(...)` calls

Each run writes:

```text
outputs/simulation_runs/run_YYYYMMDD_HHMMSS/simulation_execution_result.json
```

The execution report includes status, mode, consumed `simulation_task`, reset
state, step counts, blocking reasons, error details, and
`allow_robot_motion=false`.

This is First Simulation Execution only. It does not connect to ROS2, MoveIt,
UR5, RTDE, real robot controllers, TF, depth projection, URScript, joint
angles, trajectories, `tcp_pose_world`, `moveit_goal`, or real robot motion.

If Isaac Sim is available, run the real runtime manually:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py
```

## TETO V2.0.1 World to Cube

TETO V2.0.1 extends the same runtime boundary with the smallest observable
scene object step: `World -> cube -> simulation steps ->
simulation_execution_result`.

Use `--spawn-cube` to request the cube path explicitly while keeping the
V2.0.0 no-object command path available:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py --dry-run --steps 3 --spawn-cube
```

Dry-run and no-Isaac modes still avoid Isaac imports, but when `--spawn-cube`
is set they simulate a successful cube spawn in the report. Real Isaac mode
creates a visible cube near the world origin with safe defaults:

- `cube_prim_path=/World/TETO_Cube`
- `cube_position=[0.0, 0.0, 0.5]`
- `cube_size=0.2`

The report keeps the V2.0.0 fields and adds `simulation_object_spawned`,
`object_type`, `cube_prim_path`, `cube_position`, `cube_size`, and
`cube_spawned`. Cube creation failures are converted into a FAIL report with
`error.code=E_CUBE_SPAWN_FAILED`.

## TETO V2.0.2 Simulation Object Pose Update

TETO V2.0.2 keeps cube as the default Isaac test fixture, but the runtime
boundary is now phrased as a simulation object pose update smoke test:
`World -> simulation object -> update pose -> simulation steps ->
simulation_execution_result`.

Use `--move-object` for the default simulation object pose update smoke test.
This option implies the default fixture spawn step because a pose update needs
an object handle:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py --dry-run --steps 3 --move-object
```

`--move-cube` remains as a backward-compatible alias for the same generic
pose update path.

The default fixture is:

- `object_type=cube`
- `prim_path=/World/TETO_Cube`
- `initial_position=[0.0, 0.0, 0.5]`
- `target_position=[0.3, 0.0, 0.5]`
- `size=0.2`

Internally, the runtime uses a `SimulationObjectSpec`,
`spawn_simulation_object`, and `update_simulation_object_pose` so later smoke
tests can replace the cube fixture with another Isaac object without
rewriting the execution boundary. The report keeps V2.0.1 cube compatibility
fields and adds object movement fields including `simulation_object_moved`,
`cube_move_requested`, `cube_moved`, `cube_initial_position`,
`cube_target_position`, `cube_final_position`, and `cube_displacement`.
Move failures use `error.code=E_SIM_OBJECT_MOVE_FAILED`.

This remains a minimal pose update test only. It does not save screenshots or
video, connect ROS2, MoveIt, UR5, RTDE, or URScript, generate joint angles,
generate `tcp_pose_world`, or control a real robot.

## TETO V2.0.3 Evidence Export Pipeline

TETO V2.0.3 adds a generic execution evidence export pipeline for simulation
runs. It does not capture screenshots or video yet; it creates structured
text and manifest artifacts alongside each `simulation_execution_result.json`.

Each report-writing simulation run now writes these files in the same run
directory under `outputs/simulation_runs/run_YYYYMMDD_HHMMSS/`:

- `summary.md`
- `demo_command.txt`
- `pose_delta.md`
- `evidence_manifest.json`

`summary.md` is the human-readable run summary. It records the TETO version,
run ID, timestamp, mode, status, `error.code`, reset state, step counts,
`allow_robot_motion`, generic simulation object pose fields, and report path.

`demo_command.txt` records the CLI command when available. If a run is started
from Python rather than the CLI, it still records mode, step count, move-object
status, and object type.

`pose_delta.md` focuses on the object pose update:

- `initial_position`
- `target_position`
- `final_position`
- `displacement`
- `moved`

`evidence_manifest.json` is program-readable and uses
`schema_version=teto_evidence_manifest.v1`. It links the report and evidence
files and reserves null placeholders for future capture artifacts:

- `screenshot_before_path: null`
- `screenshot_after_path: null`
- `video_path: null`

The exporter reads `simulation_object_*` fields first and uses `cube_*` fields
only as a compatibility fallback. This keeps V2.0.3 as a generic simulation
object evidence pipeline rather than a cube-specific feature.

## TETO V2.1.0 Robot Asset Loader Contract

TETO V2.1.0 adds a robot asset loader contract and availability smoke test.
This is not a UR5 control release and it does not require a UR5 USD to exist.
The current local Isaac Sim installation can create `SimulationApp` and
`World`, but the built-in Isaac asset root points to a remote S3/Nucleus path
and this project does not depend on network assets for tests or acceptance.

Use `--check-robot-asset` for the default diagnostic path:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py --dry-run --steps 1 --check-robot-asset
```

When no local robot asset path is provided, the run is still a successful
diagnostic and writes report/evidence with:

- `status=PASS`
- `error.code=OK`
- `robot_asset_available=false`
- `robot_asset_loaded=false`
- `robot_asset_status=UNAVAILABLE`
- `robot_asset_blocking_reason=E_ROBOT_ASSET_UNAVAILABLE`

The generic robot asset contract records:

- `robot_asset_check_requested`
- `robot_asset_load_requested`
- `robot_type`
- `robot_prim_path`
- `robot_asset_path`
- `robot_asset_source`
- `robot_asset_available`
- `robot_asset_loaded`
- `robot_prim_exists`
- `robot_asset_status`
- `robot_asset_blocking_reason`

Use `--load-robot-asset --robot-asset-path <local.usd>` only when a local
USD/USDA/USDC asset exists and should be referenced into the Isaac stage. In
load mode, an invalid or missing path is a FAIL report. In check mode, missing
local assets are a PASS diagnostic. Evidence summaries and manifests include a
robot asset section, while screenshot and video paths remain null placeholders.

This remains asset preparation only. It does not download assets, depend on
network/Nucleus, connect ROS2, MoveIt, UR5 hardware, RTDE, or URScript,
generate joint angles, generate `tcp_pose_world`, read or write robot joint
state, capture screenshots or video, or control a robot.

## TETO V2.1.3 UR5e Structure Report Export

TETO V2.1.3 keeps the V2.1.2 read-only robot prim inspection and joint metadata
classification, then exports a presentation-friendly UR5e structure report. It
is intended to inspect the USD stage structure under `/World/TETO_Robot` after
a robot asset has been loaded, classify joint-like metadata entries, and
summarize asset loading, prim structure, joint classification, and safety
boundaries in a single evidence document.

This is not robot control. The inspection only reads prim paths, type names,
applied API schemas, descendant counts, link-like prims, joint-like prims,
visual-like prims, collision-like prims, articulation-root indicators, and
possible DOF names/counts derived from joint-like prim names. It does not
generate joint targets, joint angles, trajectories, URScript,
`tcp_pose_world`, ROS2 messages, MoveIt requests, RTDE commands, or real UR5
control actions.

V2.1.3 preserves the V2.1.2 joint-like metadata classification:

- UR5e arm joints: `shoulder_pan_joint`, `shoulder_lift_joint`,
  `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`
- structural joints: `root_joint`
- gripper/tool joints: `robot_gripper_joint`
- unknown joints: any future joint-like metadata name not covered above

`possible_dof_count` and `possible_dof_names` are metadata candidate counts
only. They are not joint targets, not joint commands, and do not indicate robot
control capability.

Use dry-run mode to verify the report/evidence shape without Isaac:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py --dry-run --steps 1 --inspect-robot-prim
```

Use true Isaac mode with a local UR5e USD to inspect the loaded prim:

```bash
PYTHONPATH=. /home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh scripts/harnesses/run_shadow_simulation_contract.py \
  --steps 1 \
  --load-robot-asset \
  --robot-asset-path /home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd \
  --inspect-robot-prim
```

The structured report includes:

- `robot_prim_inspection_requested`
- `robot_prim_inspection.robot_prim_path`
- `robot_prim_inspection.robot_prim_exists`
- `robot_prim_inspection.robot_root_type_name`
- `robot_prim_inspection.total_descendant_prim_count`
- `robot_prim_inspection.link_like_prim_count`
- `robot_prim_inspection.joint_like_prim_count`
- `robot_prim_inspection.visual_like_prim_count`
- `robot_prim_inspection.collision_like_prim_count`
- `robot_prim_inspection.articulation_root_found`
- `robot_prim_inspection.physics_schema_summary`
- `robot_prim_inspection.joint_names`
- `robot_prim_inspection.joint_prim_paths`
- `robot_prim_inspection.possible_dof_names`
- `robot_prim_inspection.possible_dof_count`
- `robot_prim_inspection.joint_metadata_summary`
- `robot_prim_inspection.joint_metadata_table`
- `robot_prim_inspection.inspection_status`
- `robot_prim_inspection.inspection_warnings`
- `robot_structure_report_generated`
- `robot_structure_report_path`

Evidence export adds a `Robot Prim Inspection` section to `summary.md`, a
`Joint Metadata Classification` table in `summary.md`, a
`robot_prim_inspection` object in `evidence_manifest.json`, and
`robot_prim_inspection.json` when inspection is requested. V2.1.3 also writes
`robot_structure_report.md` in the same run directory and links it from
`summary.md`, `simulation_execution_result.json`, and `evidence_manifest.json`.
Screenshot and video placeholders remain null.

`robot_structure_report.md` is designed for presentation evidence and includes:

- basic run and asset information
- Asset Load Summary
- Prim Structure Summary
- Joint Metadata Classification
- Joint Metadata Table
- Safety Boundary
- Presentation Summary

## TETO V2.2.0 Articulation Readiness Contract

TETO V2.2.0 adds a read-only articulation readiness contract on top of the
UR5e structure report. It checks whether the loaded robot prim metadata has the
minimum structure expected before future simulation-side articulation work:
the robot prim exists, an articulation-root indicator is present, the six
standard UR5e arm joint names are visible, and visual/collision prims are
present.

This is still not robot control. `READY` means the USD metadata appears ready
for later inspection or planning integration work; it does not enable control,
does not generate motion, and does not generate commands. The report always
keeps:

- `control_enabled=false`
- `motion_generated=false`
- `command_generated=false`
- `allow_robot_motion=false`

Run the dry-run evidence shape check without Isaac:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py --dry-run --steps 1 --inspect-robot-prim --check-articulation-readiness
```

Run true Isaac with the locally cached UR5e USD:

```bash
PYTHONPATH=. /home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh scripts/harnesses/run_shadow_simulation_contract.py \
  --steps 1 \
  --load-robot-asset \
  --robot-asset-path /home/newusername/Storage/isaac_assets/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd \
  --inspect-robot-prim \
  --check-articulation-readiness
```

The structured report adds:

- `articulation_readiness_requested`
- `articulation_readiness.readiness_status`
- `articulation_readiness.articulation_ready`
- `articulation_readiness.control_enabled`
- `articulation_readiness.motion_generated`
- `articulation_readiness.command_generated`
- `articulation_readiness.robot_prim_path`
- `articulation_readiness.articulation_root_found`
- `articulation_readiness.arm_joint_count`
- `articulation_readiness.required_arm_joint_count`
- `articulation_readiness.arm_joint_names`
- `articulation_readiness.missing_arm_joint_names`
- `articulation_readiness.extra_joint_like_names`
- `articulation_readiness.has_visual_prims`
- `articulation_readiness.has_collision_prims`
- `articulation_readiness.has_robot_structure_report`
- `articulation_readiness.missing_requirements`
- `articulation_readiness.warnings`
- `articulation_readiness.safety_boundary`

Evidence export adds an `Articulation Readiness` section to `summary.md`, an
`articulation_readiness` object in `evidence_manifest.json`, and
`articulation_readiness.json` when readiness is requested. If the run also
generates `robot_structure_report.md`, that report includes an `Articulation
Readiness` section as well. Screenshot and video placeholders remain null.

V2.2.0 does not connect ROS2, MoveIt, RTDE, URScript, or a real UR5. It does
not generate joint targets, joint angles, `tcp_pose_world`, trajectories,
robot commands, or any simulated robot motion.

## TETO V2.3.0 Articulation State Observation Contract

TETO V2.3.0 adds a read-only articulation state observation contract after the
V2.2.0 articulation readiness layer. It observes and organizes UR5e joint
metadata/state fields from Isaac-side robot prim information so later versions
can prepare for simulation robot motion integration without crossing into
control in this release.

This is still not robot control. Observation means metadata/state reporting
only. The V2.3.0 report always keeps:

- `metadata_only=true`
- `control_enabled=false`
- `motion_generated=false`
- `command_generated=false`
- `joint_targets_generated=false`
- `allow_robot_motion=false`

Run the dry-run observation shape check without Isaac:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py --dry-run --steps 1 --observe-articulation-state
```

Run true Isaac with the locally cached UR5e USD and the full read-only chain:

```bash
PYTHONPATH=. /home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh scripts/harnesses/run_shadow_simulation_contract.py \
  --steps 1 \
  --check-robot-asset \
  --inspect-robot-prim \
  --check-articulation-readiness \
  --observe-articulation-state
```

When a local default UR5e USD exists at the known project machine path, the
true Isaac check path can load it for this read-only inspection chain. If no
articulation is available, the run may still pass as a diagnostic while
`articulation_state.status` reports `NOT_OBSERVABLE` or `NOT_AVAILABLE`.

The structured report adds:

- `articulation_state_observation_requested`
- `articulation_state_observable`
- `articulation_state_path`
- `articulation_state_report_generated`
- `articulation_state_report_path`
- `articulation_state.status`
- `articulation_state.metadata_only`
- `articulation_state.control_enabled`
- `articulation_state.motion_generated`
- `articulation_state.command_generated`
- `articulation_state.joint_targets_generated`
- `articulation_state.arm_joint_count`
- `articulation_state.observed_joint_count`
- `articulation_state.expected_arm_joint_names`
- `articulation_state.observed_arm_joint_names`
- `articulation_state.missing_arm_joint_names`
- `articulation_state.extra_joint_names`
- `articulation_state.joint_positions_available`
- `articulation_state.joint_velocities_available`
- `articulation_state.joint_limits_available`
- `articulation_state.joint_state_table`
- `articulation_state.warnings`
- `articulation_state.errors`
- `articulation_state.safety_boundary`

Each joint state table row records the joint name, category, optional position
and velocity, optional lower/upper limits, limit availability, within-limit
status, and explicit metadata-only / no-control flags.

Evidence export adds an `Articulation State Observation` section to
`summary.md`, an `articulation_state` object in `evidence_manifest.json`,
`articulation_state.json`, and `articulation_state_report.md` when observation
is requested. Screenshot and video placeholders remain null.

V2.3.0 does not connect ROS2, MoveIt, RTDE, URScript, or a real UR5. It does
not generate joint targets, joint angles, `tcp_pose_world`, trajectories,
robot commands, or any simulated robot motion.

## TETO V2.4.0 Simulation-Only Motion Precheck Contract

TETO V2.4.0 adds a simulation-only motion precheck contract after the V2.3.0
articulation state observation layer. It is the final precheck gate before the
V2.5.0 first simulation robot micro-motion stage. V2.4.0 only answers:
if a future Isaac-only UR5e micro-motion were requested, do the current asset,
prim, readiness, state, and joint-limit observations satisfy the prerequisites?

This is still not robot control. The V2.4.0 contract is metadata/readiness/state
assessment only. It explicitly keeps:

- `metadata_only=true`
- `simulation_only=true`
- `control_enabled=false`
- `motion_generated=false`
- `command_generated=false`
- `joint_targets_generated=false`
- `trajectory_generated=false`
- `tcp_pose_world_generated=false`
- `robot_motion_executed=false`
- `real_robot_allowed=false`

Run the dry-run precheck shape without Isaac:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py --dry-run --steps 1 --check-simulation-motion-precheck
```

Run true Isaac with the full read-only chain:

```bash
PYTHONPATH=. /home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh scripts/harnesses/run_shadow_simulation_contract.py \
  --steps 1 \
  --check-robot-asset \
  --inspect-robot-prim \
  --check-articulation-readiness \
  --observe-articulation-state \
  --check-simulation-motion-precheck
```

The structured report adds:

- `simulation_motion_precheck_requested`
- `simulation_motion_precheck_status`
- `ready_for_simulation_motion`
- `simulation_motion_precheck_path`
- `simulation_motion_precheck_report_path`
- `simulation_motion_precheck_report_generated`
- `simulation_motion_precheck.status`
- `simulation_motion_precheck.ready`
- `simulation_motion_precheck.metadata_only`
- `simulation_motion_precheck.simulation_only`
- `simulation_motion_precheck.control_enabled`
- `simulation_motion_precheck.motion_generated`
- `simulation_motion_precheck.command_generated`
- `simulation_motion_precheck.joint_targets_generated`
- `simulation_motion_precheck.trajectory_generated`
- `simulation_motion_precheck.tcp_pose_world_generated`
- `simulation_motion_precheck.robot_motion_executed`
- `simulation_motion_precheck.real_robot_allowed`
- `simulation_motion_precheck.checked_requirements`
- `simulation_motion_precheck.missing_requirements`
- `simulation_motion_precheck.blocking_reasons`
- `simulation_motion_precheck.warnings`
- `simulation_motion_precheck.errors`
- `simulation_motion_precheck.joint_precheck_table`

The precheck validates that the UR5e asset and prim are available, articulation
readiness is `READY`, articulation state is observable and `OK`, six standard
UR5e arm joints exist, joint positions/velocities/limits are available, and
each arm joint position is inside its current lower/upper limit. Non-arm extra
joints such as `robot_gripper_joint` and `root_joint` are recorded as
`non_arm_extra_joints` and do not block readiness.

Evidence export adds a `Simulation Motion Precheck Summary` section to
`summary.md`, a `simulation_motion_precheck` object in
`evidence_manifest.json`, `simulation_motion_precheck.json`, and
`simulation_motion_precheck_report.md` when precheck is requested. Screenshot
and video placeholders remain null.

Version route:

- `V2.1.x` = UR5e structure understanding
- `V2.2.0` = articulation readiness contract
- `V2.3.0` = articulation state observation contract
- `V2.4.0` = simulation-only motion precheck contract
- `V2.5.0` = first simulation robot micro-motion

V2.4.0 does not move the Isaac UR5e, generate joint targets, generate
trajectories, generate `tcp_pose_world`, call ROS2, call MoveIt, call RTDE,
generate URScript, connect a real UR5, or open any real or simulated robot
motion control chain.

## TETO V2.5.0 First Simulation Robot Micro-Motion

TETO V2.5.0 performs the first UR5e joint micro-motion in Isaac Sim, with a
strict simulation-only boundary. The default request is:

- `joint_name=wrist_3_joint`
- `requested_delta_rad=0.01`
- `micro_motion_tolerance_rad=0.005`

The micro-motion path always requires the V2.4.0 precheck gate. If
`--execute-simulation-micro-motion` is passed, TETO automatically enables the
robot asset check, robot prim inspection, articulation readiness check,
articulation state observation, and simulation motion precheck. Motion is only
attempted when:

- `simulation_motion_precheck_status=READY_FOR_SIMULATION_MOTION`
- `ready_for_simulation_motion=true`
- `articulation_readiness_status=READY`
- `articulation_state_status=OK`

This stage is simulation-only. No real robot command is generated. No ROS2,
MoveIt, RTDE, URScript, Dashboard, real UR5 backend, trajectory planner, or
`tcp_pose_world` control chain is used. The only allowed execution mechanism is
the local Isaac Sim simulation API.

Run dry-run evidence without claiming real Isaac motion:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --dry-run \
  --steps 3 \
  --execute-simulation-micro-motion \
  --micro-motion-joint wrist_3_joint \
  --micro-motion-delta-rad 0.01
```

Run true Isaac simulation micro-motion:

```bash
PYTHONPATH=. /home/newusername/Storage/home/wu-zijian/下载/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh scripts/harnesses/run_shadow_simulation_contract.py \
  --steps 1 \
  --check-robot-asset \
  --inspect-robot-prim \
  --check-articulation-readiness \
  --observe-articulation-state \
  --check-simulation-motion-precheck \
  --execute-simulation-micro-motion \
  --micro-motion-joint wrist_3_joint \
  --micro-motion-delta-rad 0.01
```

When requested, evidence export adds `simulation_motion_result.json`,
`simulation_motion_report.md`, `before_articulation_state.json`, and
`after_articulation_state.json`. The report states that the motion is
simulation-only, no real robot command was generated, no ROS2 / MoveIt / RTDE /
URScript / real UR5 control chain was used, and execution happened only through
the local Isaac Sim simulation API.

## TETO V2.5.1 Motion Evidence Polish

TETO V2.5.1 does not add any new robot motion capability and does not change
the V2.5.0 simulation-only safety boundary. It polishes the micro-motion
evidence so dry-run, blocked-by-precheck, and true Isaac runs are easier to
audit.

The evidence export now highlights:

- before/after joint position diff
- `requested_delta_rad`, `actual_delta_rad`, `tolerance_rad`
- `delta_within_tolerance`
- motion evidence file paths in `summary.md`
- `motion_evidence_available`, `motion_evidence_files`, and
  `motion_diff_summary` in `evidence_manifest.json`

The generated `simulation_motion_report.md` is titled
`TETO V2.5.1 Simulation Micro-Motion Evidence Report` and includes Status,
Precheck Summary, Joint Diff Summary, Evidence Files, and Safety Boundary
sections.

V2.5.1 remains simulation-only. No real robot command is generated. No ROS2 /
MoveIt / RTDE / URScript / Dashboard / real UR5 control chain, trajectory
planner, or `tcp_pose_world` control path is used.

## TETO V2.6.0 Semantic-to-Simulation Motion Bridge

TETO V2.6.0 adds a semantic-to-simulation-motion bridge. It consumes an
existing semantic task contract JSON, runs a semantic eligibility gate, then
connects only eligible contracts to the already-validated V2.4.0 simulation
motion precheck and V2.5.x simulation-only micro-motion proof pulse.

V2.6.0 does not make the robot execute a semantic target pose. It does not use
live camera capture, live VLM/Qwen inference, ROS2, MoveIt, RTDE, URScript,
Dashboard, a real UR5, a trajectory planner, or `tcp_pose_world` execution.
If a semantic contract contains fields such as `world_point`, `pose_candidates`,
or `tcp_pose_world`, those fields are copied and audited as non-executable
contract evidence only.

The bridge flow is:

```text
semantic task contract
-> semantic bridge eligibility check
-> simulation motion precheck
-> Isaac simulation-only micro-motion
-> semantic bridge evidence + motion evidence
```

The bridge proof pulse is fixed to a local Isaac simulation API request such as
`joint_name=wrist_3_joint`, `requested_delta_rad=0.01`, and
`command_type=ISAAC_SIMULATION_API_LOCAL_ONLY`. It proves that a semantic task
can safely enter the simulation motion gate; it does not prove semantic task
completion.

Run the dry-run demo bridge:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --dry-run \
  --steps 3 \
  --semantic-simulation-bridge \
  --semantic-bridge-demo-contract
```

Run with a fixture semantic contract:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --dry-run \
  --steps 3 \
  --semantic-simulation-bridge \
  --semantic-task-json tests/fixtures/semantic_contracts/eligible_hover_to_object.json
```

Bridge evidence includes `semantic_simulation_bridge_result.json`,
`semantic_simulation_bridge_report.md`, and `semantic_task_contract_copy.json`.
`evidence_manifest.json` records `semantic_bridge_status`,
`semantic_gate_passed`, `semantic_task_id`, `semantic_intent`,
`semantic_target_label`, `semantic_bridge_files`, and whether the bridge
triggered the simulation-only micro-motion proof pulse. `summary.md` includes a
Semantic-to-Simulation Bridge Summary with the bridge status and resulting
micro-motion status.

## TETO V2.7.0 Safe Simulated Task Execution Loop

TETO V2.7.0 adds a safe simulated task execution lifecycle on top of the
V2.6.0 semantic-to-simulation bridge. It still does not prove that the robot
completed a real semantic goal such as moving above a red mug. It proves that
an eligible semantic contract can enter a safe simulation-only execution
attempt and produce structured feedback, while blocked contracts produce
structured failure evidence.

The V2.7.0 lifecycle is:

```text
semantic task contract
-> semantic gate
-> simulation motion precheck
-> simulation-only micro-motion
-> post-motion state observation
-> execution feedback
-> simulated task status
-> failure report / retry recommendation / fallback recommendation
-> replay-ready evidence
```

V2.7.0 remains simulation-only. It does not call live camera capture, live
Qwen/VLM inference, ROS2, MoveIt, RTDE, URScript, Dashboard commands, a real
UR5, a trajectory planner, or `tcp_pose_world` execution. Retry and fallback
are recommendations only; V2.7.0 does not execute automatic repeated motion.

Run the safe execution dry-run demo:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --dry-run \
  --steps 3 \
  --semantic-simulation-bridge \
  --semantic-bridge-demo-contract \
  --safe-simulated-task-execution \
  --execution-enable-retry-recommendation \
  --execution-enable-fallback-recommendation
```

Safe execution evidence includes `simulated_task_execution_result.json`,
`simulated_task_execution_report.md`, `execution_feedback.json`,
`execution_attempt_record.json`, `failure_analysis.json`, and
`retry_fallback_recommendation.json`. `summary.md` includes a Safe Simulated
Task Execution Summary, and `evidence_manifest.json` records
`simulated_task_execution_status`, `execution_feedback_status`,
`failure_reason`, retry/fallback recommendation fields, post-motion state check
status, and replay readiness.

## TETO V2.7.1 Execution Evidence Polish

TETO V2.7.1 does not add new robot motion capability and does not change the
V2.7.0 safe simulated task execution loop. It polishes the execution evidence
bundle so that each run is easier to inspect, replay, and audit.

V2.7.1 improves:

- `simulated_task_execution_report.md` with lifecycle, gate decision, motion
  verification, failure/retry/fallback, replay readiness, and safety boundary
  tables.
- `summary.md` with a human-readable execution conclusion and a dedicated
  execution evidence file list.
- `evidence_manifest.json` with `execution_evidence_available`,
  `execution_evidence_files`, `replay_ready`, `replay_bundle_files`,
  `latest_execution_summary`, `safety_boundary_confirmed`, and
  `no_automatic_retry_executed`.
- `failure_analysis.json` with `failure_category`, `blocking_stage`,
  `human_readable_message`, recommendation flags, fallback type, and
  `next_safe_action`.
- `retry_fallback_recommendation.json` with `recommendation_reason`,
  `automatic_retry_executed=false`, and `next_safe_action`.

V2.7.1 remains simulation-only. It does not call a live camera or live VLM, does
not generate real robot commands, and does not use a ROS2, MoveIt, RTDE,
URScript, Dashboard, real UR5, trajectory, or `tcp_pose_world` control chain.
Retry and fallback remain recommendations only; no automatic retry motion is
executed.

## TETO V2.8.0 Lab Backend / Camera Readiness No-Motion Preparation

TETO V2.8.0 prepares a lab backend, camera, live VLM, and shadow-mode readiness
contract for future lab UR5 computer integration. It is not a real robot
execution release. No real robot command is generated, no trajectory is
generated, no `tcp_pose_world` execution is performed, and no ROS2, MoveIt,
RTDE, URScript, Dashboard, real UR5, live camera capture, or live VLM call is
made.

Default V2.8.0 safety flags remain disabled:

- `allow_live_camera=false`
- `allow_live_vlm=false`
- `allow_real_robot_backend=false`
- `allow_robot_motion=false`

Readiness checks are config-only and shadow-mode oriented. Example safe configs
are provided in:

- `configs/lab_backend.example.yaml`
- `configs/camera.example.yaml`
- `configs/live_vlm.example.yaml`

Local lab configs such as `local.lab_backend.yaml`, `local.camera.yaml`, and
`local.live_vlm.yaml` must not be committed because they may contain real UR5
IP addresses, camera serial numbers, tokens, or local paths.

Run a no-motion readiness evidence check:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-lab-readiness \
  --check-camera-readiness \
  --check-live-vlm-readiness \
  --check-shadow-mode-readiness \
  --lab-readiness-config configs/lab_backend.example.yaml \
  --output-dir /tmp/teto_v280_readiness
```

The evidence bundle includes `lab_readiness_result.json`,
`lab_readiness_report.md`, `camera_readiness_result.json`,
`live_vlm_readiness_result.json`, `shadow_mode_readiness_result.json`,
`summary.md`, and `evidence_manifest.json`. V3.0 is the first planned boundary
where a carefully gated first real UR5 small motion may be considered.

## TETO V2.8.1 Readiness Evidence Polish

TETO V2.8.1 does not add live camera, live VLM, or real UR5 execution
capability. It only polishes the no-motion readiness evidence generated by the
V2.8.0 lab backend / camera / live VLM / shadow-mode contracts.

V2.8.1 improves:

- `lab_readiness_report.md` with version metadata, Overall Status, readiness
  contract tables, no-motion safety boundary, blocking reasons, warnings, and
  next safe action.
- `summary.md` with a Readiness Evidence Summary and PASS / BLOCKED /
  NOT_READY style readiness statuses.
- `evidence_manifest.json` with `readiness_evidence_available`,
  `readiness_evidence_files`, `no_motion_readiness_passed`,
  `readiness_statuses`, `blocking_reasons`, `safety_flags`,
  `live_camera_used=false`, `live_vlm_called=false`,
  `real_robot_motion_executed=false`, and
  `real_robot_command_enabled=false`.

V2.8.1 remains config-only and shadow-mode/no-motion preparation. It does not
capture from a live camera, does not call live Qwen or any live VLM, does not
connect to ROS2, MoveIt, RTDE, URScript, Dashboard, a trajectory planner,
`tcp_pose_world`, a real robot backend, or automatic retry motion.

## TETO V2.8.2 Camera Snapshot Contract

TETO V2.8.2 adds a camera snapshot contract so future real-scene or offline
camera frames can enter TETO as safe, verifiable, rejectable, and replayable
manifest evidence. It is real-scene pipeline preparation before live capture or
robot execution.

V2.8.2 is not live camera capture, not live VLM/Qwen inference, and not real
UR5 execution. It does not connect to ROS2, MoveIt, RTDE, URScript, Dashboard,
a trajectory planner, `tcp_pose_world`, a real robot backend, joint targets, or
automatic retry motion.

The snapshot contract validates declared fields only:

- `snapshot_id`, `scene_version`, `capture_timestamp`, and `ttl_ms`
- `source`, `frame_id`, `image_ref`, optional `depth_ref`, camera info,
  metadata, and extrinsics references
- width, height, encodings, camera frame, alignment, sync, and availability
  flags
- no-motion flags such as `live_capture_used=false`,
  `live_camera_enabled=false`, `live_vlm_called=false`,
  `real_robot_motion_executed=false`, and
  `real_robot_command_enabled=false`

Example manifest:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-camera-snapshot \
  --camera-snapshot-config configs/camera_snapshot.example.yaml \
  --camera-snapshot-report \
  --output-dir /tmp/teto_v282_camera_snapshot
```

The evidence bundle includes `camera_snapshot_result.json`,
`camera_snapshot_report.md`, `summary.md`, and `evidence_manifest.json`.
`camera_snapshot_report.md` states the no-motion safety boundary, and
`evidence_manifest.json` records `camera_snapshot_evidence_available`,
`camera_snapshot_id`, `scene_version`, `camera_snapshot_validity_status`,
`camera_snapshot_blocking_reasons`, `camera_snapshot_warnings`,
`no_motion_snapshot_passed`, and the live/real-robot safety flags.

## TETO V2.9.0 Real-Scene No-Motion Shadow Pipeline

TETO V2.9.0 adds a real-scene no-motion shadow pipeline. It joins a validated
offline/manual camera snapshot contract with an offline/mock grounding result
JSON, then exports replayable evidence about whether the semantic gate would
accept the scene.

V2.9.0 is real-scene pipeline preparation only. It is not live camera capture,
not live VLM/Qwen inference, and not real UR5 execution. It does not connect to
ROS2, MoveIt, RTDE, URScript, Dashboard, a trajectory planner,
`tcp_pose_world`, a real robot backend, joint targets, robot commands,
automatic retry motion, or any real execution request.

The shadow pipeline validates:

- camera snapshot contract status, `snapshot_id`, and `scene_version`
- offline/mock grounding result `grounding_id`, target label, object id,
  bounding box, pixel center, and confidence values
- snapshot / grounding ID match and scene version match
- rejection states such as no target, invalid box, invalid pixel center, low
  confidence, live VLM/camera flags, and forbidden robot control fields
- safety flags such as `live_camera_used=false`, `live_vlm_called=false`,
  `real_robot_motion_executed=false`, `real_robot_command_enabled=false`,
  `robot_command_generated=false`, `trajectory_generated=false`,
  `joint_targets_generated=false`, and `tcp_pose_world_generated=false`

Positive shadow smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --run-real-scene-shadow \
  --real-scene-shadow-config configs/real_scene_shadow.example.yaml \
  --real-scene-shadow-report \
  --output-dir /tmp/teto_v290_real_scene_shadow_positive
```

No-target shadow smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --run-real-scene-shadow \
  --real-scene-shadow-config configs/real_scene_shadow.example.yaml \
  --grounding-result examples/grounding_result_no_target_example.json \
  --real-scene-shadow-report \
  --output-dir /tmp/teto_v290_real_scene_shadow_no_target
```

The evidence bundle includes `real_scene_shadow_result.json`,
`real_scene_shadow_report.md`, `summary.md`, and `evidence_manifest.json`.
`real_scene_shadow_report.md` states the no-motion safety boundary, and
`evidence_manifest.json` records `real_scene_shadow_evidence_available`,
`snapshot_id`, `grounding_id`, `shadow_pipeline_status`,
`semantic_gate_passed`, `no_motion_shadow_passed`, blocking reasons, warnings,
replay readiness, and live/real-robot safety flags.

## TETO V2.9.1 Geometry Validity Contract

TETO V2.9.1 adds a geometry validity contract. It checks whether an offline/mock
VLM grounding result is geometrically usable before a future 2D-to-3D projector
handoff.

The contract validates declared evidence only:

- snapshot / grounding `snapshot_id` and `scene_version` match
- image width and height are present and valid
- `bbox_xyxy` format, image bounds, and nonzero/minimum area
- `pixel_center` format and image bounds
- camera frame and frame ID availability
- depth availability when `depth_required=true`
- confidence threshold and TTL freshness
- grounded/rejected state, live camera/VLM flags, and forbidden robot control
  fields

V2.9.1 is not live camera capture, not live VLM/Qwen inference, not a ROS2
bridge, not MoveIt planning, and not real UR5 execution. It does not generate
trajectory, `tcp_pose_world`, URScript, joint targets, robot commands,
automatic retry motion, or any real execution request.

Positive geometry validity smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-geometry-validity \
  --geometry-validity-config configs/geometry_validity.example.yaml \
  --geometry-validity-report \
  --output-dir /tmp/teto_v291_geometry_validity_positive
```

Invalid bbox geometry smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-geometry-validity \
  --geometry-validity-config configs/geometry_validity.example.yaml \
  --grounding-result examples/grounding_result_invalid_bbox_example.json \
  --geometry-validity-report \
  --output-dir /tmp/teto_v291_geometry_validity_invalid_bbox
```

The evidence bundle includes `geometry_validity_result.json`,
`geometry_validity_report.md`, `summary.md`, and `evidence_manifest.json`.
`geometry_validity_report.md` states the no-motion / no-live-camera /
no-live-VLM / no-real-robot safety boundary, and `evidence_manifest.json`
records `geometry_validity_evidence_available`, `geometry_validity_status`,
`snapshot_id`, `grounding_id`, `bbox_valid`, `pixel_center_valid`,
depth/confidence/TTL checks, blocking reasons, warnings, `next_safe_action`,
and the no-motion safety flags.

## TETO V2.9.2 2D-to-3D Projector Shadow Contract

TETO V2.9.2 adds a 2D-to-3D projector shadow contract. It converts a geometry
valid `pixel_center` plus depth, camera intrinsics, and a mock/config transform
into `camera_point_m` and `world_point_m` evidence.

This is the geometric prerequisite for the future V3.0 text-camera-UR5 demo.
It is shadow projection evidence only.

The projector shadow contract validates declared evidence only:

- `geometry_validity_status=PASS`
- `pixel_center`
- camera intrinsics `fx`, `fy`, `cx`, and `cy`
- `depth_value_m` and configured min/max depth range
- camera frame and world frame
- mock/config transform availability with `tf_source=mock_or_config`
- workspace bounds for the projected point
- live camera/VLM flags and forbidden robot control fields

V2.9.2 is not live camera capture, not live VLM/Qwen inference, not ROS2 tf2,
not MoveIt planning, and not real UR5 execution. It does not generate
trajectory, `tcp_pose_world`, URScript, joint targets, robot commands,
automatic retry motion, or any real execution request.

Positive projector shadow smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-projector-shadow \
  --projector-shadow-config configs/projector_shadow.example.yaml \
  --projector-shadow-report \
  --output-dir /tmp/teto_v292_projector_shadow_positive
```

Invalid depth projector shadow smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-projector-shadow \
  --projector-shadow-config examples/projector_invalid_depth_example.json \
  --projector-shadow-report \
  --output-dir /tmp/teto_v292_projector_shadow_invalid_depth
```

The evidence bundle includes `projector_shadow_result.json`,
`projector_shadow_report.md`, `summary.md`, and `evidence_manifest.json`.
`projector_shadow_report.md` states the no-motion / no-live-camera /
no-live-VLM / no-real-robot / no-ROS2-TF safety boundary, and
`evidence_manifest.json` records `projector_shadow_evidence_available`,
`projector_requested`, `projector_status`, `pixel_center`, `depth_value_m`,
`camera_point_m`, `world_point_m`, `projection_method`, `tf_available`,
`tf_source`, `real_tf_used=false`, `ros2_tf_used=false`, workspace checks,
blocking reasons, warnings, `next_safe_action`, and the no-motion safety flags.

## TETO V2.9.3 Camera Source Adapter / Manual Snapshot No-Motion

TETO V2.9.3 adds a camera source adapter. It converts offline file,
manual snapshot, live-disabled, and future explicitly allowed one-shot camera
source declarations into a TETO camera snapshot contract.

V2.9.3 is snapshot acquisition contract evidence only. It is not a continuous
live camera loop, not live VLM/Qwen inference, not a ROS2 bridge, not MoveIt
planning, and not real UR5 execution. It does not generate trajectory,
`tcp_pose_world`, URScript, joint targets, robot commands, automatic retry
motion, or any real execution request.

Optional live capture is disabled by default and must be explicitly enabled
with `--allow-live-camera-capture`. Even then, V2.9.3 only allows a bounded
one-shot snapshot evidence path and does not send frames to a VLM or robot
control path.

Supported source modes:

- `realsense_replay` (formal replay input)
- `offline_file` (legacy/debug contract fixture)
- `manual_snapshot` (legacy/debug contract fixture)
- `live_disabled`
- `optional_realsense_one_shot`

Offline camera source smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-camera-source-adapter \
  --camera-source-config configs/camera_source_offline.example.yaml \
  --camera-source-report \
  --output-dir /tmp/teto_v293_camera_source_offline
```

Manual snapshot smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-camera-source-adapter \
  --camera-source-config configs/camera_source_manual.example.yaml \
  --camera-source-report \
  --output-dir /tmp/teto_v293_camera_source_manual
```

Live capture remains blocked unless explicitly allowed:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-camera-source-adapter \
  --camera-source-config configs/camera_source_offline.example.yaml \
  --camera-source-mode optional_realsense_one_shot \
  --camera-source-report \
  --output-dir /tmp/teto_v293_camera_source_live_blocked
```

The evidence bundle includes `camera_source_result.json`,
`camera_source_report.md`, `summary.md`, and `evidence_manifest.json`.
`camera_source_report.md` states the no-motion / no-live-VLM /
no-real-robot / no-ROS2 / no-MoveIt safety boundary, and
`evidence_manifest.json` records `camera_source_evidence_available`,
`camera_source_status`, `source_mode`, snapshot IDs and refs,
`one_shot_capture_used`, `continuous_capture_used=false`,
`live_camera_capture_allowed`, `live_camera_capture_used`, blocking reasons,
warnings, `next_safe_action`, and no-motion safety flags.

## TETO V2.9.4 Local/Mock VLM Grounding Adapter No-Motion

TETO V2.9.4 adds a local/mock VLM grounding adapter. It converts a text
command plus a declared camera snapshot into a TETO grounding result contract.

The current implementation is mock/offline/manual by default. It does not call
live Qwen or any live VLM. `local_vlm_disabled` and
`future_local_qwen_adapter` are declaration-only modes for documenting the
boundary without invoking a model.

V2.9.4 is not live camera capture, not real VLM execution, not a ROS2 bridge,
not MoveIt planning, and not real UR5 execution. It does not generate
trajectory, `tcp_pose_world`, URScript, joint targets, robot commands,
automatic retry motion, or any real execution request.

Supported adapter modes:

- `offline_grounding_json`
- `mock_vlm`
- `manual_annotation`
- `local_vlm_disabled`
- `future_local_qwen_adapter`

Mock VLM grounding smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-vlm-grounding-adapter \
  --vlm-grounding-config configs/vlm_grounding_mock.example.yaml \
  --vlm-grounding-report \
  --user-command "hover over the red mug" \
  --output-dir /tmp/teto_v294_vlm_grounding_mock
```

Offline grounding JSON smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-vlm-grounding-adapter \
  --vlm-grounding-config configs/vlm_grounding_offline.example.yaml \
  --vlm-grounding-report \
  --output-dir /tmp/teto_v294_vlm_grounding_offline
```

The evidence bundle includes `vlm_grounding_result.json`,
`vlm_grounding_report.md`, `summary.md`, and `evidence_manifest.json`.
`vlm_grounding_report.md` states the no-motion / no-live-VLM /
no-real-robot / no-ROS2 / no-MoveIt safety boundary, and
`evidence_manifest.json` records `vlm_grounding_evidence_available`,
`vlm_grounding_status`, `grounding_id`, `snapshot_id`, `scene_version`,
`user_command`, `normalized_command`, `adapter_mode`, target fields, bbox,
pixel center, confidence fields, rejection fields, blocking reasons, warnings,
`next_safe_action`, and safety flags proving no live camera, live VLM, real
robot command, trajectory, joint targets, or `tcp_pose_world` were generated.

## TETO V2.9.5 Full Perception Shadow Pipeline

TETO V2.9.5 adds the full perception shadow pipeline. It composes a text
command, camera source adapter, camera snapshot validation, VLM grounding,
semantic gate, geometry validity, and 2D-to-3D projector shadow into one
no-motion evidence path.

The pipeline can output `world_point_m` evidence from declared `pixel_center`,
offline depth, camera intrinsics, and mock/config transform inputs. This is
perception-side shadow evidence only.

V2.9.5 is still not a ROS2 bridge, not MoveIt planning, and not real UR5
execution. It does not generate trajectory, `tcp_pose_world`, URScript, joint
targets, robot commands, automatic retry motion, or any real execution request.
It does not call live Qwen/VLM, does not open a continuous live camera loop,
and does not use a real robot backend.

Full perception positive smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --run-perception-shadow-pipeline \
  --perception-shadow-config configs/perception_shadow_mock_positive.example.yaml \
  --perception-shadow-report \
  --output-dir /tmp/teto_v295_perception_shadow_positive
```

The evidence bundle includes `perception_shadow_result.json`,
`perception_shadow_report.md`, `summary.md`, and `evidence_manifest.json`.
`perception_shadow_report.md` states the no-motion / no-live-camera /
no-live-VLM / no-real-robot / no-ROS2 / no-MoveIt safety boundary, and the
manifest records stage statuses, semantic gate, target fields, `bbox_xyxy`,
`pixel_center`, `depth_value_m`, `camera_point_m`, `world_point_m`, workspace
check, replay readiness, blocking reasons, warnings, `next_safe_action`, and
safety flags proving no robot command, trajectory, joint targets, or
`tcp_pose_world` were generated.

V2.9.5 is the final perception-side integration stage before the planned
V2.10.x ROS2 Planner Gateway shadow bridge.

## TETO V2.10.0 Planner Gateway Shadow Contract

TETO V2.10.0 adds the Planner Gateway Shadow Contract. It converts the V2.9.5
full perception shadow pipeline output, especially validated `world_point_m`,
into bounded Planner Gateway input evidence that a future gateway can consume.

The generated shadow request is planner input metadata, not an execution
command. It records fields such as `intent_name: hover_to_object`,
`target_world_point_m`, `hover_offset_m`, `bounded_target_point_m`,
`world_frame`, conservative speed and acceleration scales, `must_reconfirm:
true`, `execution_allowed: false`, and `mode: shadow_only`.

V2.10.0 is not ROS2 bridge live publish, not MoveIt planning, and not real UR5
execution. It does not publish ROS2 topics, services, or actions; does not call
MoveIt; does not connect to RTDE, URScript, Dashboard, or a real robot backend;
and does not generate trajectory, `tcp_pose_world`, URScript, joint targets, or
robot commands.

Planner Gateway shadow positive smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-planner-gateway-shadow \
  --planner-gateway-shadow-config configs/planner_gateway_shadow_positive.example.yaml \
  --planner-gateway-shadow-report \
  --perception-shadow-result examples/planner_gateway_shadow/perception_positive_result.json \
  --output-dir /tmp/teto_v2100_planner_gateway_shadow_positive
```

The evidence bundle includes `planner_gateway_shadow_result.json`,
`planner_gateway_shadow_report.md`, `summary.md`, and
`evidence_manifest.json`. The report states the no-ROS2-publish / no-MoveIt /
no-real-robot / no-trajectory safety boundary, and the manifest records the
gateway request id, task id, intent, perception identifiers, `world_point_m`,
`bounded_target_point_m`, workspace and confidence checks, manual confirmation
requirement, blocking reasons, warnings, replay readiness, and safety flags
proving no trajectory, `tcp_pose_world`, joint targets, robot commands, ROS2
publish, MoveIt call, or real robot motion occurred.

V2.10.0 is the first step of the V2.10.x ROS2/Planner Gateway shadow bridge
line. It remains shadow-only and evidence-only.

## TETO V2.10.1 ROS2 Environment / Interface Readiness Check

TETO V2.10.1 adds a ROS2 Environment / Interface Readiness Check for the
future ROS2 Planner Gateway shadow bridge. The check validates declared
configuration only: ROS2 environment declaration, ROS distro, ROS domain id,
Planner Gateway interface mode and endpoint, message schema, required frame
names, and the required shadow-only safety flags.

This prepares the future robot-system shadow bridge, but it does not publish
ROS2 messages, does not call MoveIt, and does not control a real UR5. It does
not generate trajectory, `tcp_pose_world`, joint targets, URScript, Dashboard
commands, RTDE commands, robot commands, automatic retry motion, or real robot
execution requests.

ROS2 does not need to be installed for this development-stage check. When
`allow_missing_ros2_runtime: true` is declared, an unavailable ROS2 runtime is
reported as a warning while the smoke remains no-motion and shadow-only.

ROS2 interface readiness smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-ros2-interface-readiness \
  --ros2-interface-config configs/ros2_interface.example.yaml \
  --ros2-interface-report \
  --output-dir /tmp/teto_v2101_ros2_interface_readiness_positive
```

The evidence bundle includes `ros2_interface_readiness_result.json`,
`ros2_interface_readiness_report.md`, `summary.md`, and
`evidence_manifest.json`. `summary.md` includes a `ROS2 Interface Readiness
Summary`, and `evidence_manifest.json` records readiness status, ROS2
declarations, Planner Gateway endpoint, message schema, frame names, blocking
reasons, warnings, and safety flags proving no ROS2 publish, MoveIt call,
trajectory, `tcp_pose_world`, joint target, robot command, or real robot motion
occurred.

## TETO V2.10.2 ROS2 Message Export / Fake Publish Dry-Run

TETO V2.10.2 adds a ROS2 Message Export / Fake Publish Dry-Run. It converts an
already validated Planner Gateway shadow input plus a READY_FOR_SHADOW_BRIDGE
ROS2 interface declaration into a deterministic ROS2-compatible
`PlannerRequest` JSON artifact.

The generated artifact is fake-publish evidence only. V2.10.2 does not publish
ROS2 messages, does not call `rclpy` publish, does not call MoveIt, does not
generate trajectory or `tcp_pose_world`, does not create joint targets or robot
commands, and does not control a real UR5.

ROS2 message export positive smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-planner-gateway-shadow \
  --planner-gateway-shadow-config configs/planner_gateway_shadow_positive.example.yaml \
  --planner-gateway-shadow-report \
  --perception-shadow-result examples/planner_gateway_shadow/perception_positive_result.json \
  --check-ros2-interface-readiness \
  --ros2-interface-config configs/ros2_interface.example.yaml \
  --ros2-interface-report \
  --check-ros2-message-export \
  --ros2-message-export-config configs/ros2_message_export.example.yaml \
  --ros2-message-export-report \
  --output-dir /tmp/teto_v2102_ros2_message_export_positive
```

The evidence bundle includes `ros2_message_export_result.json`,
`ros2_message_export_report.md`, `summary.md`, and `evidence_manifest.json`.
`summary.md` includes a `ROS2 Message Export / Fake Publish Summary`, and the
manifest records `message_id`, `message_schema`, fake-publish flags, Planner
Gateway interface endpoint, frame names, bounded target point, blocking
reasons, warnings, and safety flags proving no ROS2 publish, MoveIt call,
trajectory, `tcp_pose_world`, joint target, robot command, or real robot motion
occurred.

## TETO V2.11.0 Full Robot-System Shadow Bridge Dry-Run

TETO V2.11.0 adds a full robot-system shadow bridge. It merges the validated
Planner Gateway shadow input, ROS2 fake-publish `PlannerRequest`, MoveIt
plan-only declaration, and UR5 read-only state monitor declaration into one
deterministic no-motion rehearsal boundary.

The generated artifacts are shadow evidence only. V2.11.0 does not publish
ROS2 messages, does not call `rclpy` publish, does not execute MoveIt plans,
does not send controller trajectories, does not use UR driver writes, RTDE
writes, Dashboard commands, or URScript, does not generate executable
`tcp_pose_world` or joint targets, does not generate robot commands, and does
not move a real UR5.

Full robot-system shadow bridge positive smoke:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --check-planner-gateway-shadow \
  --planner-gateway-shadow-config configs/planner_gateway_shadow_positive.example.yaml \
  --planner-gateway-shadow-report \
  --perception-shadow-result examples/planner_gateway_shadow/perception_positive_result.json \
  --check-ros2-interface-readiness \
  --ros2-interface-config configs/ros2_interface.example.yaml \
  --ros2-interface-report \
  --check-ros2-message-export \
  --ros2-message-export-config configs/ros2_message_export.example.yaml \
  --ros2-message-export-report \
  --check-moveit-plan-only \
  --moveit-plan-only-config configs/moveit_plan_only.example.yaml \
  --moveit-plan-only-report \
  --check-ur5-read-only-state \
  --ur5-read-only-state-config configs/ur5_read_only_state.example.yaml \
  --ur5-read-only-state-report \
  --check-robot-system-shadow-bridge \
  --robot-system-shadow-bridge-config configs/robot_system_shadow_bridge.example.yaml \
  --robot-system-shadow-bridge-report \
  --output-dir /tmp/teto_v2110_full_robot_system_shadow_bridge
```

The evidence bundle includes `moveit_plan_only_result.json`,
`moveit_plan_only_report.md`, `ur5_read_only_state_result.json`,
`ur5_read_only_state_report.md`, `robot_system_shadow_bridge_result.json`,
`robot_system_shadow_bridge_report.md`, `summary.md`, and
`evidence_manifest.json`. `summary.md` includes `MoveIt Plan-Only Contract
Summary`, `UR5 Read-Only State Contract Summary`, and `Full Robot-System Shadow
Bridge Summary`. The manifest records the bridge readiness flags and safety
flags proving `execution_allowed=false`, `moveit_execute_called=false`,
`trajectory_sent=false`, `controller_command_sent=false`,
`ros2_publish_attempted=false`, `rtde_write_attempted=false`,
`dashboard_command_attempted=false`, `urscript_generated=false`,
`robot_command_generated=false`, `tcp_pose_world_generated=false`,
`joint_targets_generated=false`, `real_robot_motion_executed=false`, and
`automatic_retry_motion=false`.

## TETO Isaac Sim GUI Operator

The Isaac operator connects natural-language Qwen parsing to TETO validation,
bounded vector decomposition, a simulated measured gateway, and visible UR5e
motion in Isaac Sim. The formal demo is GUI-first:

```bash
bash scripts/start_teto_isaac_gui_operator.sh --gui --console
```

The console displays `TETO/Isaac>` and accepts commands such as:

```text
move forward 30 cm and left 10 cm
```

Each accepted vector is decomposed into bounded substeps. Every substep is
checked against the configured workspace and verified from Isaac Sim
forward-kinematics measurements before the next substep may run. JSON and
Markdown evidence are saved under `outputs/isaac_sim_operator_runs/`.

Use `--headless` only for software smoke tests. Formal evidence keeps
`execution_mode: isaac_sim` and records whether the runtime was `gui` or
`headless_smoke_test`. The operator always records
`real_robot_motion_executed: false`, `real_ur_connection_used: false`,
`dashboard_used: false`, `rtde_write_used: false`, and
`isaac_gui_required: true`. A real Isaac run records
`gateway_type: simulated_measured_gateway`.

Safety is fail-closed: `--real` is rejected, real UR controller addresses are
rejected, and this path does not use Dashboard, RTDE write, URScript, a real
MoveIt `ExecuteTrajectory`, or any physical UR5e connection. Workstation-only
Isaac and UR5e asset paths belong in the ignored
`configs/isaac_sim_operator.lab.local.yaml`.

## Legacy/debug RGB-only tools

The generic converter, arbitrary local-image demo, and RGB folder batch code
remain in the repository temporarily for compatibility and semantic debugging.
They are hidden from the formal launcher and unified CLI. They do not create
formal snapshots and are not recommended for RealSense-native training or
runtime.

The reusable `prepare_image_for_vlm` preprocessing remains available for
resizing and RGB normalization after an RGB frame has been selected from an
accepted RealSense snapshot. It must not be treated as a snapshot builder.

The formal launcher menu is:

```text
1. Validate RealSense snapshot replay
2. Check environment
3. Run first simulation execution
4. Quit
```

Old workspaces may still contain `outputs/image_conversion/`,
`outputs/batches/`, or RGB-only recognition runs. Those artifacts are legacy
evidence and are not RealSense Snapshot Replay datasets.

## Batch Recognition Results

All recognition and VLM demo outputs live under `outputs/results/`.

Single image recognition creates an independent result folder. Inside the
launcher, you can keep asking free-form follow-up questions about the same
image until you type `back`, `q`, `quit`, or `exit`:

```text
outputs/results/single_recognition/single_YYYYMMDD_HHMMSS/
├── result.json
└── summary.json
```

`result.json` stores the questions and answers for that image session.
When `prompt_type` is `robot_task_json`, each saved item also includes
`raw_response`, `parsed_json`, `normalized_json`, `parse_status`,
`validation_status`, `validation_errors`, and `validation_warnings`. If parsing
fails, `parse_status` is `failed`, `normalized_json.error.code` is `E_PARSE`,
and the original model text is available in `raw_response`. If parsing succeeds
but required fields, controlled vocabulary, or safety rules fail validation,
`parse_status` remains `success` and `validation_status` becomes `failed`.
`summary.json` records the total number of questions, successes, failures,
backend, and output folder. The terminal still prints each formatted
recognition result. New single recognition runs no longer append to
`outputs/results/demo_results.jsonl`; that file may exist only as a legacy
artifact.

Each regular batch image recognition experiment creates an independent run
folder under `outputs/results/batch_recognition/`:

```text
outputs/results/batch_recognition/run_YYYYMMDD_HHMMSS/
├── results.jsonl
├── summary.json
└── errors.log
```

`results.jsonl` contains one JSON object per image with the image path, prompt
type, prompt text, backend, model response, status, and error message. When
`prompt_type` is `robot_task_json`, each line also stores `raw_response`,
`parsed_json`, `normalized_json`, `parse_status`, `validation_status`,
`validation_errors`, and `validation_warnings`; inspect `raw_response` when
`parse_status` is `failed`.
`summary.json` records the run name, creation time, input folder, output folder,
prompt type, backend, total image count, success count, and failure count.
`errors.log` contains only failed images and their error reasons, so successful
runs leave it empty.

When `prompt_type` is `robot_task_json`, batch runs use a dedicated output
branch:

```text
outputs/results/robot_task_json/run_YYYYMMDD_HHMMSS/
├── input_manifest.json
├── results.jsonl
├── summary.json
├── errors.log
├── smoke_report.md
├── smoke_report.json
├── scene_index.json
└── replay_index.json
```

`input_manifest.json` records run metadata and image paths only. It does not
store image bytes.

TETO also appends one summary line per batch recognition run to:

```text
outputs/results/index.jsonl
```

Use `index.jsonl` as the recognition experiment history table. Each line tells
you the `type`, `run_name`, `created_at`, `input_dir` or `image_path`,
`output_dir`, `prompt_type`, `backend`, `total`, `success`, and `failed` values
for one recognition run. To inspect an older experiment, find its line in
`index.jsonl`, then open that run folder's result files.

Older workspaces may still contain `outputs/recognition_runs/`. New batch
recognition runs no longer write there.

## TETO V3.0.0: First Real UR5 Hover Demo

TETO V3.0.0 is a historical real text-to-camera-to-VLM-to-TETO-to-UR5
execution milestone. It is not the current default real console path. The
initial task was intentionally narrow: identify one object from a limited
command such as:

```text
hover over the red mug
```

The V3 path normalizes the command, takes a one-shot camera snapshot, grounds
the target with a VLM adapter, runs TETO semantic and geometry gates, projects
2D evidence to a bounded 3D target, prepares a planner request, checks ROS2 /
MoveIt / UR5 state evidence, uses its legacy acceptance gate, and only then
allows a low-speed hover above the object.

Run the CI-safe software/no-robot smoke path with:

```bash
python3 scripts/harnesses/run_shadow_simulation_contract.py \
  --run-v3-hover-demo \
  --v3-user-command "hover over the red mug" \
  --v3-hover-config configs/v3_hover_demo.example.yaml \
  --v3-hover-report
```

Safety boundaries:

- no arbitrary natural-language execution
- no grasping, pushing, pressing, or other contact tasks
- no raw URScript
- no RTDE write control
- no Dashboard motion command
- no raw joint target generation
- no LLM-generated `tcp_pose_world` execution
- legacy acceptance gate for this historical hover path
- low-speed MoveIt execution only
- bounded workspace checks
- evidence export required

Without the lab robot, V3.0.0 software mode validates the pipeline but cannot
claim physical demo success. The project may claim “TETO V3.0.0 First Real UR5
Hover Demo accepted” only after the lab real-hover mode exports evidence with
`real_robot_motion_executed=true`.

Use the unified CLI:

```bash
python3 -m src.cli check-env
python3 -m src.cli snapshot-replay \
  --snapshot-manifest configs/camera_snapshot.example.yaml
```

## Notes

- Real VLM inference logic belongs in `src/vlm_infer.py`.
- Snapshot RGB preprocessing logic belongs in `src/image_utils.py`.
- Prompt templates belong in `src/prompt_utils.py`.
- Local Qwen2.5-VL integration lives in `src/vlm_infer.py`.
- Future Isaac Sim / ROS2 / UR5 integration should mainly modify
  `src/robot_interface.py`.

## Extension Plan

- Connect real local Qwen2.5-VL inference
- Add RealSense D455 snapshot datasets
- Add annotation formats
- Add spatial relationship understanding tasks
- Add simple robot action plan JSON output
- V2.1.x = UR5e structure understanding
- V2.2.0 = articulation readiness contract
- V2.3.0 = articulation state observation contract
- V2.4.0 = simulation-only motion precheck contract
- V2.5.0 = first simulation robot micro-motion
- V2.5.1 = motion evidence polish
- V2.6.0 = semantic-to-simulation motion bridge
- V2.7.0 = safe simulated task execution loop
- V2.7.1 = execution evidence polish
- V2.8.0 = lab backend / camera / VLM no-motion readiness
- V2.8.1 = readiness evidence polish
- V2.8.2 = camera snapshot contract
- V2.9.0 = real-scene no-motion shadow pipeline
- V2.9.1 = geometry validity contract
- V2.9.2 = 2D-to-3D projector shadow contract
- V2.9.3 = camera source adapter / manual snapshot no-motion
- V2.9.4 = local/mock VLM grounding adapter no-motion
- V2.9.5 = full perception shadow pipeline
- V2.10.0 = planner gateway shadow contract
- V2.10.1 = ROS2 environment / interface readiness check
- V2.10.2 = ROS2 message export / fake publish dry-run
- V2.11.0 = full robot-system shadow bridge with MoveIt plan-only and UR5 read-only state declarations
- V3.0.0 = historical first real UR5 hover demo architecture; current default real console uses unified segmented measured gates
- B1/B2 migration = unified segmented real backend and real full-stack startup docs; default real console is autonomous segmented execution with measured per-segment gates, while legacy manual mode remains explicit
- RTDE write / Dashboard command / URScript / raw joint target execution remains outside the implemented safety boundary
