# TETO Project State

## Current Baseline

- Current HEAD: `b6cad38`
- Branch: `master`
- Last known full offline pytest: `843 passed`
- Working tree after migration: clean
- Migration status: GREEN

## Current Architecture

- Unified segmented operator core: `src/unified_segmented_operator.py`
- Isaac SIM_ONLY backend: `src/isaac_sim_operator.py`
- Real segmented backend: `src/real_segmented_operator_backend.py`
- Real full-stack startup docs/scripts: `docs/real_full_stack_startup.md` and `scripts/start_teto_real_full_stack.sh`
- Formal RealSense snapshot/replay direction: `scripts/build_realsense_snapshot_bundle.py` and the camera snapshot contracts
- Legacy chat / old image config removed: `src/teto_chat.py`, `tests/test_teto_chat.py`, and `config/default.yaml`

## Current Real Mode

Default real mode is autonomous segmented execution with measured per-segment gates.

Old per-command manual `y` confirmation is no longer the default real console.
Legacy manual mode still exists only as a legacy path via `--legacy-manual` /
`--legacy-manual-console`.

Real per-segment gates:

- Dashboard state
- controller active
- joint state available
- MoveIt availability
- fresh measured TCP pose
- D455 snapshot freshness / sync / newer-than-previous guard
- bounded relative motion contract
- cartesian safety gateway
- post-segment measured verification

## Isaac Status

- Isaac remains SIM_ONLY.
- Isaac evidence must not count as REAL_PATH success evidence.
- Isaac must not touch Dashboard / RTDE / MoveIt ExecuteTrajectory / UR driver.
- `SyntheticFakeGateway` is allowed only for tests/headless/smoke contexts, not real evidence.

## Motion Envelope

- `0.50 m` is the shared bounded relative motion envelope.
- `0.51 m` must BLOCK.
- Long motion must be decomposed into bounded subgoals.
- IK failure must fail closed, not fake success.

## Entrypoints

Recommended real entry:

- `bash scripts/start_teto_real_full_stack.sh`

Real console path:

- `scripts/qwen_operator_console.sh`
- `scripts/teto_operator_console.py --backend real`

Real bringup helper:

- `scripts/start_teto_qwen_real_operator.sh`

Legacy manual:

- `--legacy-manual`
- `--legacy-manual-console`

Recommended Isaac entry:

- `bash scripts/start_teto_isaac_gui_operator.sh --gui --console`

Isaac console:

- `scripts/teto_isaac_operator_console.py`

RealSense formal replay / bundle:

- `scripts/build_realsense_snapshot_bundle.py`

Old / not recommended:

- removed Just Chat path
- removed `src/teto_chat.py`
- old RGB-only paths are legacy/debug only, not formal real execution

## Safety Invariants

- Never count dry-run / plan-only / Isaac / fake / synthetic evidence as REAL_PATH success.
- Never bypass measured gates for real MoveIt execution.
- Never auto-unlock protective stop via LLM.
- Never start hardware from tests.
- Never mix SIM_ONLY changes into REAL_PATH without explicit audit.
- Always preserve fail-closed behavior.

## Next Recommended Research Phase

The next phase is not more cleanup. It is research and validation for:

- Camera-to-base TF calibration
- D455 formal snapshot/replay
- Qwen grounding to 3D target
- red mug hover / pre-hover pipeline
- TETO contract + validation + replay + re-observation
- real visual-motion closed loop only after geometry is validated
