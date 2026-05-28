# TETO V1 VLM Pipeline

TETO V1 VLM Pipeline is a lightweight Python project skeleton for learning,
showing, and extending a Vision-Language Model pipeline. It is not a formal
training framework yet.

## Current Goals

This project currently focuses on:

- Startup animation and launcher
- Automatic model-safe image preparation
- Single-image and batch image conversion
- Image input checks
- Prompt template management
- Mock local VLM inference interface
- Optional local Qwen2.5-VL inference through the existing Ollama demo path
- Simple image question-answering demo
- Follow-up question mode for repeated free-form prompts on one image
- Demo result saving
- `robot_task_json` result inspector / replay viewer for saved JSONL runs
- Placeholder dataset and annotation utilities
- Placeholder robot/simulation interface

Future work can connect Qwen2.5-VL, LLaVA, InternVL, Isaac Sim, ROS2, MoveIt2,
UR5 controllers, or other local robotics and VLM components.

## Project Structure

```text
teto_vlm/
├── teto_V1.py
├── README.md
├── requirements.txt
├── config/
│   └── default.yaml
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
│   ├── run_demo.py
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

Start the launcher:

```bash
python3 teto_V1.py
```

Check the environment:

```bash
python3 scripts/check_env.py
```

Run the mock demo:

```bash
python3 scripts/run_demo.py --image data/raw/1.jpg --prompt-type describe_image --backend mock
```

Run the local Qwen2.5-VL demo using the existing Ollama model:

```bash
python3 scripts/run_demo.py --image data/raw/1.jpg --prompt-type describe_image --backend qwen
```

By default, the Qwen backend uses `qwen2.5vl:3b`, matching the existing demo.
`--backend local` is accepted as an alias for `--backend qwen`.
To override the local model name without changing code:

```bash
python3 scripts/run_demo.py --image data/raw/1.jpg --prompt-type describe_image --backend qwen --model-path qwen2.5vl:3b
```

Use a custom prompt:

```bash
python3 scripts/run_demo.py --image data/raw/1.jpg --prompt "Please describe all objects." --backend mock
```

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

Single image SRL test:

```bash
python3 scripts/run_demo.py --image data/raw/1.jpg --prompt-type spatial_relationship_lite --backend qwen
```

Batch SRL test:

```bash
python3 scripts/batch_recognize.py --input-dir data/processed/train --prompt-type spatial_relationship_lite --backend qwen
```

Single controlled robot task JSON test:

```bash
python3 scripts/run_demo.py --image data/raw/1.jpg --prompt-type robot_task_json --prompt "pick the red cup" --backend qwen
```

Batch controlled robot task JSON test:

```bash
python3 scripts/batch_recognize.py --input-dir data/processed/train --prompt-type robot_task_json --prompt "pick the red cup" --backend qwen
```

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

When `robot_task_json` batch recognition is run from `python3 teto_V1.py`, the
launcher prints the inspector summary for that exact run directory after the
batch completes. It then asks whether to show detailed item inspection. Normal
batch recognition does not trigger the robot task inspector.

## TETO V1.1.4 recommended smoke test

Use the Qwen backend to create a small real-output smoke set without connecting
to ROS2, MoveIt, UR5, or any robot controller. Prepare four image folders or a
single folder containing:

- a normal manipulable object, such as a cup or box
- a person / human image
- a bird / animal image
- an empty scene or scene with no clear manipulation target

Run the controlled JSON batch:

```bash
python3 scripts/batch_recognize.py --input-dir data/processed/smoke_robot_task --prompt-type robot_task_json --prompt "pick the safe target" --backend qwen
```

Then inspect the saved run:

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

## TETO V1.3.0 scene snapshot contract preparation

`robot_task_json` normalized output now includes a lightweight scene snapshot
contract for later planner_gateway / semantic_task_server stages:

```json
{
  "scene": {
    "scene_version": "run_YYYYMMDD_HHMMSS_item_001",
    "capture_timestamp": "ISO8601 string or unknown",
    "image_path": "input image path",
    "image_width": 640,
    "image_height": 480,
    "source": "single_image",
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

This is still only a software-readable VLM intermediate representation. It
does not read depth, compute camera/world coordinates, publish TF, call MoveIt,
connect to UR5, generate URScript, generate joint angles, generate trajectories,
or send robot control commands.

In the `python3 teto_V1.py` launcher, single image recognition and batch image
recognition also show prompt helper keywords. You can type a built-in prompt
type, a shortcut keyword, or a free-form prompt. Useful shortcuts include:

- `describe`: `describe_image`
- `objects` or `locate`: `locate_objects`
- `spatial` or `relation`: `spatial_relationship`
- `srl`: `spatial_relationship_lite`
- `srl_json` or `json`: `spatial_relationship_json`
- `manipulation` or `grasp`: `manipulation_spatial_analysis`
- `candidate`: `manipulation_candidate`
- `robot_json` or `task_json`: `robot_task_json`

Demo commands accept common image formats directly. TETO automatically
creates a cached RGB JPEG under `data/processed/auto/`, with EXIF orientation
applied, long edge resized, animated images reduced to the first frame, and
transparent pixels flattened for model compatibility.

For the Qwen/Ollama backend, TETO uses a smaller default image size than the
mock path and retries with smaller images if the local model runner stops due
to resource limits. You can force a lower size for constrained GPUs:

```bash
TETO_VLM_MAX_SIZE=512 python3 scripts/run_demo.py --image data/raw/1.jpg --backend qwen
```

Prepare a larger image directory for training or offline processing:

```bash
python3 -m src.cli prepare-images \
  --input-dir data/raw \
  --output-dir data/processed/train \
  --manifest outputs/results/train_images.jsonl
```

Training code can also call the same preparation layer directly:

```python
from src.image_utils import prepare_image_for_vlm, prepare_image_dataset

safe_image = prepare_image_for_vlm("data/raw/example.heic")
stats = prepare_image_dataset(
    "data/raw",
    output_dir="data/processed/train",
    manifest_path="outputs/results/train_images.jsonl",
)
```

The manifest is JSONL. Each line maps the original file to the prepared model
input path with `source_image_path` and `image_path`.

In the `python3 teto_V1.py` launcher, use the image conversion menu entry:

```text
1. Convert images
2. Run the demo
3. Just chat with TETO
4. Check environment
5. Quit
```

Choose `Just chat with TETO` for a text-only chat loop. It defaults to the
existing local Qwen/Ollama backend and uses the same model name as the VLM demo
unless `TETO_QWEN_MODEL` overrides it. It does not need an image and does not
write recognition results. Type `back`, `q`, `quit`, or `exit` to return to the
main menu. Select `mock` only when you want the simple local fallback.

Choose `Convert images`, then select a conversion mode:

```text
1. Single image
2. Batch images
3. Back
```

For batch conversion, choose `Batch convert images` and enter an input folder.
Press Enter to use the default `data/raw`. TETO reads `image.max_size` and
`image.quality` from `config/default.yaml`; if the config cannot be read, it
falls back to `max_size=1024` and `quality=85`.

Single image conversion creates a timestamped folder under:

```text
outputs/image_conversion/single/single_YYYYMMDD_HHMMSS/
├── processed/
├── summary.json
└── errors.log
```

Each batch conversion creates a new folder so older results are not overwritten:

```text
outputs/image_conversion/batch/batch_YYYYMMDD_HHMMSS/
├── processed/
├── manifest.jsonl
├── summary.json
└── errors.log
```

`processed/` contains converted JPEG images. `manifest.jsonl` records each
source image, output image, status, and error message. `summary.json` records
the batch input, output, totals, max size, and quality. `errors.log` lists
failed images and reasons. Batch conversion supports `.jpg`, `.jpeg`, `.png`,
and `.webp` inputs.

Older workspaces may still contain `outputs/batches/`. New batch conversion
runs no longer write there.

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
└── errors.log
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

Use the unified CLI:

```bash
python3 -m src.cli check-env
python3 -m src.cli demo --image data/raw/1.jpg --prompt-type describe_image
python3 -m src.cli prepare-images --input-dir data/raw --output-dir data/processed/train
```

## Notes

- `teto_V1.py` is the visual launcher, not a training main program.
- Real VLM inference logic belongs in `src/vlm_infer.py`.
- Image processing logic belongs in `src/image_utils.py`.
- Prompt templates belong in `src/prompt_utils.py`.
- Local Qwen2.5-VL integration lives in `src/vlm_infer.py`.
- Future Isaac Sim / ROS2 / UR5 integration should mainly modify
  `src/robot_interface.py`.

## Extension Plan

- Connect real local Qwen2.5-VL inference
- Add lab image datasets
- Add annotation formats
- Add spatial relationship understanding tasks
- Add simple robot action plan JSON output
- Connect ROS2 / MoveIt2 / Isaac Sim / UR5 controller
