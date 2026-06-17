import json
import subprocess
import urllib.error
from pathlib import Path

from src.qwen_motion_parser import (
    DEFAULT_ENDPOINT,
    DEFAULT_MODEL_NAME,
    E_EXCESSIVE_CARTESIAN_MOTION,
    E_INVALID_AXIS,
    E_INVALID_DIRECTION,
    E_INVALID_DISTANCE,
    E_INVALID_JSON,
    E_LLM_CALL_FAILED,
    E_LOW_CONFIDENCE,
    E_UNSUPPORTED_INTENT,
    E_UNSUPPORTED_OR_FORBIDDEN_COMMAND,
    QwenMotionParserRequest,
    evaluate_qwen_motion_parser,
)


def test_qwen_parser_uses_local_server_defaults(monkeypatch):
    monkeypatch.setattr("src.qwen_motion_parser._call_qwen", lambda **_kwargs: _json_success())

    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="raise the tcp by 5 millimeters",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
        )
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["model_name"] == DEFAULT_MODEL_NAME
    assert result["qwen_endpoint"] == DEFAULT_ENDPOINT
    assert result["delta_m"] == [0.0, 0.0, 0.005]


def test_qwen_generate_endpoint_response_normalizes(monkeypatch):
    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def read(self):
            return json.dumps({"model": DEFAULT_MODEL_NAME, "response": _json_success(), "done": True}).encode("utf-8")

    monkeypatch.setattr("urllib.request.urlopen", lambda _request, timeout: FakeResponse())

    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="raise the tcp by 5 millimeters",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
            endpoint=DEFAULT_ENDPOINT,
        )
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["raw_llm_output"] == _json_success()
    assert result["normalized_contract"]["delta_m"] == [0.0, 0.0, 0.005]


def test_qwen_valid_json_produces_relative_delta():
    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="raise the tcp by 5 millimeters",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
            model_name="qwen-test",
            endpoint="http://127.0.0.1:11434",
            llm_callable=lambda _prompt: json.dumps(
                {
                    "intent": "relative_cartesian_motion",
                    "axis": "z",
                    "direction": "+",
                    "distance_m": 0.005,
                    "confidence": 0.94,
                    "reason": "small upward relative motion",
                }
            ),
        )
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["llm_called"] is True
    assert result["parser_source"] == "qwen_llm"
    assert result["delta_m"] == [0.0, 0.0, 0.005]
    assert result["normalized_contract"]["delta_m"] == [0.0, 0.0, 0.005]


def test_qwen_fenced_json_with_language_label_produces_relative_delta():
    result = _parse_text(
        """```json
{"intent":"relative_cartesian_motion","axis":"z","direction":"-","distance_m":0.002,"confidence":1.0}
```"""
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["delta_m"] == [0.0, 0.0, -0.002]
    assert result["normalized_contract"]["delta_m"] == [0.0, 0.0, -0.002]


def test_qwen_fenced_json_without_language_label_produces_relative_delta():
    result = _parse_text(
        """```
{"intent":"relative_cartesian_motion","axis":"x","direction":"+","distance_m":0.001,"confidence":1.0}
```"""
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["delta_m"] == [0.001, 0.0, 0.0]


def test_qwen_fenced_json_with_surrounding_whitespace_produces_relative_delta():
    result = _parse_text(
        """

```json
{"intent":"relative_cartesian_motion","axis":"y","direction":"+","distance_m":0.003,"confidence":1.0}
```

"""
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["delta_m"] == [0.0, 0.003, 0.0]


def test_qwen_short_prose_plus_single_fenced_json_produces_relative_delta():
    result = _parse_text(
        """Here is the parsed command:
```json
{"intent":"relative_cartesian_motion","axis":"z","direction":"+","distance_m":0.004,"confidence":1.0}
```
Done."""
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["delta_m"] == [0.0, 0.0, 0.004]


def test_qwen_short_prose_plus_single_json_object_produces_relative_delta():
    result = _parse_text(
        'Parsed command: {"intent":"relative_cartesian_motion","axis":"z","direction":"+","distance_m":0.001,"confidence":1.0}'
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["delta_m"] == [0.0, 0.0, 0.001]


def test_qwen_invalid_json_blocks():
    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="move up 5 mm",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
            llm_callable=lambda _prompt: "not json",
        )
    )

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_JSON in result["parser_blocking_reasons"]


def test_qwen_empty_response_blocks():
    result = _parse_text("")

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_JSON in result["parser_blocking_reasons"]


def test_qwen_no_json_object_blocks():
    result = _parse_text("I cannot parse this command.")

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_JSON in result["parser_blocking_reasons"]


def test_qwen_multiple_json_objects_blocks_as_ambiguous():
    result = _parse_text(
        '{"intent":"relative_cartesian_motion","axis":"z","direction":"+","distance_m":0.001,"confidence":1.0}'
        '{"intent":"relative_cartesian_motion","axis":"z","direction":"-","distance_m":0.001,"confidence":1.0}'
    )

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_JSON in result["parser_blocking_reasons"]
    assert any("ambiguous" in warning for warning in result["warnings"])


def test_qwen_multiple_fenced_json_objects_blocks_as_ambiguous():
    result = _parse_text(
        """```json
{"intent":"relative_cartesian_motion","axis":"z","direction":"+","distance_m":0.001,"confidence":1.0}
```
```json
{"intent":"relative_cartesian_motion","axis":"z","direction":"-","distance_m":0.001,"confidence":1.0}
```"""
    )

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_JSON in result["parser_blocking_reasons"]
    assert any("ambiguous" in warning for warning in result["warnings"])


def test_qwen_top_level_array_blocks():
    result = _parse_text('[{"intent":"relative_cartesian_motion","axis":"z","direction":"+","distance_m":0.001,"confidence":1.0}]')

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_JSON in result["parser_blocking_reasons"]


def test_qwen_missing_intent_blocks():
    result = _parse_payload({"axis": "z", "direction": "+", "distance_m": 0.005, "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_UNSUPPORTED_INTENT in result["parser_blocking_reasons"]


def test_qwen_low_confidence_blocks():
    result = _parse_payload({"intent": "relative_cartesian_motion", "axis": "z", "direction": "+", "distance_m": 0.005, "confidence": 0.50})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_LOW_CONFIDENCE in result["parser_blocking_reasons"]


def test_qwen_unsupported_intent_blocks():
    result = _parse_payload({"intent": "hover_to_object", "axis": "z", "direction": "+", "distance_m": 0.005, "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_UNSUPPORTED_INTENT in result["parser_blocking_reasons"]


def test_qwen_unsupported_axis_blocks():
    result = _parse_payload({"intent": "relative_cartesian_motion", "axis": "yaw", "direction": "+", "distance_m": 0.005, "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_AXIS in result["parser_blocking_reasons"]


def test_qwen_unsupported_direction_blocks():
    result = _parse_payload({"intent": "relative_cartesian_motion", "axis": "z", "direction": "up", "distance_m": 0.005, "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_DIRECTION in result["parser_blocking_reasons"]


def test_qwen_missing_distance_blocks():
    result = _parse_payload({"intent": "relative_cartesian_motion", "axis": "z", "direction": "+", "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_DISTANCE in result["parser_blocking_reasons"]


def test_qwen_non_numeric_distance_blocks():
    result = _parse_payload({"intent": "relative_cartesian_motion", "axis": "z", "direction": "+", "distance_m": "five millimeters", "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_DISTANCE in result["parser_blocking_reasons"]


def test_qwen_negative_distance_blocks():
    result = _parse_payload({"intent": "relative_cartesian_motion", "axis": "z", "direction": "+", "distance_m": -0.001, "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_INVALID_DISTANCE in result["parser_blocking_reasons"]


def test_qwen_reject_intent_blocks():
    result = _parse_payload({"intent": "reject", "axis": None, "direction": None, "distance_m": 0.0, "confidence": 0.0, "reason": "unsupported"})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_UNSUPPORTED_INTENT in result["parser_blocking_reasons"]


def test_qwen_hover_above_red_mug_blocks_even_with_valid_shape():
    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="hover above red mug",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
            llm_callable=lambda _prompt: json.dumps(
                {
                    "intent": "relative_cartesian_motion",
                    "axis": "z",
                    "direction": "+",
                    "distance_m": 0.005,
                    "confidence": 0.95,
                    "reason": "object command",
                }
            ),
        )
    )

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_UNSUPPORTED_OR_FORBIDDEN_COMMAND in result["parser_blocking_reasons"]


def test_qwen_move_up_20_mm_blocks():
    result = _parse_payload({"intent": "relative_cartesian_motion", "axis": "z", "direction": "+", "distance_m": 0.02, "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_EXCESSIVE_CARTESIAN_MOTION in result["parser_blocking_reasons"]


def test_qwen_output_with_urscript_blocks():
    result = _parse_payload(
        {
            "intent": "relative_cartesian_motion",
            "axis": "z",
            "direction": "+",
            "distance_m": 0.005,
            "confidence": 0.95,
            "reason": "use URScript movel",
        }
    )

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_UNSUPPORTED_OR_FORBIDDEN_COMMAND in result["parser_blocking_reasons"]


def test_qwen_endpoint_unavailable_fails_closed(monkeypatch):
    def fake_urlopen(_request, timeout):
        raise urllib.error.URLError("connection refused")

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)

    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="raise the tcp by 5 millimeters",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
            endpoint=DEFAULT_ENDPOINT,
            timeout_s=0.01,
        )
    )

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_LLM_CALL_FAILED in result["parser_blocking_reasons"]
    assert result["raw_llm_output"] is None


def test_qwen_wrapper_scripts_have_valid_bash_syntax():
    repo_root = Path(__file__).resolve().parents[1]
    for script in (
        "scripts/run_qwen_motion_server.sh",
        "scripts/run_text_to_ur5e_real.sh",
        "scripts/run_text_to_ur5e_dry_run.sh",
    ):
        completed = subprocess.run(["bash", "-n", script], cwd=repo_root, text=True, capture_output=True)
        assert completed.returncode == 0, completed.stderr


def _parse_payload(payload):
    return _parse_text(json.dumps(payload))


def _parse_text(text):
    return evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="move up 5 mm",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
            llm_callable=lambda _prompt: text,
        )
    )


def _json_success():
    return json.dumps(
        {
            "intent": "relative_cartesian_motion",
            "axis": "z",
            "direction": "+",
            "distance_m": 0.005,
            "confidence": 0.94,
            "reason": "small upward relative motion",
        }
    )


def test_qwen_prompt_requests_semantic_schema_and_fuzzy_examples():
    from src.qwen_motion_parser import build_qwen_motion_prompt

    prompt = build_qwen_motion_prompt("drop the tool a tiny bit")

    assert "teto_motion_semantics.v1" in prompt
    assert "direction_semantic" in prompt
    assert "fuzzy_small" in prompt
    assert "把末端降低 2 厘米" in prompt
    assert "移动 5 厘米" in prompt
    assert "go up 5 cm and right 2 cm" in prompt
    assert "先上再下 5 厘米" in prompt
    assert "never infer a direction from distance alone" in prompt
    assert "do not approve execution" in prompt.lower()


def test_qwen_semantic_json_produces_canonical_relative_delta():
    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="把末端降低 2 厘米",
            max_distance_m=0.05,
            hard_safety_limit_m=0.05,
            llm_callable=lambda _prompt: json.dumps(
                {
                    "schema_version": "teto_motion_semantics.v1",
                    "intent_status": "ok",
                    "intent_type": "relative_cartesian_motion",
                    "motion": {
                        "reference": "end_effector",
                        "direction_semantic": "down",
                        "distance": {"value": 2, "unit": "cm", "meters": 0.02, "quality": "explicit"},
                        "fuzzy_magnitude": "unspecified",
                        "frame_hint": "base_link",
                    },
                    "clarification": {"required": False, "reason": ""},
                    "unsupported": {"reason": ""},
                    "confidence": {"intent": 0.96, "direction": 0.96, "distance": 0.96, "overall": 0.96},
                    "language": "zh",
                    "notes": "relative lowering",
                }
            ),
        )
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["axis"] == "z"
    assert result["direction"] == "-"
    assert result["distance_m"] == 0.02
    assert result["delta_m"] == [0.0, 0.0, -0.02]
    assert result["qwen_semantic_parse_used"] is True
    assert result["execution_permission_decided_by_parser"] is False
    assert result["safety_gate_still_required"] is True


def test_qwen_semantic_fuzzy_small_uses_default_step():
    result = evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="drop the tool a tiny bit",
            max_distance_m=0.05,
            hard_safety_limit_m=0.05,
            llm_callable=lambda _prompt: json.dumps(
                {
                    "schema_version": "teto_motion_semantics.v1",
                    "intent_status": "ok",
                    "intent_type": "relative_cartesian_motion",
                    "motion": {
                        "reference": "tool",
                        "direction_semantic": "down",
                        "distance": {"value": None, "unit": "unspecified", "meters": None, "quality": "fuzzy_small"},
                        "fuzzy_magnitude": "tiny",
                        "frame_hint": "base_link",
                    },
                    "clarification": {"required": False, "reason": ""},
                    "unsupported": {"reason": ""},
                    "confidence": {"intent": 0.93, "direction": 0.93, "distance": 0.90, "overall": 0.92},
                    "language": "en",
                    "notes": "fuzzy small relative lowering",
                }
            ),
        )
    )

    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["distance_m"] == 0.01
    assert result["distance_source"] == "inferred_default"
    assert result["delta_m"] == [0.0, 0.0, -0.01]
