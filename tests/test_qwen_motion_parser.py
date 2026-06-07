import json
import subprocess
import urllib.error
from pathlib import Path

from src.qwen_motion_parser import (
    DEFAULT_ENDPOINT,
    DEFAULT_MODEL_NAME,
    E_EXCESSIVE_CARTESIAN_MOTION,
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


def test_qwen_low_confidence_blocks():
    result = _parse_payload({"intent": "relative_cartesian_motion", "axis": "z", "direction": "+", "distance_m": 0.005, "confidence": 0.50})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_LOW_CONFIDENCE in result["parser_blocking_reasons"]


def test_qwen_unsupported_intent_blocks():
    result = _parse_payload({"intent": "hover_to_object", "axis": "z", "direction": "+", "distance_m": 0.005, "confidence": 0.95})

    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert E_UNSUPPORTED_INTENT in result["parser_blocking_reasons"]


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
    return evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="move up 5 mm",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
            llm_callable=lambda _prompt: json.dumps(payload),
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
