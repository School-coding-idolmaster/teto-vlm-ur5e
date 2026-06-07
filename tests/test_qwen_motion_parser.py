import json

from src.qwen_motion_parser import (
    E_EXCESSIVE_CARTESIAN_MOTION,
    E_INVALID_JSON,
    E_LOW_CONFIDENCE,
    E_UNSUPPORTED_INTENT,
    E_UNSUPPORTED_OR_FORBIDDEN_COMMAND,
    QwenMotionParserRequest,
    evaluate_qwen_motion_parser,
)


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


def _parse_payload(payload):
    return evaluate_qwen_motion_parser(
        QwenMotionParserRequest(
            user_text="move up 5 mm",
            max_distance_m=0.005,
            hard_safety_limit_m=0.01,
            llm_callable=lambda _prompt: json.dumps(payload),
        )
    )
