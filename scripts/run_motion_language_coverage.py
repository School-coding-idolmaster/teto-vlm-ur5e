#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Any
import urllib.error
import urllib.request


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.motion_command_normalizer import normalize_motion_command  # noqa: E402
from src.qwen_motion_parser import build_qwen_motion_prompt  # noqa: E402


DEFAULT_ENDPOINT = "http://127.0.0.1:18080"
DEFAULT_MODEL = "Qwen/Qwen2.5-VL-3B-Instruct"
DEFAULT_OUTPUT_DIR = "outputs/qwen_motion_language_coverage"

PASS_CANONICAL_RELATIVE_MOTION = "PASS_CANONICAL_RELATIVE_MOTION"
PASS_FUZZY_DEFAULT_STEP = "PASS_FUZZY_DEFAULT_STEP"
NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"
UNSUPPORTED_VISION_INTENT = "UNSUPPORTED_VISION_INTENT"
UNSUPPORTED_MANIPULATION_INTENT = "UNSUPPORTED_MANIPULATION_INTENT"
UNSUPPORTED_OTHER = "UNSUPPORTED_OTHER"
PARSER_ERROR = "PARSER_ERROR"
NORMALIZER_ERROR = "NORMALIZER_ERROR"


CORPUS: list[dict[str, Any]] = [
    *[
        {"category": "A_explicit_safe_relative_motion", "raw_command": command}
        for command in [
            "raise the tcp by 5 cm",
            "lift the tool 2 centimeters",
            "move the robot hand slightly down by 10 mm",
            "drop the end effector 2 centimeters",
            "shift the arm tip left by 10 mm",
            "move forward 5 centimeters",
            "go backward by 3 cm",
            "raise it by two centimeters",
            "lower it by five millimeters",
            "move the tcp to the right by 1 cm",
            "move the tool up 15 mm",
            "shift the tip right by 20 millimeters",
            "move the end-effector down 0.03 meters",
            "bring the tool higher by 4 cm",
            "slide the tcp left by 2 cm",
        ]
    ],
    *[
        {"category": "B_fuzzy_small_step", "raw_command": command}
        for command in [
            "go down a little",
            "move up a bit",
            "lift the tool slightly",
            "nudge the tcp left",
            "shift the end effector forward a little",
            "bring the robot hand slightly higher",
            "drop the tool a tiny bit",
            "move the arm tip backward slightly",
            "nudge the tool right",
            "lower the tcp just a little",
            "raise the end effector a bit",
            "move the tip forward slightly",
            "slide the tool left a little",
            "move the robot hand down a small step",
            "bring the tcp up slightly",
        ]
    ],
    *[
        {"category": "C_long_distance_safety_decided_later", "raw_command": command}
        for command in [
            "move forward 10 cm",
            "move forward 20 cm",
            "lower the tcp by 10 centimeters",
            "raise the tool by 20 centimeters",
            "shift the end effector left by 15 cm",
            "move backward 25 centimeters",
            "move up 0.2 meters",
            "drop the tcp by 0.15 meters",
            "move right 12 cm",
            "go forward by 18 centimeters",
        ]
    ],
    *[
        {"category": "D_ambiguity_clarification", "raw_command": command}
        for command in [
            "move it over there",
            "move 5 cm",
            "move up and down 5 cm",
            "move left and right a little",
            "go somewhere safe",
            "move closer",
            "move away",
            "put it there",
            "move to that side",
            "go over the object",
        ]
    ],
    *[
        {"category": "E_object_vision_manipulation_rejection", "raw_command": command}
        for command in [
            "move to the mug",
            "go to the red cup",
            "hover above the bottle",
            "grab the cup",
            "pick up the mug",
            "touch the red object",
            "push the box",
            "inspect the marker",
            "point at the screwdriver",
            "move to the object on the left",
        ]
    ],
    *[
        {"category": "F_chinese_natural_language", "raw_command": command}
        for command in [
            "把 tcp 抬高 5 厘米",
            "末端往下移动一点",
            "工具向左移动 1 厘米",
            "机械臂末端往前一点",
            "tcp 往右移动 10 毫米",
            "把末端降低 2 厘米",
            "往后退一点",
            "往上抬一点",
            "向前移动 20 厘米",
            "移动到杯子上方",
            "抓起杯子",
            "往那里移动",
            "移动 5 厘米",
            "先上再下 5 厘米",
            "往左边稍微挪一下",
        ]
    ],
    *[
        {"category": "G_mixed_conversational", "raw_command": command}
        for command in [
            "can you move the tool a little lower",
            "please raise the end effector by 3 cm",
            "just nudge it forward a tiny bit",
            "bring the robot hand down, maybe 1 cm",
            "move the tip higher, but only slightly",
            "I want the tcp to go left by two centimeters",
            "make the arm tip go backward a little",
            "lower the tool toward the table by 2 cm",
            "move it toward the mug",
            "carefully go up 5 cm",
            "slowly move down 10 mm",
            "shift a little to the right",
            "can you put the gripper over the cup",
            "move forward, no, actually move backward 5 cm",
            "go up 5 cm and right 2 cm",
        ]
    ],
]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline TETO natural-language motion coverage report.")
    parser.add_argument("commands", nargs="*", help="Commands to normalize. Uses built-in corpus when omitted.")
    parser.add_argument("--default-small-step-m", type=float, default=0.01)
    parser.add_argument("--use-qwen", action="store_true", help="Call the local Qwen motion server for every case.")
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--timeout-s", type=float, default=60.0)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--max-cases", type=int)
    parser.add_argument("--save-json", action="store_true")
    parser.add_argument("--save-md", action="store_true")
    args = parser.parse_args(argv)

    cases = _cases_from_args(args.commands)
    if args.max_cases is not None:
        cases = cases[: max(0, int(args.max_cases))]

    started_at = datetime.now(timezone.utc)
    endpoint = _endpoint_base(args.endpoint)
    health = _health(endpoint) if args.use_qwen else None
    results = [
        _evaluate_case(
            case_id=index + 1,
            category=str(case["category"]),
            raw_command=str(case["raw_command"]),
            use_qwen=bool(args.use_qwen),
            endpoint=endpoint,
            model=str(args.model),
            timeout_s=float(args.timeout_s),
            default_small_step_m=float(args.default_small_step_m),
        )
        for index, case in enumerate(cases)
    ]
    summary = _summary(results)
    report = {
        "report_version": "teto_v3_0_11_llm_first_motion_semantics_coverage_report_v1",
        "generated_at": started_at.isoformat(),
        "repo_root": str(REPO_ROOT),
        "qwen_endpoint": endpoint,
        "qwen_model_name": str(args.model),
        "qwen_health": health,
        "robot_execution_invoked": False,
        "moveit_invoked": False,
        "execute_trajectory_called": False,
        "trajectory_sent": False,
        "real_substep_execution_enabled": False,
        "manual_confirmation_requested": False,
        "summary": summary,
        "cases": results,
    }

    paths = _write_reports(report, output_dir=Path(args.output_dir), save_json=args.save_json, save_md=args.save_md)
    _print_table(results)
    print(json.dumps({"summary": summary, "artifacts": paths}, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


def _cases_from_args(commands: list[str]) -> list[dict[str, Any]]:
    if not commands:
        return list(CORPUS)
    return [{"category": "custom", "raw_command": command} for command in commands]


def _evaluate_case(
    *,
    case_id: int,
    category: str,
    raw_command: str,
    use_qwen: bool,
    endpoint: str,
    model: str,
    timeout_s: float,
    default_small_step_m: float,
) -> dict[str, Any]:
    start = time.monotonic()
    qwen_raw = None
    qwen_payload = None
    qwen_status = "NOT_CALLED"
    qwen_error = None
    if use_qwen:
        try:
            qwen_raw = _call_qwen(endpoint=endpoint, model=model, command=raw_command, timeout_s=timeout_s)
            qwen_payload = _parse_json_object(qwen_raw)
            qwen_status = "PASS" if isinstance(qwen_payload, dict) else "MALFORMED"
        except Exception as exc:
            qwen_error = str(exc)
            qwen_status = "ERROR"
    try:
        normalized = normalize_motion_command(
            raw_command,
            default_small_step_m=default_small_step_m,
            qwen_semantic=qwen_payload if isinstance(qwen_payload, dict) else None,
            parser_source="qwen_llm" if isinstance(qwen_payload, dict) else "rule_based",
        )
        fallback_normalized = normalize_motion_command(raw_command, default_small_step_m=default_small_step_m, parser_source="rule_based")
        classification = _classification(normalized, qwen_status=qwen_status)
        normalizer_error = None
    except Exception as exc:
        normalized = {}
        fallback_normalized = {}
        classification = NORMALIZER_ERROR
        normalizer_error = str(exc)
    elapsed_ms = round((time.monotonic() - start) * 1000.0, 3)
    mapping_conflict = normalized.get("qwen_fallback_conflict") is True
    return {
        "case_id": case_id,
        "category": category,
        "classification": classification,
        "raw_command": raw_command,
        "qwen_called": use_qwen,
        "qwen_endpoint": endpoint if use_qwen else None,
        "qwen_model_name": model if use_qwen else None,
        "qwen_raw_response": qwen_raw,
        "qwen_parse_status": qwen_status,
        "qwen_payload": qwen_payload,
        "qwen_error": qwen_error,
        "normalized_command": normalized.get("normalized_command"),
        "parse_status": normalized.get("parse_status"),
        "canonical_intent": normalized.get("intent"),
        "direction_axis": normalized.get("direction_axis"),
        "direction_sign": normalized.get("direction_sign"),
        "requested_distance_m": normalized.get("requested_distance_m"),
        "distance_source": normalized.get("distance_source"),
        "direction_source": normalized.get("direction_source"),
        "requires_confirmation": normalized.get("requires_confirmation"),
        "clarification_reason": normalized.get("clarification_reason"),
        "unsupported_intent_reason": normalized.get("unsupported_intent_reason"),
        "parser_confidence": normalized.get("parser_confidence"),
        "motion_parse_confidence": normalized.get("motion_parse_confidence"),
        "semantic_alignment_version": normalized.get("semantic_alignment_version"),
        "qwen_semantic_schema_version": normalized.get("qwen_semantic_schema_version"),
        "qwen_intent_status": normalized.get("qwen_intent_status"),
        "qwen_intent_type": normalized.get("qwen_intent_type"),
        "qwen_direction_semantic": normalized.get("qwen_direction_semantic"),
        "qwen_distance_quality": normalized.get("qwen_distance_quality"),
        "qwen_distance_m": normalized.get("qwen_distance_m"),
        "qwen_language": normalized.get("qwen_language"),
        "qwen_confidence_overall": normalized.get("qwen_confidence_overall"),
        "qwen_semantic_parse_used": normalized.get("qwen_semantic_parse_used"),
        "fallback_parse_used": normalized.get("fallback_parse_used"),
        "qwen_fallback_conflict": normalized.get("qwen_fallback_conflict"),
        "qwen_fallback_conflict_reason": normalized.get("qwen_fallback_conflict_reason"),
        "canonicalization_source": normalized.get("canonicalization_source"),
        "fallback_parse_status": fallback_normalized.get("parse_status"),
        "fallback_direction_axis": fallback_normalized.get("direction_axis"),
        "fallback_direction_sign": fallback_normalized.get("direction_sign"),
        "fallback_requested_distance_m": fallback_normalized.get("requested_distance_m"),
        "execution_permission_decided_by_parser": normalized.get("execution_permission_decided_by_parser"),
        "safety_gate_still_required": normalized.get("safety_gate_still_required"),
        "elapsed_ms": elapsed_ms,
        "normalizer_error": normalizer_error,
        "qwen_direction_mapping_conflict": mapping_conflict,
    }


def _classification(normalized: dict[str, Any], *, qwen_status: str) -> str:
    if qwen_status in {"ERROR", "MALFORMED"}:
        return PARSER_ERROR
    status = normalized.get("parse_status")
    reason = str(normalized.get("unsupported_intent_reason") or "")
    if status == "PASS" and normalized.get("distance_source") == "inferred_default":
        return PASS_FUZZY_DEFAULT_STEP
    if status == "PASS":
        return PASS_CANONICAL_RELATIVE_MOTION
    if status == "NEEDS_CLARIFICATION":
        return NEEDS_CLARIFICATION
    if "VISION" in reason or reason == "NEEDS_VISION":
        return UNSUPPORTED_VISION_INTENT
    if "MANIPULATION" in reason:
        return UNSUPPORTED_MANIPULATION_INTENT
    return UNSUPPORTED_OTHER


def _call_qwen(*, endpoint: str, model: str, command: str, timeout_s: float) -> str:
    payload = {
        "model": model,
        "prompt": _qwen_prompt(command),
        "stream": False,
        "options": {"temperature": 0.0, "num_predict": 512},
    }
    request = urllib.request.Request(
        f"{endpoint.rstrip('/')}/api/generate",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        body = json.loads(response.read().decode("utf-8"))
    return str(body.get("response") or "")


def _qwen_prompt(command: str) -> str:
    return build_qwen_motion_prompt(command)


def _parse_json_object(text: str | None) -> dict[str, Any] | None:
    if not isinstance(text, str) or not text.strip():
        return None
    stripped = text.strip()
    if stripped.startswith("```"):
        stripped = stripped.strip("`")
        if stripped.lower().startswith("json"):
            stripped = stripped[4:].strip()
    try:
        parsed = json.loads(stripped)
    except json.JSONDecodeError:
        start = stripped.find("{")
        end = stripped.rfind("}")
        if start < 0 or end <= start:
            return None
        try:
            parsed = json.loads(stripped[start : end + 1])
        except json.JSONDecodeError:
            return None
    return parsed if isinstance(parsed, dict) else None


def _health(endpoint: str) -> dict[str, Any] | None:
    try:
        with urllib.request.urlopen(f"{endpoint.rstrip('/')}/health", timeout=5.0) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except (OSError, urllib.error.URLError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _summary(results: list[dict[str, Any]]) -> dict[str, Any]:
    latencies = [float(item["elapsed_ms"]) for item in results if isinstance(item.get("elapsed_ms"), (int, float))]
    by_class = _counts(item.get("classification") for item in results)
    by_category = _counts(item.get("category") for item in results)
    return {
        "total_cases": len(results),
        "qwen_called_count": sum(1 for item in results if item.get("qwen_called") is True),
        "qwen_success_count": sum(1 for item in results if item.get("qwen_parse_status") == "PASS"),
        "canonical_relative_motion_count": by_class.get(PASS_CANONICAL_RELATIVE_MOTION, 0),
        "fuzzy_default_count": by_class.get(PASS_FUZZY_DEFAULT_STEP, 0),
        "clarification_count": by_class.get(NEEDS_CLARIFICATION, 0),
        "unsupported_vision_manipulation_count": by_class.get(UNSUPPORTED_VISION_INTENT, 0) + by_class.get(UNSUPPORTED_MANIPULATION_INTENT, 0),
        "parser_normalizer_error_count": by_class.get(PARSER_ERROR, 0) + by_class.get(NORMALIZER_ERROR, 0),
        "classification_counts": by_class,
        "category_counts": by_category,
        "english_explicit_success_rate": _rate(results, "A_explicit_safe_relative_motion", {PASS_CANONICAL_RELATIVE_MOTION}),
        "english_fuzzy_success_rate": _rate(results, "B_fuzzy_small_step", {PASS_FUZZY_DEFAULT_STEP}),
        "long_distance_parse_success_rate": _rate(results, "C_long_distance_safety_decided_later", {PASS_CANONICAL_RELATIVE_MOTION}),
        "ambiguity_correct_rejection_rate": _rate(results, "D_ambiguity_clarification", {NEEDS_CLARIFICATION, UNSUPPORTED_OTHER, UNSUPPORTED_VISION_INTENT}),
        "vision_manipulation_correct_rejection_rate": _rate(results, "E_object_vision_manipulation_rejection", {UNSUPPORTED_VISION_INTENT, UNSUPPORTED_MANIPULATION_INTENT, UNSUPPORTED_OTHER}),
        "chinese_breakdown": _counts(item.get("classification") for item in results if item.get("category") == "F_chinese_natural_language"),
        "average_latency_ms": round(statistics.mean(latencies), 3) if latencies else None,
        "p50_latency_ms": _percentile(latencies, 50),
        "p90_latency_ms": _percentile(latencies, 90),
        "worst_10_cases": sorted(
            [
                {
                    "case_id": item.get("case_id"),
                    "raw_command": item.get("raw_command"),
                    "classification": item.get("classification"),
                    "reason": item.get("clarification_reason") or item.get("unsupported_intent_reason") or item.get("qwen_error") or item.get("normalizer_error"),
                    "elapsed_ms": item.get("elapsed_ms"),
                }
                for item in results
            ],
            key=lambda item: float(item.get("elapsed_ms") or 0.0),
            reverse=True,
        )[:10],
        "qwen_parsed_but_normalizer_rejected": [
            item["case_id"]
            for item in results
            if item.get("qwen_parse_status") == "PASS"
            and item.get("qwen_intent_status") == "ok"
            and item.get("parse_status") != "PASS"
        ],
        "qwen_malformed_cases": [item["case_id"] for item in results if item.get("qwen_parse_status") == "MALFORMED"],
        "qwen_direction_conflict_cases": [item["case_id"] for item in results if item.get("qwen_direction_mapping_conflict") is True],
        "qwen_semantic_used_count": sum(1 for item in results if item.get("qwen_semantic_parse_used") is True),
        "fallback_used_count": sum(1 for item in results if item.get("fallback_parse_used") is True),
        "qwen_fallback_conflict_cases": [item["case_id"] for item in results if item.get("qwen_fallback_conflict") is True],
        "qwen_classified_clarification_cases": [item["case_id"] for item in results if item.get("qwen_intent_status") == "needs_clarification"],
        "qwen_classified_unsupported_cases": [item["case_id"] for item in results if item.get("qwen_intent_status") == "unsupported"],
    }


def _counts(values: Any) -> dict[str, int]:
    counts: dict[str, int] = {}
    for value in values:
        key = str(value)
        counts[key] = counts.get(key, 0) + 1
    return counts


def _rate(results: list[dict[str, Any]], category: str, passing: set[str]) -> float | None:
    subset = [item for item in results if item.get("category") == category]
    if not subset:
        return None
    return round(sum(1 for item in subset if item.get("classification") in passing) / len(subset), 6)


def _percentile(values: list[float], percentile: int) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * (percentile / 100.0)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return round(ordered[int(index)], 3)
    fraction = index - lower
    return round(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction, 3)


def _write_reports(report: dict[str, Any], *, output_dir: Path, save_json: bool, save_md: bool) -> dict[str, str | None]:
    if not save_json and not save_md:
        return {"json": None, "md": None}
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = output_dir / f"qwen_motion_language_coverage_{stamp}.json"
    md_path = output_dir / f"qwen_motion_language_coverage_{stamp}.md"
    if save_json:
        json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if save_md:
        md_path.write_text(_markdown_report(report), encoding="utf-8")
    return {"json": str(json_path) if save_json else None, "md": str(md_path) if save_md else None}


def _markdown_report(report: dict[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        "# Qwen Motion Language Coverage",
        "",
        f"- generated_at: {report['generated_at']}",
        f"- qwen_endpoint: {report['qwen_endpoint']}",
        f"- qwen_model_name: {report['qwen_model_name']}",
        f"- total_cases: {summary['total_cases']}",
        f"- qwen_called_count: {summary['qwen_called_count']}",
        f"- qwen_success_count: {summary['qwen_success_count']}",
        f"- canonical_relative_motion_count: {summary['canonical_relative_motion_count']}",
        f"- fuzzy_default_count: {summary['fuzzy_default_count']}",
        f"- clarification_count: {summary['clarification_count']}",
        f"- unsupported_vision_manipulation_count: {summary['unsupported_vision_manipulation_count']}",
        f"- parser_normalizer_error_count: {summary['parser_normalizer_error_count']}",
        f"- qwen_semantic_used_count: {summary.get('qwen_semantic_used_count')}",
        f"- fallback_used_count: {summary.get('fallback_used_count')}",
        f"- qwen_fallback_conflict_cases: {summary.get('qwen_fallback_conflict_cases')}",
        f"- average_latency_ms: {summary['average_latency_ms']}",
        f"- p50_latency_ms: {summary['p50_latency_ms']}",
        f"- p90_latency_ms: {summary['p90_latency_ms']}",
        "",
        "| id | category | classification | command | qwen_semantic | canonical | distance_m | source | reason |",
        "| - | - | - | - | - | - | - | - | - |",
    ]
    for item in report["cases"]:
        canonical = f"{item.get('direction_axis') or ''}{item.get('direction_sign') or ''}"
        semantic = item.get("qwen_direction_semantic") or ""
        reason = item.get("clarification_reason") or item.get("unsupported_intent_reason") or item.get("qwen_fallback_conflict_reason") or item.get("qwen_error") or ""
        lines.append(
            f"| {item['case_id']} | {item['category']} | {item['classification']} | "
            f"{_md(item['raw_command'])} | {semantic} | {canonical} | {item.get('requested_distance_m') or ''} | "
            f"{item.get('canonicalization_source') or ''} | {_md(reason)} |"
        )
    return "\n".join(lines) + "\n"


def _md(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def _print_table(results: list[dict[str, Any]]) -> None:
    rows = [("id", "category", "classification", "direction", "distance_m", "command")]
    for item in results:
        direction = f"{item.get('direction_axis') or ''}{item.get('direction_sign') or ''}"
        rows.append(
            (
                str(item.get("case_id")),
                str(item.get("category")),
                str(item.get("classification")),
                direction,
                "" if item.get("requested_distance_m") is None else str(item.get("requested_distance_m")),
                str(item.get("raw_command")),
            )
        )
    widths = [min(max(len(row[index]) for row in rows), 42) for index in range(len(rows[0]))]
    for row in rows:
        cells = [row[index][: widths[index]].ljust(widths[index]) for index in range(len(row))]
        print(" | ".join(cells))


def _endpoint_base(endpoint: str) -> str:
    if endpoint.endswith("/api/generate"):
        return endpoint[: -len("/api/generate")]
    return endpoint.rstrip("/")


if __name__ == "__main__":
    raise SystemExit(main())
