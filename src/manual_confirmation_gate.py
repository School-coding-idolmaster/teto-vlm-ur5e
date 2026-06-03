from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict


CURRENT_MANUAL_CONFIRMATION_GATE_VERSION = "TETO V3.0.0"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUIRED = "NOT_REQUIRED"

E_MANUAL_CONFIRMATION_REQUIRED = "E_MANUAL_CONFIRMATION_REQUIRED"
E_MANUAL_CONFIRMATION_TOKEN_MISSING = "E_MANUAL_CONFIRMATION_TOKEN_MISSING"
E_MANUAL_CONFIRMATION_TOKEN_INVALID = "E_MANUAL_CONFIRMATION_TOKEN_INVALID"
E_MANUAL_CONFIRMATION_EXPIRED = "E_MANUAL_CONFIRMATION_EXPIRED"
E_MANUAL_CONFIRMATION_TASK_MISMATCH = "E_MANUAL_CONFIRMATION_TASK_MISMATCH"
E_MANUAL_CONFIRMATION_TARGET_MISMATCH = "E_MANUAL_CONFIRMATION_TARGET_MISMATCH"
E_MANUAL_CONFIRMATION_POINT_MISMATCH = "E_MANUAL_CONFIRMATION_POINT_MISMATCH"
E_MANUAL_CONFIRMATION_PHRASE_MISSING = "E_MANUAL_CONFIRMATION_PHRASE_MISSING"

DEFAULT_CONFIRMATION_TOKEN = "CONFIRM_REAL_UR5_HOVER"
DEFAULT_UNDERSTANDING_PHRASE = "I understand this will move the real UR5"
DEFAULT_CONFIRMATION_TIMEOUT_S = 30


@dataclass(frozen=True)
class ManualConfirmationRequest:
    manual_confirmation_required: bool = True
    expected_token: str = DEFAULT_CONFIRMATION_TOKEN
    expected_task_id: str | None = None
    expected_target_label: str | None = None
    expected_bounded_target_point_m: list[float] | None = None
    timeout_s: int = DEFAULT_CONFIRMATION_TIMEOUT_S
    confirmation: Dict[str, Any] | None = None
    now_epoch_s: float | None = None
    required_phrase: str = DEFAULT_UNDERSTANDING_PHRASE


def evaluate_manual_confirmation_gate(request: ManualConfirmationRequest | None = None) -> Dict[str, Any]:
    request = request or ManualConfirmationRequest()
    if not request.manual_confirmation_required:
        return _result(request, STATUS_NOT_REQUIRED, [], False)

    confirmation = request.confirmation if isinstance(request.confirmation, dict) else {}
    blocking_reasons: list[str] = []
    token = _string(confirmation.get("confirmation_token") or confirmation.get("token"))
    if not confirmation:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_REQUIRED)
    if not token:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_TOKEN_MISSING)
    elif token != request.expected_token:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_TOKEN_INVALID)
    if request.expected_task_id and _string(confirmation.get("task_id")) != request.expected_task_id:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_TASK_MISMATCH)
    if request.expected_target_label and _string(confirmation.get("target_label")) != request.expected_target_label:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_TARGET_MISMATCH)
    if request.expected_bounded_target_point_m is not None and not _same_point(
        confirmation.get("bounded_target_point_m"), request.expected_bounded_target_point_m
    ):
        blocking_reasons.append(E_MANUAL_CONFIRMATION_POINT_MISMATCH)
    understanding = _string(confirmation.get("understanding")) or ""
    if request.required_phrase and request.required_phrase not in understanding:
        blocking_reasons.append(E_MANUAL_CONFIRMATION_PHRASE_MISSING)
    if _expired(confirmation, request):
        blocking_reasons.append(E_MANUAL_CONFIRMATION_EXPIRED)

    blocking_reasons = _unique(blocking_reasons)
    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    return _result(request, status, blocking_reasons, status == STATUS_PASS)


def build_manual_confirmation(
    *,
    token: str,
    task_id: str,
    target_label: str,
    bounded_target_point_m: list[float],
    understanding: str = DEFAULT_UNDERSTANDING_PHRASE,
    confirmed_at_epoch_s: float | None = None,
) -> Dict[str, Any]:
    return {
        "confirmation_token": token,
        "task_id": task_id,
        "target_label": target_label,
        "bounded_target_point_m": bounded_target_point_m,
        "understanding": understanding,
        "confirmed_at_epoch_s": confirmed_at_epoch_s if confirmed_at_epoch_s is not None else datetime.now(timezone.utc).timestamp(),
    }


def _result(
    request: ManualConfirmationRequest,
    status: str,
    blocking_reasons: list[str],
    accepted: bool,
) -> Dict[str, Any]:
    confirmation = request.confirmation if isinstance(request.confirmation, dict) else {}
    return {
        "teto_version": CURRENT_MANUAL_CONFIRMATION_GATE_VERSION,
        "manual_confirmation_gate_status": status,
        "manual_confirmation_required": request.manual_confirmation_required,
        "manual_confirmation_accepted": accepted,
        "expected_task_id": request.expected_task_id,
        "expected_target_label": request.expected_target_label,
        "expected_bounded_target_point_m": request.expected_bounded_target_point_m,
        "confirmation_task_id": confirmation.get("task_id"),
        "confirmation_target_label": confirmation.get("target_label"),
        "confirmation_bounded_target_point_m": confirmation.get("bounded_target_point_m"),
        "confirmation_timeout_s": request.timeout_s,
        "blocking_reasons": blocking_reasons,
    }


def _expired(confirmation: Dict[str, Any], request: ManualConfirmationRequest) -> bool:
    timestamp = _float(confirmation.get("confirmed_at_epoch_s"))
    if timestamp is None:
        return bool(confirmation)
    now = request.now_epoch_s if request.now_epoch_s is not None else datetime.now(timezone.utc).timestamp()
    return now - timestamp > request.timeout_s


def _same_point(value: Any, expected: list[float]) -> bool:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return False
    try:
        return all(abs(float(left) - float(right)) <= 1e-6 for left, right in zip(value, expected))
    except (TypeError, ValueError):
        return False


def _string(value: Any) -> str | None:
    if isinstance(value, str):
        return value.strip() or None
    if value is None:
        return None
    return str(value).strip() or None


def _float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value and value not in seen:
            seen.add(value)
            output.append(value)
    return output
