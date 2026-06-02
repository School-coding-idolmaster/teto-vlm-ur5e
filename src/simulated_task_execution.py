from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any, Dict


SIMULATED_TASK_STATUS_SUCCEEDED = "SUCCEEDED"
SIMULATED_TASK_STATUS_BLOCKED_BY_SEMANTIC_GATE = "BLOCKED_BY_SEMANTIC_GATE"
SIMULATED_TASK_STATUS_BLOCKED_BY_PRECHECK = "BLOCKED_BY_PRECHECK"
SIMULATED_TASK_STATUS_MOTION_FAILED = "MOTION_FAILED"
SIMULATED_TASK_STATUS_POST_CHECK_FAILED = "POST_CHECK_FAILED"
SIMULATED_TASK_STATUS_DRY_RUN_ONLY = "DRY_RUN_ONLY"
SIMULATED_TASK_STATUS_FAILED = "FAILED"

EXECUTION_FEEDBACK_STATUS_OK = "OK"
EXECUTION_FEEDBACK_STATUS_BLOCKED = "BLOCKED"
EXECUTION_FEEDBACK_STATUS_WARNING = "WARNING"
EXECUTION_FEEDBACK_STATUS_FAILED = "FAILED"

FAILURE_REASON_NONE = "NONE"
FAILURE_REASON_SEMANTIC_GATE_BLOCKED = "E_SEMANTIC_GATE_BLOCKED"
FAILURE_REASON_PRECHECK_NOT_READY = "E_SIMULATION_MOTION_PRECHECK_NOT_READY"
FAILURE_REASON_MICRO_MOTION_FAILED = "E_SIMULATION_MICRO_MOTION_FAILED"
FAILURE_REASON_POST_STATE_MISSING = "E_POST_MOTION_STATE_MISSING"
FAILURE_REASON_DELTA_OUT_OF_TOLERANCE = "E_DELTA_OUT_OF_TOLERANCE"
FAILURE_REASON_UNEXPECTED_STATE = "E_UNEXPECTED_EXECUTION_STATE"
FAILURE_REASON_DRY_RUN_ONLY = "E_DRY_RUN_ONLY"

FALLBACK_TYPE_NONE = "NONE"
FALLBACK_TYPE_REOBSERVE = "REOBSERVE"
FALLBACK_TYPE_REVALIDATE_SEMANTIC_CONTRACT = "REVALIDATE_SEMANTIC_CONTRACT"
FALLBACK_TYPE_RECHECK_SIMULATION_PRECHECK = "RECHECK_SIMULATION_PRECHECK"
FALLBACK_TYPE_MANUAL_REVIEW = "MANUAL_REVIEW"
FALLBACK_TYPE_BLOCK_EXECUTION = "BLOCK_EXECUTION"


@dataclass(frozen=True)
class SimulatedTaskExecutionRequest:
    requested: bool = False
    execution_attempt_id: str | None = None
    execution_max_attempts: int = 1
    execution_attempt_index: int = 1
    retry_recommendation_enabled: bool = False
    fallback_recommendation_enabled: bool = False


@dataclass(frozen=True)
class PostMotionStateCheck:
    post_motion_state_check_status: str
    before_state_available: bool
    after_state_available: bool
    actual_delta_available: bool
    delta_within_tolerance: bool
    post_check_passed: bool
    blocking_reasons: list[str]
    warnings: list[str]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ExecutionFeedback:
    execution_feedback_status: str
    message: str
    failure_reason: str
    replay_ready: bool
    warnings: list[str]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class FailureAnalysis:
    failure_reason: str
    failure_category: str
    blocking_stage: str
    human_readable_message: str
    failure_summary: str
    retry_recommended: bool
    fallback_recommended: bool
    fallback_type: str
    next_safe_action: str
    semantic_blocking_reasons: list[str]
    precheck_blocking_reasons: list[str]
    motion_blocking_reasons: list[str]
    post_check_blocking_reasons: list[str]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RetryFallbackRecommendation:
    retry_recommended: bool
    fallback_recommended: bool
    fallback_type: str
    recommendation_reason: str
    recommendation_summary: str
    automatic_retry_executed: bool
    next_safe_action: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ExecutionAttemptRecord:
    execution_attempt_id: str
    execution_attempt_index: int
    execution_max_attempts: int
    semantic_bridge_status: str
    semantic_gate_passed: bool
    simulation_motion_precheck_status: str
    simulation_micro_motion_status: str
    simulated_task_status: str
    robot_motion_executed: bool
    real_robot_motion_executed: bool
    replay_ready: bool

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SimulatedTaskExecutionResult:
    safe_simulated_task_execution_requested: bool
    execution_attempt_id: str | None
    execution_max_attempts: int
    execution_attempt_index: int
    simulated_task_status: str
    execution_feedback_status: str
    failure_reason: str
    retry_recommended: bool
    fallback_recommended: bool
    fallback_type: str
    replay_ready: bool
    post_motion_state_check: Dict[str, Any]
    execution_feedback: Dict[str, Any]
    failure_analysis: Dict[str, Any]
    retry_fallback_recommendation: Dict[str, Any]
    execution_attempt_record: Dict[str, Any]
    simulated_task_execution_result_path: str | None = None
    simulated_task_execution_report_path: str | None = None
    execution_feedback_path: str | None = None
    execution_attempt_record_path: str | None = None
    failure_analysis_path: str | None = None
    retry_fallback_recommendation_path: str | None = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def build_execution_attempt_id(context: Dict[str, Any], requested_id: str | None = None) -> str:
    if requested_id:
        return str(requested_id)
    task_id = context.get("semantic_task_id") or "semantic_task"
    started_at = str(context.get("started_at") or "unknown_time").replace(" ", "_").replace(":", "")
    return f"{task_id}_attempt_1_{started_at}"


def execute_safe_simulated_task(
    request: SimulatedTaskExecutionRequest,
    context: Dict[str, Any],
) -> Dict[str, Any]:
    if not request.requested:
        return _not_requested_result(request)

    normalized_request = SimulatedTaskExecutionRequest(
        requested=True,
        execution_attempt_id=build_execution_attempt_id(context, request.execution_attempt_id),
        execution_max_attempts=int(request.execution_max_attempts or 1),
        execution_attempt_index=int(request.execution_attempt_index or 1),
        retry_recommendation_enabled=request.retry_recommendation_enabled,
        fallback_recommendation_enabled=request.fallback_recommendation_enabled,
    )
    post_check = evaluate_post_motion_state(context)
    simulated_status = classify_simulated_task_status(context, post_check=post_check)
    failure_analysis = analyze_execution_failure(context, simulated_status, post_check=post_check)
    recommendation = recommend_retry_or_fallback(
        failure_analysis,
        retry_enabled=normalized_request.retry_recommendation_enabled,
        fallback_enabled=normalized_request.fallback_recommendation_enabled,
    )
    failure_analysis = {
        **failure_analysis,
        "retry_recommended": recommendation["retry_recommended"],
        "fallback_recommended": recommendation["fallback_recommended"],
        "fallback_type": recommendation["fallback_type"],
        "next_safe_action": recommendation["next_safe_action"],
    }
    feedback = _build_execution_feedback(simulated_status, failure_analysis, post_check)
    attempt = ExecutionAttemptRecord(
        execution_attempt_id=str(normalized_request.execution_attempt_id),
        execution_attempt_index=normalized_request.execution_attempt_index,
        execution_max_attempts=normalized_request.execution_max_attempts,
        semantic_bridge_status=str(context.get("semantic_bridge_status") or "NOT_REQUESTED"),
        semantic_gate_passed=context.get("semantic_gate_passed") is True,
        simulation_motion_precheck_status=str(context.get("simulation_motion_precheck_status") or "NOT_REQUESTED"),
        simulation_micro_motion_status=str(context.get("simulation_micro_motion_status") or "NOT_REQUESTED"),
        simulated_task_status=simulated_status,
        robot_motion_executed=context.get("robot_motion_executed") is True,
        real_robot_motion_executed=context.get("real_robot_motion_executed") is True,
        replay_ready=True,
    ).to_dict()
    result = SimulatedTaskExecutionResult(
        safe_simulated_task_execution_requested=True,
        execution_attempt_id=str(normalized_request.execution_attempt_id),
        execution_max_attempts=normalized_request.execution_max_attempts,
        execution_attempt_index=normalized_request.execution_attempt_index,
        simulated_task_status=simulated_status,
        execution_feedback_status=feedback["execution_feedback_status"],
        failure_reason=failure_analysis["failure_reason"],
        retry_recommended=recommendation["retry_recommended"],
        fallback_recommended=recommendation["fallback_recommended"],
        fallback_type=recommendation["fallback_type"],
        replay_ready=True,
        post_motion_state_check=post_check,
        execution_feedback=feedback,
        failure_analysis=failure_analysis,
        retry_fallback_recommendation=recommendation,
        execution_attempt_record=attempt,
    ).to_dict()
    result["safety_boundary"] = _safety_boundary()
    return result


def evaluate_post_motion_state(context: Dict[str, Any], *, dry_run: bool | None = None) -> Dict[str, Any]:
    is_dry_run = context.get("mode") == "dry_run" if dry_run is None else dry_run
    before_state = context.get("before_articulation_state")
    after_state = context.get("after_articulation_state")
    motion = context.get("motion") if isinstance(context.get("motion"), dict) else {}
    actual_delta = context.get("actual_delta_rad", motion.get("actual_delta_rad"))
    delta_within_tolerance = context.get("delta_within_tolerance", motion.get("delta_within_tolerance")) is True
    before_available = isinstance(before_state, dict) and bool(before_state)
    after_available = isinstance(after_state, dict) and bool(after_state)
    actual_delta_available = actual_delta is not None
    blockers: list[str] = []
    warnings: list[str] = []

    if is_dry_run:
        return PostMotionStateCheck(
            post_motion_state_check_status="DRY_RUN_ONLY",
            before_state_available=before_available,
            after_state_available=after_available,
            actual_delta_available=actual_delta_available,
            delta_within_tolerance=delta_within_tolerance,
            post_check_passed=False,
            blocking_reasons=[],
            warnings=["Dry-run only; post-motion state change is not required or claimed."],
        ).to_dict()

    if not before_available or not after_available:
        blockers.append(FAILURE_REASON_POST_STATE_MISSING)
    if not actual_delta_available:
        blockers.append(FAILURE_REASON_POST_STATE_MISSING)
    if actual_delta_available and not delta_within_tolerance:
        blockers.append(FAILURE_REASON_DELTA_OUT_OF_TOLERANCE)

    passed = not blockers
    return PostMotionStateCheck(
        post_motion_state_check_status="OK" if passed else "FAILED",
        before_state_available=before_available,
        after_state_available=after_available,
        actual_delta_available=actual_delta_available,
        delta_within_tolerance=delta_within_tolerance,
        post_check_passed=passed,
        blocking_reasons=_unique(blockers),
        warnings=warnings,
    ).to_dict()


def classify_simulated_task_status(
    context: Dict[str, Any],
    *,
    post_check: Dict[str, Any] | None = None,
) -> str:
    post_check = post_check if isinstance(post_check, dict) else evaluate_post_motion_state(context)
    if context.get("semantic_bridge_status") == "BLOCKED_BY_SEMANTIC_GATE" or context.get("semantic_gate_passed") is False:
        return SIMULATED_TASK_STATUS_BLOCKED_BY_SEMANTIC_GATE
    if context.get("simulation_micro_motion_status") == "DRY_RUN_ONLY" or post_check.get("post_motion_state_check_status") == "DRY_RUN_ONLY":
        return SIMULATED_TASK_STATUS_DRY_RUN_ONLY
    if context.get("simulation_micro_motion_status") == "BLOCKED_BY_PRECHECK":
        return SIMULATED_TASK_STATUS_BLOCKED_BY_PRECHECK
    if context.get("simulation_motion_precheck_status") not in {"READY_FOR_SIMULATION_MOTION", None, "NOT_REQUESTED"}:
        return SIMULATED_TASK_STATUS_BLOCKED_BY_PRECHECK
    if context.get("simulation_micro_motion_status") not in {"OK", "NOT_REQUESTED"}:
        return SIMULATED_TASK_STATUS_MOTION_FAILED
    if post_check.get("post_check_passed") is not True:
        if FAILURE_REASON_DELTA_OUT_OF_TOLERANCE in post_check.get("blocking_reasons", []):
            return SIMULATED_TASK_STATUS_MOTION_FAILED
        return SIMULATED_TASK_STATUS_POST_CHECK_FAILED
    if context.get("simulation_micro_motion_status") == "OK":
        return SIMULATED_TASK_STATUS_SUCCEEDED
    return SIMULATED_TASK_STATUS_FAILED


def analyze_execution_failure(
    context: Dict[str, Any],
    simulated_task_status: str,
    *,
    post_check: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    post_check = post_check if isinstance(post_check, dict) else evaluate_post_motion_state(context)
    semantic_reasons = _list(context.get("semantic_bridge_blocking_reasons"))
    precheck = context.get("simulation_motion_precheck") if isinstance(context.get("simulation_motion_precheck"), dict) else {}
    motion_reasons = _list(context.get("simulation_micro_motion_blocking_reasons"))
    post_reasons = _list(post_check.get("blocking_reasons"))
    if simulated_task_status == SIMULATED_TASK_STATUS_SUCCEEDED:
        reason = FAILURE_REASON_NONE
        summary = "The simulated execution attempt succeeded."
        category = "NONE"
        stage = "NONE"
    elif simulated_task_status == SIMULATED_TASK_STATUS_DRY_RUN_ONLY:
        reason = FAILURE_REASON_DRY_RUN_ONLY
        summary = "Dry-run only; no real Isaac state change is claimed."
        category = "DRY_RUN"
        stage = "DRY_RUN"
    elif simulated_task_status == SIMULATED_TASK_STATUS_BLOCKED_BY_SEMANTIC_GATE:
        reason = FAILURE_REASON_SEMANTIC_GATE_BLOCKED
        summary = "The semantic gate blocked the simulated execution attempt."
        category = "SEMANTIC_GATE"
        stage = "SEMANTIC_GATE"
    elif simulated_task_status == SIMULATED_TASK_STATUS_BLOCKED_BY_PRECHECK:
        reason = FAILURE_REASON_PRECHECK_NOT_READY
        summary = "The simulation motion precheck was not ready."
        category = "SIMULATION_PRECHECK"
        stage = "SIMULATION_MOTION_PRECHECK"
    elif simulated_task_status == SIMULATED_TASK_STATUS_MOTION_FAILED:
        reason = (
            FAILURE_REASON_DELTA_OUT_OF_TOLERANCE
            if FAILURE_REASON_DELTA_OUT_OF_TOLERANCE in post_reasons
            else FAILURE_REASON_MICRO_MOTION_FAILED
        )
        summary = "The simulation-only micro-motion did not produce acceptable evidence."
        category = "SIMULATION_MOTION"
        stage = "SIMULATION_MICRO_MOTION"
    elif simulated_task_status == SIMULATED_TASK_STATUS_POST_CHECK_FAILED:
        reason = FAILURE_REASON_POST_STATE_MISSING
        summary = "Post-motion state evidence was missing or incomplete."
        category = "POST_MOTION_STATE"
        stage = "POST_MOTION_STATE_CHECK"
    else:
        reason = FAILURE_REASON_UNEXPECTED_STATE
        summary = "The simulated execution attempt ended in an unexpected state."
        category = "UNEXPECTED"
        stage = "EXECUTION_CLASSIFICATION"
    return FailureAnalysis(
        failure_reason=reason,
        failure_category=category,
        blocking_stage=stage,
        human_readable_message=summary,
        failure_summary=summary,
        retry_recommended=False,
        fallback_recommended=False,
        fallback_type=FALLBACK_TYPE_NONE,
        next_safe_action=_next_safe_action(reason, FALLBACK_TYPE_NONE),
        semantic_blocking_reasons=semantic_reasons,
        precheck_blocking_reasons=_list(precheck.get("blocking_reasons")),
        motion_blocking_reasons=motion_reasons,
        post_check_blocking_reasons=post_reasons,
    ).to_dict()


def recommend_retry_or_fallback(
    failure_analysis: Dict[str, Any],
    *,
    retry_enabled: bool = False,
    fallback_enabled: bool = False,
) -> Dict[str, Any]:
    reason = failure_analysis.get("failure_reason")
    retry = False
    fallback = False
    fallback_type = FALLBACK_TYPE_NONE
    summary = "No retry or fallback is recommended."
    reason_text = summary
    if reason == FAILURE_REASON_NONE:
        pass
    elif reason == FAILURE_REASON_DRY_RUN_ONLY:
        retry = bool(retry_enabled)
        fallback = bool(fallback_enabled)
        fallback_type = FALLBACK_TYPE_RECHECK_SIMULATION_PRECHECK if fallback else FALLBACK_TYPE_NONE
        summary = "Dry-run evidence is replay-ready; run Isaac validation for state-change evidence."
        reason_text = summary
    elif reason == FAILURE_REASON_SEMANTIC_GATE_BLOCKED:
        semantic_reasons = set(failure_analysis.get("semantic_blocking_reasons") or [])
        retry = bool(retry_enabled and ("E_STATE_STALE" in semantic_reasons or "E_LOW_CONFIDENCE" in semantic_reasons))
        fallback = bool(fallback_enabled)
        if "E_NO_TARGET" in semantic_reasons or "E_MISSING_TARGET" in semantic_reasons:
            fallback_type = FALLBACK_TYPE_REOBSERVE
        elif "E_UNSAFE_TARGET" in semantic_reasons:
            fallback_type = FALLBACK_TYPE_BLOCK_EXECUTION
        else:
            fallback_type = FALLBACK_TYPE_REVALIDATE_SEMANTIC_CONTRACT
        summary = "Semantic evidence should be reobserved or revalidated before another execution attempt."
        reason_text = summary
    elif reason == FAILURE_REASON_PRECHECK_NOT_READY:
        retry = bool(retry_enabled)
        fallback = bool(fallback_enabled)
        fallback_type = FALLBACK_TYPE_RECHECK_SIMULATION_PRECHECK
        summary = "Recheck robot asset, articulation readiness, and observed state before another attempt."
        reason_text = summary
    elif reason in {FAILURE_REASON_MICRO_MOTION_FAILED, FAILURE_REASON_DELTA_OUT_OF_TOLERANCE}:
        retry = bool(retry_enabled)
        fallback = bool(fallback_enabled)
        fallback_type = FALLBACK_TYPE_MANUAL_REVIEW
        summary = "Manual review is recommended before retrying simulation motion."
        reason_text = summary
    else:
        retry = False
        fallback = bool(fallback_enabled)
        fallback_type = FALLBACK_TYPE_MANUAL_REVIEW
        summary = "Manual review is recommended."
        reason_text = summary
    return RetryFallbackRecommendation(
        retry_recommended=retry,
        fallback_recommended=fallback,
        fallback_type=fallback_type,
        recommendation_reason=reason_text,
        recommendation_summary=summary,
        automatic_retry_executed=False,
        next_safe_action=_next_safe_action(str(reason), fallback_type),
    ).to_dict()


def format_simulated_task_execution_report(
    result: Dict[str, Any],
    *,
    evidence_files: list[Dict[str, str | None]] | None = None,
) -> str:
    evidence_files = evidence_files or []
    post_check = result.get("post_motion_state_check") if isinstance(result.get("post_motion_state_check"), dict) else {}
    feedback = result.get("execution_feedback") if isinstance(result.get("execution_feedback"), dict) else {}
    failure = result.get("failure_analysis") if isinstance(result.get("failure_analysis"), dict) else {}
    recommendation = (
        result.get("retry_fallback_recommendation")
        if isinstance(result.get("retry_fallback_recommendation"), dict)
        else {}
    )
    attempt = (
        result.get("execution_attempt_record")
        if isinstance(result.get("execution_attempt_record"), dict)
        else {}
    )
    return "\n".join(
        [
            "# TETO V2.8.2 Safe Simulated Task Execution Evidence Report",
            "",
            "## Execution Attempt Summary",
            "",
            f"- execution_attempt_id: {_format_value(result.get('execution_attempt_id'))}",
            f"- simulated_task_status: {_format_value(result.get('simulated_task_status'))}",
            f"- replay_ready: {_format_value(result.get('replay_ready'))}",
            "",
            "## Execution Lifecycle Table",
            "",
            "| Step | Status | Evidence |",
            "| --- | --- | --- |",
            f"| semantic gate | {_format_value(result.get('semantic_bridge_status'))} | semantic_gate_passed={_format_value(result.get('semantic_gate_passed'))} |",
            f"| simulation precheck | {_format_value(result.get('simulation_motion_precheck_status'))} | ready_for_simulation_motion={_format_value(result.get('ready_for_simulation_motion'))} |",
            f"| simulation micro-motion | {_format_value(result.get('simulation_micro_motion_status'))} | robot_motion_executed={_format_value(attempt.get('robot_motion_executed'))}; real_robot_motion_executed={_format_value(attempt.get('real_robot_motion_executed'))} |",
            f"| post-motion state check | {_format_value(post_check.get('post_motion_state_check_status'))} | post_check_passed={_format_value(post_check.get('post_check_passed'))} |",
            f"| execution feedback | {_format_value(feedback.get('execution_feedback_status'))} | simulated_task_status={_format_value(result.get('simulated_task_status'))} |",
            "",
            "## Gate Decision Table",
            "",
            "| Gate | Decision | Blocking Reasons |",
            "| --- | --- | --- |",
            f"| semantic gate | {_format_value(result.get('semantic_gate_passed'))} | {_format_value(failure.get('semantic_blocking_reasons'))} |",
            f"| simulation precheck | {_format_value(result.get('ready_for_simulation_motion'))} | {_format_value(failure.get('precheck_blocking_reasons'))} |",
            "",
            "## Motion Verification Table",
            "",
            "| Metric | Value |",
            "| --- | --- |",
            f"| simulation_micro_motion_status | {_format_value(result.get('simulation_micro_motion_status'))} |",
            f"| actual_delta_rad | {_format_value(result.get('actual_delta_rad'))} |",
            f"| delta_within_tolerance | {_format_value(result.get('delta_within_tolerance'))} |",
            f"| post_motion_state_check_status | {_format_value(post_check.get('post_motion_state_check_status'))} |",
            f"| before_state_available | {_format_value(post_check.get('before_state_available'))} |",
            f"| after_state_available | {_format_value(post_check.get('after_state_available'))} |",
            f"| actual_delta_available | {_format_value(post_check.get('actual_delta_available'))} |",
            "",
            "## Failure / Retry / Fallback Table",
            "",
            "| Field | Value |",
            "| --- | --- |",
            f"| failure_reason | {_format_value(failure.get('failure_reason'))} |",
            f"| failure_category | {_format_value(failure.get('failure_category'))} |",
            f"| blocking_stage | {_format_value(failure.get('blocking_stage'))} |",
            f"| human_readable_message | {_format_value(failure.get('human_readable_message'))} |",
            f"| retry_recommended | {_format_value(recommendation.get('retry_recommended'))} |",
            f"| fallback_recommended | {_format_value(recommendation.get('fallback_recommended'))} |",
            f"| fallback_type | {_format_value(recommendation.get('fallback_type'))} |",
            f"| automatic_retry_executed | {_format_value(recommendation.get('automatic_retry_executed'))} |",
            f"| next_safe_action | {_format_value(recommendation.get('next_safe_action'))} |",
            "",
            "## Replay Readiness Summary",
            "",
            f"- replay_ready: {_format_value(result.get('replay_ready'))}",
            f"- execution_attempt_id: {_format_value(result.get('execution_attempt_id'))}",
            f"- evidence_file_count: {_format_value(len(evidence_files))}",
            "- replay scope: semantic contract, gate outcome, simulation precheck, micro-motion evidence, post-motion check, feedback, failure analysis, retry/fallback recommendation",
            "",
            "## Semantic Gate Summary",
            "",
            f"- semantic_bridge_status: {_format_value(result.get('semantic_bridge_status'))}",
            f"- semantic_gate_passed: {_format_value(result.get('semantic_gate_passed'))}",
            "",
            "## Simulation Precheck Summary",
            "",
            f"- simulation_motion_precheck_status: {_format_value(result.get('simulation_motion_precheck_status'))}",
            f"- ready_for_simulation_motion: {_format_value(result.get('ready_for_simulation_motion'))}",
            "",
            "## Simulation Micro-Motion Summary",
            "",
            f"- simulation_micro_motion_status: {_format_value(result.get('simulation_micro_motion_status'))}",
            f"- actual_delta_rad: {_format_value(result.get('actual_delta_rad'))}",
            f"- delta_within_tolerance: {_format_value(result.get('delta_within_tolerance'))}",
            "",
            "## Post-Motion State Check",
            "",
            f"- post_motion_state_check_status: {_format_value(post_check.get('post_motion_state_check_status'))}",
            f"- before_state_available: {_format_value(post_check.get('before_state_available'))}",
            f"- after_state_available: {_format_value(post_check.get('after_state_available'))}",
            f"- actual_delta_available: {_format_value(post_check.get('actual_delta_available'))}",
            f"- post_check_passed: {_format_value(post_check.get('post_check_passed'))}",
            "",
            "## Execution Feedback",
            "",
            f"- execution_feedback_status: {_format_value(feedback.get('execution_feedback_status'))}",
            f"- message: {_format_value(feedback.get('message'))}",
            "",
            "## Failure Analysis",
            "",
            f"- failure_reason: {_format_value(failure.get('failure_reason'))}",
            f"- failure_category: {_format_value(failure.get('failure_category'))}",
            f"- blocking_stage: {_format_value(failure.get('blocking_stage'))}",
            f"- human_readable_message: {_format_value(failure.get('human_readable_message'))}",
            f"- failure_summary: {_format_value(failure.get('failure_summary'))}",
            f"- next_safe_action: {_format_value(failure.get('next_safe_action'))}",
            "",
            "## Retry / Fallback Recommendation",
            "",
            f"- retry_recommended: {_format_value(recommendation.get('retry_recommended'))}",
            f"- fallback_recommended: {_format_value(recommendation.get('fallback_recommended'))}",
            f"- fallback_type: {_format_value(recommendation.get('fallback_type'))}",
            f"- recommendation_reason: {_format_value(recommendation.get('recommendation_reason'))}",
            f"- automatic_retry_executed: {_format_value(recommendation.get('automatic_retry_executed'))}",
            f"- next_safe_action: {_format_value(recommendation.get('next_safe_action'))}",
            "",
            "## Evidence Files",
            "",
            *[f"- {item.get('name')}: {_format_value(item.get('path'))}" for item in evidence_files],
            "",
            "## Safety Boundary",
            "",
            "This is a safe simulated task execution evidence report.",
            "It consumes an existing semantic task contract.",
            "It does not call a live camera or live VLM.",
            "It does not execute target poses, tcp_pose_world, trajectories, MoveIt goals, URScript, Dashboard commands, or real robot commands.",
            "It only performs a local Isaac Sim simulation-only micro-motion proof pulse after semantic gate and simulation precheck pass.",
            "Retry and fallback are recommendations only; no automatic repeated motion is executed.",
            "",
        ]
    )


def _not_requested_result(request: SimulatedTaskExecutionRequest) -> Dict[str, Any]:
    return SimulatedTaskExecutionResult(
        safe_simulated_task_execution_requested=False,
        execution_attempt_id=request.execution_attempt_id,
        execution_max_attempts=int(request.execution_max_attempts or 1),
        execution_attempt_index=int(request.execution_attempt_index or 1),
        simulated_task_status="NOT_REQUESTED",
        execution_feedback_status="NOT_REQUESTED",
        failure_reason=FAILURE_REASON_NONE,
        retry_recommended=False,
        fallback_recommended=False,
        fallback_type=FALLBACK_TYPE_NONE,
        replay_ready=False,
        post_motion_state_check={},
        execution_feedback={},
        failure_analysis={},
        retry_fallback_recommendation={},
        execution_attempt_record={},
    ).to_dict()


def _build_execution_feedback(
    simulated_status: str,
    failure_analysis: Dict[str, Any],
    post_check: Dict[str, Any],
) -> Dict[str, Any]:
    if simulated_status == SIMULATED_TASK_STATUS_SUCCEEDED:
        status = EXECUTION_FEEDBACK_STATUS_OK
        message = "Safe simulated task execution attempt succeeded."
    elif simulated_status == SIMULATED_TASK_STATUS_DRY_RUN_ONLY:
        status = EXECUTION_FEEDBACK_STATUS_WARNING
        message = "Dry-run only; no true Isaac post-motion state change is claimed."
    elif simulated_status in {SIMULATED_TASK_STATUS_BLOCKED_BY_SEMANTIC_GATE, SIMULATED_TASK_STATUS_BLOCKED_BY_PRECHECK}:
        status = EXECUTION_FEEDBACK_STATUS_BLOCKED
        message = failure_analysis.get("failure_summary", "Execution attempt was blocked.")
    else:
        status = EXECUTION_FEEDBACK_STATUS_FAILED
        message = failure_analysis.get("failure_summary", "Execution attempt failed.")
    return ExecutionFeedback(
        execution_feedback_status=status,
        message=message,
        failure_reason=str(failure_analysis.get("failure_reason") or FAILURE_REASON_UNEXPECTED_STATE),
        replay_ready=True,
        warnings=_list(post_check.get("warnings")),
    ).to_dict()


def _safety_boundary() -> Dict[str, bool]:
    return {
        "simulation_only": True,
        "no_live_camera_used": True,
        "no_live_vlm_used": True,
        "no_ros2_used": True,
        "no_moveit_used": True,
        "no_rtde_used": True,
        "no_urscript_used": True,
        "no_dashboard_used": True,
        "no_real_ur5_used": True,
        "no_trajectory_generated": True,
        "no_tcp_pose_world_executed": True,
        "automatic_retry_executed": False,
    }


def _next_safe_action(reason: str, fallback_type: str) -> str:
    if reason == FAILURE_REASON_NONE:
        return "Archive the replay-ready evidence bundle; no retry or fallback is needed."
    if reason == FAILURE_REASON_DRY_RUN_ONLY:
        return "Run true Isaac validation if state-change evidence is required; do not claim real joint motion from dry-run evidence."
    if fallback_type == FALLBACK_TYPE_REOBSERVE:
        return "Reobserve or refresh the semantic task contract, then rerun the safety gates."
    if fallback_type == FALLBACK_TYPE_REVALIDATE_SEMANTIC_CONTRACT:
        return "Revalidate the semantic contract before considering another simulated attempt."
    if fallback_type == FALLBACK_TYPE_RECHECK_SIMULATION_PRECHECK:
        return "Recheck simulation robot asset, articulation readiness, and observed state before another simulated attempt."
    if fallback_type == FALLBACK_TYPE_BLOCK_EXECUTION:
        return "Keep execution blocked and perform manual safety review."
    if fallback_type == FALLBACK_TYPE_MANUAL_REVIEW:
        return "Perform manual review of the evidence bundle before any further simulated attempt."
    return "Keep the evidence bundle for review; no automatic retry motion is permitted."


def _list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]


def _unique(values: list[str]) -> list[str]:
    unique_values: list[str] = []
    for value in values:
        if value not in unique_values:
            unique_values.append(value)
    return unique_values


def _format_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False)
    return str(value)
