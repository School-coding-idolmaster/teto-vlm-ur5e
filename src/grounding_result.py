"""Compatibility shim for the grounding result contract API."""

from src.grounding.result import (
    CONTRACT_VERSION,
    CURRENT_GROUNDING_VERSION,
    FORBIDDEN_ROBOT_CONTROL_FIELDS,
    GroundingResultRequest,
    build_grounding_result_request,
    evaluate_grounding_result_contract,
    load_grounding_result,
)

__all__ = [
    "CONTRACT_VERSION",
    "CURRENT_GROUNDING_VERSION",
    "FORBIDDEN_ROBOT_CONTROL_FIELDS",
    "GroundingResultRequest",
    "load_grounding_result",
    "build_grounding_result_request",
    "evaluate_grounding_result_contract",
]
