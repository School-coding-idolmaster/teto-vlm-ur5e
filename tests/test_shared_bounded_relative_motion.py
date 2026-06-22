import math

import pytest

from src.bounded_relative_motion import (
    E_RELATIVE_MOTION_RANGE_EXCEEDED,
    build_bounded_relative_motion_contract,
)
from src.qwen_motion_parser import evaluate_shared_motion_parser


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ("move forward 0.5 meters", [0.5, 0.0, 0.0]),
        ("move left 50 cm", [0.0, 0.5, 0.0]),
        ("move backward 0.4 meters", [-0.4, 0.0, 0.0]),
        ("move up 0.2 meters", [0.0, 0.0, 0.2]),
        ("move x positive 0.5 meters", [0.5, 0.0, 0.0]),
        ("move y negative 0.2 meters", [0.0, -0.2, 0.0]),
        ("move vector [0.3, 0.4, 0.0] meters", [0.3, 0.4, 0.0]),
    ],
)
def test_shared_parser_accepts_bounded_relative_motion(command, expected):
    result = evaluate_shared_motion_parser(command)
    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["delta_m"] == expected
    assert result["motion_contract_type"] == "decomposed_relative_motion"
    assert result["distance_within_shared_envelope"] is True
    assert result["shared_max_total_distance_m"] == 0.5


def test_shared_parser_normalizes_combined_direction_to_requested_distance():
    result = evaluate_shared_motion_parser("move forward and left 0.3 meters")
    assert result["qwen_motion_parser_status"] == "PASS"
    assert result["requested_distance_m"] == 0.3
    assert math.sqrt(sum(value * value for value in result["delta_m"])) == pytest.approx(0.3, abs=2e-6)
    assert result["normalized_direction_vector"] == [0.707107, 0.707107, 0.0]


@pytest.mark.parametrize("distance", [0.51, 1.0])
def test_shared_parser_blocks_above_envelope(distance):
    result = evaluate_shared_motion_parser(f"move forward {distance} meters")
    assert result["qwen_motion_parser_status"] == "BLOCKED"
    assert result["blocking_reasons"] == [E_RELATIVE_MOTION_RANGE_EXCEEDED]


@pytest.mark.parametrize(("max_substep", "count"), [(0.05, 10), (0.10, 5)])
def test_shared_decomposition_has_bounded_subgoals(max_substep, count):
    contract = build_bounded_relative_motion_contract([0.3, 0.4, 0.0], max_substep_m=max_substep)
    assert contract["subgoal_count"] == count
    assert all(
        math.sqrt(sum(value * value for value in step)) <= max_substep + 1e-9
        for step in contract["decomposed_substeps_m"]
    )
    assert [
        round(sum(step[axis] for step in contract["decomposed_substeps_m"]), 6)
        for axis in range(3)
    ] == [0.3, 0.4, 0.0]
    assert contract["normalized_direction_vector"] == [0.6, 0.8, 0.0]
