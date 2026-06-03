from src.manual_confirmation_gate import (
    DEFAULT_CONFIRMATION_TOKEN,
    E_MANUAL_CONFIRMATION_EXPIRED,
    E_MANUAL_CONFIRMATION_REQUIRED,
    E_MANUAL_CONFIRMATION_TOKEN_INVALID,
    ManualConfirmationRequest,
    build_manual_confirmation,
    evaluate_manual_confirmation_gate,
)


def test_manual_confirmation_missing_blocks():
    result = evaluate_manual_confirmation_gate(
        ManualConfirmationRequest(
            expected_task_id="task_1",
            expected_target_label="red mug",
            expected_bounded_target_point_m=[0.1, 0.2, 0.3],
        )
    )

    assert result["manual_confirmation_accepted"] is False
    assert E_MANUAL_CONFIRMATION_REQUIRED in result["blocking_reasons"]


def test_manual_confirmation_invalid_token_blocks():
    confirmation = build_manual_confirmation(
        token="WRONG",
        task_id="task_1",
        target_label="red mug",
        bounded_target_point_m=[0.1, 0.2, 0.3],
        confirmed_at_epoch_s=100.0,
    )

    result = evaluate_manual_confirmation_gate(
        ManualConfirmationRequest(
            expected_task_id="task_1",
            expected_target_label="red mug",
            expected_bounded_target_point_m=[0.1, 0.2, 0.3],
            confirmation=confirmation,
            now_epoch_s=101.0,
        )
    )

    assert result["manual_confirmation_accepted"] is False
    assert E_MANUAL_CONFIRMATION_TOKEN_INVALID in result["blocking_reasons"]


def test_manual_confirmation_expired_blocks():
    confirmation = build_manual_confirmation(
        token=DEFAULT_CONFIRMATION_TOKEN,
        task_id="task_1",
        target_label="red mug",
        bounded_target_point_m=[0.1, 0.2, 0.3],
        confirmed_at_epoch_s=100.0,
    )

    result = evaluate_manual_confirmation_gate(
        ManualConfirmationRequest(
            expected_task_id="task_1",
            expected_target_label="red mug",
            expected_bounded_target_point_m=[0.1, 0.2, 0.3],
            confirmation=confirmation,
            now_epoch_s=200.0,
            timeout_s=30,
        )
    )

    assert result["manual_confirmation_accepted"] is False
    assert E_MANUAL_CONFIRMATION_EXPIRED in result["blocking_reasons"]


def test_manual_confirmation_matching_request_passes():
    confirmation = build_manual_confirmation(
        token=DEFAULT_CONFIRMATION_TOKEN,
        task_id="task_1",
        target_label="red mug",
        bounded_target_point_m=[0.1, 0.2, 0.3],
        confirmed_at_epoch_s=100.0,
    )

    result = evaluate_manual_confirmation_gate(
        ManualConfirmationRequest(
            expected_task_id="task_1",
            expected_target_label="red mug",
            expected_bounded_target_point_m=[0.1, 0.2, 0.3],
            confirmation=confirmation,
            now_epoch_s=101.0,
        )
    )

    assert result["manual_confirmation_accepted"] is True
    assert result["blocking_reasons"] == []
