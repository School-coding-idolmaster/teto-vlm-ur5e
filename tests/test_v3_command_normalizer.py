from src.v3_command_normalizer import (
    E_TARGET_DESCRIPTION_MISSING,
    E_UNSUPPORTED_COMMAND,
    E_UNSUPPORTED_CONTACT_TASK,
    INTENT_HOVER_TO_OBJECT,
    INTENT_RETURN_HOME,
    INTENT_STOP,
    normalize_v3_command,
)


def test_english_hover_command_accepted():
    result = normalize_v3_command("hover over the red mug")

    assert result["accepted"] is True
    assert result["intent_name"] == INTENT_HOVER_TO_OBJECT
    assert result["target_query"] == "red mug"
    assert result["target_label_hint"] == "red_mug"


def test_chinese_hover_command_accepted():
    result = normalize_v3_command("把机械臂移动到红色杯子上方")

    assert result["accepted"] is True
    assert result["intent_name"] == INTENT_HOVER_TO_OBJECT
    assert result["language"] == "zh"
    assert result["target_query"] == "红色杯子"


def test_japanese_hover_command_accepted():
    result = normalize_v3_command("紅いマグカップの上で止まって")

    assert result["accepted"] is True
    assert result["intent_name"] == INTENT_HOVER_TO_OBJECT
    assert result["language"] == "ja"
    assert result["target_query"] == "紅いマグカップ"


def test_return_home_command_accepted():
    result = normalize_v3_command("return home")

    assert result["accepted"] is True
    assert result["intent_name"] == INTENT_RETURN_HOME


def test_stop_command_accepted():
    result = normalize_v3_command("halt")

    assert result["accepted"] is True
    assert result["intent_name"] == INTENT_STOP


def test_grasp_command_rejected_as_contact_task():
    result = normalize_v3_command("pick up the red mug")

    assert result["rejected"] is True
    assert result["error_code"] == E_UNSUPPORTED_CONTACT_TASK


def test_push_and_press_commands_rejected_as_contact_tasks():
    for command in ("push the red mug", "press the button", "推红色杯子", "按按钮"):
        result = normalize_v3_command(command)

        assert result["rejected"] is True
        assert result["error_code"] == E_UNSUPPORTED_CONTACT_TASK


def test_unknown_command_rejected():
    result = normalize_v3_command("wave at the mug")

    assert result["rejected"] is True
    assert result["error_code"] == E_UNSUPPORTED_COMMAND


def test_missing_target_rejected():
    result = normalize_v3_command("hover over")

    assert result["rejected"] is True
    assert result["error_code"] == E_TARGET_DESCRIPTION_MISSING
