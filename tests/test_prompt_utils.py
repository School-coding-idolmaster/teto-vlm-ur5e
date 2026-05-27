from src.prompt_utils import build_prompt, get_prompt, list_prompt_types


def test_get_prompt_describe_image_returns_string():
    prompt = get_prompt("describe_image")
    assert isinstance(prompt, str)
    assert prompt


def test_list_prompt_types_contains_describe_image():
    assert "describe_image" in list_prompt_types()


def test_spatial_prompt_types_are_available():
    prompt_types = list_prompt_types()

    assert "spatial_relationship_lite" in prompt_types
    assert "spatial_relationship_json" in prompt_types
    assert "manipulation_spatial_analysis" in prompt_types
    assert get_prompt("spatial_relationship_lite")


def test_robot_task_json_prompt_type_is_available():
    prompt = get_prompt("robot_task_json")

    assert "robot_task_json" in list_prompt_types()
    assert "teto_robot_task.v1" in prompt
    assert "URScript" in prompt
    assert "Humans must not be selected" in prompt
    assert "candidate must be true or false" in prompt


def test_build_robot_task_prompt_embeds_user_instruction():
    prompt = build_prompt("robot_task_json", "pick the red cup")

    assert "pick the red cup" in prompt
    assert "Copy the user instruction" in prompt
