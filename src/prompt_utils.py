PROMPT_TEMPLATES = {
    "describe_image": "Describe this image briefly.",
    "locate_objects": "List the visible objects and their approximate positions in the image.",
    "spatial_relationship": "Describe the spatial relationships between the main objects in the image.",
    "spatial_relationship_lite": (
        "List the main visible objects in the image and describe their approximate spatial relationships "
        "using simple terms such as left, right, front, behind, on, under, near, far, center, and edge. "
        "If you are uncertain about a relationship, say \"uncertain\"."
    ),
    "spatial_relationship_json": (
        "Identify the main visible objects in the image and output their approximate spatial relationships "
        "in JSON format.\n\n"
        "Use this schema:\n\n"
        "{\n"
        "  \"objects\": [\n"
        "    {\n"
        "      \"name\": \"object name\",\n"
        "      \"approx_position\": \"left / right / center / top / bottom / front / behind / uncertain\",\n"
        "      \"relations\": [\n"
        "        {\n"
        "          \"target\": \"another object name\",\n"
        "          \"relation\": \"left_of / right_of / in_front_of / behind / on / under / near / far / overlapping / uncertain\",\n"
        "          \"confidence\": \"high / medium / low\"\n"
        "        }\n"
        "      ]\n"
        "    }\n"
        "  ],\n"
        "  \"overall_scene\": \"brief scene description\",\n"
        "  \"uncertainties\": [\"list uncertain relationships here\"]\n"
        "}\n\n"
        "Do not invent objects that are not clearly visible. If a relationship is unclear, use \"uncertain\"."
    ),
    "robot_instruction_parse": (
        "Convert the user's natural language instruction into a simple robot action plan. "
        "Output the result in JSON format."
    ),
    "manipulation_candidate": "Which visible object is suitable for robot manipulation? Explain briefly.",
    "manipulation_spatial_analysis": (
        "Analyze the image for a robot manipulation task.\n\n"
        "Please identify:\n"
        "1. Main visible objects.\n"
        "2. Approximate position of each object in the image.\n"
        "3. Spatial relationships between objects.\n"
        "4. Which object seems easiest to manipulate with a robotic arm.\n"
        "5. Possible obstacles around the target object.\n\n"
        "Use simple spatial terms: left, right, front, behind, on, under, near, far, center, edge, "
        "occluded, clear.\n\n"
        "If the image does not provide enough information for safe manipulation, say "
        "\"insufficient visual information\"."
    ),
    "robot_task_json": (
        "Analyze the image for a controlled robot task intermediate representation.\n"
        "Output strict JSON only. Do not output Markdown. Do not output explanation text. "
        "Do not output robot control commands, URScript, joint angles, trajectories, or motion plans.\n\n"
        "Use only the controlled vocabulary shown in the schema. If unsure, use \"unknown\".\n"
        "If there is no explicit user instruction, set user_instruction to \"unknown\".\n\n"
        "Safety rules:\n"
        "- Humans must not be selected as robot manipulation candidates.\n"
        "- Living animals must not be selected as robot manipulation candidates.\n"
        "- Fragile, dangerous, sharp, hot, liquid-filled, transparent, reflective, or unclear objects "
        "should be marked as unsafe or hard when appropriate.\n"
        "- Do not simply choose the most visually salient object. Choose only if it is suitable for "
        "a robot manipulation task.\n"
        "- candidate must be true or false. Do not default to true.\n"
        "- Set candidate=false for humans, animals, unsafe objects, or unsuitable targets.\n"
        "- If the target is not suitable for robot manipulation, set "
        "manipulation_assessment.candidate=false, manipulation_assessment.difficulty=\"unsafe\", "
        "and error.code=\"E_UNSAFE\".\n"
        "- If the image does not contain a suitable manipulation target, set target.label=\"unknown\" "
        "or the visible object label if relevant, manipulation_assessment.candidate=false, and "
        "error.code=\"E_NO_TARGET\" or \"E_UNSAFE\".\n"
        "- If unsure, use target.label=\"unknown\" and low confidence.\n\n"
        "2D grounding rules:\n"
        "- Estimate target.bbox_xyxy when the target location is visually clear.\n"
        "- bbox_xyxy must be [x_min, y_min, x_max, y_max] in input-image pixel coordinates, or null.\n"
        "- Estimate geometry_2d.pixel_center as [cx, cy], ideally the bbox center, or null.\n"
        "- geometry_2d.confidence is a number from 0.0 to 1.0 for rough 2D localization only.\n"
        "- image_width and image_height may be null; the program will fill them from the input image.\n"
        "- These 2D fields are rough visual grounding hints only. Do not output 3D coordinates, "
        "camera/world coordinates, robot goals, robot actions, trajectories, or control commands.\n\n"
        "Schema:\n"
        "{\n"
        "  \"schema_version\": \"teto_robot_task.v1\",\n"
        "  \"task_type\": \"target_analysis\",\n"
        "  \"user_instruction\": \"string\",\n"
        "  \"target\": {\n"
        "    \"label\": \"string or unknown\",\n"
        "    \"bbox_xyxy\": [x_min, y_min, x_max, y_max] or null,\n"
        "    \"approx_position\": \"left/right/center/top/bottom/front/back/edge/unknown\",\n"
        "    \"visibility\": \"clear/partially_occluded/heavily_occluded/unknown\"\n"
        "  },\n"
        "  \"geometry_2d\": {\n"
        "    \"pixel_center\": [cx, cy] or null,\n"
        "    \"image_width\": integer or null,\n"
        "    \"image_height\": integer or null,\n"
        "    \"confidence\": number from 0.0 to 1.0 or null\n"
        "  },\n"
        "  \"spatial_context\": {\n"
        "    \"surface\": \"table/floor/shelf/unknown\",\n"
        "    \"nearby_objects\": [\"string\"],\n"
        "    \"relations\": [\n"
        "      {\n"
        "        \"object\": \"string\",\n"
        "        \"relation\": \"left_of/right_of/in_front_of/behind/on/under/near/far/overlapping/unknown\",\n"
        "        \"target\": \"string\",\n"
        "        \"confidence\": \"high/medium/low/unknown\"\n"
        "      }\n"
        "    ],\n"
        "    \"obstacles\": [\"string\"]\n"
        "  },\n"
        "  \"manipulation_assessment\": {\n"
        "    \"candidate\": true or false,\n"
        "    \"difficulty\": \"easy/medium/hard/unsafe/unknown\",\n"
        "    \"reason\": \"string\"\n"
        "  },\n"
        "  \"confidence\": {\n"
        "    \"semantic\": \"high/medium/low/unknown\",\n"
        "    \"spatial\": \"high/medium/low/unknown\",\n"
        "    \"overall\": \"high/medium/low/unknown\"\n"
        "  },\n"
        "  \"error\": {\n"
        "    \"code\": \"OK/E_NO_TARGET/E_UNCLEAR_IMAGE/E_AMBIGUOUS_TARGET/E_UNSAFE/E_PARSE\",\n"
        "    \"message\": \"string\"\n"
        "  }\n"
        "}"
    ),
}


def list_prompt_types() -> list[str]:
    return sorted(PROMPT_TEMPLATES.keys())


def get_prompt(prompt_type: str) -> str:
    try:
        return PROMPT_TEMPLATES[prompt_type]
    except KeyError as exc:
        options = ", ".join(list_prompt_types())
        raise ValueError(f"Unknown prompt type '{prompt_type}'. Available prompt types: {options}") from exc


def build_prompt(prompt_type: str, user_instruction: str | None = None) -> str:
    prompt = get_prompt(prompt_type)
    if prompt_type != "robot_task_json":
        return user_instruction if user_instruction else prompt

    instruction = (user_instruction or "").strip() or "unknown"
    return (
        f"{prompt}\n\n"
        f"User instruction: {instruction}\n\n"
        "Copy the user instruction string into the user_instruction field."
    )
