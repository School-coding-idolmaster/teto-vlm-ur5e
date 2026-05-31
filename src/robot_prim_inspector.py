from __future__ import annotations

from typing import Any, Dict, Iterable


INSPECTION_STATUS_NOT_REQUESTED = "NOT_REQUESTED"
INSPECTION_STATUS_OK = "OK"
INSPECTION_STATUS_PRIM_NOT_FOUND = "E_ROBOT_PRIM_NOT_FOUND"
INSPECTION_STATUS_FAILED = "E_ROBOT_PRIM_INSPECTION_FAILED"


def build_robot_prim_inspection_report(
    *,
    requested: bool = False,
    robot_prim_path: str | None = None,
    robot_prim_exists: bool = False,
    robot_root_type_name: str | None = None,
    total_descendant_prim_count: int = 0,
    link_like_prim_count: int = 0,
    joint_like_prim_count: int = 0,
    visual_like_prim_count: int = 0,
    collision_like_prim_count: int = 0,
    articulation_root_found: bool = False,
    physics_schema_summary: list[str] | None = None,
    joint_names: list[str] | None = None,
    joint_prim_paths: list[str] | None = None,
    possible_dof_names: list[str] | None = None,
    possible_dof_count: int | None = None,
    inspection_status: str | None = None,
    inspection_warnings: list[str] | None = None,
) -> Dict[str, Any]:
    if inspection_status is None:
        if not requested:
            inspection_status = INSPECTION_STATUS_NOT_REQUESTED
        elif robot_prim_exists:
            inspection_status = INSPECTION_STATUS_OK
        else:
            inspection_status = INSPECTION_STATUS_PRIM_NOT_FOUND

    normalized_joint_names = joint_names or []
    normalized_dof_names = possible_dof_names if possible_dof_names is not None else normalized_joint_names
    return {
        "requested": requested,
        "robot_prim_path": robot_prim_path,
        "robot_prim_exists": robot_prim_exists,
        "robot_root_type_name": robot_root_type_name,
        "total_descendant_prim_count": int(total_descendant_prim_count),
        "link_like_prim_count": int(link_like_prim_count),
        "joint_like_prim_count": int(joint_like_prim_count),
        "visual_like_prim_count": int(visual_like_prim_count),
        "collision_like_prim_count": int(collision_like_prim_count),
        "articulation_root_found": articulation_root_found,
        "physics_schema_summary": sorted(set(physics_schema_summary or [])),
        "joint_names": normalized_joint_names,
        "joint_prim_paths": joint_prim_paths or [],
        "possible_dof_names": normalized_dof_names,
        "possible_dof_count": int(possible_dof_count if possible_dof_count is not None else len(normalized_dof_names)),
        "inspection_status": inspection_status,
        "inspection_warnings": inspection_warnings or [],
    }


def inspect_robot_prim(world=None, *, stage=None, robot_prim_path: str) -> Dict[str, Any]:
    try:
        resolved_stage = stage or _stage_from_world(world)
        if resolved_stage is None or not hasattr(resolved_stage, "GetPrimAtPath"):
            return build_robot_prim_inspection_report(
                requested=True,
                robot_prim_path=robot_prim_path,
                robot_prim_exists=False,
                inspection_warnings=["No readable USD stage was available for robot prim inspection."],
            )

        root_prim = resolved_stage.GetPrimAtPath(robot_prim_path)
        if not _prim_is_valid(root_prim):
            return build_robot_prim_inspection_report(
                requested=True,
                robot_prim_path=robot_prim_path,
                robot_prim_exists=False,
            )

        descendants = list(_iter_descendants(root_prim))
        all_prims = [root_prim, *descendants]
        joint_prims = [prim for prim in descendants if _is_joint_like(prim)]
        joint_names = [_prim_name(prim) for prim in joint_prims]
        joint_prim_paths = [_prim_path(prim) for prim in joint_prims]
        physics_schemas = sorted(
            {
                schema
                for prim in all_prims
                for schema in _applied_schemas(prim)
                if _is_physics_schema(schema)
            }
        )

        return build_robot_prim_inspection_report(
            requested=True,
            robot_prim_path=robot_prim_path,
            robot_prim_exists=True,
            robot_root_type_name=_type_name(root_prim),
            total_descendant_prim_count=len(descendants),
            link_like_prim_count=sum(1 for prim in descendants if _is_link_like(prim)),
            joint_like_prim_count=len(joint_prims),
            visual_like_prim_count=sum(1 for prim in descendants if _is_visual_like(prim)),
            collision_like_prim_count=sum(1 for prim in descendants if _is_collision_like(prim)),
            articulation_root_found=any(_is_articulation_root_like(prim) for prim in all_prims),
            physics_schema_summary=physics_schemas,
            joint_names=joint_names,
            joint_prim_paths=joint_prim_paths,
            possible_dof_names=joint_names,
            possible_dof_count=len(joint_names),
            inspection_status=INSPECTION_STATUS_OK,
        )
    except Exception as exc:
        return build_robot_prim_inspection_report(
            requested=True,
            robot_prim_path=robot_prim_path,
            robot_prim_exists=False,
            inspection_status=INSPECTION_STATUS_FAILED,
            inspection_warnings=[str(exc)],
        )


def _stage_from_world(world):
    stage = getattr(world, "stage", None)
    if stage is not None:
        return stage
    try:
        import omni.usd

        return omni.usd.get_context().get_stage()
    except Exception:
        return None


def _iter_descendants(prim) -> Iterable[Any]:
    for child in _children(prim):
        yield child
        yield from _iter_descendants(child)


def _children(prim) -> list[Any]:
    get_children = getattr(prim, "GetChildren", None)
    if not callable(get_children):
        return []
    return list(get_children())


def _prim_is_valid(prim) -> bool:
    if prim is None:
        return False
    is_valid = getattr(prim, "IsValid", None)
    return bool(is_valid()) if callable(is_valid) else bool(prim)


def _prim_name(prim) -> str:
    get_name = getattr(prim, "GetName", None)
    if callable(get_name):
        return str(get_name())
    path = _prim_path(prim)
    return path.rsplit("/", 1)[-1] if path else ""


def _prim_path(prim) -> str:
    get_path = getattr(prim, "GetPath", None)
    if callable(get_path):
        return str(get_path())
    return str(getattr(prim, "path", ""))


def _type_name(prim) -> str | None:
    get_type_name = getattr(prim, "GetTypeName", None)
    if callable(get_type_name):
        type_name = str(get_type_name())
        return type_name or None
    return getattr(prim, "type_name", None)


def _applied_schemas(prim) -> list[str]:
    get_applied_schemas = getattr(prim, "GetAppliedSchemas", None)
    if callable(get_applied_schemas):
        return [str(schema) for schema in get_applied_schemas()]
    return [str(schema) for schema in getattr(prim, "applied_schemas", [])]


def _classification_text(prim) -> str:
    return " ".join(
        [
            _prim_name(prim).lower(),
            _prim_path(prim).lower(),
            (_type_name(prim) or "").lower(),
            " ".join(schema.lower() for schema in _applied_schemas(prim)),
        ]
    )


def _is_link_like(prim) -> bool:
    text = _classification_text(prim)
    return "link" in text or "rigidbodyapi" in text or "rigid_body" in text


def _is_joint_like(prim) -> bool:
    name = _prim_name(prim).lower()
    type_name = (_type_name(prim) or "").lower()
    schemas = " ".join(schema.lower() for schema in _applied_schemas(prim))
    return "joint" in type_name or "joint" in schemas or name.endswith("_joint") or "_joint_" in name


def _is_visual_like(prim) -> bool:
    text = _classification_text(prim)
    type_name = (_type_name(prim) or "").lower()
    return "visual" in text or type_name in {"mesh", "cube", "sphere", "cylinder", "capsule"}


def _is_collision_like(prim) -> bool:
    text = _classification_text(prim)
    return "collision" in text or "collisionapi" in text


def _is_articulation_root_like(prim) -> bool:
    text = _classification_text(prim)
    return "articulationrootapi" in text or "articulation_root" in text


def _is_physics_schema(schema: str) -> bool:
    lowered = schema.lower()
    return any(
        keyword in lowered
        for keyword in (
            "physics",
            "physx",
            "rigidbody",
            "collision",
            "articulation",
            "joint",
            "drive",
        )
    )
