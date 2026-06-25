import src.projector as projector_package
import src.projector.shadow as current_shadow
from src.projector.shadow import (
    CONTRACT_VERSION,
    E_NO_DEPTH,
    CURRENT_PROJECTOR_SHADOW_VERSION,
    FORBIDDEN_ROBOT_CONTROL_FIELDS,
    PROJECTOR_FIELDS,
    STATUS_PASS,
    ProjectorShadowRequest,
    build_projector_shadow_request,
    evaluate_projector_shadow,
    evaluate_projector_shadow_from_contracts,
    format_projector_shadow_report,
    load_projector_shadow_config,
)


def test_projector_package_root_keeps_empty_export_policy():
    assert projector_package.__all__ == []
    assert not hasattr(projector_package, "evaluate_projector_shadow")


def test_projector_shadow_current_import_exports_public_api():
    names = [
        "CONTRACT_VERSION",
        "CURRENT_PROJECTOR_SHADOW_VERSION",
        "STATUS_PASS",
        "STATUS_BLOCKED",
        "STATUS_NOT_REQUESTED",
        "E_GEOMETRY_NOT_VALID",
        "E_PIXEL_CENTER_MISSING",
        "E_CAMERA_INFO_MISSING",
        "E_INVALID_CAMERA_INTRINSICS",
        "E_NO_DEPTH",
        "E_INVALID_DEPTH",
        "E_DEPTH_OUT_OF_RANGE",
        "E_CAMERA_FRAME_MISSING",
        "E_WORLD_FRAME_MISSING",
        "E_TF_UNAVAILABLE",
        "E_INVALID_PROJECTION",
        "E_OUT_OF_WORKSPACE",
        "E_LIVE_CAMERA_DISABLED",
        "E_LIVE_VLM_DISABLED",
        "E_ROBOT_COMMAND_NOT_ALLOWED",
        "DEFAULT_MIN_DEPTH_M",
        "DEFAULT_MAX_DEPTH_M",
        "DEFAULT_PROJECTION_METHOD",
        "FORBIDDEN_ROBOT_CONTROL_FIELDS",
        "PROJECTOR_FIELDS",
        "ProjectorShadowRequest",
        "load_projector_shadow_config",
        "build_projector_shadow_request",
        "evaluate_projector_shadow",
        "evaluate_projector_shadow_from_contracts",
        "format_projector_shadow_report",
    ]

    for name in names:
        assert hasattr(current_shadow, name)


def test_projector_shadow_current_import_preserves_key_object_identity():
    assert CONTRACT_VERSION is current_shadow.CONTRACT_VERSION
    assert CURRENT_PROJECTOR_SHADOW_VERSION is current_shadow.CURRENT_PROJECTOR_SHADOW_VERSION
    assert STATUS_PASS is current_shadow.STATUS_PASS
    assert E_NO_DEPTH is current_shadow.E_NO_DEPTH
    assert FORBIDDEN_ROBOT_CONTROL_FIELDS is current_shadow.FORBIDDEN_ROBOT_CONTROL_FIELDS
    assert PROJECTOR_FIELDS is current_shadow.PROJECTOR_FIELDS
    assert ProjectorShadowRequest is current_shadow.ProjectorShadowRequest
    assert load_projector_shadow_config is current_shadow.load_projector_shadow_config
    assert build_projector_shadow_request is current_shadow.build_projector_shadow_request
    assert evaluate_projector_shadow is current_shadow.evaluate_projector_shadow
    assert evaluate_projector_shadow_from_contracts is current_shadow.evaluate_projector_shadow_from_contracts
    assert format_projector_shadow_report is current_shadow.format_projector_shadow_report
