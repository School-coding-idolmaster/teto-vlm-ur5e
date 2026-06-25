import src.projector as projector_package
import src.projector.shadow as current_shadow
import src.projector_shadow as legacy_shadow


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


def test_projector_shadow_legacy_shim_preserves_key_object_identity():
    names = [
        "CONTRACT_VERSION",
        "CURRENT_PROJECTOR_SHADOW_VERSION",
        "STATUS_PASS",
        "E_NO_DEPTH",
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
        assert getattr(legacy_shadow, name) is getattr(current_shadow, name)
