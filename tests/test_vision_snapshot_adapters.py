import importlib


def test_camera_snapshot_package_exports_public_symbols():
    module = importlib.import_module("src.vision.snapshot.camera_snapshot")

    _assert_public_symbols(
        module,
        [
            "CONTRACT_VERSION",
            "CURRENT_CAMERA_SNAPSHOT_VERSION",
            "STATUS_PASS",
            "STATUS_BLOCKED",
            "STATUS_NOT_REQUESTED",
            "FORMAL_REALSENSE_SOURCES",
            "E_CAMERA_SNAPSHOT_INVALID",
            "E_SCENE_VERSION_MISSING",
            "E_CAPTURE_TIMESTAMP_MISSING",
            "E_CAMERA_SNAPSHOT_STALE",
            "E_CAMERA_FRAME_MISSING",
            "E_IMAGE_REF_MISSING",
            "E_DEPTH_REF_MISSING",
            "E_ALIGNED_DEPTH_REQUIRED",
            "E_CAMERA_INFO_REF_MISSING",
            "E_METADATA_REF_MISSING",
            "E_TF_SNAPSHOT_REF_MISSING",
            "E_LIVE_CAMERA_DISABLED",
            "E_ROBOT_COMMAND_NOT_ALLOWED",
            "FORBIDDEN_ROBOT_CONTROL_FIELDS",
            "SNAPSHOT_FIELDS",
            "CameraSnapshotRequest",
            "load_camera_snapshot_config",
            "build_camera_snapshot_request",
            "evaluate_formal_snapshot_replay",
            "evaluate_camera_snapshot_contract",
            "format_camera_snapshot_report",
        ],
    )


def test_camera_source_adapter_package_exports_public_symbols():
    module = importlib.import_module("src.vision.snapshot.camera_source_adapter")

    _assert_public_symbols(
        module,
        [
            "CONTRACT_VERSION",
            "CURRENT_CAMERA_SOURCE_VERSION",
            "STATUS_PASS",
            "STATUS_BLOCKED",
            "STATUS_SAFE_DISABLED",
            "STATUS_NOT_REQUESTED",
            "MODE_OFFLINE_FILE",
            "MODE_MANUAL_SNAPSHOT",
            "MODE_LIVE_DISABLED",
            "MODE_REALSENSE_REPLAY",
            "MODE_REALSENSE_ONE_SHOT",
            "E_LIVE_CAMERA_CAPTURE_NOT_ALLOWED",
            "E_CONTINUOUS_CAPTURE_DISABLED",
            "E_CAMERA_BACKEND_UNAVAILABLE",
            "E_IMAGE_REF_MISSING",
            "E_CAMERA_INFO_MISSING",
            "E_CAMERA_FRAME_MISSING",
            "E_CAPTURE_TIMESTAMP_MISSING",
            "E_LIVE_VLM_DISABLED",
            "E_ROBOT_COMMAND_NOT_ALLOWED",
            "E_UNSUPPORTED_SOURCE_MODE",
            "FORBIDDEN_ROBOT_CONTROL_FIELDS",
            "CAMERA_SOURCE_FIELDS",
            "CameraSourceAdapterRequest",
            "load_camera_source_config",
            "build_camera_source_adapter_request",
            "evaluate_camera_source_adapter",
            "format_camera_source_report",
        ],
    )


def test_realsense_snapshot_builder_package_exports_public_symbols():
    module = importlib.import_module("src.vision.snapshot.realsense_snapshot_builder")

    _assert_public_symbols(
        module,
        [
            "RealSenseSnapshotBundleRequest",
            "SnapshotBundleError",
            "build_realsense_snapshot_bundle",
        ],
    )


def test_vision_snapshot_package_root_remains_conservative():
    package_root = importlib.import_module("src.vision.snapshot")

    for symbol in (
        "CameraSnapshotRequest",
        "CameraSourceAdapterRequest",
        "RealSenseSnapshotBundleRequest",
        "evaluate_camera_snapshot_contract",
        "evaluate_camera_source_adapter",
        "build_realsense_snapshot_bundle",
    ):
        assert not hasattr(package_root, symbol)


def _assert_public_symbols(module, names):
    assert module.__all__ == names
    for name in names:
        assert hasattr(module, name)
