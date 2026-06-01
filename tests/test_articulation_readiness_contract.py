from src.articulation_readiness_contract import (
    READINESS_STATUS_NOT_READY,
    READINESS_STATUS_READY,
    build_articulation_readiness_report,
)


STANDARD_UR5E_INSPECTION = {
    "requested": True,
    "robot_prim_path": "/World/TETO_Robot",
    "robot_prim_exists": True,
    "articulation_root_found": True,
    "visual_like_prim_count": 7,
    "collision_like_prim_count": 7,
    "joint_names": [
        "robot_gripper_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        "root_joint",
    ],
    "joint_metadata_summary": {
        "arm_joint_count": 6,
        "arm_joint_names": [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
        "possible_dof_names": [
            "robot_gripper_joint",
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "root_joint",
        ],
    },
}


def test_articulation_readiness_not_ready_when_robot_prim_missing():
    report = build_articulation_readiness_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection={"robot_prim_exists": False},
    )

    assert report["readiness_status"] == READINESS_STATUS_NOT_READY
    assert report["articulation_ready"] is False
    assert "robot_prim" in report["missing_requirements"]
    assert report["control_enabled"] is False
    assert report["motion_generated"] is False
    assert report["command_generated"] is False


def test_articulation_readiness_not_ready_without_articulation_root():
    inspection = dict(STANDARD_UR5E_INSPECTION)
    inspection["articulation_root_found"] = False

    report = build_articulation_readiness_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection=inspection,
    )

    assert report["readiness_status"] == READINESS_STATUS_NOT_READY
    assert "articulation_root" in report["missing_requirements"]


def test_articulation_readiness_not_ready_with_missing_arm_joints():
    inspection = dict(STANDARD_UR5E_INSPECTION)
    inspection["joint_metadata_summary"] = {
        "arm_joint_names": ["shoulder_pan_joint", "shoulder_lift_joint"],
        "possible_dof_names": ["shoulder_pan_joint", "shoulder_lift_joint"],
    }

    report = build_articulation_readiness_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection=inspection,
    )

    assert report["readiness_status"] == READINESS_STATUS_NOT_READY
    assert report["arm_joint_count"] == 2
    assert "six_standard_ur5e_arm_joints" in report["missing_requirements"]
    assert "elbow_joint" in report["missing_arm_joint_names"]


def test_articulation_readiness_ready_for_standard_ur5e_metadata():
    report = build_articulation_readiness_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_inspection=STANDARD_UR5E_INSPECTION,
        has_robot_structure_report=True,
    )

    assert report["readiness_status"] == READINESS_STATUS_READY
    assert report["articulation_ready"] is True
    assert report["arm_joint_count"] == 6
    assert report["required_arm_joint_count"] == 6
    assert report["missing_arm_joint_names"] == []
    assert report["missing_requirements"] == []
    assert report["has_visual_prims"] is True
    assert report["has_collision_prims"] is True
    assert report["has_robot_structure_report"] is True
    assert report["control_enabled"] is False
    assert report["motion_generated"] is False
    assert report["command_generated"] is False
    assert report["safety_boundary"]["read_only"] is True
    assert report["safety_boundary"]["no_robot_motion"] is True
    assert report["safety_boundary"]["no_joint_targets"] is True
    assert report["safety_boundary"]["no_tcp_pose_world"] is True
    assert report["safety_boundary"]["no_ros2_moveit_rtde_urscript"] is True
