from src.robot_prim_inspector import (
    INSPECTION_STATUS_OK,
    INSPECTION_STATUS_PRIM_NOT_FOUND,
    build_robot_prim_inspection_report,
    inspect_robot_prim,
)


class FakePrim:
    def __init__(self, path, type_name="Xform", schemas=None, children=None, valid=True):
        self.path = path
        self.type_name = type_name
        self.applied_schemas = schemas or []
        self.children = children or []
        self.valid = valid

    def GetPath(self):
        return self.path

    def GetName(self):
        return self.path.rsplit("/", 1)[-1]

    def GetTypeName(self):
        return self.type_name

    def GetAppliedSchemas(self):
        return self.applied_schemas

    def GetChildren(self):
        return self.children

    def IsValid(self):
        return self.valid


class FakeStage:
    def __init__(self, prims):
        self.prims = prims

    def GetPrimAtPath(self, path):
        return self.prims.get(path, FakePrim(path, valid=False))


def test_inspect_robot_prim_returns_not_found_without_crashing():
    report = inspect_robot_prim(stage=FakeStage({}), robot_prim_path="/World/TETO_Robot")

    assert report["requested"] is True
    assert report["robot_prim_path"] == "/World/TETO_Robot"
    assert report["robot_prim_exists"] is False
    assert report["inspection_status"] == INSPECTION_STATUS_PRIM_NOT_FOUND
    assert report["total_descendant_prim_count"] == 0


def test_inspect_robot_prim_summarizes_read_only_structure():
    collision = FakePrim(
        "/World/TETO_Robot/base_link/collisions/base_collision",
        type_name="Mesh",
        schemas=["PhysicsCollisionAPI"],
    )
    visual = FakePrim("/World/TETO_Robot/base_link/visuals/base_visual", type_name="Mesh")
    link = FakePrim(
        "/World/TETO_Robot/base_link",
        schemas=["PhysicsRigidBodyAPI"],
        children=[visual, collision],
    )
    joint = FakePrim(
        "/World/TETO_Robot/shoulder_pan_joint",
        type_name="PhysicsRevoluteJoint",
        schemas=["PhysicsJointAPI"],
    )
    root = FakePrim(
        "/World/TETO_Robot",
        schemas=["PhysicsArticulationRootAPI"],
        children=[link, joint],
    )

    report = inspect_robot_prim(stage=FakeStage({"/World/TETO_Robot": root}), robot_prim_path="/World/TETO_Robot")

    assert report["inspection_status"] == INSPECTION_STATUS_OK
    assert report["robot_prim_exists"] is True
    assert report["robot_root_type_name"] == "Xform"
    assert report["total_descendant_prim_count"] == 4
    assert report["link_like_prim_count"] >= 1
    assert report["joint_like_prim_count"] == 1
    assert report["visual_like_prim_count"] >= 1
    assert report["collision_like_prim_count"] >= 1
    assert report["articulation_root_found"] is True
    assert report["joint_names"] == ["shoulder_pan_joint"]
    assert report["joint_prim_paths"] == ["/World/TETO_Robot/shoulder_pan_joint"]
    assert report["possible_dof_count"] == 1
    assert "PhysicsJointAPI" in report["physics_schema_summary"]


def test_robot_prim_inspection_report_has_no_motion_control_fields():
    report = build_robot_prim_inspection_report(
        requested=True,
        robot_prim_path="/World/TETO_Robot",
        robot_prim_exists=True,
        joint_names=["shoulder_pan_joint"],
    )

    serialized_keys = " ".join(report.keys())
    for forbidden in (
        "joint_target",
        "joint_angles",
        "tcp_pose_world",
        "actual_TCP_pose",
        "urscript",
        "rtde",
        "moveit",
        "ros2",
    ):
        assert forbidden not in serialized_keys

    for value in report.values():
        assert "command" not in str(value).lower()

