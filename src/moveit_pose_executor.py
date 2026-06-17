from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict


CONTRACT_VERSION = "teto_moveit_pose_executor.v1"

STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_NOT_REQUESTED = "NOT_REQUESTED"

DEFAULT_FRAME = "base_link"
DEFAULT_PLANNING_GROUP = "ur_manipulator"
DEFAULT_END_EFFECTOR_LINK = "tool0"
DEFAULT_MOVE_GROUP_ACTION = "/move_action"
DEFAULT_EXECUTE_TRAJECTORY_ACTION = "/execute_trajectory"
DEFAULT_MAX_TRANSLATION_M = 0.20
DEFAULT_HARD_SAFETY_LIMIT_M = DEFAULT_MAX_TRANSLATION_M
DEFAULT_POSITION_TOLERANCE_M = 0.002
DEFAULT_ORIENTATION_TOLERANCE_RAD = 0.01
REAL_MOTION_TOLERANCE_DISTANCE_RATIO = 0.10
MOTION_LIMIT_EPS = 1e-9
DEFAULT_WORKSPACE_BOUNDS = {
    "x": [-1.0, 1.0],
    "y": [-1.0, 1.0],
    "z": [0.0, 2.0],
}
ALLOWED_FRAMES = {"base_link"}
PLANNER_MODE_JOINT_SPACE_POSE_GOAL = "joint_space_pose_goal"
MOVEIT_GOAL_TYPE_POSE_CONSTRAINTS = "move_group_pose_goal_constraints"
START_STATE_SOURCE_IMPLICIT_PLANNING_SCENE = "implicit_planning_scene"
WRIST_JOINT_NAMES = ("wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
SUSPICIOUS_WRIST_JOINT_DELTA_RAD = 1.0

E_TARGET_POSE_MISSING = "E_TARGET_POSE_MISSING"
E_INVALID_TARGET_POSE = "E_INVALID_TARGET_POSE"
E_INVALID_FRAME = "E_INVALID_FRAME"
E_CURRENT_TCP_POSE_MISSING = "E_CURRENT_TCP_POSE_MISSING"
E_INVALID_CURRENT_TCP_POSE = "E_INVALID_CURRENT_TCP_POSE"
E_EXCESSIVE_CARTESIAN_MOTION = "E_EXCESSIVE_CARTESIAN_MOTION"
E_OUT_OF_WORKSPACE = "E_OUT_OF_WORKSPACE"
E_ROBOT_STATE_NOT_OK = "E_ROBOT_STATE_NOT_OK"
E_SAFETY_STATUS_NOT_OK = "E_SAFETY_STATUS_NOT_OK"
E_PROTECTIVE_STOP_ACTIVE = "E_PROTECTIVE_STOP_ACTIVE"
E_EMERGENCY_STOP_ACTIVE = "E_EMERGENCY_STOP_ACTIVE"
E_SPEED_SCALING_UNSAFE = "E_SPEED_SCALING_UNSAFE"
E_MANUAL_CONFIRMATION_REQUIRED = "E_MANUAL_CONFIRMATION_REQUIRED"
E_FORBIDDEN_CONTROL_ARTIFACT = "E_FORBIDDEN_CONTROL_ARTIFACT"
E_ROS2_IMPORT_FAILED = "E_ROS2_IMPORT_FAILED"
E_MOVEIT_ACTION_SERVER_UNAVAILABLE = "E_MOVEIT_ACTION_SERVER_UNAVAILABLE"
E_MOVEIT_GOAL_REJECTED = "E_MOVEIT_GOAL_REJECTED"
E_MOVEIT_PLAN_FAILED = "E_MOVEIT_PLAN_FAILED"
E_MOVEIT_EXECUTE_FAILED = "E_MOVEIT_EXECUTE_FAILED"
E_MOVEIT_API_ERROR = "E_MOVEIT_API_ERROR"
E_SMALL_MOTION_TOLERANCE_POLICY_VIOLATION = "E_SMALL_MOTION_TOLERANCE_POLICY_VIOLATION"


@dataclass(frozen=True)
class MoveItPoseExecutorRequest:
    requested: bool = False
    target_pose: Dict[str, Any] | None = None
    current_tcp_pose: Dict[str, Any] | list[float] | None = None
    config: Dict[str, Any] | None = None
    execute: bool = False
    manual_confirmation_result: Dict[str, Any] | None = None
    robot_state_result: Dict[str, Any] | None = None


def evaluate_moveit_pose_plan(request: MoveItPoseExecutorRequest | None = None) -> Dict[str, Any]:
    request = request or MoveItPoseExecutorRequest()
    if not request.requested:
        return _not_requested(plan=True)

    config = request.config if isinstance(request.config, dict) else {}
    target_pose = _normalize_pose(request.target_pose)
    current_pose = _normalize_pose(request.current_tcp_pose if request.current_tcp_pose is not None else config.get("current_tcp_pose"))
    validation = _validate_request(
        target_pose=target_pose,
        current_pose=current_pose,
        config=config,
        robot_state=request.robot_state_result,
        execute=False,
        confirmation=request.manual_confirmation_result,
    )
    if validation["blocking_reasons"]:
        return _blocked_result(plan=True, validation=validation, config=config)
    config = _config_with_resolved_tolerance(config, validation)

    try:
        api_result = _plan_with_move_group_action(target_pose, config)
    except ImportError as exc:
        return _api_blocked(plan=True, config=config, reason=E_ROS2_IMPORT_FAILED, error=str(exc), validation=validation)
    except Exception as exc:  # pragma: no cover - exercised against the lab ROS graph.
        return _api_blocked(plan=True, config=config, reason=E_MOVEIT_API_ERROR, error=str(exc), validation=validation)

    blocking_reasons = []
    if api_result.get("action_server_available") is not True:
        blocking_reasons.append(E_MOVEIT_ACTION_SERVER_UNAVAILABLE)
    elif api_result.get("goal_accepted") is not True:
        blocking_reasons.append(E_MOVEIT_GOAL_REJECTED)
    elif api_result.get("success") is not True:
        blocking_reasons.append(E_MOVEIT_PLAN_FAILED)

    status = STATUS_PASS if not blocking_reasons else STATUS_BLOCKED
    plan_audit = _plan_audit_result(config=config, validation=validation, api_result=api_result)
    return {
        **_common_result(config=config, target_pose=target_pose, validation=validation),
        **plan_audit,
        "moveit_pose_plan_requested": True,
        "moveit_pose_execute_requested": False,
        "moveit_pose_executor_status": status,
        "plan_success": api_result.get("success") is True,
        "execute_success": False,
        "moveit_plan_called": api_result.get("action_call_attempted") is True,
        "moveit_execute_called": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "real_robot_motion_executed": False,
        "action_server_available": api_result.get("action_server_available") is True,
        "goal_accepted": api_result.get("goal_accepted") is True,
        "moveit_error_code": api_result.get("error_code"),
        "moveit_error_code_name": api_result.get("error_code_name"),
        "planning_time_s": api_result.get("planning_time_s"),
        "trajectory_point_count": api_result.get("trajectory_point_count", 0),
        "blocking_reasons": _unique(validation["blocking_reasons"] + blocking_reasons),
        "warnings": _unique(validation["warnings"] + _string_list(api_result.get("warnings"))),
    }


def evaluate_moveit_pose_execute(request: MoveItPoseExecutorRequest | None = None) -> Dict[str, Any]:
    request = request or MoveItPoseExecutorRequest()
    if not request.requested:
        return _not_requested(plan=False)

    config = request.config if isinstance(request.config, dict) else {}
    target_pose = _normalize_pose(request.target_pose)
    current_pose = _normalize_pose(request.current_tcp_pose if request.current_tcp_pose is not None else config.get("current_tcp_pose"))
    validation = _validate_request(
        target_pose=target_pose,
        current_pose=current_pose,
        config=config,
        robot_state=request.robot_state_result,
        execute=True,
        confirmation=request.manual_confirmation_result,
    )
    if validation["blocking_reasons"]:
        return _blocked_result(plan=False, validation=validation, config=config)
    config = _config_with_resolved_tolerance(config, validation)

    try:
        plan_result = _plan_with_move_group_action(target_pose, config)
    except ImportError as exc:
        return _api_blocked(plan=False, config=config, reason=E_ROS2_IMPORT_FAILED, error=str(exc), validation=validation)
    except Exception as exc:  # pragma: no cover - exercised against the lab ROS graph.
        return _api_blocked(plan=False, config=config, reason=E_MOVEIT_API_ERROR, error=str(exc), validation=validation)

    blocking_reasons = []
    if plan_result.get("action_server_available") is not True:
        blocking_reasons.append(E_MOVEIT_ACTION_SERVER_UNAVAILABLE)
    elif plan_result.get("goal_accepted") is not True:
        blocking_reasons.append(E_MOVEIT_GOAL_REJECTED)
    elif plan_result.get("success") is not True:
        blocking_reasons.append(E_MOVEIT_PLAN_FAILED)

    execute_result = {
        "action_call_attempted": False,
        "action_server_available": False,
        "goal_accepted": False,
        "success": False,
        "error_code": None,
        "error_code_name": None,
    }
    if not blocking_reasons:
        try:
            execute_result = _execute_trajectory_action(plan_result.get("planned_trajectory"), config)
        except ImportError as exc:
            blocking_reasons.append(E_ROS2_IMPORT_FAILED)
            execute_result["error"] = str(exc)
        except Exception as exc:  # pragma: no cover - exercised against the lab ROS graph.
            blocking_reasons.append(E_MOVEIT_API_ERROR)
            execute_result["error"] = str(exc)

    if not blocking_reasons:
        if execute_result.get("action_server_available") is not True:
            blocking_reasons.append(E_MOVEIT_ACTION_SERVER_UNAVAILABLE)
        elif execute_result.get("goal_accepted") is not True:
            blocking_reasons.append(E_MOVEIT_GOAL_REJECTED)
        elif execute_result.get("success") is not True:
            blocking_reasons.append(E_MOVEIT_EXECUTE_FAILED)

    execute_success = not blocking_reasons and execute_result.get("success") is True
    status = STATUS_PASS if execute_success else STATUS_BLOCKED
    plan_audit = _plan_audit_result(config=config, validation=validation, api_result=plan_result)
    return {
        **_common_result(config=config, target_pose=target_pose, validation=validation),
        **plan_audit,
        "moveit_pose_plan_requested": True,
        "moveit_pose_execute_requested": True,
        "moveit_pose_executor_status": status,
        "plan_success": plan_result.get("success") is True,
        "execute_success": execute_success,
        "moveit_plan_called": plan_result.get("action_call_attempted") is True,
        "moveit_execute_called": execute_result.get("action_call_attempted") is True,
        "trajectory_send_allowed": True,
        "trajectory_sent": execute_result.get("action_call_attempted") is True,
        "controller_command_sent": execute_result.get("action_call_attempted") is True,
        "real_robot_motion_executed": execute_success,
        "action_server_available": (
            plan_result.get("action_server_available") is True
            and execute_result.get("action_server_available") is True
        ),
        "goal_accepted": plan_result.get("goal_accepted") is True and execute_result.get("goal_accepted") is True,
        "moveit_plan_error_code": plan_result.get("error_code"),
        "moveit_plan_error_code_name": plan_result.get("error_code_name"),
        "moveit_execute_error_code": execute_result.get("error_code"),
        "moveit_execute_error_code_name": execute_result.get("error_code_name"),
        "planning_time_s": plan_result.get("planning_time_s"),
        "trajectory_point_count": plan_result.get("trajectory_point_count", 0),
        "blocking_reasons": _unique(validation["blocking_reasons"] + blocking_reasons),
        "warnings": _unique(validation["warnings"] + _string_list(plan_result.get("warnings")) + _string_list(execute_result.get("warnings"))),
    }


def _validate_request(
    *,
    target_pose: Dict[str, Any] | None,
    current_pose: Dict[str, Any] | None,
    config: Dict[str, Any],
    robot_state: Dict[str, Any] | None,
    execute: bool,
    confirmation: Dict[str, Any] | None,
) -> Dict[str, Any]:
    blocking_reasons: list[str] = []
    warnings: list[str] = []
    allowed_frames = _allowed_frames(config)
    workspace_bounds = _workspace_bounds(config)
    max_translation_m = _optional_float(config.get("max_translation_m")) or DEFAULT_MAX_TRANSLATION_M
    hard_safety_limit_m = _optional_float(config.get("hard_safety_limit_m")) or DEFAULT_HARD_SAFETY_LIMIT_M
    configured_max_distance_m = _optional_float(config.get("configured_max_distance_m")) or max_translation_m

    if not target_pose:
        blocking_reasons.append(E_TARGET_POSE_MISSING)
    elif not _valid_pose(target_pose, allowed_frames):
        blocking_reasons.append(E_INVALID_TARGET_POSE)
        if _string(target_pose.get("frame")) not in allowed_frames:
            blocking_reasons.append(E_INVALID_FRAME)

    if current_pose is None:
        blocking_reasons.append(E_CURRENT_TCP_POSE_MISSING)
    elif not _valid_pose(current_pose, allowed_frames):
        blocking_reasons.append(E_INVALID_CURRENT_TCP_POSE)

    translation_distance_m = None
    current_position_m = None
    target_position_m = None
    if target_pose and current_pose and _valid_pose(target_pose, allowed_frames) and _valid_pose(current_pose, allowed_frames):
        current_position_m = list(current_pose["position_m"])
        target_position_m = list(target_pose["position_m"])
        translation_distance_m = _distance_between(current_pose["position_m"], target_pose["position_m"])
        if translation_distance_m > hard_safety_limit_m + MOTION_LIMIT_EPS:
            blocking_reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)
        elif translation_distance_m > max_translation_m + MOTION_LIMIT_EPS:
            blocking_reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)
        if not _point_in_workspace(target_pose["position_m"], workspace_bounds):
            blocking_reasons.append(E_OUT_OF_WORKSPACE)

    tolerance = _resolved_tolerance_policy(config, translation_distance_m)
    if (
        tolerance["small_motion_tolerance_policy_enabled"] is True
        and tolerance["requested_distance_m"] is not None
        and tolerance["position_tolerance_policy_limit_m"] is not None
        and (
            tolerance["moveit_position_tolerance_m"] > tolerance["position_tolerance_policy_limit_m"] + MOTION_LIMIT_EPS
            or tolerance["moveit_position_tolerance_m"] >= tolerance["requested_distance_m"] - MOTION_LIMIT_EPS
        )
    ):
        blocking_reasons.append(E_SMALL_MOTION_TOLERANCE_POLICY_VIOLATION)

    require_robot_state = execute or config.get("require_robot_state_for_plan") is True
    if require_robot_state:
        blocking_reasons.extend(_robot_state_blockers(config, robot_state if isinstance(robot_state, dict) else {}))

    if execute:
        if config.get("manual_confirmation_required", True) is not True:
            blocking_reasons.append(E_MANUAL_CONFIRMATION_REQUIRED)
        if not isinstance(confirmation, dict) or confirmation.get("manual_confirmation_accepted") is not True:
            blocking_reasons.append(E_MANUAL_CONFIRMATION_REQUIRED)

    if _forbidden_artifact(config):
        blocking_reasons.append(E_FORBIDDEN_CONTROL_ARTIFACT)
        warnings.append("forbidden_control_artifact_detected")

    return {
        "blocking_reasons": _unique(blocking_reasons),
        "warnings": _unique(warnings),
        "workspace_bounds": workspace_bounds,
        "max_translation_m": max_translation_m,
        "configured_max_distance_m": configured_max_distance_m,
        "hard_safety_limit_m": hard_safety_limit_m,
        "translation_distance_m": round(float(translation_distance_m), 6) if translation_distance_m is not None else None,
        "motion_check_source": "moveit_pose_executor",
        "motion_check_current_position_m": current_position_m,
        "motion_check_target_position_m": target_position_m,
        "current_tcp_frame": current_pose.get("frame") if isinstance(current_pose, dict) else None,
        "motion_check_distance_m": round(float(translation_distance_m), 6) if translation_distance_m is not None else None,
        "motion_check_max_distance_m": max_translation_m,
        "motion_check_hard_limit_m": hard_safety_limit_m,
        "motion_check_eps": MOTION_LIMIT_EPS,
        "requested_distance_within_configured_limit": (
            translation_distance_m is not None
            and translation_distance_m <= configured_max_distance_m + MOTION_LIMIT_EPS
        ),
        "safety_policy_source": _string(config.get("safety_policy_source")),
        "safety_policy_name": _string(config.get("safety_policy_name")),
        "workspace_check_passed": bool(target_pose and _point_in_workspace(target_pose["position_m"], workspace_bounds)),
        "requested_start_tcp_pose": current_pose,
        "requested_target_tcp_pose": target_pose,
        **_target_orientation_evidence(target_pose=target_pose, current_pose=current_pose, config=config),
        **tolerance,
    }


def _plan_with_move_group_action(target_pose: Dict[str, Any], config: Dict[str, Any]) -> Dict[str, Any]:
    rclpy, ActionClient = _import_ros_action_client()
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes

    action_name = _string(config.get("move_group_action_name")) or DEFAULT_MOVE_GROUP_ACTION
    node_name = _string(config.get("node_name")) or "teto_moveit_pose_planner"
    server_timeout_s = _optional_float(config.get("action_server_timeout_s")) or 5.0
    result_timeout_s = _optional_float(config.get("action_result_timeout_s")) or 30.0
    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True
    node = rclpy.create_node(node_name)
    client = ActionClient(node, MoveGroup, action_name)
    try:
        if not client.wait_for_server(timeout_sec=server_timeout_s):
            return _action_unavailable(action_name)
        goal = _build_move_group_goal(target_pose, config)
        if config.get("debug_moveit") is True:
            _debug_print_move_group_goal(goal, target_pose)
        send_future = client.send_goal_async(goal)
        if not _spin_until_future(rclpy, node, send_future, server_timeout_s):
            return _goal_timeout(action_name)
        goal_handle = send_future.result()
        accepted = bool(goal_handle and goal_handle.accepted)
        if not accepted:
            return _goal_rejected(action_name)
        result_future = goal_handle.get_result_async()
        if not _spin_until_future(rclpy, node, result_future, result_timeout_s):
            return _result_timeout(action_name)
        result_message = result_future.result().result
        error_code = int(result_message.error_code.val)
        planned_trajectory = result_message.planned_trajectory
        point_count = _trajectory_point_count(planned_trajectory)
        if config.get("debug_moveit") is True:
            _debug_print_move_group_result(error_code, result_message.planning_time, point_count)
        return {
            "action_name": action_name,
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": error_code == MoveItErrorCodes.SUCCESS and point_count > 0,
            "error_code": error_code,
            "error_code_name": _moveit_error_code_name(error_code),
            "planning_time_s": float(result_message.planning_time),
            "trajectory_point_count": point_count,
            "planned_trajectory": planned_trajectory,
        }
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()


def _execute_trajectory_action(planned_trajectory: Any, config: Dict[str, Any]) -> Dict[str, Any]:
    rclpy, ActionClient = _import_ros_action_client()
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import MoveItErrorCodes

    action_name = _string(config.get("execute_trajectory_action_name")) or DEFAULT_EXECUTE_TRAJECTORY_ACTION
    node_name = _string(config.get("execute_node_name")) or "teto_moveit_pose_executor"
    server_timeout_s = _optional_float(config.get("action_server_timeout_s")) or 5.0
    result_timeout_s = _optional_float(config.get("execute_result_timeout_s")) or 60.0
    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True
    node = rclpy.create_node(node_name)
    client = ActionClient(node, ExecuteTrajectory, action_name)
    try:
        if not client.wait_for_server(timeout_sec=server_timeout_s):
            return _action_unavailable(action_name)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = planned_trajectory
        send_future = client.send_goal_async(goal)
        if not _spin_until_future(rclpy, node, send_future, server_timeout_s):
            return _goal_timeout(action_name)
        goal_handle = send_future.result()
        accepted = bool(goal_handle and goal_handle.accepted)
        if not accepted:
            return _goal_rejected(action_name)
        result_future = goal_handle.get_result_async()
        if not _spin_until_future(rclpy, node, result_future, result_timeout_s):
            return _result_timeout(action_name)
        result_message = result_future.result().result
        error_code = int(result_message.error_code.val)
        return {
            "action_name": action_name,
            "action_call_attempted": True,
            "action_server_available": True,
            "goal_accepted": True,
            "success": error_code == MoveItErrorCodes.SUCCESS,
            "error_code": error_code,
            "error_code_name": _moveit_error_code_name(error_code),
        }
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()


def _build_move_group_goal(target_pose: Dict[str, Any], config: Dict[str, Any]) -> Any:
    from geometry_msgs.msg import Pose
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
    from shape_msgs.msg import SolidPrimitive

    goal = MoveGroup.Goal()
    request = goal.request
    frame = _string(target_pose.get("frame")) or DEFAULT_FRAME
    position = target_pose["position_m"]
    orientation = target_pose["orientation_xyzw"]
    workspace = _workspace_bounds(config)

    request.group_name = _string(config.get("planning_group")) or DEFAULT_PLANNING_GROUP
    request.num_planning_attempts = int(config.get("num_planning_attempts", 5))
    request.allowed_planning_time = float(_optional_float(config.get("allowed_planning_time_s")) or 5.0)
    request.max_velocity_scaling_factor = float(_optional_float(config.get("max_speed_scale")) or 0.10)
    request.max_acceleration_scaling_factor = float(_optional_float(config.get("max_acc_scale")) or 0.10)
    request.start_state.is_diff = True
    pipeline_id = _string(config.get("pipeline_id"))
    if pipeline_id:
        request.pipeline_id = pipeline_id
    planner_id = _string(config.get("planner_id"))
    if planner_id:
        request.planner_id = planner_id
    request.workspace_parameters.header.frame_id = frame
    request.workspace_parameters.min_corner.x = workspace["x"][0]
    request.workspace_parameters.min_corner.y = workspace["y"][0]
    request.workspace_parameters.min_corner.z = workspace["z"][0]
    request.workspace_parameters.max_corner.x = workspace["x"][1]
    request.workspace_parameters.max_corner.y = workspace["y"][1]
    request.workspace_parameters.max_corner.z = workspace["z"][1]

    link_name = _string(config.get("end_effector_link")) or _string(config.get("end_effector_frame")) or DEFAULT_END_EFFECTOR_LINK
    tolerance = _resolved_tolerance_policy(config)
    position_tolerance_m = tolerance["moveit_position_tolerance_m"]
    orientation_tolerance_rad = tolerance["moveit_orientation_tolerance_rad"]

    region_pose = Pose()
    region_pose.position.x = float(position[0])
    region_pose.position.y = float(position[1])
    region_pose.position.z = float(position[2])
    region_pose.orientation.w = 1.0

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [position_tolerance_m]

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = frame
    position_constraint.link_name = link_name
    position_constraint.constraint_region.primitives.append(primitive)
    position_constraint.constraint_region.primitive_poses.append(region_pose)
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = frame
    orientation_constraint.link_name = link_name
    orientation_constraint.orientation.x = float(orientation[0])
    orientation_constraint.orientation.y = float(orientation[1])
    orientation_constraint.orientation.z = float(orientation[2])
    orientation_constraint.orientation.w = float(orientation[3])
    orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance_rad
    orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance_rad
    orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance_rad
    orientation_constraint.weight = 1.0

    constraints = Constraints()
    constraints.name = "teto_cartesian_pose_goal"
    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)
    request.goal_constraints.append(constraints)

    goal.planning_options.plan_only = True
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.planning_scene_diff.is_diff = True
    return goal


def _debug_print_move_group_goal(goal: Any, target_pose: Dict[str, Any]) -> None:
    request = goal.request
    constraints = request.goal_constraints[0] if request.goal_constraints else None
    position_constraint = constraints.position_constraints[0] if constraints and constraints.position_constraints else None
    orientation_constraint = constraints.orientation_constraints[0] if constraints and constraints.orientation_constraints else None
    frame_id = position_constraint.header.frame_id if position_constraint else target_pose.get("frame")
    link_name = position_constraint.link_name if position_constraint else None
    print("[TETO MoveIt DEBUG] submitting MoveGroup.Goal", flush=True)
    print(f"[TETO MoveIt DEBUG] position_xyz={target_pose.get('position_m')}", flush=True)
    print(f"[TETO MoveIt DEBUG] orientation_xyzw={target_pose.get('orientation_xyzw')}", flush=True)
    print(f"[TETO MoveIt DEBUG] frame_id={frame_id}", flush=True)
    print(f"[TETO MoveIt DEBUG] group_name={request.group_name}", flush=True)
    print(f"[TETO MoveIt DEBUG] link_name={link_name}", flush=True)
    print(f"[TETO MoveIt DEBUG] pipeline_id={request.pipeline_id or None}", flush=True)
    print(f"[TETO MoveIt DEBUG] planner_id={request.planner_id or None}", flush=True)
    if position_constraint:
        primitive = position_constraint.constraint_region.primitives[0]
        pose = position_constraint.constraint_region.primitive_poses[0]
        print(f"[TETO MoveIt DEBUG] position_constraint_radius={list(primitive.dimensions)}", flush=True)
        print(
            "[TETO MoveIt DEBUG] position_constraint_center="
            f"[{pose.position.x}, {pose.position.y}, {pose.position.z}]",
            flush=True,
        )
    if orientation_constraint:
        print(
            "[TETO MoveIt DEBUG] orientation_tolerances="
            f"[{orientation_constraint.absolute_x_axis_tolerance}, "
            f"{orientation_constraint.absolute_y_axis_tolerance}, "
            f"{orientation_constraint.absolute_z_axis_tolerance}]",
            flush=True,
        )


def _debug_print_move_group_result(error_code: int, planning_time_s: float, trajectory_point_count: int) -> None:
    print("[TETO MoveIt DEBUG] MoveGroup result", flush=True)
    print(f"[TETO MoveIt DEBUG] error_code={error_code}", flush=True)
    print(f"[TETO MoveIt DEBUG] error_code_name={_moveit_error_code_name(error_code)}", flush=True)
    print(f"[TETO MoveIt DEBUG] planning_time={float(planning_time_s)}", flush=True)
    print(f"[TETO MoveIt DEBUG] trajectory_point_count={trajectory_point_count}", flush=True)


def _import_ros_action_client() -> tuple[Any, Any]:
    import rclpy
    from rclpy.action import ActionClient

    return rclpy, ActionClient


def _spin_until_future(rclpy: Any, node: Any, future: Any, timeout_s: float) -> bool:
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    return bool(future.done())


def _common_result(*, config: Dict[str, Any], target_pose: Dict[str, Any] | None, validation: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "moveit_pose_executor_requested": True,
        "selected_moveit_interface": "ros2_action_clients",
        "move_group_action_name": _string(config.get("move_group_action_name")) or DEFAULT_MOVE_GROUP_ACTION,
        "execute_trajectory_action_name": _string(config.get("execute_trajectory_action_name")) or DEFAULT_EXECUTE_TRAJECTORY_ACTION,
        **_planner_mode_evidence(config),
        **_start_state_evidence(config),
        "planning_group": _string(config.get("planning_group")) or DEFAULT_PLANNING_GROUP,
        "planning_frame": target_pose.get("frame") if target_pose else _string(config.get("planning_frame")) or DEFAULT_FRAME,
        "end_effector_link": _string(config.get("end_effector_link")) or _string(config.get("end_effector_frame")) or DEFAULT_END_EFFECTOR_LINK,
        "requested_distance_m": validation.get("requested_distance_m"),
        "configured_max_distance_m": validation.get("configured_max_distance_m"),
        "hard_safety_limit_m": validation.get("hard_safety_limit_m"),
        "requested_distance_within_configured_limit": validation.get("requested_distance_within_configured_limit"),
        "safety_policy_source": validation.get("safety_policy_source"),
        "safety_policy_name": validation.get("safety_policy_name"),
        "moveit_position_tolerance_m": validation.get("moveit_position_tolerance_m"),
        "moveit_orientation_tolerance_rad": validation.get("moveit_orientation_tolerance_rad"),
        "tolerance_to_requested_distance_ratio": validation.get("tolerance_to_requested_distance_ratio"),
        "small_motion_tolerance_policy": validation.get("small_motion_tolerance_policy"),
        "target_frame": target_pose.get("frame") if target_pose else None,
        "current_tcp_frame": validation.get("current_tcp_frame"),
        "target_orientation_source": validation.get("target_orientation_source"),
        "orientation_mode": validation.get("orientation_mode"),
        "orientation_locked": validation.get("orientation_locked"),
        "requested_start_tcp_pose": validation.get("requested_start_tcp_pose"),
        "requested_target_tcp_pose": validation.get("requested_target_tcp_pose"),
        "moveit_end_effector_link": _string(config.get("end_effector_link")) or _string(config.get("end_effector_frame")) or DEFAULT_END_EFFECTOR_LINK,
        "moveit_planning_frame": target_pose.get("frame") if target_pose else _string(config.get("planning_frame")) or DEFAULT_FRAME,
        "moveit_group_name": _string(config.get("planning_group")) or DEFAULT_PLANNING_GROUP,
        "target_pose": target_pose,
        "translation_distance_m": validation.get("translation_distance_m"),
        "max_translation_m": validation.get("max_translation_m"),
        "motion_check_source": validation.get("motion_check_source"),
        "motion_check_current_position_m": validation.get("motion_check_current_position_m"),
        "motion_check_target_position_m": validation.get("motion_check_target_position_m"),
        "motion_check_distance_m": validation.get("motion_check_distance_m"),
        "motion_check_max_distance_m": validation.get("motion_check_max_distance_m"),
        "motion_check_hard_limit_m": validation.get("motion_check_hard_limit_m"),
        "motion_check_eps": validation.get("motion_check_eps"),
        "workspace_bounds": validation.get("workspace_bounds"),
        "workspace_check_passed": validation.get("workspace_check_passed") is True,
        **_empty_plan_audit(),
        "plan_success_source": "actual_moveit_action_result",
        "execute_success_source": "actual_execute_trajectory_action_result",
        "target_pose_generated_by_llm": False,
        "target_pose_generated_by_teto": True,
        "urscript_generated": False,
        "rtde_write_attempted": False,
        "dashboard_command_attempted": False,
        "raw_joint_targets_generated": False,
    }


def _blocked_result(*, plan: bool, validation: Dict[str, Any], config: Dict[str, Any]) -> Dict[str, Any]:
    return {
        **_common_result(config=config, target_pose=None, validation=validation),
        "moveit_pose_plan_requested": True,
        "moveit_pose_execute_requested": not plan,
        "moveit_pose_executor_status": STATUS_BLOCKED,
        "plan_success": False,
        "execute_success": False,
        "moveit_plan_called": False,
        "moveit_execute_called": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": validation["blocking_reasons"],
        "warnings": validation["warnings"],
    }


def _api_blocked(*, plan: bool, config: Dict[str, Any], reason: str, error: str, validation: Dict[str, Any]) -> Dict[str, Any]:
    return {
        **_common_result(config=config, target_pose=None, validation=validation),
        "moveit_pose_plan_requested": True,
        "moveit_pose_execute_requested": not plan,
        "moveit_pose_executor_status": STATUS_BLOCKED,
        "plan_success": False,
        "execute_success": False,
        "moveit_plan_called": False,
        "moveit_execute_called": False,
        "trajectory_send_allowed": False,
        "trajectory_sent": False,
        "controller_command_sent": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": _unique(validation["blocking_reasons"] + [reason]),
        "warnings": _unique(validation["warnings"] + [error]),
    }


def _planner_mode_evidence(config: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "planner_mode": PLANNER_MODE_JOINT_SPACE_POSE_GOAL,
        "planning_pipeline_id": _string(config.get("pipeline_id")),
        "planner_id": _string(config.get("planner_id")),
        "moveit_goal_type": MOVEIT_GOAL_TYPE_POSE_CONSTRAINTS,
        "joint_space_pose_goal_used": True,
        "cartesian_path_used": False,
        "cartesian_path_fraction": None,
        "joint_space_fallback_used": False,
        "joint_space_fallback_reason": None,
    }


def _start_state_evidence(config: Dict[str, Any]) -> Dict[str, Any]:
    current_joint_state = config.get("current_joint_state")
    joint_state_available = isinstance(current_joint_state, dict)
    return {
        "start_state_source": START_STATE_SOURCE_IMPLICIT_PLANNING_SCENE,
        "start_state_is_diff": True,
        "explicit_start_state_provided": False,
        "current_joint_state_available": joint_state_available,
        "current_joint_state_source": _string(config.get("current_joint_state_source")) if joint_state_available else None,
        "current_joint_state_age_s": _optional_float(config.get("current_joint_state_age_s")) if joint_state_available else None,
    }


def _target_orientation_evidence(
    *,
    target_pose: Dict[str, Any] | None,
    current_pose: Dict[str, Any] | None,
    config: Dict[str, Any],
) -> Dict[str, Any]:
    configured_source = _string(config.get("target_orientation_source"))
    configured_mode = _string(config.get("orientation_mode"))
    target_orientation = target_pose.get("orientation_xyzw") if isinstance(target_pose, dict) else None
    current_orientation = current_pose.get("orientation_xyzw") if isinstance(current_pose, dict) else None
    preserved = _same_quaternion(target_orientation, current_orientation)
    return {
        "target_orientation_source": configured_source or ("copied_from_current_tcp_pose" if preserved else None),
        "orientation_mode": configured_mode or ("keep_current_orientation" if preserved else None),
        "orientation_locked": preserved,
    }


def _empty_plan_audit() -> Dict[str, Any]:
    return {
        "planned_joint_names": None,
        "planned_start_joint_positions": None,
        "planned_final_joint_positions": None,
        "per_joint_delta_rad": None,
        "max_joint_delta_rad": None,
        "wrist_joint_names": list(WRIST_JOINT_NAMES),
        "wrist_joint_delta_rad": None,
        "max_wrist_joint_delta_rad": None,
        "joint_wrap_suspected": None,
        "joint_delta_audit_status": "NOT_AVAILABLE",
        "joint_delta_audit_reason": "planned_joint_trajectory_not_available",
        "planned_waypoint_count": None,
        "planned_joint_path_length_rad": None,
        "estimated_cartesian_path_length_m": None,
        "path_length_ratio": None,
        "path_metric_source": "not_available",
    }


def _plan_audit_result(
    *,
    config: Dict[str, Any],
    validation: Dict[str, Any],
    api_result: Dict[str, Any],
) -> Dict[str, Any]:
    joint_audit = _joint_delta_audit(api_result)
    path_audit = _path_metric_audit(api_result, validation, joint_audit)
    warnings = []
    max_wrist_delta = joint_audit.get("max_wrist_joint_delta_rad")
    if max_wrist_delta is not None and max_wrist_delta > SUSPICIOUS_WRIST_JOINT_DELTA_RAD + MOTION_LIMIT_EPS:
        warnings.append("W_SUSPICIOUS_WRIST_JOINT_DELTA_FOR_CARTESIAN_STEP")
    return {
        **_planner_mode_evidence(config),
        **_start_state_evidence(config),
        **joint_audit,
        **path_audit,
        "planner_audit_warnings": warnings,
    }


def _joint_delta_audit(api_result: Dict[str, Any]) -> Dict[str, Any]:
    names = _joint_trajectory_names(api_result)
    points = _joint_trajectory_points(api_result)
    if len(points) < 2:
        return _empty_plan_audit()
    start = points[0]
    final = points[-1]
    if len(start) != len(final):
        result = _empty_plan_audit()
        result["joint_delta_audit_reason"] = "planned_joint_trajectory_point_size_mismatch"
        return result
    if names is not None and len(names) != len(start):
        names = None
    deltas = [round(float(right) - float(left), 6) for left, right in zip(start, final)]
    max_delta = round(max(abs(value) for value in deltas), 6) if deltas else None
    per_joint_delta = (
        {name: delta for name, delta in zip(names, deltas)}
        if names is not None
        else None
    )
    wrist_delta = (
        {name: per_joint_delta[name] for name in WRIST_JOINT_NAMES if name in per_joint_delta}
        if per_joint_delta is not None
        else None
    )
    max_wrist_delta = (
        round(max(abs(value) for value in wrist_delta.values()), 6)
        if wrist_delta
        else None
    )
    return {
        **_empty_plan_audit(),
        "planned_joint_names": names,
        "planned_start_joint_positions": [round(float(value), 6) for value in start],
        "planned_final_joint_positions": [round(float(value), 6) for value in final],
        "per_joint_delta_rad": per_joint_delta,
        "max_joint_delta_rad": max_delta,
        "wrist_joint_names": [name for name in WRIST_JOINT_NAMES if wrist_delta and name in wrist_delta],
        "wrist_joint_delta_rad": wrist_delta,
        "max_wrist_joint_delta_rad": max_wrist_delta,
        "joint_wrap_suspected": any(abs(value) > math.pi for value in deltas),
        "joint_delta_audit_status": "AVAILABLE",
        "joint_delta_audit_reason": None,
    }


def _path_metric_audit(
    api_result: Dict[str, Any],
    validation: Dict[str, Any],
    joint_audit: Dict[str, Any],
) -> Dict[str, Any]:
    points = _joint_trajectory_points(api_result)
    waypoint_count = api_result.get("trajectory_point_count")
    if not isinstance(waypoint_count, int) or isinstance(waypoint_count, bool):
        waypoint_count = len(points) if points else None
    joint_path_length = _joint_path_length(points)
    requested_distance = validation.get("requested_distance_m")
    path_length_ratio = None
    if joint_path_length is not None and requested_distance is not None and requested_distance > 0.0:
        path_length_ratio = round(joint_path_length / float(requested_distance), 6)
    estimated_cartesian_path_length = _optional_float(api_result.get("estimated_cartesian_path_length_m"))
    source = "joint_trajectory" if joint_path_length is not None else "not_available"
    if estimated_cartesian_path_length is not None and source == "not_available":
        source = "provided_cartesian_path_metric"
    return {
        "planned_waypoint_count": waypoint_count,
        "planned_joint_path_length_rad": joint_path_length,
        "estimated_cartesian_path_length_m": estimated_cartesian_path_length,
        "path_length_ratio": path_length_ratio,
        "path_metric_source": source,
        "joint_delta_audit_status": joint_audit.get("joint_delta_audit_status"),
        "joint_delta_audit_reason": joint_audit.get("joint_delta_audit_reason"),
    }


def _not_requested(*, plan: bool) -> Dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "schema_version": CONTRACT_VERSION,
        "moveit_pose_executor_requested": False,
        "moveit_pose_plan_requested": False,
        "moveit_pose_execute_requested": False if plan else True,
        "moveit_pose_executor_status": STATUS_NOT_REQUESTED,
        "plan_success": False,
        "execute_success": False,
        "moveit_plan_called": False,
        "moveit_execute_called": False,
        "real_robot_motion_executed": False,
        "blocking_reasons": [],
        "warnings": [],
    }


def _robot_state_blockers(config: Dict[str, Any], state: Dict[str, Any]) -> list[str]:
    blockers: list[str] = []
    if _flag_from(config, state, "robot_state_ok", state.get("read_only_state_contract_ready")) is not True:
        blockers.append(E_ROBOT_STATE_NOT_OK)
    if _flag_from(config, state, "safety_status_ok", True) is not True:
        blockers.append(E_SAFETY_STATUS_NOT_OK)
    if _flag_from(config, state, "protective_stop", False) is True:
        blockers.append(E_PROTECTIVE_STOP_ACTIVE)
    if _flag_from(config, state, "emergency_stop", False) is True:
        blockers.append(E_EMERGENCY_STOP_ACTIVE)
    speed_scaling = _optional_float(config.get("speed_scaling", state.get("speed_scaling")))
    max_speed_scale = _optional_float(config.get("max_speed_scale")) or 0.10
    max_acc_scale = _optional_float(config.get("max_acc_scale")) or 0.10
    if speed_scaling is not None and (speed_scaling < 0.0 or speed_scaling > max_speed_scale):
        blockers.append(E_SPEED_SCALING_UNSAFE)
    if max_speed_scale > 0.10 or max_acc_scale > 0.10:
        blockers.append(E_SPEED_SCALING_UNSAFE)
    return blockers


def _action_unavailable(action_name: str) -> Dict[str, Any]:
    return {
        "action_name": action_name,
        "action_call_attempted": False,
        "action_server_available": False,
        "goal_accepted": False,
        "success": False,
        "error_code": None,
        "error_code_name": None,
        "warnings": [f"action server unavailable: {action_name}"],
    }


def _goal_timeout(action_name: str) -> Dict[str, Any]:
    result = _action_unavailable(action_name)
    result["action_call_attempted"] = True
    result["action_server_available"] = True
    result["warnings"] = [f"action goal timeout: {action_name}"]
    return result


def _result_timeout(action_name: str) -> Dict[str, Any]:
    result = _goal_timeout(action_name)
    result["warnings"] = [f"action result timeout: {action_name}"]
    return result


def _goal_rejected(action_name: str) -> Dict[str, Any]:
    return {
        "action_name": action_name,
        "action_call_attempted": True,
        "action_server_available": True,
        "goal_accepted": False,
        "success": False,
        "error_code": None,
        "error_code_name": None,
        "warnings": [f"action goal rejected: {action_name}"],
    }


def _trajectory_point_count(trajectory: Any) -> int:
    joint_points = getattr(getattr(trajectory, "joint_trajectory", None), "points", [])
    multi_dof_points = getattr(getattr(trajectory, "multi_dof_joint_trajectory", None), "points", [])
    return len(joint_points or []) + len(multi_dof_points or [])


def _joint_trajectory_names(api_result: Dict[str, Any]) -> list[str] | None:
    raw_names = api_result.get("planned_joint_names") or api_result.get("joint_names")
    if isinstance(raw_names, list) and all(isinstance(name, str) and name for name in raw_names):
        return list(raw_names)
    trajectory = api_result.get("planned_trajectory")
    if isinstance(trajectory, dict):
        joint_trajectory = trajectory.get("joint_trajectory")
        raw_names = joint_trajectory.get("joint_names") if isinstance(joint_trajectory, dict) else None
    else:
        joint_trajectory = getattr(trajectory, "joint_trajectory", None)
        raw_names = getattr(joint_trajectory, "joint_names", None)
    if isinstance(raw_names, list) and all(isinstance(name, str) and name for name in raw_names):
        return list(raw_names)
    return None


def _joint_trajectory_points(api_result: Dict[str, Any]) -> list[list[float]]:
    raw_points = api_result.get("joint_trajectory_points") or api_result.get("planned_joint_positions")
    if raw_points is None:
        trajectory = api_result.get("planned_trajectory")
        if isinstance(trajectory, dict):
            joint_trajectory = trajectory.get("joint_trajectory")
            raw_points = joint_trajectory.get("points") if isinstance(joint_trajectory, dict) else None
        else:
            joint_trajectory = getattr(trajectory, "joint_trajectory", None)
            raw_points = getattr(joint_trajectory, "points", None)
    if not isinstance(raw_points, list):
        return []
    points: list[list[float]] = []
    for raw in raw_points:
        positions = raw.get("positions") if isinstance(raw, dict) else getattr(raw, "positions", raw)
        if not isinstance(positions, list) or not positions:
            continue
        values = [_optional_float(value) for value in positions]
        if any(value is None for value in values):
            continue
        points.append([float(value) for value in values if value is not None])
    return points


def _joint_path_length(points: list[list[float]]) -> float | None:
    if len(points) < 2:
        return None
    total = 0.0
    for left, right in zip(points, points[1:]):
        if len(left) != len(right):
            continue
        total += sum(abs(float(rvalue) - float(lvalue)) for lvalue, rvalue in zip(left, right))
    return round(total, 6)


def _moveit_error_code_name(value: int | None) -> str | None:
    if value is None:
        return None
    known = {
        1: "SUCCESS",
        99999: "FAILURE",
        -1: "PLANNING_FAILED",
        -2: "INVALID_MOTION_PLAN",
        -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        -4: "CONTROL_FAILED",
        -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
        -6: "TIMED_OUT",
        -7: "PREEMPTED",
        -10: "START_STATE_IN_COLLISION",
        -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
        -12: "GOAL_IN_COLLISION",
        -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
        -14: "GOAL_CONSTRAINTS_VIOLATED",
        -15: "INVALID_GROUP_NAME",
        -16: "INVALID_GOAL_CONSTRAINTS",
        -17: "INVALID_ROBOT_STATE",
        -18: "INVALID_LINK_NAME",
        -19: "INVALID_OBJECT_NAME",
        -21: "FRAME_TRANSFORM_FAILURE",
        -22: "COLLISION_CHECKING_UNAVAILABLE",
        -23: "ROBOT_STATE_STALE",
        -24: "SENSOR_INFO_STALE",
        -31: "NO_IK_SOLUTION",
    }
    return known.get(int(value), f"UNKNOWN_{value}")


def _normalize_pose(value: Any) -> Dict[str, Any] | None:
    if isinstance(value, dict):
        position = value.get("position_m") or value.get("position") or value.get("xyz")
        orientation = value.get("orientation_xyzw") or value.get("orientation") or value.get("quat_xyzw")
        return {
            "frame": _string(value.get("frame")) or DEFAULT_FRAME,
            "position_m": list(position) if isinstance(position, (list, tuple)) else None,
            "orientation_xyzw": list(orientation) if isinstance(orientation, (list, tuple)) else [0.0, 0.0, 0.0, 1.0],
        }
    if isinstance(value, (list, tuple)) and len(value) in {3, 7}:
        orientation = list(value[3:7]) if len(value) == 7 else [0.0, 0.0, 0.0, 1.0]
        return {"frame": DEFAULT_FRAME, "position_m": list(value[:3]), "orientation_xyzw": orientation}
    return None


def _valid_pose(pose: Dict[str, Any], allowed_frames: set[str]) -> bool:
    return (
        _string(pose.get("frame")) in allowed_frames
        and _valid_vector3(pose.get("position_m"))
        and _valid_quaternion(pose.get("orientation_xyzw"))
    )


def _valid_vector3(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 3
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(float(item)) for item in value)
    )


def _valid_quaternion(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 4
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(float(item)) for item in value)
    )


def _same_quaternion(left: Any, right: Any) -> bool:
    if not _valid_quaternion(left) or not _valid_quaternion(right):
        return False
    return all(abs(float(lvalue) - float(rvalue)) <= MOTION_LIMIT_EPS for lvalue, rvalue in zip(left, right))


def _allowed_frames(config: Dict[str, Any]) -> set[str]:
    frames = config.get("allowed_frames") or config.get("allowed_cartesian_frames")
    if isinstance(frames, list):
        return {frame for frame in (_string(item) for item in frames) if frame}
    return set(ALLOWED_FRAMES)


def _workspace_bounds(config: Dict[str, Any]) -> Dict[str, list[float]]:
    raw = config.get("workspace_bounds") if isinstance(config.get("workspace_bounds"), dict) else {}
    return {
        "x": _pair(raw.get("x"), DEFAULT_WORKSPACE_BOUNDS["x"]),
        "y": _pair(raw.get("y"), DEFAULT_WORKSPACE_BOUNDS["y"]),
        "z": _pair(raw.get("z"), DEFAULT_WORKSPACE_BOUNDS["z"]),
    }


def _pair(value: Any, default: list[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        low = _optional_float(value[0])
        high = _optional_float(value[1])
        if low is not None and high is not None and low <= high:
            return [low, high]
    return list(default)


def _point_in_workspace(point: list[float], bounds: Dict[str, list[float]]) -> bool:
    return (
        bounds["x"][0] <= point[0] <= bounds["x"][1]
        and bounds["y"][0] <= point[1] <= bounds["y"][1]
        and bounds["z"][0] <= point[2] <= bounds["z"][1]
    )


def _distance_between(left: list[float], right: list[float]) -> float:
    return sum((float(lvalue) - float(rvalue)) ** 2 for lvalue, rvalue in zip(left, right)) ** 0.5


def _resolved_tolerance_policy(config: Dict[str, Any], fallback_distance_m: float | None = None) -> Dict[str, Any]:
    requested_distance = _optional_float(config.get("requested_distance_m"))
    if requested_distance is None:
        requested_distance = fallback_distance_m
    position_tolerance = float(_optional_float(config.get("position_tolerance_m")) or DEFAULT_POSITION_TOLERANCE_M)
    orientation_tolerance = float(_optional_float(config.get("orientation_tolerance_rad")) or DEFAULT_ORIENTATION_TOLERANCE_RAD)
    configured_policy = _string(config.get("small_motion_tolerance_policy")) or ""
    policy_enabled = configured_policy.startswith(
        (
            "real_motion_safety_policy_v1",
            "real_small_motion_strict_v3.0.3",
            "tiny_relative_cartesian_strict_v3.0.3",
        )
    )
    policy_limit = None
    policy = "default_moveit_pose_tolerance"
    if policy_enabled and requested_distance is not None and requested_distance > 0.0:
        policy_limit = float(requested_distance) * REAL_MOTION_TOLERANCE_DISTANCE_RATIO
        position_tolerance = min(position_tolerance, policy_limit)
        policy = (
            "real_motion_safety_policy_v1:"
            "position_tolerance_m=min(configured_position_tolerance_m,requested_distance_m*0.10);"
            "orientation_tolerance_rad=configured"
        )
    ratio = None
    if requested_distance is not None and requested_distance > 0.0:
        ratio = round(position_tolerance / float(requested_distance), 6)
    return {
        "requested_distance_m": round(float(requested_distance), 6) if requested_distance is not None else None,
        "moveit_position_tolerance_m": round(float(position_tolerance), 6),
        "moveit_orientation_tolerance_rad": round(float(orientation_tolerance), 6),
        "tolerance_to_requested_distance_ratio": ratio,
        "small_motion_tolerance_policy": policy,
        "small_motion_tolerance_policy_enabled": policy_enabled,
        "position_tolerance_policy_limit_m": round(float(policy_limit), 6) if policy_limit is not None else None,
    }


def _config_with_resolved_tolerance(config: Dict[str, Any], validation: Dict[str, Any]) -> Dict[str, Any]:
    return {
        **config,
        "requested_distance_m": validation.get("requested_distance_m"),
        "position_tolerance_m": validation.get("moveit_position_tolerance_m"),
        "orientation_tolerance_rad": validation.get("moveit_orientation_tolerance_rad"),
        "small_motion_tolerance_policy": config.get("small_motion_tolerance_policy"),
    }


def _flag_from(config: Dict[str, Any], state: Dict[str, Any], name: str, default: Any = None) -> Any:
    if name in config:
        return config.get(name)
    if name in state:
        return state.get(name)
    return default


def _forbidden_artifact(config: Dict[str, Any]) -> bool:
    return any(
        config.get(name) is True
        for name in (
            "urscript_generated",
            "rtde_write_attempted",
            "dashboard_command_attempted",
            "raw_joint_targets_generated",
            "target_pose_generated_by_llm",
        )
    )


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _string(value: Any) -> str | None:
    if isinstance(value, str):
        return value.strip() or None
    if value is None:
        return None
    return str(value).strip() or None


def _string_list(value: Any) -> list[str]:
    return [str(item) for item in value] if isinstance(value, list) else []


def _unique(values: list[str]) -> list[str]:
    output: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value and value not in seen:
            output.append(value)
            seen.add(value)
    return output
