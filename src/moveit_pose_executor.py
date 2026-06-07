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
DEFAULT_WORKSPACE_BOUNDS = {
    "x": [-1.0, 1.0],
    "y": [-1.0, 1.0],
    "z": [0.0, 2.0],
}
ALLOWED_FRAMES = {"base_link"}

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
    return {
        **_common_result(config=config, target_pose=target_pose, validation=validation),
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
    return {
        **_common_result(config=config, target_pose=target_pose, validation=validation),
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
    if target_pose and current_pose and _valid_pose(target_pose, allowed_frames) and _valid_pose(current_pose, allowed_frames):
        translation_distance_m = _distance_between(current_pose["position_m"], target_pose["position_m"])
        if translation_distance_m > max_translation_m:
            blocking_reasons.append(E_EXCESSIVE_CARTESIAN_MOTION)
        if not _point_in_workspace(target_pose["position_m"], workspace_bounds):
            blocking_reasons.append(E_OUT_OF_WORKSPACE)

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
        "translation_distance_m": round(float(translation_distance_m), 6) if translation_distance_m is not None else None,
        "workspace_check_passed": bool(target_pose and _point_in_workspace(target_pose["position_m"], workspace_bounds)),
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
    position_tolerance_m = float(_optional_float(config.get("position_tolerance_m")) or 0.005)
    orientation_tolerance_rad = float(_optional_float(config.get("orientation_tolerance_rad")) or 0.05)

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
        "planning_group": _string(config.get("planning_group")) or DEFAULT_PLANNING_GROUP,
        "planning_frame": target_pose.get("frame") if target_pose else _string(config.get("planning_frame")) or DEFAULT_FRAME,
        "end_effector_link": _string(config.get("end_effector_link")) or _string(config.get("end_effector_frame")) or DEFAULT_END_EFFECTOR_LINK,
        "target_pose": target_pose,
        "translation_distance_m": validation.get("translation_distance_m"),
        "max_translation_m": validation.get("max_translation_m"),
        "workspace_bounds": validation.get("workspace_bounds"),
        "workspace_check_passed": validation.get("workspace_check_passed") is True,
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
