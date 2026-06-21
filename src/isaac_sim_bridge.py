from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any
from urllib.parse import unquote

from src.isaac_sim_operator import GATEWAY_SIMULATED_MEASURED


DEFAULT_ISAAC_HOME_POSE_RAD = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.57,
    "elbow_joint": 1.57,
    "wrist_1_joint": -1.57,
    "wrist_2_joint": -1.57,
    "wrist_3_joint": 0.0,
}


def _named_joint_target(
    joint_names: list[str],
    current_positions: list[float],
    desired_positions: dict[str, Any],
) -> tuple[list[float], list[str], list[float]]:
    target = list(current_positions)
    applied_names: list[str] = []
    applied_positions: list[float] = []
    missing = [name for name in desired_positions if name not in joint_names]
    if missing:
        raise RuntimeError(f"E_ISAAC_HOME_JOINTS_MISSING: {', '.join(sorted(missing))}")
    for name, raw_position in desired_positions.items():
        index = joint_names.index(name)
        position = float(raw_position)
        target[index] = position
        applied_names.append(name)
        applied_positions.append(position)
    return target, applied_names, applied_positions


def _visual_timing(config: dict[str, Any], *, headless: bool, substep_count: int) -> dict[str, Any]:
    enabled = bool(config.get("visual_demo_slowdown_enabled", True)) and not headless
    fps = max(float(config.get("visual_demo_fps") or 60.0), 1.0)
    duration = max(float(config.get("motion_duration_sec") or 2.4), 0.0)
    pause = max(float(config.get("substep_pause_sec") or 0.25), 0.0)
    count = max(int(substep_count), 1)
    motion_frames = max(int(round((duration / count) * fps)), 1) if enabled else max(
        int(config.get("frames_per_substep", 90)),
        1,
    )
    pause_frames = int(round(pause * fps)) if enabled else 0
    return {
        "visual_demo_slowdown_enabled": enabled,
        "motion_duration_sec": duration,
        "substep_pause_sec": pause,
        "visual_demo_fps": fps,
        "motion_frames_per_substep": motion_frames,
        "pause_frames_per_substep": pause_frames,
    }


def _missing_local_usd_dependencies(asset_path: Path, sdf_module) -> list[Path]:
    layer = sdf_module.Layer.FindOrOpen(str(asset_path))
    if layer is None:
        return [asset_path]
    missing: list[Path] = []
    for dependency in layer.GetExternalReferences():
        dependency_text = unquote(str(dependency))
        if "://" in dependency_text:
            continue
        dependency_path = Path(dependency_text)
        if not dependency_path.is_absolute():
            dependency_path = asset_path.parent / dependency_path
        dependency_path = dependency_path.resolve()
        if not dependency_path.is_file():
            missing.append(dependency_path)
    return sorted(set(missing), key=str)


def _require_articulation(stage, prim_path: str, usd_module, articulation_root_api) -> str:
    root_prim = stage.GetPrimAtPath(prim_path)
    if root_prim and root_prim.IsValid():
        articulation_paths = [
            prim.GetPath().pathString
            for prim in usd_module.PrimRange(root_prim)
            if prim.HasAPI(articulation_root_api)
        ]
        if articulation_paths:
            return articulation_paths[0]
    raise RuntimeError(f"E_ISAAC_ARTICULATION_NOT_FOUND: prim_path={prim_path}")


class IsaacSimMeasuredBridge:
    """Isaac-only articulation/IK bridge. Import after SimulationApp is created."""

    gateway_type = GATEWAY_SIMULATED_MEASURED

    def __init__(self, *, simulation_app, config: dict[str, Any], headless: bool = False) -> None:
        self.simulation_app = simulation_app
        self.config = config
        self.headless = bool(headless)
        self.world = None
        self.robot = None
        self.ik_solver = None
        self.controller = None
        self.home_joint_positions: list[float] = []
        self.initial_home_pose_applied = False
        self.initial_home_pose_source = "disabled"
        self.initial_home_joint_names: list[str] = []
        self.initial_home_joint_positions: list[float] = []
        self.visual_debug = None
        self.visual_markers_enabled = False
        self.trajectory_trace_enabled = False
        self.last_visual_timing = _visual_timing(self.config, headless=self.headless, substep_count=1)
        self._initialize()

    def _initialize(self) -> None:
        import numpy as np
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.robot_motion.motion_generation import (
            ArticulationKinematicsSolver,
            LulaKinematicsSolver,
        )
        from isaacsim.robot_motion.motion_generation.interface_config_loader import (
            load_supported_lula_kinematics_solver_config,
        )
        from pxr import Sdf, Usd, UsdPhysics

        asset_mode = str(self.config.get("asset_mode") or "usd_reference")
        print(f"[TETO Isaac] bridge asset_mode={asset_mode}", flush=True)
        asset_path = Path(str(self.config["ur5e_asset_path"])).expanduser().resolve()
        if not asset_path.is_file():
            raise FileNotFoundError(f"E_UR5E_ASSET_NOT_FOUND: {asset_path}")
        prim_path = str(self.config.get("robot_prim_path") or "/World/TETO_UR5e")
        self.world = World(stage_units_in_meters=1.0)
        try:
            self.world.scene.add_default_ground_plane()
        except Exception:
            pass
        if asset_mode == "urdf_import":
            import omni.kit.commands

            urdf_path = Path(str(self.config.get("ur5e_urdf_path") or "")).expanduser().resolve()
            if not urdf_path.is_file():
                raise FileNotFoundError(f"E_UR5E_URDF_NOT_FOUND: {urdf_path}")
            ros_package_path = str(self.config.get("ros_package_path_for_asset_resolution") or "").strip()
            if ros_package_path:
                os.environ["ROS_PACKAGE_PATH"] = ros_package_path
            _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            import_config.merge_fixed_joints = False
            import_config.fix_base = True
            import_config.make_default_prim = False
            import_config.create_physics_scene = False
            _, imported_prim_path = omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=str(urdf_path),
                import_config=import_config,
                get_articulation_root=True,
            )
            prim_path = str(imported_prim_path)
            self.config["robot_prim_path"] = prim_path
            self.config["resolved_asset_source"] = "isaac_urdf_imported_to_usd_stage"
        else:
            missing_dependencies = _missing_local_usd_dependencies(asset_path, Sdf)
            if missing_dependencies:
                missing_text = ", ".join(str(path) for path in missing_dependencies)
                raise RuntimeError(
                    f"E_ISAAC_USD_DEPENDENCY_MISSING: asset={asset_path}; missing={missing_text}"
                )
            print(f"[TETO Isaac] loading UR5e USD reference: {asset_path}", flush=True)
            add_reference_to_stage(usd_path=str(asset_path), prim_path=prim_path)
            self.config["resolved_asset_source"] = "local_usd_reference"
            prim_path = _require_articulation(
                self.world.stage,
                prim_path,
                Usd,
                UsdPhysics.ArticulationRootAPI,
            )
            self.config["robot_prim_path"] = prim_path
        self.robot = self.world.scene.add(SingleArticulation(prim_path=prim_path, name="teto_isaac_ur5e"))
        self.world.reset()
        for _ in range(int(self.config.get("startup_render_frames", 30))):
            self.world.step(render=not self.headless)
        self.robot.initialize()
        self.controller = self.robot.get_articulation_controller()
        self._apply_initial_home_pose()
        kinematics_config = load_supported_lula_kinematics_solver_config("UR5e")
        lula = LulaKinematicsSolver(**kinematics_config)
        end_effector = str(self.config.get("tool_frame") or "tool0")
        self.ik_solver = ArticulationKinematicsSolver(self.robot, lula, end_effector)
        self.home_joint_positions = self._joint_positions()
        self._setup_visual_debug()
        base_position = np.asarray(self.config.get("robot_base_position_m") or [0.0, 0.0, 0.0], dtype=float)
        if any(abs(float(item)) > 0.0 for item in base_position):
            self.robot.set_world_pose(position=base_position)
            self.world.step(render=not self.headless)
        print("[TETO Isaac] measured bridge initialized; console may enter REPL", flush=True)

    def status(self) -> dict[str, Any]:
        try:
            pose = self._tcp_pose()
            return {
                "connection_status": "ISAAC_SIM_CONNECTED",
                "current_tcp_pose": pose,
                "joint_state": self._joint_state(),
                "robot_prim_path": self.config.get("robot_prim_path"),
                "ur5e_asset_path": self.config.get("ur5e_asset_path"),
                "isaac_initial_home_pose_applied": self.initial_home_pose_applied,
                "isaac_initial_home_pose_source": self.initial_home_pose_source,
                "isaac_initial_home_joint_names": list(self.initial_home_joint_names),
                "isaac_initial_home_joint_positions_rad": list(self.initial_home_joint_positions),
                "isaac_visual_markers_enabled": self.visual_markers_enabled,
                "isaac_trajectory_trace_enabled": self.trajectory_trace_enabled,
                "isaac_visual_timing": dict(self.last_visual_timing),
            }
        except Exception as exc:
            return {
                "connection_status": "ISAAC_SIM_ERROR",
                "error": str(exc),
                "current_tcp_pose": None,
                "joint_state": self._joint_state(),
            }

    def execute_relative_substep(
        self,
        *,
        delta_m: list[float],
        target_tcp_pose: dict[str, Any],
        substep_index: int,
        substep_count: int,
    ) -> dict[str, Any]:
        import numpy as np

        before = self._tcp_pose()
        self._draw_motion_markers(before["position_m"], target_tcp_pose["position_m"])
        action, success = self.ik_solver.compute_inverse_kinematics(
            target_position=np.asarray(target_tcp_pose["position_m"], dtype=float),
            target_orientation=np.asarray(
                [
                    target_tcp_pose["orientation_xyzw"][3],
                    target_tcp_pose["orientation_xyzw"][0],
                    target_tcp_pose["orientation_xyzw"][1],
                    target_tcp_pose["orientation_xyzw"][2],
                ],
                dtype=float,
            ),
            position_tolerance=float(self.config.get("ik_position_tolerance_m", 0.002)),
            orientation_tolerance=float(self.config.get("ik_orientation_tolerance_rad", 0.05)),
        )
        if not success:
            return {
                "execution_status": "FAILED",
                "abort_reason": "E_ISAAC_IK_DID_NOT_CONVERGE",
                "measured_tcp_before": before,
                "measured_tcp_after": self._tcp_pose(),
                "joint_state_after": self._joint_state(),
                "simulated_only": True,
            }
        timing = _visual_timing(self.config, headless=self.headless, substep_count=substep_count)
        self.last_visual_timing = timing
        target_positions = getattr(action, "joint_positions", None)
        joint_indices = getattr(action, "joint_indices", None)
        current_positions = np.asarray(self.robot.get_joint_positions(), dtype=float)
        if target_positions is not None:
            from isaacsim.core.utils.types import ArticulationAction

            targets = np.asarray(target_positions, dtype=float)
            if joint_indices is None:
                indices = np.arange(len(targets), dtype=int)
            else:
                indices = np.asarray(joint_indices, dtype=int)
            starts = current_positions[indices]
        else:
            indices = None
            starts = None
            targets = None
            self.controller.apply_action(action)
        motion_frames = int(timing["motion_frames_per_substep"])
        sleep_per_motion_frame = (
            float(timing["motion_duration_sec"]) / max(substep_count, 1) / motion_frames
            if timing["visual_demo_slowdown_enabled"]
            else 0.0
        )
        for frame_index in range(motion_frames):
            if not self.simulation_app.is_running():
                return {
                    "execution_status": "FAILED",
                    "abort_reason": "E_ISAAC_APP_CLOSED",
                    "measured_tcp_before": before,
                    "measured_tcp_after": self._tcp_pose(),
                    "joint_state_after": self._joint_state(),
                    "simulated_only": True,
                }
            if targets is not None and starts is not None:
                progress = float(frame_index + 1) / float(motion_frames)
                interpolated = starts + (targets - starts) * progress
                self.controller.apply_action(
                    ArticulationAction(
                        joint_positions=interpolated,
                        joint_indices=indices,
                    )
                )
            self.world.step(render=not self.headless)
            if sleep_per_motion_frame > 0.0:
                time.sleep(sleep_per_motion_frame)
        pause_frames = int(timing["pause_frames_per_substep"])
        sleep_per_pause_frame = (
            float(timing["substep_pause_sec"]) / pause_frames
            if pause_frames > 0
            else 0.0
        )
        for _ in range(pause_frames):
            self.world.step(render=not self.headless)
            if sleep_per_pause_frame > 0.0:
                time.sleep(sleep_per_pause_frame)
        after = self._tcp_pose()
        self._draw_motion_trace(before["position_m"], after["position_m"], target_tcp_pose["position_m"])
        return {
            "execution_status": "PASS",
            "abort_reason": None,
            "measured_tcp_before": before,
            "measured_tcp_after": after,
            "joint_state_after": self._joint_state(),
            "substep_index": substep_index,
            "substep_count": substep_count,
            "simulated_only": True,
            "isaac_local_api_used": True,
            "moveit_execute_trajectory_called": False,
            "isaac_visual_timing": timing,
            "isaac_visual_markers_enabled": self.visual_markers_enabled,
            "isaac_trajectory_trace_enabled": self.trajectory_trace_enabled,
        }

    def home(self) -> dict[str, Any]:
        if not self.home_joint_positions:
            return {"status": "BLOCKED", "abort_reason": "E_HOME_STATE_UNAVAILABLE", "simulated_only": True}
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        positions = np.asarray(self.home_joint_positions, dtype=float)
        self.robot.set_joint_positions(positions)
        self.controller.apply_action(ArticulationAction(joint_positions=positions))
        for _ in range(30):
            self.world.step(render=not self.headless)
        return {"status": "PASS", "current_tcp_pose": self._tcp_pose(), "simulated_only": True}

    def reset(self) -> dict[str, Any]:
        self.world.reset()
        self.robot.initialize()
        self.controller = self.robot.get_articulation_controller()
        self._apply_initial_home_pose()
        for _ in range(30):
            self.world.step(render=not self.headless)
        return {"status": "PASS", "current_tcp_pose": self._tcp_pose(), "simulated_only": True}

    def render_once(self) -> bool:
        if not self.simulation_app.is_running():
            return False
        self.world.step(render=not self.headless)
        return True

    def _apply_initial_home_pose(self) -> None:
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        if not bool(self.config.get("apply_initial_home_pose", True)):
            self.initial_home_pose_applied = False
            self.initial_home_pose_source = "config_disabled"
            return
        names = [str(item) for item in (self.robot.dof_names or [])]
        current = self._joint_positions()
        configured = self.config.get("initial_home_pose_rad")
        desired = configured if isinstance(configured, dict) else DEFAULT_ISAAC_HOME_POSE_RAD
        target, applied_names, applied_positions = _named_joint_target(names, current, desired)
        target_array = np.asarray(target, dtype=float)
        self.robot.set_joints_default_state(positions=target_array)
        self.robot.set_joint_positions(target_array)
        if self.controller is not None:
            self.controller.apply_action(ArticulationAction(joint_positions=target_array))
        for _ in range(max(int(self.config.get("home_pose_settle_frames", 30)), 1)):
            self.world.step(render=not self.headless)
        self.initial_home_pose_applied = True
        self.initial_home_pose_source = (
            "config.initial_home_pose_rad" if isinstance(configured, dict) else "isaac_default_natural_home"
        )
        self.initial_home_joint_names = applied_names
        self.initial_home_joint_positions = [round(value, 8) for value in applied_positions]

    def _setup_visual_debug(self) -> None:
        requested = bool(self.config.get("visual_markers_enabled", True)) and not self.headless
        if not requested:
            return
        try:
            from isaacsim.util.debug_draw import _debug_draw

            self.visual_debug = _debug_draw.acquire_debug_draw_interface()
            self.visual_markers_enabled = True
            self.trajectory_trace_enabled = bool(self.config.get("trajectory_trace_enabled", True))
            current = self._tcp_pose()["position_m"]
            self.visual_debug.draw_points([tuple(current)], [(0.0, 1.0, 0.0, 1.0)], [12.0])
        except Exception as exc:
            print(f"[TETO Isaac] visual debug unavailable: {type(exc).__name__}: {exc}", flush=True)
            self.visual_debug = None
            self.visual_markers_enabled = False
            self.trajectory_trace_enabled = False

    def _draw_motion_markers(self, current: list[float], target: list[float]) -> None:
        if self.visual_debug is None:
            return
        self.visual_debug.clear_points()
        self.visual_debug.draw_points(
            [tuple(current), tuple(target)],
            [(0.0, 1.0, 0.0, 1.0), (1.0, 0.8, 0.0, 1.0)],
            [12.0, 16.0],
        )

    def _draw_motion_trace(self, before: list[float], after: list[float], target: list[float]) -> None:
        if self.visual_debug is None:
            return
        self.visual_debug.clear_points()
        self.visual_debug.draw_points(
            [tuple(after), tuple(target)],
            [(0.0, 1.0, 1.0, 1.0), (1.0, 0.8, 0.0, 1.0)],
            [12.0, 16.0],
        )
        if self.trajectory_trace_enabled:
            self.visual_debug.draw_lines(
                [tuple(before)],
                [tuple(after)],
                [(0.0, 0.7, 1.0, 1.0)],
                [4.0],
            )

    def _tcp_pose(self) -> dict[str, Any]:
        from isaacsim.core.utils.rotations import rot_matrix_to_quat

        position, rotation = self.ik_solver.compute_end_effector_pose()
        orientation_wxyz = rot_matrix_to_quat(rotation)
        orientation_xyzw = [
            orientation_wxyz[1],
            orientation_wxyz[2],
            orientation_wxyz[3],
            orientation_wxyz[0],
        ]
        return {
            "frame": str(self.config.get("base_frame") or "base_link"),
            "position_m": [round(float(item), 6) for item in position],
            "orientation_xyzw": [round(float(item), 8) for item in orientation_xyzw],
            "source": "isaac_sim_forward_kinematics_measured",
        }

    def _joint_positions(self) -> list[float]:
        values = self.robot.get_joint_positions()
        return [float(item) for item in values.tolist()]

    def _joint_state(self) -> dict[str, Any]:
        names = [str(item) for item in (self.robot.dof_names or [])]
        positions = self._joint_positions() if self.robot is not None else []
        return {
            "names": names,
            "positions_rad": [round(value, 8) for value in positions],
            "source": "isaac_sim_articulation_measured",
        }
