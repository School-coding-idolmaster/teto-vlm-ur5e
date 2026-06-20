from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from src.isaac_sim_operator import GATEWAY_SIMULATED_MEASURED


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

        asset_mode = str(self.config.get("asset_mode") or "usd_reference")
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
            add_reference_to_stage(usd_path=str(asset_path), prim_path=prim_path)
            self.config["resolved_asset_source"] = "local_usd_reference"
        self.robot = self.world.scene.add(SingleArticulation(prim_path=prim_path, name="teto_isaac_ur5e"))
        self.world.reset()
        for _ in range(int(self.config.get("startup_render_frames", 30))):
            self.world.step(render=not self.headless)
        self.robot.initialize()
        kinematics_config = load_supported_lula_kinematics_solver_config("UR5e")
        lula = LulaKinematicsSolver(**kinematics_config)
        end_effector = str(self.config.get("tool_frame") or "tool0")
        self.ik_solver = ArticulationKinematicsSolver(self.robot, lula, end_effector)
        self.controller = self.robot.get_articulation_controller()
        self.home_joint_positions = self._joint_positions()
        base_position = np.asarray(self.config.get("robot_base_position_m") or [0.0, 0.0, 0.0], dtype=float)
        if any(abs(float(item)) > 0.0 for item in base_position):
            self.robot.set_world_pose(position=base_position)
            self.world.step(render=not self.headless)

    def status(self) -> dict[str, Any]:
        try:
            pose = self._tcp_pose()
            return {
                "connection_status": "ISAAC_SIM_CONNECTED",
                "current_tcp_pose": pose,
                "joint_state": self._joint_state(),
                "robot_prim_path": self.config.get("robot_prim_path"),
                "ur5e_asset_path": self.config.get("ur5e_asset_path"),
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
        self.controller.apply_action(action)
        frames = int(self.config.get("frames_per_substep", 90))
        for _ in range(max(frames, 1)):
            if not self.simulation_app.is_running():
                return {
                    "execution_status": "FAILED",
                    "abort_reason": "E_ISAAC_APP_CLOSED",
                    "measured_tcp_before": before,
                    "measured_tcp_after": self._tcp_pose(),
                    "joint_state_after": self._joint_state(),
                    "simulated_only": True,
                }
            self.world.step(render=not self.headless)
        return {
            "execution_status": "PASS",
            "abort_reason": None,
            "measured_tcp_before": before,
            "measured_tcp_after": self._tcp_pose(),
            "joint_state_after": self._joint_state(),
            "substep_index": substep_index,
            "substep_count": substep_count,
            "simulated_only": True,
            "isaac_local_api_used": True,
            "moveit_execute_trajectory_called": False,
        }

    def home(self) -> dict[str, Any]:
        if not self.home_joint_positions:
            return {"status": "BLOCKED", "abort_reason": "E_HOME_STATE_UNAVAILABLE", "simulated_only": True}
        self.robot.set_joint_positions(self.home_joint_positions)
        for _ in range(30):
            self.world.step(render=not self.headless)
        return {"status": "PASS", "current_tcp_pose": self._tcp_pose(), "simulated_only": True}

    def reset(self) -> dict[str, Any]:
        self.world.reset()
        for _ in range(30):
            self.world.step(render=not self.headless)
        return {"status": "PASS", "current_tcp_pose": self._tcp_pose(), "simulated_only": True}

    def render_once(self) -> bool:
        if not self.simulation_app.is_running():
            return False
        self.world.step(render=not self.headless)
        return True

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
