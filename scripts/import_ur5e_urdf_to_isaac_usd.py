#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any
from urllib.parse import unquote


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Import a resolved UR5e URDF into an Isaac Sim USD asset.")
    parser.add_argument("--urdf", required=True, help="Resolved URDF input path.")
    parser.add_argument("--out", required=True, help="Destination USD path.")
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing generated USD.")
    parser.add_argument(
        "--clean-no-tool",
        action="store_true",
        help="Remove workstation demo attachments while preserving the standard UR5e flange/tool0 chain.",
    )
    parser.add_argument(
        "--native-kit",
        action="store_true",
        help="Use the importer native binding inside a minimal Kit app.",
    )
    return parser


def _missing_local_dependencies(layer, asset_path: Path) -> list[str]:
    missing: list[str] = []
    for dependency in layer.GetExternalReferences():
        dependency_text = unquote(str(dependency))
        if "://" in dependency_text:
            continue
        dependency_path = Path(dependency_text)
        if not dependency_path.is_absolute():
            dependency_path = asset_path.parent / dependency_path
        dependency_path = dependency_path.resolve()
        if not dependency_path.exists():
            missing.append(str(dependency_path))
    return sorted(set(missing))


def _sanitized_urdf_copy(
    urdf_path: Path,
    *,
    clean_no_tool: bool = False,
) -> tuple[tempfile.TemporaryDirectory, Path, dict[str, Any]]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    removed_empty_geometry = 0
    removed_links: list[str] = []
    removed_joints: list[str] = []
    if clean_no_tool:
        removed_link_names = {"End_E", "stage_3", "camera_link", "ground_plane"}
        for link in list(root.findall("link")):
            link_name = str(link.get("name") or "")
            if link_name in removed_link_names:
                root.remove(link)
                removed_links.append(link_name)
        for joint in list(root.findall("joint")):
            parent = joint.find("parent")
            child = joint.find("child")
            parent_link = str(parent.get("link") or "") if parent is not None else ""
            child_link = str(child.get("link") or "") if child is not None else ""
            if parent_link in removed_link_names or child_link in removed_link_names:
                root.remove(joint)
                removed_joints.append(str(joint.get("name") or ""))
        for gazebo in list(root.findall("gazebo")):
            if str(gazebo.get("reference") or "") in removed_link_names:
                root.remove(gazebo)
    for link in root.findall("link"):
        for element_name in ("visual", "collision"):
            for element in list(link.findall(element_name)):
                geometry = element.find("geometry")
                if geometry is None or not list(geometry):
                    link.remove(element)
                    removed_empty_geometry += 1
    temp_dir = tempfile.TemporaryDirectory(prefix="teto_isaac_urdf_")
    sanitized_path = Path(temp_dir.name) / urdf_path.name
    tree.write(sanitized_path, encoding="utf-8", xml_declaration=True)
    return temp_dir, sanitized_path, {
        "clean_no_tool": clean_no_tool,
        "removed_empty_geometry_elements": removed_empty_geometry,
        "removed_links": sorted(removed_links),
        "removed_joints": sorted(removed_joints),
    }


def main() -> int:
    args = build_parser().parse_args()
    urdf_path = Path(args.urdf).expanduser().resolve()
    output_path = Path(args.out).expanduser().resolve()

    if not urdf_path.is_file():
        raise FileNotFoundError(f"E_UR5E_URDF_NOT_FOUND: {urdf_path}")
    if output_path.exists() and not args.overwrite:
        raise FileExistsError(f"E_ISAAC_USD_OUTPUT_EXISTS: {output_path}; pass --overwrite to replace it")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        output_path.unlink()

    simulation_app = None
    temp_dir = None
    try:
        from pxr import Sdf, Usd, UsdPhysics

        temp_dir, import_urdf_path, sanitization = _sanitized_urdf_copy(
            urdf_path,
            clean_no_tool=args.clean_no_tool,
        )
        if args.native_kit:
            isaac_root = os.environ.get("ISAAC_PATH")
            if not isaac_root and os.environ.get("CARB_APP_PATH"):
                isaac_root = str(Path(os.environ["CARB_APP_PATH"]).resolve().parent)
            if not isaac_root:
                raise RuntimeError("E_ISAAC_PATH_UNAVAILABLE")
            isaac_path = Path(isaac_root).resolve()
            binding_dir = (
                isaac_path / "exts/isaacsim.asset.importer.urdf/isaacsim/asset/importer/urdf"
            )
            binding_candidates = sorted(binding_dir.glob("_urdf*.so"))
            if not binding_candidates:
                raise RuntimeError(f"E_ISAAC_URDF_BINDING_NOT_FOUND: {binding_dir}")
            binding_path = binding_candidates[0]
            plugin_path = (
                isaac_path
                / "exts/isaacsim.asset.importer.urdf/bin"
                / "libisaacsim.asset.importer.urdf.plugin.so"
            )
            spec = importlib.util.spec_from_file_location("_urdf", binding_path)
            if spec is None or spec.loader is None:
                raise RuntimeError(f"E_ISAAC_URDF_BINDING_LOAD_FAILED: {binding_path}")
            urdf_binding = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(urdf_binding)
            urdf_interface = urdf_binding.acquire_urdf_interface(library_path=str(plugin_path))
            import_config = urdf_binding.ImportConfig()
        else:
            from isaacsim import SimulationApp

            simulation_app = SimulationApp({"headless": True})
            import omni.kit.app
            import omni.kit.commands

            extension_manager = omni.kit.app.get_app().get_extension_manager()
            extension_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
            simulation_app.update()
            status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            if not status:
                raise RuntimeError("E_ISAAC_URDF_IMPORT_CONFIG_FAILED")
        import_config.merge_fixed_joints = False
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.create_physics_scene = False
        import_config.import_inertia_tensor = True
        import_config.distance_scale = 1.0

        if args.native_kit:
            imported_robot = urdf_interface.parse_urdf(
                str(import_urdf_path.parent),
                import_urdf_path.name,
                import_config,
            )
            imported_prim_path = urdf_interface.import_robot(
                str(import_urdf_path.parent),
                import_urdf_path.name,
                imported_robot,
                import_config,
                str(output_path),
                True,
            )
            status = bool(imported_prim_path)
        else:
            status, imported_prim_path = omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=str(import_urdf_path),
                import_config=import_config,
                dest_path=str(output_path),
                get_articulation_root=True,
            )
            simulation_app.update()
        if not status or not output_path.is_file():
            raise RuntimeError(
                f"E_ISAAC_URDF_IMPORT_FAILED: status={status!r}; imported_prim_path={imported_prim_path!r}"
            )

        layer = Sdf.Layer.FindOrOpen(str(output_path))
        if layer is None:
            raise RuntimeError(f"E_ISAAC_USD_STAGE_OPEN_FAILED: {output_path}")
        missing_dependencies = _missing_local_dependencies(layer, output_path)
        if missing_dependencies:
            raise RuntimeError(
                "E_ISAAC_USD_DEPENDENCY_MISSING: " + ", ".join(missing_dependencies)
            )

        stage = Usd.Stage.Open(str(output_path))
        if stage is None:
            raise RuntimeError(f"E_ISAAC_USD_STAGE_OPEN_FAILED: {output_path}")
        prims = list(stage.TraverseAll())
        articulation_paths = [
            prim.GetPath().pathString
            for prim in prims
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        if not articulation_paths:
            raise RuntimeError("E_ISAAC_ARTICULATION_NOT_FOUND")
        joint_paths = [
            prim.GetPath().pathString
            for prim in prims
            if prim.IsA(UsdPhysics.Joint)
        ]
        rigid_body_paths = [
            prim.GetPath().pathString
            for prim in prims
            if prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        main_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        imported_joint_names = {prim.GetName() for prim in prims if prim.IsA(UsdPhysics.Joint)}
        missing_main_joints = [
            name for name in main_joint_names if name not in imported_joint_names
        ]
        tool0_paths = [
            prim.GetPath().pathString
            for prim in prims
            if prim.GetName() == "tool0"
        ]
        forbidden_clean_paths = [
            prim.GetPath().pathString
            for prim in prims
            if prim.GetName() in {"End_E", "stage_3", "camera_link", "ground_plane"}
        ]
        if args.clean_no_tool and missing_main_joints:
            raise RuntimeError(
                "E_ISAAC_CLEAN_ASSET_MAIN_JOINTS_MISSING: "
                + ", ".join(missing_main_joints)
            )
        if args.clean_no_tool and not tool0_paths:
            raise RuntimeError("E_ISAAC_CLEAN_ASSET_TOOL0_MISSING")
        if args.clean_no_tool and forbidden_clean_paths:
            raise RuntimeError(
                "E_ISAAC_CLEAN_ASSET_ATTACHMENT_REMAINS: "
                + ", ".join(forbidden_clean_paths)
            )
        robot_prims = [
            prim.GetPath().pathString
            for prim in prims
            if prim.GetPath().pathString.count("/") == 1
        ]
        used_layers = [
            layer.realPath
            for layer in stage.GetUsedLayers()
            if layer.realPath
        ]
        all_missing_dependencies: list[str] = []
        for used_layer in stage.GetUsedLayers():
            if not used_layer.realPath:
                continue
            all_missing_dependencies.extend(
                _missing_local_dependencies(used_layer, Path(used_layer.realPath))
            )
        all_missing_dependencies = sorted(set(all_missing_dependencies))
        if all_missing_dependencies:
            raise RuntimeError(
                "E_ISAAC_USD_DEPENDENCY_MISSING: " + ", ".join(all_missing_dependencies)
            )

        print(
            json.dumps(
                {
                    "status": "PASS",
                    "urdf": str(urdf_path),
                    "usd": str(output_path),
                    "imported_prim_path": str(imported_prim_path),
                    "sanitization": sanitization,
                    "robot_prims": robot_prims,
                    "articulation_roots": articulation_paths,
                    "joint_count": len(joint_paths),
                    "joints": joint_paths,
                    "rigid_body_count": len(rigid_body_paths),
                    "rigid_bodies": rigid_body_paths,
                    "main_joint_names": main_joint_names,
                    "missing_main_joints": missing_main_joints,
                    "tool0_paths": tool0_paths,
                    "forbidden_clean_paths": forbidden_clean_paths,
                    "used_layers": sorted(used_layers),
                    "missing_dependencies": all_missing_dependencies,
                },
                indent=2,
            ),
            flush=True,
        )
        return 0
    finally:
        if simulation_app is not None:
            simulation_app.close()
        if temp_dir is not None:
            temp_dir.cleanup()


if __name__ == "__main__":
    main()
