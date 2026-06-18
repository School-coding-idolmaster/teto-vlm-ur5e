import argparse
import json

from src.camera_snapshot import build_camera_snapshot_request, evaluate_camera_snapshot_contract


FORMAL_REALSENSE_SOURCES = {"realsense_d455", "realsense_replay"}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="TETO formal CLI for RealSense D455 snapshot validation and safe utilities."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("check-env", help="Check Python, dependencies, and GPU status")

    snapshot_parser = subparsers.add_parser(
        "snapshot-replay",
        help="Validate a RealSense D455 snapshot or snapshot replay manifest.",
    )
    snapshot_parser.add_argument(
        "--snapshot-manifest",
        required=True,
        help="YAML/JSON manifest with RGB, aligned depth, camera_info, metadata, TF, and timestamps.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "check-env":
        from scripts.check_env import print_env_info

        print_env_info()
        return 0

    if args.command == "snapshot-replay":
        result = evaluate_camera_snapshot_contract(
            build_camera_snapshot_request(
                requested=True,
                config_path=args.snapshot_manifest,
            )
        )
        source = result.get("source")
        if source not in FORMAL_REALSENSE_SOURCES:
            result = {
                **result,
                "validity_status": "BLOCKED",
                "formal_visual_entry_status": "BLOCKED",
                "formal_visual_entry_reason": "E_FORMAL_VISUAL_SOURCE_NOT_REALSENSE",
                "allowed_formal_sources": sorted(FORMAL_REALSENSE_SOURCES),
            }
        else:
            result["formal_visual_entry_status"] = result.get("validity_status")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get("formal_visual_entry_status") == "PASS" else 2

    parser.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
