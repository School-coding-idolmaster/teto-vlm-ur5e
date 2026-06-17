#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.motion_command_normalizer import normalize_motion_command  # noqa: E402


DEFAULT_COMMANDS = [
    "raise the tcp by 5 cm",
    "lift the tool 2 centimeters",
    "move the robot hand slightly down by 10 mm",
    "drop the end effector 2 centimeters",
    "shift the arm tip left by 10 mm",
    "move forward 5 centimeters",
    "go backward by 3 cm",
    "go down a little",
    "move up a bit",
    "nudge the tcp left",
    "move it over there",
    "move to the mug",
    "grab the cup",
    "move up and down 5 cm",
]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline TETO natural-language motion coverage report.")
    parser.add_argument("commands", nargs="*", help="Commands to normalize. Uses built-in examples when omitted.")
    parser.add_argument("--default-small-step-m", type=float, default=0.01)
    args = parser.parse_args(argv)

    commands = args.commands or DEFAULT_COMMANDS
    rows = [
        (
            "raw_command",
            "parse_status",
            "direction",
            "distance_m",
            "distance_source",
            "parser_source",
            "clarification_reason",
        )
    ]
    for command in commands:
        result = normalize_motion_command(command, default_small_step_m=float(args.default_small_step_m))
        direction = (
            f"{result.get('direction_axis')}{result.get('direction_sign')}"
            if result.get("direction_axis") and result.get("direction_sign")
            else ""
        )
        rows.append(
            (
                command,
                str(result.get("parse_status") or ""),
                direction,
                "" if result.get("requested_distance_m") is None else str(result.get("requested_distance_m")),
                str(result.get("distance_source") or ""),
                str(result.get("parser_source") or ""),
                str(result.get("clarification_reason") or result.get("unsupported_intent_reason") or ""),
            )
        )

    widths = [max(len(row[index]) for row in rows) for index in range(len(rows[0]))]
    for row in rows:
        print(" | ".join(value.ljust(widths[index]) for index, value in enumerate(row)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
