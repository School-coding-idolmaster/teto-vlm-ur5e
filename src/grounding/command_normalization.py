from __future__ import annotations

import re


def normalize_command(command: str | None) -> str | None:
    if not isinstance(command, str):
        return None
    normalized = re.sub(r"\s+", " ", command.strip().lower())
    return normalized or None


__all__ = ["normalize_command"]
