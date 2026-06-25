"""Compatibility shim for the projector shadow implementation.

New code should import from ``src.projector.shadow``. This module remains only
to preserve the historical root-level import path during the package migration.
"""

from src.projector.shadow import *  # noqa: F401,F403
