"""Validation for public MiniSky identifiers."""

from __future__ import annotations

import re

_PLUGIN_ID = re.compile(r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
_RESERVED_PLUGIN_IDS = frozenset({"sim", "traf"})


def validate_plugin_id(value: object) -> str:
    """Validate a canonical lowercase plugin ID."""
    if not isinstance(value, str) or not _PLUGIN_ID.fullmatch(value):
        raise ValueError(f"invalid plugin id: {value!r}")
    if value in _RESERVED_PLUGIN_IDS:
        raise ValueError(f"reserved plugin id: {value!r}")
    return value


_PUBLIC_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")


def normalize_public_name(value: str) -> str:
    """Normalize a command or replacement name."""
    name = value.strip().upper()
    if not _PUBLIC_NAME.fullmatch(name):
        raise ValueError(f"invalid public name: {value!r}")
    return name
