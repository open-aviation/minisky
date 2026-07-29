from __future__ import annotations

from typing import TYPE_CHECKING, Any

from minisky_tangram import TangramBridge, convert_snapshot, extract_command
from minisky_tangram import init_plugin as _init_plugin

if TYPE_CHECKING:
    from minisky import MiniSky


def init_plugin(runtime: MiniSky) -> tuple[dict[str, Any], dict[str, list[Any]]]:
    return _init_plugin(runtime)


__all__ = ("TangramBridge", "convert_snapshot", "extract_command", "init_plugin")
