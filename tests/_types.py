from __future__ import annotations

from collections.abc import Callable
from typing import Protocol


class RunCommand(Protocol):
    """Execute a queued command after one or more simulation steps."""

    def __call__(self, cmd: str, steps: int = 1) -> str: ...


class StepUntil(Protocol):
    """Step the simulation until a predicate succeeds."""

    def __call__(self, pred: Callable[[], bool], max_steps: int = 600) -> int: ...
