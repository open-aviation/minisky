from __future__ import annotations

from minisky import MiniSky
from minisky.command import command
from minisky.result import Ok


def test_overlapping_overloads_use_left_to_right_order(runtime: MiniSky) -> None:
    received: list[tuple[str, int | float]] = []

    class Component:
        @command(name="TESTAMBIG")
        def integer(self, value: int) -> None:
            received.append(("int", value))

        @command(name="TESTAMBIG")
        def number(self, value: float) -> None:
            received.append(("float", value))

    (prepared,) = runtime.commands.prepare_component(Component())

    assert isinstance(prepared.command("7"), Ok)
    assert isinstance(prepared.command("7.5"), Ok)
    assert received == [("int", 7), ("float", 7.5)]
