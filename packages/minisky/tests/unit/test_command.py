from __future__ import annotations

from typing import Literal

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


def test_literal_form_usage_and_order(runtime: MiniSky) -> None:
    received: list[tuple[object, ...]] = []

    class Component:
        @command(name="TESTFORM")
        def query(self) -> None:
            received.append(("query",))

        @command(name="TESTFORM")
        def set_value(self, action: Literal["SET"], value: int) -> None:
            received.append((action, value))

    (prepared,) = runtime.commands.prepare_component(Component())
    command_obj = prepared.command

    assert "TESTFORM SET,value" in command_obj.helptext()
    assert isinstance(command_obj(""), Ok)
    assert isinstance(command_obj("set 7"), Ok)
    assert received == [("query",), ("SET", 7)]
