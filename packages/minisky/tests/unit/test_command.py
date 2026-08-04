from __future__ import annotations

from typing import Literal

import pytest
from minisky import MiniSky
from minisky.command import command, split_commands
from minisky.result import Err, Ok


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


@pytest.mark.parametrize(
    ("text", "commands"),
    [
        ("OP; HOLD", ("OP", "HOLD")),
        ('ECHO "ONE;TWO"; OP', ('ECHO "ONE;TWO"', "OP")),
        ("ECHO it's ready; OP", ("ECHO it's ready", "OP")),
        (" ; OP ;; HOLD ; ", ("OP", "HOLD")),
    ],
)
def test_semicolons_ignored(text: str, commands: tuple[str, ...]) -> None:
    result = split_commands(text)
    assert isinstance(result, Ok)
    assert result.ok() == commands


def test_unclosed_quote() -> None:
    result = split_commands('ECHO "unfinished')
    assert isinstance(result, Err)
    issue = result.err()
    assert issue.message == 'expected a closing " quote, but got end of input'
    assert issue.source_text == 'ECHO "unfinished'
    assert issue.span is not None
    assert issue.span.start == 5
