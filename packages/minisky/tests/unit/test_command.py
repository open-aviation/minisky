from __future__ import annotations

from typing import Literal

import pytest
from minisky import MiniSky
from minisky.command import (
    ArgumentIssue,
    HeadingDeg,
    LatLonDeg,
    LatLonDegrees,
    MagneticHeadingDeg,
    TrueHeadingDeg,
    command,
    split_commands,
)
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


def test_empty_comma_field_uses_parameter_default(runtime: MiniSky) -> None:
    received: list[tuple[int, int]] = []

    class Component:
        @command(name="TESTDEFAULT")
        def record(self, first: int = 7, second: int = 9) -> None:
            received.append((first, second))

    (prepared,) = runtime.commands.prepare_component(Component())
    assert isinstance(prepared.command(",3"), Ok)
    assert received == [(7, 3)]


def test_required_nullable_field_accepts_only_explicit_omission(runtime: MiniSky) -> None:
    received: list[tuple[int | None, int]] = []

    class Component:
        @command(name="TESTNULLABLE")
        def record(self, first: int | None, second: int) -> None:
            received.append((first, second))

    (prepared,) = runtime.commands.prepare_component(Component())
    parameter = prepared.command.forms[0].parameters[0]
    assert parameter.nullable
    assert isinstance(prepared.command(",7"), Ok)
    assert isinstance(prepared.command(""), Err)
    assert received == [(None, 7)]


def test_nullable_default_allows_argument_omission(runtime: MiniSky) -> None:
    received: list[int | None] = []

    class Component:
        @command(name="TESTOPTIONAL")
        def record(self, value: int | None = None) -> None:
            received.append(value)

    (prepared,) = runtime.commands.prepare_component(Component())
    assert "TESTOPTIONAL [value]" in prepared.command.helptext()
    assert isinstance(prepared.command(""), Ok)
    assert isinstance(prepared.command(","), Ok)
    assert received == [None, None]


def test_heading_parser_preserves_reference_frame(runtime: MiniSky) -> None:
    received: list[TrueHeadingDeg | MagneticHeadingDeg] = []

    class Component:
        @command(name="TESTHDG")
        def record(self, heading: HeadingDeg) -> None:
            received.append(heading)

    (prepared,) = runtime.commands.prepare_component(Component())
    assert isinstance(prepared.command("090"), Ok)
    assert isinstance(prepared.command("090T"), Ok)
    assert isinstance(prepared.command("090M"), Ok)
    assert isinstance(prepared.command("09M0"), Err)
    assert received == [
        TrueHeadingDeg(90.0),
        TrueHeadingDeg(90.0),
        MagneticHeadingDeg(90.0),
    ]


def test_resolved_position_rejects_ambiguous_waypoint_without_ui_reference(
    runtime: MiniSky,
) -> None:
    received: list[LatLonDegrees] = []

    class Component:
        @command(name="TESTPOS")
        def record(self, position: LatLonDeg) -> None:
            received.append(position)

    runtime.navigation.defwpt("ZZDUPPOS", 52.0, 4.0)
    runtime.navigation.defwpt("ZZDUPPOS", 53.0, 5.0)
    try:
        (prepared,) = runtime.commands.prepare_component(Component())
        result = prepared.command("ZZDUPPOS")

        assert isinstance(result, Err)
        issue = result.err()
        assert isinstance(issue, ArgumentIssue)
        assert "unambiguous waypoint id" in issue.message
        assert received == []
    finally:
        runtime.navigation.delwpt("ZZDUPPOS")
        runtime.navigation.delwpt("ZZDUPPOS")
