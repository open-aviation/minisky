"""Focused tests for the BlueSky scenario command boundary."""

from __future__ import annotations

from typing import Annotated, Any, Literal

import pytest
from annotated_types import Gt
from minisky import MiniSky
from minisky import quantities as q
from minisky._internal.command import (
    ArgumentIssue,
    CommandCursor,
    CoordinateWaypoint,
    HeadingDeg,
    LatLonDeg,
    NamedWaypoint,
    RunwayHeadingRequest,
    SourceSpan,
    UseRunwayHeading,
    Wpt,
    build_command_schema,
    command,
)
from minisky._internal.result import Err, Ok
from minisky.types import (
    GroundTrackDeg,
    LatLonDegrees,
    MagneticHeadingDeg,
    MslAltM,
    StdPressureAltM,
    TrueHeadingDeg,
)
from tests._types import RunCommand


@pytest.mark.parametrize(
    ("text", "value", "remaining"),
    [
        ("ONE TWO", "ONE", "TWO"),
        ("ONE, TWO", "ONE", "TWO"),
        ("ONE ,TWO", "ONE", "TWO"),
        (",TWO", None, "TWO"),
        ('"",TWO', "", "TWO"),
        ("ONE,,TWO", "ONE", ",TWO"),
        ('"ONE TWO",THREE', "ONE TWO", "THREE"),
        ("'ONE,TWO' THREE", "ONE,TWO", "THREE"),
        ("N52'14'12' E004'23'10'", "N52'14'12'", "E004'23'10'"),
    ],
)
def test_cursor_field_grammar(text: str, value: str | None, remaining: str) -> None:
    cursor = CommandCursor(text)
    result = cursor.next_field()
    assert isinstance(result, Ok)
    field = result.ok()
    assert field is not None
    assert field.value == value
    assert cursor.remaining == remaining


def test_quoted_span() -> None:
    cursor = CommandCursor('  "ONE TWO",THREE')
    result = cursor.next_field()
    assert isinstance(result, Ok)
    field = result.ok()
    assert field is not None
    assert (field.span.start, field.span.end) == (2, 11)


@pytest.mark.parametrize(
    ("text", "commands"),
    [
        ("OP; HOLD", ("OP", "HOLD")),
        ('ECHO "ONE;TWO"; OP', ('ECHO "ONE;TWO"', "OP")),
        ("ECHO it's ready; OP", ("ECHO it's ready", "OP")),
        (" ; OP ;; HOLD ; ", ("OP", "HOLD")),
    ],
)
def test_cursor_command_grammar(text: str, commands: tuple[str, ...]) -> None:
    cursor = CommandCursor(text)
    parsed: list[str] = []
    while True:
        result = cursor.next_command()
        assert isinstance(result, Ok)
        command = result.ok()
        if command is None:
            break
        parsed.append(command.value)
    assert tuple(parsed) == commands


def test_unclosed_quote_has_source_span() -> None:
    text = 'OP\nECHO "unfinished'
    cursor = CommandCursor(text, 3)
    result = cursor.next_command()
    assert isinstance(result, Err)
    issue = result.err()
    assert issue.message == 'expected a closing " quote, but got end of input'
    assert issue.span == SourceSpan(8, len(text))


def test_command_cursor_failed_read_does_not_advance() -> None:
    cursor = CommandCursor('"unfinished')
    assert isinstance(cursor.next_field(), Err)
    assert cursor.pos == 0


def test_empty_comma_field_uses_parameter_default(runtime: MiniSky) -> None:
    received: list[tuple[str, str]] = []

    def record(first: str = "DEFAULT", second: str = "SECOND") -> None:
        received.append((first, second))

    prepared = runtime.commands.prepare_command(record, name="TESTDEFAULT")
    result = prepared(",provided")

    assert isinstance(result, Ok)
    assert received == [("DEFAULT", "provided")]


def test_required_nullable_field_accepts_only_explicit_omission(runtime: MiniSky) -> None:
    received: list[tuple[int | None, int]] = []

    def record(first: int | None, second: int) -> None:
        received.append((first, second))

    prepared = runtime.commands.prepare_command(record, name="TESTNULLABLE")
    first = prepared.forms[0].parameters[0]
    assert first.name == "first"
    assert first.nullable
    assert isinstance(prepared(",7"), Ok)
    assert received == [(None, 7)]
    assert isinstance(prepared(""), Err)


def test_nullable_default_allows_argument_omission(runtime: MiniSky) -> None:
    received: list[int | None] = []

    def record(value: int | None = None) -> None:
        received.append(value)

    prepared = runtime.commands.prepare_command(record, name="TESTOPTIONAL")
    assert isinstance(prepared(""), Ok)
    assert isinstance(prepared(","), Ok)
    assert received == [None, None]


def test_none_default_requires_nullable_annotation(runtime: MiniSky) -> None:
    none_default: Any = None

    def invalid(value: int = none_default) -> None:
        pass

    with pytest.raises(TypeError, match="defaults to None"):
        runtime.commands.prepare_command(invalid, name="TESTNONEDEFAULT")


def test_annotated_constraint_is_checked_before_callback(runtime: MiniSky) -> None:
    received: list[int] = []

    def record(value: Annotated[int, Gt(0)]) -> None:
        received.append(value)

    prepared = runtime.commands.prepare_command(record, name="TESTPOSITIVE")
    assert isinstance(prepared("0"), Err)
    assert isinstance(prepared("1"), Ok)
    assert received == [1]


def test_variadic_is_zero_or_more(runtime: MiniSky) -> None:
    received: list[tuple[int, ...]] = []

    def record(*values: int) -> None:
        received.append(values)

    prepared = runtime.commands.prepare_command(record, name="TESTREPEAT")
    parameter = prepared.forms[0].parameters[0]
    assert parameter.name == "values"
    assert parameter.repeat

    empty = prepared("")
    multiple = prepared("1, 2 3")

    assert isinstance(empty, Ok)
    assert isinstance(multiple, Ok)
    assert received == [(), (1, 2, 3)]


def test_altitude_reference_is_preserved(runtime: MiniSky) -> None:
    def record(value: StdPressureAltM | MslAltM) -> None:
        pass

    prepared = runtime.commands.prepare_command(record, name="TESTALTREF")
    assert prepared.parse_arguments("FL100") == Ok((StdPressureAltM(q.ft_to_m(10000.0)),))
    assert prepared.parse_arguments("3048M[STD]") == Ok((StdPressureAltM(3048.0),))
    assert prepared.parse_arguments("3048M[MSL]") == Ok((MslAltM(3048.0),))


def test_ground_track_requires_explicit_reference(runtime: MiniSky) -> None:
    def record(value: GroundTrackDeg) -> None:
        pass

    prepared = runtime.commands.prepare_command(record, name="TESTTRK")
    assert prepared.parse_arguments("090TRK") == Ok((GroundTrackDeg(90.0),))
    assert isinstance(prepared.parse_arguments("090"), Err)


def test_waypoint_parser_preserves_named_and_coordinate_structure(runtime: MiniSky) -> None:
    received: list[object] = []

    def record(waypoint: Wpt) -> None:
        received.append(waypoint)

    prepared = runtime.commands.prepare_command(record, name="TESTWPT")

    assert isinstance(prepared("EHAM"), Ok)
    assert isinstance(prepared("52.5,5.0"), Ok)
    assert received == [
        NamedWaypoint("EHAM"),
        CoordinateWaypoint(LatLonDegrees(52.5, 5.0), "52.5,5.0"),
    ]


def test_resolved_position_rejects_ambiguous_waypoint_without_ui_reference(
    runtime: MiniSky,
) -> None:
    received: list[LatLonDegrees] = []

    def record(position: LatLonDeg) -> None:
        received.append(position)

    runtime.navigation.defwpt("ZZDUPPOS", 52.0, 4.0)
    runtime.navigation.defwpt("ZZDUPPOS", 53.0, 5.0)
    try:
        prepared = runtime.commands.prepare_command(record, name="TESTPOS")
        result = prepared("ZZDUPPOS")

        assert isinstance(result, Err)
        issue = result.err()
        assert isinstance(issue, ArgumentIssue)
        assert "unambiguous waypoint id" in issue.message
        assert received == []
    finally:
        runtime.navigation.delwpt("ZZDUPPOS")
        runtime.navigation.delwpt("ZZDUPPOS")


def test_union_uses_left_to_right_choice(runtime: MiniSky) -> None:
    received: list[object] = []

    def record(value: HeadingDeg | UseRunwayHeading) -> None:
        received.append(value)

    prepared = runtime.commands.prepare_command(record, name="TESTUNION")

    assert isinstance(prepared("*"), Ok)
    assert isinstance(prepared("090"), Ok)
    assert isinstance(prepared("090M"), Ok)
    assert isinstance(received[0], RunwayHeadingRequest)
    assert received[1] == TrueHeadingDeg(90.0)
    assert received[2] == MagneticHeadingDeg(90.0)


def test_union_falls_through_to_later_parser(runtime: MiniSky) -> None:
    received: list[int | str] = []

    def record(value: int | str) -> None:
        received.append(value)

    prepared = runtime.commands.prepare_command(record, name="TESTUNION")

    assert isinstance(prepared("7"), Ok)
    assert isinstance(prepared("word"), Ok)
    assert received == [7, "word"]


def test_union_order_is_semantic(runtime: MiniSky) -> None:
    received: list[str | int] = []

    def record(value: str | int) -> None:
        received.append(value)

    prepared = runtime.commands.prepare_command(record, name="TESTUNION")

    assert isinstance(prepared("7"), Ok)
    assert received == ["7"]


def test_command_overloads_use_left_to_right_choice(runtime: MiniSky) -> None:
    received: list[tuple[object, ...]] = []

    class Component:
        @command(name="TESTOVER")
        def query(self) -> None:
            """Query the current value."""
            received.append(("query",))

        @command(name="TESTOVER")
        def set_value(self, action: Literal["SET"], value: int) -> None:
            """Set an integer value."""
            received.append((action, value))

        @command(name="TESTOVER")
        def named(self, name: str) -> None:
            """Select a named value."""
            received.append(("name", name))

    (prepared,) = runtime.commands.mount_component(Component())
    try:
        help_result = runtime.commands.show_help("TESTOVER")
        assert isinstance(help_result, Ok)
        help_text = help_result.ok()
        assert "Query the current value.\nUsage:\nTESTOVER" in help_text
        assert "Set an integer value.\nUsage:\nTESTOVER SET,value" in help_text
        assert "Select a named value.\nUsage:\nTESTOVER name" in help_text
        assert isinstance(prepared(""), Ok)
        assert isinstance(prepared("other"), Ok)
        assert isinstance(prepared("set 7"), Ok)
        assert received == [("query",), ("name", "other"), ("SET", 7)]

        # Ordered choice is intentionally permissive: when the narrower SET form
        # fails, the later generic name form may still accept the same text.
        assert isinstance(prepared("set"), Ok)
        assert received == [("query",), ("name", "other"), ("SET", 7), ("name", "set")]
    finally:
        runtime.commands.remove_commands((prepared,))


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

    assert isinstance(prepared("7"), Ok)
    assert isinstance(prepared("7.5"), Ok)
    assert received == [("int", 7), ("float", 7.5)]


def test_nullable_overload_can_accept_omitted_field_after_required_fails(runtime: MiniSky) -> None:
    received: list[int | None] = []

    class Component:
        @command(name="TESTNULLAMBIG")
        def required(self, value: int) -> None:
            received.append(value)

        @command(name="TESTNULLAMBIG")
        def nullable(self, value: int | None) -> None:
            received.append(value)

    (prepared,) = runtime.commands.prepare_component(Component())

    assert isinstance(prepared("7"), Ok)
    assert isinstance(prepared(","), Ok)
    assert received == [7, None]


def test_cre_omitted_heading_field(run_cmd: RunCommand, runtime: MiniSky) -> None:
    output = run_cmd("CRE COMPAT1,B738,52,4,,FL100,250KT[CAS]")

    assert "created" in output.lower()
    index = runtime.traffic.idx("COMPAT1")
    assert index >= 0
    assert runtime.traffic.hdg[index] == pytest.approx(45.0)


def test_cre_explicit_runway_heading_marker_is_resolved_by_command(
    run_cmd: RunCommand, runtime: MiniSky
) -> None:
    output = run_cmd("CRE RWYREF,A320,EHAM,RWY18L,*,0FT[STD],250KT[CAS]")

    assert "created" in output.lower()
    index = runtime.traffic.idx("RWYREF")
    runway_heading = runtime.navigation.rwythresholds["EHAM"]["18L"][2]
    assert runtime.traffic.hdg[index] == pytest.approx(runway_heading)


def test_heading_wildcard_is_not_global_heading_syntax(
    runtime: MiniSky, run_cmd: RunCommand
) -> None:
    run_cmd("CRE HDGREF,A320,52,4,90,FL100,250KT[CAS]")
    heading = runtime.commands.cmddict["HDG"]

    result = heading.parse_arguments("HDGREF *")

    assert isinstance(result, Err)
    assert "expected a heading" in result.err().message


def test_cre_runway_heading_marker_requires_runway(runtime: MiniSky) -> None:
    result = runtime.commands.cmddict["CRE"]("NORWY,A320,52,4,*,0FT[STD],250KT[CAS]")

    assert isinstance(result, Err)
    assert result.err() == "CRE: heading * requires a runway position"


def test_wind_error_span_points_to_invalid_profile_field(runtime: MiniSky) -> None:
    result = runtime.commands.cmddict["WIND"].parse_arguments("52 4 100FT[STD] 180 BAD")
    assert isinstance(result, Err)
    issue = result.err()
    text = "WIND 52 4 100FT[STD] 180 BAD"
    assert issue.span is not None
    assert text[issue.span.start : issue.span.end] == "BAD"


def test_route_waypoint_membership_is_validated_by_route_command(
    runtime: MiniSky, run_cmd: RunCommand
) -> None:
    run_cmd("CRE ROUTE1,A320,52,4,90,FL100,250KT[CAS]")
    direct = runtime.commands.cmddict["DIRECT"]

    parsed = direct.parse_arguments("ROUTE1 MISSING")
    result = direct("ROUTE1 MISSING")

    assert isinstance(parsed, Ok)
    assert isinstance(result, Err)
    error = result.err()
    assert isinstance(error, str)
    assert "Waypoint MISSING not found in the route of ROUTE1" in error


def test_command_schema_describes_existing_forms(runtime: MiniSky) -> None:
    def record(value: int) -> None:
        """Record one integer."""

    command = runtime.commands.prepare_command(record, name="TESTSCHEMA", aliases=("TS",))
    schema = build_command_schema((command,))

    assert tuple(schema.commands) == ("TESTSCHEMA",)
    entry = schema.commands["TESTSCHEMA"]
    assert entry.aliases == ("TS",)
    assert entry.forms[0].syntax == "TESTSCHEMA value"
    assert entry.forms[0].doc == "Record one integer."
