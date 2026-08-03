"""Integration tests for the command stack (queueing, processing, echo output)."""

from __future__ import annotations

import asyncio
from io import StringIO
from pathlib import Path

import pytest
from minisky import MiniSky
from minisky.simulation import Simulation
from minisky.stack import Command
from tests._types import RunCommand

FT = 0.3048
KTS = 0.514444


class TestQueueing:
    def test_stack_only_queues(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.commands.stack("CRE KL204,B744,52,4,45,FL250,350")
        assert runtime.traffic.ntraf == 0  # not executed yet

    def test_command_executes_on_step(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.commands.stack("CRE KL204,B744,52,4,45,FL250,350")
        runtime.simulation.step()
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.callsign[0] == "KL204"

    def test_command_stacked_during_processing_is_kept(
        self, runtime: MiniSky, sim: Simulation
    ) -> None:
        # A stack() call that lands while process() is draining the stack
        # (e.g. from a plugin I/O thread) must not be lost: commands() detaches
        # the pending list up front, so late arrivals run on the next step.
        runtime.commands.stack("ECHO first")
        drain = runtime.commands.commands()
        assert next(drain) == "ECHO first"
        runtime.commands.stack("CRE KL204,B744,52,4,45,FL250,350")  # racing append
        with pytest.raises(StopIteration):
            next(drain)
        runtime.simulation.step()
        assert runtime.traffic.ntraf == 1

    def test_awaitable_command_pauses_step_and_preserves_order(
        self, runtime: MiniSky, sim: Simulation
    ) -> None:
        async def exercise() -> None:
            events: list[str] = []
            release = asyncio.Event()

            async def pause() -> None:
                events.append("pause:start")
                await release.wait()
                events.append("pause:end")

            def queued() -> None:
                events.append("queued")

            def late() -> None:
                events.append("late")

            prepared = (
                runtime.commands.prepare_command(pause, name="TESTPAUSE"),
                runtime.commands.prepare_command(queued, name="TESTQUEUED"),
                runtime.commands.prepare_command(late, name="TESTLATE"),
            )
            runtime.commands.validate_commands(prepared)
            runtime.commands.install_commands(prepared)
            try:
                runtime.simulation.op()
                start = runtime.simulation.simt
                runtime.commands.stack("TESTPAUSE;TESTQUEUED")

                assert not runtime.simulation.step()
                await asyncio.sleep(0)
                assert events == ["pause:start"]
                assert runtime.commands.command_pending
                assert runtime.simulation.simt == start

                runtime.commands.stack("TESTLATE")
                assert not runtime.simulation.step()
                assert runtime.simulation.simt == start

                release.set()
                await runtime.commands.wait_for_pending()
                assert runtime.simulation.step()
                assert events == ["pause:start", "pause:end", "queued", "late"]
                assert runtime.simulation.simt == start + runtime.simulation.simdt
            finally:
                runtime.commands.remove_commands(prepared)
                runtime.commands.reset()
                await asyncio.sleep(0)

        asyncio.run(exercise())


class TestCommands:
    def test_cre_via_stack(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.alt[0] == pytest.approx(25000 * FT, rel=1e-3)

    def test_pos_outputs_callsign(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        output = run_cmd("POS KL204")
        assert "KL204" in output

    def test_bare_callsign_defaults_to_pos(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        output = run_cmd("KL204")
        assert "KL204" in output

    def test_alt_sets_selected_altitude(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("ALT KL204 FL260")
        assert runtime.traffic.selalt[0] == pytest.approx(26000 * FT, rel=1e-3)

    def test_hdg_sets_autopilot_track(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("HDG KL204 340")
        assert runtime.traffic.ap.trk[0] == pytest.approx(340.0)
        assert not runtime.traffic.swlnav[0]

    def test_spd_sets_selected_speed(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("SPD KL204 300")
        assert runtime.traffic.selspd[0] == pytest.approx(300 * KTS, rel=1e-3)

    def test_del_removes_aircraft(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert runtime.traffic.ntraf == 1
        run_cmd("DEL KL204")
        assert runtime.traffic.ntraf == 0

    def test_mcre_via_stack(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("MCRE 3")
        assert runtime.traffic.ntraf == 3


class TestTypedCommands:
    def test_echo_alias_consumes_remainder_verbatim(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        command = runtime.commands.cmddict["ECHO"]
        assert runtime.commands.cmddict["PRINT"] is command
        assert command.callback == runtime.console.echo
        assert run_cmd('PRINT "quoted text", still text') == '"quoted text", still text'

    def test_seed_parses_integer_field(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        command = runtime.commands.cmddict["SEED"]
        assert command.callback == runtime.simulation.setseed
        assert run_cmd('SEED "17"') == "random seed set"

    def test_seed_reports_source_for_invalid_integer(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        output = run_cmd("SEED nope")
        assert output == (
            "Error: argument `value`: expected an integer, but got nope\nSEED nope\n     ^^^^"
        )


class TestReadscn:
    def test_short_command_line_survives(self, runtime: MiniSky) -> None:
        # "0:00:00>OP" is only 10 characters; it used to be dropped by a
        # minimum-length check meant to skip empty lines.
        lines = list(runtime.commands.readscn(StringIO("0:00:00>OP\n")))
        assert lines == [(0.0, "OP")]

    def test_blank_and_comment_lines_skipped(self, runtime: MiniSky) -> None:
        scn = StringIO("# a comment\n\n0:00:01>HOLD\n")
        lines = list(runtime.commands.readscn(scn))
        assert lines == [(1.0, "HOLD")]


class TestHelp:
    def test_help_writes_command_reference(
        self, runtime: MiniSky, sim: Simulation, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        # HELP >filename writes the reference to ./docs/<filename>
        monkeypatch.chdir(tmp_path)
        (tmp_path / "docs").mkdir()
        result = runtime.commands.showhelp(">ref.txt")
        assert result.is_ok(), result.err()
        ref = tmp_path / "docs" / "ref.txt"
        assert ref.exists(), result.ok()
        content = ref.read_text()
        assert content.startswith("Command\tDescription\tUsage")
        assert "\nCRE\t" in content


class TestVarExplorer:
    def test_variable_get_without_index(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        v = runtime.variables.findvar("traf.ntraf")
        assert v is not None
        assert v.get() == 1
        assert v.get_type() == "int"

    def test_variable_get_with_index(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        v = runtime.variables.findvar("traf.callsign[0]")
        assert v is not None
        assert v.get() == ["KL204"]


class TestSynonyms:
    def test_airway_synonyms_point_to_pos(self, runtime: MiniSky) -> None:
        cmddict = runtime.commands.cmddict
        assert cmddict["AIRWAY"] is cmddict["POS"]
        assert cmddict["AIRWAYS"] is cmddict["POS"]


class TestErrors:
    def test_unknown_command_echoes_error(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        output = run_cmd("BOGUSCMD 42")
        assert "unknown command" in output.lower()

    def test_command_on_missing_aircraft_reports_error(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        output = run_cmd("ALT NOSUCH FL100")
        assert output  # some error text is echoed
        assert runtime.traffic.ntraf == 0

    def test_sim_survives_bad_command(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("THISDOESNOTEXIST")
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert runtime.traffic.ntraf == 1


class TestArgumentSpecs:
    def test_all_registered_specs_resolve_to_parsers(self, runtime: MiniSky) -> None:
        # Several commands (AT, DIRECT, AFTER, RESOOFF, ...) were registered
        # with argument specs containing whitespace or free-form help text;
        # their parameters were silently dropped, making the commands
        # unusable from the stack. Every annotation token must resolve to a
        # parser (or be a documented placeholder).
        argparsers = runtime.commands.argument_parser.parsers
        placeholders = {"...", "lon", "*"}  # consumed by the preceding parser
        seen = set()
        bad = []
        for cmd in runtime.commands.cmddict.values():
            if id(cmd) in seen or not isinstance(cmd, Command):
                continue
            seen.add(id(cmd))
            for annot, _isopt in cmd.arguments:
                message = f"{cmd.name}: whitespace/empty annotation token {annot!r}"
                assert annot == annot.strip(), message
                assert annot, message
                if annot in placeholders:
                    continue
                if all(argparsers.get(part) is None for part in annot.split("/")):
                    bad.append((cmd.name, annot))
        assert not bad, f"annotation tokens without a parser: {bad}"
