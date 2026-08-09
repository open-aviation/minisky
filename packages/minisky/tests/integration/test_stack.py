"""Integration tests for the command stack (queueing, processing, echo output)."""

from __future__ import annotations

import asyncio
import gc
import weakref
from io import StringIO

import numpy as np
import pytest
from minisky import Err, MiniSky, Ok
from minisky.command import ArgumentIssue, command
from minisky.simulation import Simulation
from minisky.stack import ScheduledCommand
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

    def test_malformed_batch(self, runtime: MiniSky) -> None:
        runtime.commands.stack('ECHO "unterminated')
        assert "expected a closing" in runtime.console.read_output_buffer().lower()
        assert runtime.commands.cmdstack == []


class TestCommandMounting:
    def test_mount_components_rejects_duplicates_atomically(self, runtime: MiniSky) -> None:
        # Provider mounting is a batch operation. A conflict discovered in the
        # second provider must not leave commands from the first installed.
        class First:
            @command(name="DUPLICATE")
            def command(self) -> bool:
                return True

        class Second:
            @command(name="DUPLICATE")
            def command(self) -> bool:
                return True

        with pytest.raises(ValueError, match="repeated in batch"):
            runtime.commands.mount_components((First(), Second()))
        assert "DUPLICATE" not in runtime.commands.cmddict

    def test_replaceable_dispatch_does_not_retain_previous_implementation(
        self, runtime: MiniSky
    ) -> None:
        # Command callbacks are compiled before replacements are selected. The
        # dispatch closure must follow the slot without retaining the old object.
        command = runtime.commands.cmddict["RMETHH"]
        previous = runtime.traffic.cr
        previous_ref = weakref.ref(previous)

        before = command("")
        assert isinstance(before, (Ok, Err))
        assert before.is_err()
        assert runtime.replaceables.select("ConflictResolution", "MVP").is_ok()
        after = command("")
        assert isinstance(after, (Ok, Err))
        assert after.is_ok()

        del previous
        gc.collect()
        assert previous_ref() is None


class TestTypedGrammar:
    def test_area_commands_keep_multi_token_aviation_values(self, runtime: MiniSky) -> None:
        # Position and altitude parsers consume a variable number of tokens.
        # Plain float varargs silently lost hemisphere and flight-level syntax.
        result = runtime.commands.cmddict["BOX"]("TEST N52 E004 N53 E005 FL100 FL200")
        assert isinstance(result, (Ok, Err))
        assert result.is_ok(), result.err()
        assert runtime.areas.has_area("TEST")

    def test_group_selection_is_kept_for_compatible_commands(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        run_cmd("CRE ONE A320 52 4 90 FL100 250")
        run_cmd("CRE TWO A320 53 5 90 FL100 250")
        run_cmd("GROUP TEAM ONE TWO")

        run_cmd("ALT TEAM FL200")
        run_cmd("BANK TEAM 20")
        assert runtime.traffic.selalt.tolist() == pytest.approx([20000 * FT, 20000 * FT])
        assert np.degrees(runtime.traffic.ap.bankdef).tolist() == pytest.approx([20.0, 20.0])

        result = runtime.commands.cmddict["DELRTE"]("TEAM")
        assert isinstance(result, Err)
        issue = result.err()
        assert isinstance(issue, ArgumentIssue)
        assert issue.message == "argument `acidx`: expected an aircraft, but got group TEAM"


class TestReadscn:
    def test_short_command_line_survives(self, runtime: MiniSky) -> None:
        # "0:00:00>OP" is only 10 characters; it used to be dropped by a
        # minimum-length check meant to skip empty lines.
        lines = list(runtime.commands.readscn(StringIO("0:00:00>OP\n")))
        assert lines == [ScheduledCommand(0.0, "OP")]

    def test_blank_and_comment_lines_skipped(self, runtime: MiniSky) -> None:
        scn = StringIO("# a comment\n\n0:00:01>HOLD\n")
        lines = list(runtime.commands.readscn(scn))
        assert lines == [ScheduledCommand(1.0, "HOLD")]


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
