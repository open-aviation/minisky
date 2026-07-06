"""Integration tests for the command stack (queueing, processing, echo output)."""

from io import StringIO

import pytest

import minisky

FT = 0.3048
KTS = 0.514444


class TestQueueing:
    def test_stack_only_queues(self, bs, sim):
        minisky.stack.stack("CRE KL204,B744,52,4,45,FL250,350")
        assert bs.traf.ntraf == 0  # not executed yet

    def test_command_executes_on_step(self, bs, sim):
        minisky.stack.stack("CRE KL204,B744,52,4,45,FL250,350")
        bs.sim.step()
        assert bs.traf.ntraf == 1
        assert bs.traf.callsign[0] == "KL204"


class TestCommands:
    def test_cre_via_stack(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert bs.traf.ntraf == 1
        assert bs.traf.alt[0] == pytest.approx(25000 * FT, rel=1e-3)

    def test_pos_outputs_callsign(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        output = run_cmd("POS KL204")
        assert "KL204" in output

    def test_bare_callsign_defaults_to_pos(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        output = run_cmd("KL204")
        assert "KL204" in output

    def test_alt_sets_selected_altitude(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("ALT KL204 FL260")
        assert bs.traf.selalt[0] == pytest.approx(26000 * FT, rel=1e-3)

    def test_hdg_sets_autopilot_track(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("HDG KL204 340")
        assert bs.traf.ap.trk[0] == pytest.approx(340.0)
        assert not bs.traf.swlnav[0]

    def test_spd_sets_selected_speed(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("SPD KL204 300")
        assert bs.traf.selspd[0] == pytest.approx(300 * KTS, rel=1e-3)

    def test_del_removes_aircraft(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert bs.traf.ntraf == 1
        run_cmd("DEL KL204")
        assert bs.traf.ntraf == 0

    def test_mcre_via_stack(self, bs, run_cmd):
        run_cmd("MCRE 3")
        assert bs.traf.ntraf == 3


class TestReadscn:
    def test_short_command_line_survives(self, bs):
        # "0:00:00>OP" is only 10 characters; it used to be dropped by a
        # minimum-length check meant to skip empty lines.
        lines = list(minisky.stack.readscn(StringIO("0:00:00>OP\n")))
        assert lines == [(0.0, "OP")]

    def test_blank_and_comment_lines_skipped(self, bs):
        scn = StringIO("# a comment\n\n0:00:01>HOLD\n")
        lines = list(minisky.stack.readscn(scn))
        assert lines == [(1.0, "HOLD")]


class TestHelp:
    def test_help_writes_command_reference(self, bs, sim, tmp_path, monkeypatch):
        # HELP >filename writes the reference to ./docs/<filename>
        monkeypatch.chdir(tmp_path)
        (tmp_path / "docs").mkdir()
        success, msg = minisky.stack.showhelp(">ref.txt")
        assert success
        ref = tmp_path / "docs" / "ref.txt"
        assert ref.exists(), msg
        content = ref.read_text()
        assert content.startswith("Command\tDescription\tUsage")
        assert "\nCRE\t" in content


class TestVarExplorer:
    def test_variable_get_without_index(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        v = minisky.core.varexplorer.findvar("traf.ntraf")
        assert v is not None
        assert v.get() == 1
        assert v.get_type() == "int"

    def test_variable_get_with_index(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        v = minisky.core.varexplorer.findvar("traf.callsign[0]")
        assert v is not None
        assert v.get() == ["KL204"]


class TestSynonyms:
    def test_airway_synonyms_point_to_pos(self, bs):
        cmddict = minisky.stack.Command.cmddict
        assert cmddict["AIRWAY"] is cmddict["POS"]
        assert cmddict["AIRWAYS"] is cmddict["POS"]


class TestErrors:
    def test_unknown_command_echoes_error(self, bs, run_cmd):
        output = run_cmd("BOGUSCMD 42")
        assert "unknown command" in output.lower()

    def test_command_on_missing_aircraft_reports_error(self, bs, run_cmd):
        output = run_cmd("ALT NOSUCH FL100")
        assert output  # some error text is echoed
        assert bs.traf.ntraf == 0

    def test_sim_survives_bad_command(self, bs, run_cmd):
        run_cmd("THISDOESNOTEXIST")
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert bs.traf.ntraf == 1


class TestArgumentSpecs:
    def test_all_registered_specs_resolve_to_parsers(self, bs):
        # Several commands (AT, DIRECT, AFTER, RESOOFF, ...) were registered
        # with argument specs containing whitespace or free-form help text;
        # their parameters were silently dropped, making the commands
        # unusable from the stack. Every annotation token must resolve to a
        # parser (or be a documented placeholder).
        from minisky.stack import Command
        from minisky.stack.argparser import argparsers

        placeholders = {"...", "lon", "*"}  # consumed by the preceding parser
        seen = set()
        bad = []
        for cmd in Command.cmddict.values():
            if id(cmd) in seen:
                continue
            seen.add(id(cmd))
            for annot, _isopt in cmd.arguments:
                assert annot == annot.strip() and annot, (
                    f"{cmd.name}: whitespace/empty annotation token {annot!r}"
                )
                if annot in placeholders:
                    continue
                if all(argparsers.get(part) is None for part in annot.split("/")):
                    bad.append((cmd.name, annot))
        assert not bad, f"annotation tokens without a parser: {bad}"
