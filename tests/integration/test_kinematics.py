"""Integration tests for the replaceable Kinematics entity (Phase 1).

Kinematics was factored out of Traffic so that flight-state integration
becomes hot-swappable via SELECTIMPL KINEMATICS <IMPL>. These tests cover
the base entity wiring and the replaceable round-trip (select + revert).
"""

from __future__ import annotations

from minisky import MiniSky
from minisky.simulation import Simulation
from minisky.traffic.kinematics import Kinematics


class TaggedKinematics(Kinematics):
    """Trivial Kinematics subclass used to exercise SELECTIMPL KINEMATICS.

    Registered runtime-locally in the test (implementations no longer
    auto-register on subclass definition); it only becomes active when
    explicitly selected.
    """

    def update(self) -> None:
        super().update()
        self.tag = "tagged"


class TestKinematicsEntity:
    def test_kinematics_owns_ax_array(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
        sim.step()
        # acceleration array lives on the kinematics entity, sized per-aircraft
        assert len(runtime.traffic.kinematics.ax) == 1

    def test_base_kinematics_integrates_position(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
        lon0 = float(runtime.traffic.lon[0])
        sim.step()
        # heading 090 at positive ground speed advances longitude eastward
        assert runtime.traffic.lon[0] > lon0

    def test_selectimpl_lists_base_implementation(self, runtime: MiniSky, sim: Simulation) -> None:
        ok, msg = runtime.replaceables.select("KINEMATICS")
        assert ok
        assert "KINEMATICS" in msg.upper()

    def test_select_subclass_takes_effect_and_reverts_on_reset(
        self, runtime: MiniSky, sim: Simulation
    ) -> None:
        runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)

        prepared = runtime.replaceables.prepare(TaggedKinematics)
        runtime.replaceables.validate((prepared,))
        runtime.replaceables.install((prepared,))
        try:
            ok, msg = runtime.replaceables.select("KINEMATICS", "TAGGEDKINEMATICS")
            assert ok, msg
            assert isinstance(runtime.traffic.kinematics, TaggedKinematics)
            # per-aircraft arrays carry over to the new instance
            assert len(runtime.traffic.kinematics.ax) == 1

            sim.step()
            assert getattr(runtime.traffic.kinematics, "tag", None) == "tagged"

            # reset restores the default implementation
            runtime.simulation.reset()
            assert type(runtime.traffic.kinematics) is Kinematics
        finally:
            runtime.replaceables.remove((prepared,))
