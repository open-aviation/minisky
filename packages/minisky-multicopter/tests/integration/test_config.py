"""Plugin configuration: user performance TOML and plugin-scoped settings.

These tests build their own runtimes with a `[plugins.multicopter]` table,
so they do not share the module-wide fixture runtime.
"""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
from pathlib import Path

import pytest
from minisky import MiniSky
from minisky.core.config import MiniSkyConfig
from minisky_multicopter.config import MulticopterTypeSpec, load_type_table
from minisky_multicopter.entity import get_multicopter
from minisky_multicopter.perf import MulticopterPerf
from pydantic import ValidationError

USER_TOML = """
# Override a built-in type and add a brand-new one.
[types.MAVIC]
battery_wh = 60.0

[types.MYDRONE]
battery_wh = 100.0
cds = 0.02
oew = 5.0
mtow = 8.0
n_engines = 4
engine_kw = 0.5
v_min = -10.0
v_max = 20.0
vs_min = -5.0
vs_max = 5.0
h_max = 3000.0
d_range_max = 20.0
"""


@pytest.fixture
def user_toml(tmp_path: Path) -> Path:
    path = tmp_path / "multicopter.toml"
    path.write_text(USER_TOML)
    return path


@pytest.fixture
def configured_runtime(user_toml: Path) -> Iterator[MiniSky]:
    """Runtime with the plugin configured from a `[plugins.multicopter]` table."""
    config = MiniSkyConfig.model_validate(
        {
            "plugins": {
                "multicopter": {
                    "capture_radius": 25.0,
                    "performance_path": str(user_toml),
                    "soc_low": 0.5,
                }
            }
        }
    )
    instance = MiniSky(config)
    result = asyncio.run(instance.plugins.load("MULTICOPTER"))
    assert result.is_ok(), result.err()
    # The reset hook selects the multicopter implementations immediately,
    # before any aircraft is created (preupdate only fires in OP state).
    instance.simulation.reset()
    yield instance
    asyncio.run(instance.aclose())


class TestTypeTable:
    def test_builtin_table_loads_and_carries_membership(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        # Isolate from any real user file in the platform cache directory.
        monkeypatch.setattr(
            "minisky_multicopter.config.default_performance_path",
            lambda: tmp_path / "absent.toml",
        )
        table = load_type_table()
        assert {"MAVIC", "PHAN4", "M100", "M200", "M600", "MNET", "AMZN", "HORSEFLY"} <= set(table)
        assert table["MAVIC"].battery_wh == pytest.approx(43.6)
        assert table["AMZN"].battery_wh is None  # range-derived

    def test_user_file_overrides_and_extends(self, user_toml: Path) -> None:
        table = load_type_table(user_toml)
        assert table["MAVIC"].battery_wh == pytest.approx(60.0)
        assert table["MYDRONE"].has_airframe()
        assert "PHAN4" in table  # built-ins not named in the user file survive

    def test_explicit_path_must_exist(self, tmp_path: Path) -> None:
        with pytest.raises(FileNotFoundError):
            load_type_table(tmp_path / "missing.toml")

    def test_incomplete_airframe_block_is_rejected(self) -> None:
        with pytest.raises(ValidationError, match="incomplete airframe data"):
            MulticopterTypeSpec(oew=5.0, mtow=8.0)

    def test_unknown_keys_are_rejected(self) -> None:
        with pytest.raises(ValidationError):
            MulticopterTypeSpec.model_validate({"battery_hw": 60.0})


class TestConfiguredRuntime:
    def test_plugin_settings_reach_the_entity(self, configured_runtime: MiniSky) -> None:
        mc = get_multicopter(configured_runtime.traffic)
        assert mc is not None
        assert mc.config.capture_radius == pytest.approx(25.0)
        assert mc.config.soc_low == pytest.approx(0.5)
        assert mc.config.lowbatt_spd_factor == pytest.approx(0.6)  # default survives
        assert mc.typespecs["MAVIC"].battery_wh == pytest.approx(60.0)

    def test_user_defined_type_flies_with_battery(self, configured_runtime: MiniSky) -> None:
        configured_runtime.commands.stack("CRE D1,MYDRONE,52,4,90,50,10")
        for _ in range(3):
            configured_runtime.simulation.step()

        traf = configured_runtime.traffic
        mc = get_multicopter(traf)
        assert mc is not None
        assert bool(mc.ismulticopter[0])
        perf = traf.perf
        assert isinstance(perf, MulticopterPerf)
        assert perf.capacity[0] == pytest.approx(100.0 * 3600.0)
        assert perf.mass[0] == pytest.approx(6.5)  # 0.5 * (oew + mtow)
        assert perf.vmax[0] == pytest.approx(20.0)
        assert perf.power[0] > 0.0

    def test_invalid_config_fails_plugin_load(self, user_toml: Path) -> None:
        config = MiniSkyConfig.model_validate(
            {"plugins": {"multicopter": {"capture_radius": -1.0}}}
        )
        instance = MiniSky(config)
        try:
            result = asyncio.run(instance.plugins.load("MULTICOPTER"))
            assert result.is_err()
        finally:
            asyncio.run(instance.aclose())
