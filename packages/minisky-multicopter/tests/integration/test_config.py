from __future__ import annotations

import asyncio
from pathlib import Path

import pytest
from minisky import MiniSky
from minisky.core.config import MiniSkyConfig
from minisky_multicopter.config import load_type_table
from minisky_multicopter.entity import get_multicopter
from minisky_multicopter.perf import MulticopterPerf

USER_TOML = """
[types.MYDRONE]
battery_energy = 100.0

[types.MYDRONE.airframe]
oew = 5.0
mtow = 8.0
n_engines = 4
engine_power = 500.0
v_min = -10.0
v_max = 20.0
vs_min = -5.0
vs_max = 5.0
h_max = 3000.0
range_max = 20000.0
"""


@pytest.fixture
def user_toml(tmp_path: Path) -> Path:
    path = tmp_path / "config.toml"
    path.write_text(USER_TOML)
    return path


def test_default_user_file(monkeypatch: pytest.MonkeyPatch, user_toml: Path) -> None:
    monkeypatch.setattr(
        "minisky_multicopter.config.default_user_performance_path",
        lambda: user_toml,
    )
    assert set(load_type_table()) == {"MYDRONE"}


def test_user_defined_type(user_toml: Path) -> None:
    config = MiniSkyConfig.model_validate(
        {"plugins": {"multicopter": {"performance_path": str(user_toml)}}}
    )
    instance = MiniSky(config)
    try:
        result = asyncio.run(instance.plugins.load("MULTICOPTER"))
        assert result.is_ok(), result.err()
        instance.simulation.reset()
        instance.commands.stack("CRE D1,MYDRONE,52,4,90,50,10KT[CAS]")
        for _ in range(3):
            instance.simulation.step()

        mc = get_multicopter(instance.traffic)
        assert mc is not None
        assert bool(mc.ismulticopter[0])
        perf = instance.traffic.perf
        assert isinstance(perf, MulticopterPerf)
        assert perf.capacity[0] == pytest.approx(100.0 * 3600.0)
        assert perf.power[0] > 0.0
    finally:
        asyncio.run(instance.aclose())
