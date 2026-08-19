"""Integration tests owned by the custom-autopilot example package."""

import pytest
from minisky import Autopilot, MiniSky, MiniSkyConfig
from minisky_example_customautopilot import CustomAutoPilot


@pytest.mark.anyio
async def test_replacement_is_runtime_local_and_removed_on_shutdown() -> None:
    runtime_a = MiniSky(MiniSkyConfig())
    runtime_b = MiniSky(MiniSkyConfig())
    try:
        assert runtime_a.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT").is_err()
        assert runtime_b.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT").is_err()

        alt_callback = runtime_a.commands.cmddict["ALT"].forms[0].callback
        result = await runtime_a.plugins.load("CUSTOMAUTOPILOT")
        assert result.is_ok(), result.err()
        assert runtime_a.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT").is_ok()
        assert type(runtime_a.traffic.ap) is CustomAutoPilot
        assert runtime_b.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT").is_err()

        await runtime_a.plugins.aclose()
        assert type(runtime_a.traffic.ap) is Autopilot
        assert runtime_a.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT").is_err()
        assert runtime_a.commands.cmddict["ALT"].forms[0].callback is alt_callback
    finally:
        await runtime_a.aclose()
        await runtime_b.aclose()
