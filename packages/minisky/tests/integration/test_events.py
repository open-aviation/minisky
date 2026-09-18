from minisky import MiniSky, RuntimeEvent, TextOutput
from minisky._internal.simulation import Simulation


def test_subscriptions_filter_and_isolate_failures(runtime: MiniSky, sim: Simulation) -> None:
    text_events: list[RuntimeEvent] = []
    sink_failures = 0
    filter_failures = 0

    def fail_sink(_event: RuntimeEvent) -> None:
        nonlocal sink_failures
        sink_failures += 1
        raise RuntimeError("sink failed")

    def fail_filter(_event: RuntimeEvent) -> bool:
        nonlocal filter_failures
        filter_failures += 1
        raise RuntimeError("filter failed")

    with (
        runtime.events.subscribe(fail_sink),
        runtime.events.subscribe(lambda _event: None, where=fail_filter),
        runtime.events.subscribe(
            text_events.append, where=lambda event: isinstance(event.payload, TextOutput)
        ),
    ):
        sim.reset()
        runtime.commands.stack("ECHO first")
        assert runtime.commands.process()
        runtime.commands.stack("ECHO second")
        assert runtime.commands.process()

    assert sink_failures == 1
    assert filter_failures == 1
    assert [event.payload for event in text_events] == [TextOutput("first"), TextOutput("second")]
