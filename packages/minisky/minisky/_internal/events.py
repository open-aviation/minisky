"""Structured runtime events and in-process fan-out.

It is heavily inspired by [`tracing_subscriber`](https://docs.rs/tracing-subscriber/latest/tracing_subscriber/).
Note that subscribers run on the publisher's thread. If the sink fails, it is
unsubscribed automatically. If you are using slow I/O, put it behind a
thread-safe queue.

To help applications track where exactly an event originated from, emitters
can create as many childs as they want. Conceptually it is similar to
[`tracing::span`](https://docs.rs/tracing/latest/tracing/span/index.html).
"""

from __future__ import annotations

import traceback
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import StrEnum
from threading import Lock
from typing import Annotated, Self, TypeAlias, assert_never

from annotated_doc import Doc

from minisky._internal.command import Text, command


class Severity(StrEnum):
    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


@dataclass(frozen=True, slots=True, init=False)
class EventPath:
    parts: tuple[str, ...]

    def __init__(self, *parts: str) -> None:
        object.__setattr__(self, "parts", parts)

    def child(self, *parts: str) -> EventPath:
        return EventPath(*self.parts, *parts)

    def __str__(self) -> str:
        return ".".join(self.parts)


@dataclass(frozen=True, slots=True)
class RuntimeSource:  # for core minisky
    path: EventPath = EventPath()


@dataclass(frozen=True, slots=True)
class PluginSource:
    plugin: str
    path: EventPath = EventPath()


EventSource: TypeAlias = RuntimeSource | PluginSource


@dataclass(frozen=True, slots=True)
class TextOutput:
    """User-facing text intentionally emitted outside a command response."""

    text: str


@dataclass(frozen=True, slots=True)
class Diagnostic:
    """Structured runtime diagnostic with optional captured exception context."""

    severity: Severity  # using tagged unions so we can easily serialise for now
    message: str
    # TODO(abraham): in tangram design a DTO for traceback
    exception: traceback.TracebackException | None = None


EventPayload: TypeAlias = TextOutput | Diagnostic


@dataclass(frozen=True, slots=True)
class RuntimeEvent:
    source: EventSource
    payload: EventPayload


EventSink: TypeAlias = Callable[[RuntimeEvent], None]
EventFilter: TypeAlias = Callable[[RuntimeEvent], bool]


@dataclass(slots=True)
class _EmitterLease:
    events: _EventBus | None


@dataclass(frozen=True, slots=True)
class EventEmitter:
    _lease: _EmitterLease
    _source: EventSource

    def child(self, *path: str) -> EventEmitter:
        """Derive an emitter whose lifetime remains tied to this capability."""
        if self._lease.events is None:
            raise RuntimeError("event emitter is revoked")
        return EventEmitter(self._lease, replace(self._source, path=self._source.path.child(*path)))

    def emit(self, payload: EventPayload) -> None:
        if (events := self._lease.events) is None:
            raise RuntimeError("event emitter is revoked")
        events._emit(RuntimeEvent(self._source, payload))

    def _revoke(self) -> None:
        self._lease.events = None


class EventSubscription:
    def __init__(self, events: _EventBus, token: int) -> None:
        self._events: _EventBus | None = events
        self._token = token

    def close(self) -> None:
        events, self._events = self._events, None
        if events is not None:
            events._unsubscribe(self._token)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()


@dataclass(frozen=True, slots=True)
class _Subscriber:
    sink: EventSink
    where: EventFilter | None


class _EventBus:
    def __init__(self) -> None:
        self._subscribers: dict[int, _Subscriber] = {}
        self._next_subscription = 0
        self._lock = Lock()

    def _emit(self, event: RuntimeEvent) -> None:
        with self._lock:
            subscribers = tuple(self._subscribers.items())
        for token, subscriber in subscribers:
            try:
                if subscriber.where is None or subscriber.where(event):
                    subscriber.sink(event)
            except Exception:  # ruff: ignore[BLE001] subscribers are external observers
                self._unsubscribe(token)

    def _emitter(self, source: EventSource) -> EventEmitter:
        return EventEmitter(_EmitterLease(self), source)

    def subscribe(self, sink: EventSink, *, where: EventFilter | None = None) -> EventSubscription:
        with self._lock:
            token = self._next_subscription
            self._next_subscription += 1
            self._subscribers[token] = _Subscriber(sink, where)
        return EventSubscription(self, token)

    def _unsubscribe(self, token: int) -> None:
        with self._lock:
            self._subscribers.pop(token, None)


@dataclass(frozen=True, slots=True)
class EventStream:
    """Observation-only view of runtime events."""

    _bus: _EventBus

    def subscribe(self, sink: EventSink, *, where: EventFilter | None = None) -> EventSubscription:
        """Register a sink, optionally filtering events."""
        return self._bus.subscribe(sink, where=where)


class EventCommands:
    def __init__(self, events: EventEmitter) -> None:
        self.events = events

    @command(name="ECHO", aliases=("PRINT",))
    def echo(
        self,
        text: Annotated[Text, Doc("Message to broadcast; each line is printed separately.")],
        *,
        flag: Annotated[int, Doc("Compatibility argument for minisky 0.0.1 (ignored).")] = 0,
    ) -> None:
        del flag
        self.events.emit(TextOutput(text))


# TODO(abraham): move this inside the CLI and have a tangram-specific handler.
# until then, we are not exposing it to the public API
def _render_event(  # pyright: ignore[reportUnusedFunction]
    event: RuntimeEvent,
) -> str:
    """Render an event for text-oriented sinks."""
    match event.payload:
        case TextOutput(text):
            return text
        case Diagnostic(severity, message, exception):
            match event.source:
                case RuntimeSource(path):
                    source = str(path)
                case PluginSource(plugin, path):
                    source = str(path.child(plugin)) if not path.parts else f"{plugin}.{path}"
                case _:
                    assert_never(event.source)
            heading = f"{severity.value.upper()} {source}: {message}"
            if exception is None:
                return heading
            return f"{heading}\n{''.join(exception.format()).rstrip()}"
    assert_never(event.payload)
