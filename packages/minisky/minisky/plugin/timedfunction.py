"""Timed function infrastructure for MiniSky plugins.

Provides hooks that are triggered at specific points in the simulation cycle:
- preupdate: Before traffic update each step
- update: After traffic update each step
- reset: On simulation reset
- hold: When simulation pauses

Each `TimedFunctionManager` owns the hooks and timers for one runtime.
"""
# TODO(abraham): delete this module with `init_plugin(runtime)`.

from __future__ import annotations

import functools
import inspect
from collections import OrderedDict
from collections.abc import Callable


class _Hook(OrderedDict[str, Callable[[], None]]):
    """Ordered dictionary of callbacks that can be triggered."""

    def trigger(self) -> None:
        """Call all registered callbacks."""
        for callback in tuple(self.values()):
            callback()


class Timer:
    """Timer class for simulation-time periodic functions.

    A timer fires every `dt` simulation seconds, quantised to whole simulation
    timesteps: the requested interval is converted to a step count relative to the
    current `sim.simdt`, so the actual interval is never smaller than one timestep.

    Attributes:
        name: Unique name of the timer (also the registry key).
        dt_default: Interval the timer was created with [s].
        dt_requested: Currently requested interval [s].
        dt_act: Actual interval after quantisation to whole timesteps [s].
        rel_freq: Number of simulation steps between firings.
        readynext: True when the timer fires on the current step.
    """

    def __init__(self, name: str, dt: float, get_simdt: Callable[[], float]) -> None:
        self.name = name
        self.dt_default = dt
        self.dt_requested = dt
        self.dt_act = dt
        self.counter = 0
        self.rel_freq = 1
        self.readynext = True
        self._get_simdt = get_simdt
        self._update_freq()

    def _update_freq(self) -> None:
        """Update the relative frequency based on current simdt."""
        simdt = self._get_simdt()
        self.rel_freq = max(1, int(self.dt_requested / simdt))
        self.dt_act = self.rel_freq * simdt

    def reset(self) -> None:
        """Reset timer to default state."""
        self.dt_requested = self.dt_default
        self.counter = 0
        self._update_freq()
        self.readynext = True

    def step(self) -> None:
        """Step is called each base timestep to update this timer."""
        self.counter = (self.counter or self.rel_freq) - 1
        self.readynext = self.counter == 0


class TimedFunctionManager:
    """Central manager for plugin lifecycle events.

    Provides a clean interface for simulation.py to trigger this runtime's
    plugin hooks without knowing about Timer or hook internals.
    """

    _hook_names = ("preupdate", "update", "reset", "hold")

    def __init__(self, get_simdt: Callable[[], float]) -> None:
        self._get_simdt = get_simdt
        self.timers: dict[str, Timer] = {}
        self.preupdate_hooks = _Hook()
        self.update_hooks = _Hook()
        self.reset_hooks = _Hook()
        self.hold_hooks = _Hook()

    def _hook(self, name: str) -> _Hook:
        if name not in self._hook_names:
            raise KeyError(f"No timing hook found with name {name}")
        return getattr(self, f"{name}_hooks")

    def register(
        self,
        func: Callable[..., None],
        *,
        name: str = "",
        dt: float = 0,
        hook: str | tuple[str, ...] = "update",
    ) -> Callable[..., None]:
        """Turn a function into a periodically timed function.

        Args:
            func: The function to register.
            name: Name for the timer (auto-generated if not provided).
            dt: Update interval in seconds (0 means every step).
            hook: Which hook to attach to (`update`, `preupdate`, `reset`, or
                `hold`).

        Returns:
            The original function.
        """
        # Generate a name if none is provided.
        timer_name = name or self._callback_name(func)
        hook_names = (hook,) if isinstance(hook, str) else hook

        if any(hook_name in ("update", "preupdate") for hook_name in hook_names):
            # Create a timer for update/preupdate hooks.
            timer = Timer(timer_name, dt, self._get_simdt)
            self.timers[timer_name] = timer
        else:
            timer = None

        # Check if function accepts dt argument.
        has_dt_param = "dt" in inspect.signature(func).parameters

        @functools.wraps(func)
        def callback() -> None:
            if timer is None:
                func()
            elif timer.readynext:
                if has_dt_param:
                    func(dt=float(timer.dt_act))
                else:
                    func()

        # Add callback to appropriate hook(s).
        for hook_name in hook_names:
            target = self._hook(hook_name)
            # For reset/hold, store the original function; for
            # update/preupdate, store the timed callback.
            registered = func if hook_name in ("reset", "hold") else callback
            target.setdefault(timer_name, registered)

        return func

    @staticmethod
    def _callback_name(func: Callable[..., None]) -> str:
        if inspect.ismethod(func):
            owner = func.__self__ if inspect.isclass(func.__self__) else type(func.__self__)
            return f"{owner.__name__}.{func.__name__}"
        return f"{func.__module__}.{func.__name__}"

    def preupdate(self) -> None:
        """Called before traffic update each simulation step."""
        for timer in self.timers.values():
            timer.step()
        self.preupdate_hooks.trigger()

    def update(self) -> None:
        """Called after traffic update each simulation step."""
        self.update_hooks.trigger()

    def reset(self) -> None:
        """Called on simulation reset."""
        for timer in self.timers.values():
            timer.reset()
        self.reset_hooks.trigger()

    def hold(self) -> None:
        """Called when simulation pauses."""
        self.hold_hooks.trigger()

    def clear(self) -> None:
        """Remove all callbacks and timers owned by this manager."""
        self.timers.clear()
        for name in self._hook_names:
            self._hook(name).clear()
