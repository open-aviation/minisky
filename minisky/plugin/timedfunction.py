"""Timed function infrastructure for MiniSky plugins.

Provides hooks that are triggered at specific points in the simulation cycle:
- preupdate: Before traffic update each step
- update: After traffic update each step
- reset: On simulation reset
- hold: When simulation pauses
"""
import inspect
import functools
from collections import OrderedDict
from types import SimpleNamespace

import minisky


class _Hook(OrderedDict):
    """Ordered dictionary of callbacks that can be triggered."""

    def trigger(self):
        """Call all registered callbacks."""
        for callback in self.values():
            callback()


# Dictionaries of timed functions for different trigger points
hooks = SimpleNamespace(
    update=_Hook(),
    preupdate=_Hook(),
    hold=_Hook(),
    reset=_Hook()
)


class Timer:
    """Timer class for simulation-time periodic functions.

    A timer fires every ``dt`` simulation seconds, quantised to whole simulation
    timesteps: the requested interval is converted to a step count relative to the
    current ``sim.simdt``, so the actual interval is never smaller than one timestep.

    Attributes:
        name: Unique name of the timer (also the registry key).
        dt_default: Interval the timer was created with [s].
        dt_requested: Currently requested interval [s].
        dt_act: Actual interval after quantisation to whole timesteps [s].
        rel_freq: Number of simulation steps between firings.
        readynext: True when the timer fires on the current step.
    """

    _timers = {}

    def __init__(self, name, dt):
        self.name = name
        self.dt_default = dt
        self.dt_requested = dt
        self.dt_act = dt
        self.counter = 0
        self.rel_freq = 1
        self.readynext = True
        Timer._timers[name] = self
        self._update_freq()

    def _update_freq(self):
        """Update the relative frequency based on current simdt."""
        simdt = getattr(minisky.sim, 'simdt', 1.0) if minisky.sim else 1.0
        self.rel_freq = max(1, int(self.dt_requested / simdt))
        self.dt_act = self.rel_freq * simdt

    def reset(self):
        """Reset timer to default state."""
        self.dt_requested = self.dt_default
        self.counter = 0
        self._update_freq()
        self.readynext = True

    def step(self):
        """Step is called each base timestep to update this timer."""
        self.counter = (self.counter or self.rel_freq) - 1
        self.readynext = self.counter == 0

    @classmethod
    def timers(cls):
        """Return all registered timers."""
        return cls._timers.values()

    @classmethod
    def step_all(cls):
        """Step all timers."""
        for timer in cls._timers.values():
            timer.step()

    @classmethod
    def reset_all(cls):
        """Reset all timers."""
        for timer in cls._timers.values():
            timer.reset()


def timed_function(func=None, name='', dt=0, hook='update'):
    """Decorator to turn a function into a periodically timed function.

    Args:
        func: The function to decorate
        name: Name for the timer (auto-generated if not provided)
        dt: Update interval in seconds (0 means every step)
        hook: Which hook to attach to ('update', 'preupdate', 'reset', 'hold')

    Example:
        @timed_function(name='myplugin', dt=5, hook='update')
        def my_update():
            # Called every 5 simulation seconds
            pass
    """
    def deco(func):
        # Generate a name if none is provided
        if not name:
            if inspect.ismethod(func):
                if inspect.isclass(func.__self__):
                    tname = f'{func.__self__.__name__}.{func.__name__}'
                else:
                    tname = f'{func.__self__.__class__.__name__}.{func.__name__}'
            else:
                tname = f'{func.__module__}.{func.__name__}'
        else:
            tname = name

        if 'update' in hook or 'preupdate' in hook:
            # Create a timer for update/preupdate hooks
            timer = Timer(tname, dt)

            # Check if function accepts dt argument
            has_dt_param = 'dt' in inspect.signature(func).parameters

            if has_dt_param:
                @functools.wraps(func)
                def callback(*args):
                    if timer.readynext:
                        func(*args, dt=float(timer.dt_act))
            else:
                @functools.wraps(func)
                def callback(*args):
                    if timer.readynext:
                        func(*args)
        else:
            # For reset/hold hooks, just wrap the function directly
            @functools.wraps(func)
            def callback(*args):
                func(*args)

        # Add callback to appropriate hook(s)
        hooknames = hook if isinstance(hook, (list, tuple)) else (hook,)
        for hookname in hooknames:
            target = getattr(hooks, hookname, None)
            if target is None:
                raise KeyError(f'No timing hook found with name {hookname}')
            if tname not in target:
                # For reset/hold, store the original function, for update/preupdate store callback
                target[tname] = func if hookname in ('reset', 'hold') else callback

        return func

    # Allow both @timed_function and @timed_function(args)
    return deco(func) if func else deco


class PluginManager:
    """Central manager for plugin lifecycle events.

    Provides a clean interface for simulation.py to trigger plugin hooks
    without knowing about Timer or hooks internals.
    """

    @staticmethod
    def preupdate():
        """Called before traffic update each simulation step."""
        Timer.step_all()
        hooks.preupdate.trigger()

    @staticmethod
    def update():
        """Called after traffic update each simulation step."""
        hooks.update.trigger()

    @staticmethod
    def reset():
        """Called on simulation reset."""
        Timer.reset_all()
        hooks.reset.trigger()

    @staticmethod
    def hold():
        """Called when simulation pauses."""
        hooks.hold.trigger()
