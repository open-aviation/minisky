"""Entity base class for MiniSky singleton plugins.

Entity extends TrafficArrays to add:
- Singleton behavior (only one instance per class)
- Proxy support for runtime hot-swapping of implementations

Since TrafficArrays already provides the replaceable pattern,
Entity just adds singleton semantics on top.

Usage:
    class MyPlugin(Entity):
        def __init__(self):
            super().__init__()
            with self.settrafarrays():
                self.mydata = np.array([])

For non-singleton TrafficArrays that are replaceable,
just inherit from TrafficArrays directly.
"""
import inspect
from typing import ClassVar, Optional

from minisky.core.trafficarrays import TrafficArrays


class Proxy:
    """Proxy class for replaceable singleton entities.

    Allows plugins to replace core functionality by routing all
    attribute access through the currently selected implementation.
    """

    def __init__(self):
        self.__dict__['_refobj'] = None
        self.__dict__['_proxied'] = list()

    def _selected(self):
        """Return the class of the currently selected implementation."""
        return self._refobj.__class__

    def _replace(self, refobj):
        """Replace the reference object with a new implementation."""
        self.__dict__['_refobj'] = refobj
        # Clear all proxied functions/methods
        for name in self._proxied:
            delattr(self, name)
        self._proxied.clear()
        # Copy all public functions/methods of reference object
        for name, value in inspect.getmembers(refobj, callable):
            if name[0] != '_':
                self.__dict__[name] = value
                self._proxied.append(name)

    def __getattr__(self, attr):
        return getattr(self._refobj, attr)

    def __setattr__(self, name, value):
        return setattr(self._refobj, name, value)


def isproxied(obj):
    """Returns True if obj is a proxied object."""
    return isinstance(obj, Proxy)


def getproxied(obj):
    """Return wrapped proxy object if proxied, otherwise the original object."""
    return obj.__dict__['_refobj'] if isinstance(obj, Proxy) else obj


class EntityMeta(type):
    """Meta class to make Entity subclasses singletons."""

    def __call__(cls, *args, **kwargs):
        """Object creation with proxy wrapping and singleton behavior."""
        # Create singleton instance if it doesn't exist yet
        if not cls.is_instantiated():
            super().__call__(*args, **kwargs)

        # When proxied, calling base constructor returns the proxy
        if cls._proxy and cls is cls._baseimpl:
            if getproxied(cls._proxy) is None:
                cls.select(cls._instance)
            return cls._proxy

        return cls._instance


class Entity(TrafficArrays, metaclass=EntityMeta):
    """Base class for MiniSky singleton entities with TrafficArrays.

    Combines TrafficArrays (replaceable per-aircraft data) with
    singleton behavior (one instance per class).

    Usage:
        class MyPlugin(Entity):
            def __init__(self):
                super().__init__()
                with self.settrafarrays():
                    self.mydata = np.array([])

            def create(self, n=1):
                super().create(n)
                self.mydata[-n:] = default_values
    """

    # Singleton instance tracking
    _proxy: ClassVar[Optional[Proxy]] = None
    _instance: ClassVar[Optional['Entity']] = None

    def __init_subclass__(cls, **kwargs):
        """Called when a subclass is defined."""
        super().__init_subclass__(**kwargs)

        # Each Entity subclass keeps its own singleton instance
        cls._instance = None

        # First-level Entity subclasses get a proxy for hot-swapping
        if not hasattr(cls, '_proxy') or cls._proxy is None:
            if cls._baseimpl is not None:  # Is this a replaceable class?
                cls._proxy = Proxy()

    @classmethod
    def select(cls, instance=None):
        """Select this class/instance as the active implementation."""
        # Call parent's select to update _generator
        super().select()

        # Handle singleton instance
        if instance is None:
            instance = cls._instance or cls()

        cls._baseimpl._instance = instance
        if cls._proxy:
            cls._proxy._replace(instance)

    @classmethod
    def is_instantiated(cls):
        """Returns True if the singleton has been instantiated."""
        return cls._instance is not None

    @classmethod
    def instance(cls):
        """Return the current instance (proxy if replaceable, else instance)."""
        return cls._proxy or cls._instance

    def __init__(self):
        super().__init__()
        cls = type(self)
        if cls._instance is None:
            cls._instance = self
