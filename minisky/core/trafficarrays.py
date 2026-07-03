"""TrafficArrays: Base class for per-aircraft data arrays.

Classes that derive from TrafficArrays get automated create, delete, and reset
functionality for all registered child arrays. All subclasses are automatically
replaceable via SELECTIMPL - see minisky/plugin/ for usage examples.
"""

from collections.abc import Collection
from typing import ClassVar, Optional

import numpy as np

defaults = {"float": 0.0, "int": 0, "uint": 0, "bool": False, "S": "", "str": ""}

# Global dictionary of replaceable classes
replaceables: dict[str, type['TrafficArrays']] = {}


def reset_replaceables():
    """Reset all replaceables to their default implementation and reinstantiate on traf."""
    for base in replaceables.values():
        base.selectdefault()
        # Reinstantiate on traf with default implementation
        _replace_instance_on_traf(base, base._generator)


def select_implementation(basename='', implname=''):
    """Select an implementation for a replaceable class.

    Arguments:
    - basename: Name of the replaceable base class (e.g., 'AUTOPILOT')
    - implname: Name of the implementation to select (e.g., 'MYAUTOPILOT')

    Returns: (success, message) tuple
    """
    if not basename:
        return True, 'Replaceable classes in MiniSky:\n' + ', '.join(replaceables)

    base = replaceables.get(basename.upper())
    if not base:
        return False, f'Replaceable {basename} not found.'

    impls = base.derived()
    if not implname:
        current = base._generator.__name__
        return True, (f'Current implementation for {basename}: {current}\n'
                      f'Available implementations: {", ".join(impls)}')

    impl = impls.get(base.__name__ if implname.upper() == 'BASE' else implname.upper())
    if not impl:
        return False, f'Implementation {implname} not found for {basename}.'

    impl.select()

    # Replace existing instance on traf if it exists
    _replace_instance_on_traf(base, impl)

    return True, f'Selected {implname} for {basename}'


def _replace_instance_on_traf(base, impl):
    """Replace existing instance of base class on traf with new impl instance.

    This ensures SELECTIMPL takes effect immediately, not just for future instantiations.
    """
    import minisky
    if minisky.traf is None:
        return

    # Find attribute on traf that is an instance of the base class
    for attr_name, attr_value in minisky.traf.__dict__.items():
        if isinstance(attr_value, base):
            # Create new instance of selected implementation
            new_instance = impl()
            # Copy over any per-aircraft array data from old instance (if they exist)
            for arr_var in getattr(attr_value, '_ArrVars', []):
                if hasattr(new_instance, arr_var):
                    setattr(new_instance, arr_var, getattr(attr_value, arr_var))
            for lst_var in getattr(attr_value, '_LstVars', []):
                if hasattr(new_instance, lst_var):
                    setattr(new_instance, lst_var, getattr(attr_value, lst_var))
            # Replace on traf
            setattr(minisky.traf, attr_name, new_instance)
            # Stack commands registered as bound methods of the old instance
            # would silently mutate the orphaned object; rebind them
            _rebind_stack_commands(attr_value, new_instance)
            break


def _rebind_stack_commands(old_instance, new_instance):
    """Rebind stack command callbacks from old_instance to new_instance."""
    import inspect

    from minisky.stack import Command

    for cmdobj in set(Command.cmddict.values()):
        callback = cmdobj.callback
        if inspect.ismethod(callback) and callback.__self__ is old_instance:
            cmdobj.callback = getattr(
                new_instance, callback.__func__.__name__, callback
            )


class RegisterElementParameters:
    """Class to use in 'with'-syntax. This class automatically
    calls for the _init_trafarrays function of the
    DynamicArray, with all parameters defined in 'with'."""

    def __init__(self, parent):
        self._parent = parent
        self.keys0 = set(parent.__dict__.keys())

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_value, tb):
        self._parent._init_trafarrays(set(self._parent.__dict__.keys()) - self.keys0)


class TrafficArrays:
    """Parent class to use separate arrays and lists to allow
    vectorizing but still maintain and object like benefits
    for creation and deletion of an element for all parameters.

    Supports the replaceable pattern when subclassed with replaceable=True:
        class Autopilot(TrafficArrays, replaceable=True):
            ...
    """

    # The TrafficArrays class keeps track of all of the constructed
    # TrafficArray objects
    root = None
    ntraf = 0

    # Replaceable pattern class variables (set per-subclass)
    _baseimpl: ClassVar[Optional[type]] = None
    _generator: ClassVar[type]
    _default: ClassVar[str] = ''

    @staticmethod
    def setroot(obj):
        """This function is used to set the root of the tree of TrafficArray
        objects (which is the traffic object.)"""
        TrafficArrays.root = obj

    def __init_subclass__(cls, **kwargs):
        """Called when a subclass is defined.

        This is the key to load-order independence: Python calls this
        automatically when any class subclasses TrafficArrays or its descendants.
        The subclass is registered and can be selected later.

        All first-level subclasses become replaceable base implementations.
        Further subclasses inherit the _baseimpl and can be selected as
        alternative implementations.
        """
        super().__init_subclass__(**kwargs)

        # Each subclass can generate instances of itself
        cls._generator = cls

        # First-level subclasses become base implementations (all are replaceable)
        if not hasattr(cls, '_baseimpl') or cls._baseimpl is None:
            cls._baseimpl = cls
            cls._default = ''
            replaceables[cls.__name__.upper()] = cls

    def __new__(cls, *args, **kwargs):
        """Factory method: calling base class instantiates selected implementation.

        This is what makes the replaceable pattern work:
        - When you call Autopilot(), if Autopilot is the base class,
          it actually creates an instance of _generator (the selected impl)
        - When you call MyAutopilot() directly, it creates MyAutopilot
        """
        # Only apply factory pattern for replaceable classes
        if cls._baseimpl is not None:
            # If calling the base class, use the generator (selected implementation)
            # If calling a subclass directly, use that subclass
            generator = cls._generator if cls is cls._baseimpl else cls
            return object.__new__(generator)
        return object.__new__(cls)

    @classmethod
    def setdefault(cls, name):
        """Set a default implementation by name."""
        if cls._baseimpl is None:
            return
        impl = cls._baseimpl.derived().get(name.upper())
        if impl:
            cls._baseimpl._default = name.upper()
            cls._baseimpl._generator = impl

    @classmethod
    def getdefault(cls):
        """Get the default implementation class."""
        if cls._baseimpl is None:
            return cls
        default = cls._baseimpl._default
        return cls._baseimpl.derived().get(default) if default else cls._baseimpl

    @classmethod
    def selectdefault(cls):
        """Select the default implementation."""
        if cls._baseimpl is None:
            return
        base = cls._baseimpl
        base.derived().get(base._default, base).select()

    @classmethod
    def select(cls):
        """Select this class as the active implementation."""
        if cls._baseimpl is None:
            return
        cls._baseimpl._generator = cls

    @classmethod
    def selected(cls):
        """Return the currently selected implementation class."""
        if cls._baseimpl is None:
            return cls
        return cls._baseimpl._generator

    @classmethod
    def derived(cls):
        """Recursively find all derived classes."""
        ret = {cls.__name__.upper(): cls}
        for sub in cls.__subclasses__():
            ret.update(sub.derived())
        return ret

    def __init__(self):
        super().__init__()
        self._parent = TrafficArrays.root
        if self._parent:
            self._parent._children.append(self)
        self._children = []
        self._ArrVars = []
        self._LstVars = []

    def reparent(self, newparent):
        """Give TrafficArrays object a new parent."""
        # Remove myself from the parent list of children, and add to new parent
        self._parent._children.pop(self._parent._children.index(self))
        newparent._children.append(self)
        self._parent = newparent

    def settrafarrays(self):
        """Convenience function for with-style traffic array registration."""
        return RegisterElementParameters(self)

    def _init_trafarrays(self, keys):
        for key in keys:
            if isinstance(self.__dict__[key], list):
                self._LstVars.append(key)
            elif isinstance(self.__dict__[key], np.ndarray):
                self._ArrVars.append(key)
            elif isinstance(self.__dict__[key], TrafficArrays):
                self.__dict__[key].reparent(self)

        # In plugins and replaceable classes it could be that their instance
        # is created when the simulation is already running, and traffic is
        # present. Size traffic arrays accordingly here
        if TrafficArrays.root.ntraf:
            self.create(TrafficArrays.root.ntraf)

    def create(self, n=1):
        """Append n elements (aircraft) to all lists and arrays."""

        for v in self._LstVars:  # Lists (mostly used for strings)
            lst = self.__dict__.get(v)
            vartype = type(lst[0]).__name__ if lst else "str"
            lst.extend([defaults.get(vartype)] * n)

        for v in self._ArrVars:  # Numpy array
            # Get type without byte length
            vartype = "".join(c for c in str(self.__dict__[v].dtype) if c.isalpha())
            self.__dict__[v] = np.append(
                self.__dict__[v], [defaults.get(vartype, 0)] * n
            )

    def istrafarray(self, name):
        """Returns true if parameter 'name' is a traffic array."""
        return name in self._LstVars or name in self._ArrVars

    def create_children(self, n=1):
        """Call create (aircraft create) on all children."""
        for child in self._children:
            child.create(n)
            child.create_children(n)

    def delete(self, idx):
        """Aircraft delete."""
        # Remove element (aircraft) idx from all lists and arrays
        for child in self._children:
            child.delete(idx)

        for v in self._ArrVars:
            self.__dict__[v] = np.delete(self.__dict__[v], idx)

        if self._LstVars:
            if isinstance(idx, Collection):
                for i in reversed(idx):
                    for v in self._LstVars:
                        del self.__dict__[v][i]
            else:
                for v in self._LstVars:
                    del self.__dict__[v][idx]

    def reset(self):
        """Delete all elements from arrays and start at 0 aircraft."""
        for child in self._children:
            child.reset()

        for v in self._ArrVars:
            self.__dict__[v] = np.array([], dtype=self.__dict__[v].dtype)

        for v in self._LstVars:
            self.__dict__[v] = []
