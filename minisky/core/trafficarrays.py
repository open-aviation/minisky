"""TrafficArrays: Base class for per-aircraft data arrays.

Classes that derive from TrafficArrays get automated create, delete, and reset
functionality for all registered child arrays. All subclasses are automatically
replaceable via SELECTIMPL - see minisky/plugin/ for usage examples.

MiniSky stores aircraft state as parallel numpy arrays and lists, where
index i in every array belongs to the same aircraft. Per-aircraft
parameters are registered by assigning them inside a
``with self.settrafarrays():`` block (implemented by
RegisterElementParameters): every list or numpy array created inside the
block is recorded in _LstVars or _ArrVars, and every nested TrafficArrays
instance is re-parented to form a tree rooted at the traffic object.

When aircraft are created, ``create(n)`` appends n default-valued elements
to every registered list and array; when aircraft are deleted,
``delete(idx)`` removes the corresponding elements from all of them, and
``reset()`` empties everything back to zero aircraft. Each of these
operations recurses through the tree of children, so all per-aircraft data
in the simulation grows and shrinks in lockstep.
"""

from typing import ClassVar

import numpy as np

defaults = {"float": 0.0, "int": 0, "uint": 0, "bool": False, "S": "", "str": ""}

# Global dictionary of replaceable classes
replaceables: dict[str, type["TrafficArrays"]] = {}


def reset_replaceables() -> None:
    """Reset all replaceables to their default implementation and reinstantiate on traf."""
    for base in replaceables.values():
        base.selectdefault()
        # Reinstantiate on traf with default implementation
        _replace_instance_on_traf(base, base._generator)


def select_implementation(basename: str = "", implname: str = "") -> tuple[bool, str]:
    """Select an implementation for a replaceable class.

    Arguments:
    - basename: Name of the replaceable base class (e.g., 'AUTOPILOT')
    - implname: Name of the implementation to select (e.g., 'MYAUTOPILOT')

    Returns: (success, message) tuple
    """
    if not basename:
        return True, "Replaceable classes in MiniSky:\n" + ", ".join(replaceables)

    base = replaceables.get(basename.upper())
    if not base:
        return False, f"Replaceable {basename} not found."

    impls = base.derived()
    if not implname:
        current = base._generator.__name__
        return True, (
            f"Current implementation for {basename}: {current}\n"
            f"Available implementations: {', '.join(impls)}"
        )

    impl = impls.get(base.__name__ if implname.upper() == "BASE" else implname.upper())
    if not impl:
        return False, f"Implementation {implname} not found for {basename}."

    impl.select()

    # Replace existing instance on traf if it exists
    _replace_instance_on_traf(base, impl)

    return True, f"Selected {implname} for {basename}"


def _replace_instance_on_traf(base: type["TrafficArrays"], impl: type["TrafficArrays"]) -> None:
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
            for arr_var in getattr(attr_value, "_ArrVars", []):
                if hasattr(new_instance, arr_var):
                    setattr(new_instance, arr_var, getattr(attr_value, arr_var))
            for lst_var in getattr(attr_value, "_LstVars", []):
                if hasattr(new_instance, lst_var):
                    setattr(new_instance, lst_var, getattr(attr_value, lst_var))
            # Replace on traf
            setattr(minisky.traf, attr_name, new_instance)
            # Stack commands registered as bound methods of the old instance
            # would silently mutate the orphaned object; rebind them
            _rebind_stack_commands(attr_value, new_instance)
            break


def _rebind_stack_commands(old_instance: "TrafficArrays", new_instance: "TrafficArrays") -> None:
    """Rebind stack command callbacks from old_instance to new_instance."""
    import inspect

    from minisky.stack import Command

    for cmdobj in set(Command.cmddict.values()):
        callback = cmdobj.callback
        if inspect.ismethod(callback) and callback.__self__ is old_instance:
            cmdobj.callback = getattr(new_instance, callback.__func__.__name__, callback)


class RegisterElementParameters:
    """Context manager that registers per-aircraft parameters on a TrafficArrays object.

    Class to use in 'with'-syntax (through TrafficArrays.settrafarrays()).
    On construction it takes a snapshot of the attributes already present
    on the parent object; on exit it passes all newly created attributes to
    the parent's _init_trafarrays(), which registers lists and numpy arrays
    as per-aircraft variables that automatically grow and shrink with
    aircraft creation and deletion.
    """

    def __init__(self, parent: "TrafficArrays") -> None:
        self._parent = parent
        self.keys0 = set(parent.__dict__.keys())

    def __enter__(self) -> None:
        """No-op: the attribute snapshot is already taken in __init__."""
        pass

    def __exit__(self, exc_type, exc_value, tb) -> None:
        """Register all attributes created inside the with-block as traffic arrays."""
        self._parent._init_trafarrays(set(self._parent.__dict__.keys()) - self.keys0)


class TrafficArrays:
    """Parent class to use separate arrays and lists to allow
    vectorizing but still maintain and object like benefits
    for creation and deletion of an element for all parameters.

    TrafficArrays objects form a tree (rooted at the traffic object) in
    which aircraft creation, deletion, and reset propagate recursively, so
    that all registered per-aircraft arrays in the simulation keep the same
    length as the number of aircraft.

    Supports the replaceable pattern when subclassed with replaceable=True:
        class Autopilot(TrafficArrays, replaceable=True):
            ...

    Attributes:
        root: Class attribute; root of the TrafficArrays tree (the traffic
            object), set with setroot().
        ntraf: Class attribute; the current number of aircraft.
        _parent: Parent node of this object in the tree.
        _children: Child TrafficArrays objects of this object.
        _ArrVars: Names of the registered numpy-array parameters.
        _LstVars: Names of the registered list parameters.
    """

    # The TrafficArrays class keeps track of all of the constructed
    # TrafficArray objects
    root = None
    ntraf = 0

    # Replaceable pattern class variables (set per-subclass)
    _baseimpl: ClassVar[type | None] = None
    _generator: ClassVar[type]
    _default: ClassVar[str] = ""

    @staticmethod
    def setroot(obj: "TrafficArrays") -> None:
        """This function is used to set the root of the tree of TrafficArray
        objects (which is the traffic object.)"""
        TrafficArrays.root = obj

    def __init_subclass__(cls, **kwargs) -> None:
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
        if not hasattr(cls, "_baseimpl") or cls._baseimpl is None:
            cls._baseimpl = cls
            cls._default = ""
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
    def setdefault(cls, name: str) -> None:
        """Set a default implementation by name."""
        if cls._baseimpl is None:
            return
        impl = cls._baseimpl.derived().get(name.upper())
        if impl:
            cls._baseimpl._default = name.upper()
            cls._baseimpl._generator = impl

    @classmethod
    def getdefault(cls) -> "type[TrafficArrays] | None":
        """Get the default implementation class."""
        if cls._baseimpl is None:
            return cls
        default = cls._baseimpl._default
        return cls._baseimpl.derived().get(default) if default else cls._baseimpl

    @classmethod
    def selectdefault(cls) -> None:
        """Select the default implementation."""
        if cls._baseimpl is None:
            return
        base = cls._baseimpl
        base.derived().get(base._default, base).select()

    @classmethod
    def select(cls) -> None:
        """Select this class as the active implementation."""
        if cls._baseimpl is None:
            return
        cls._baseimpl._generator = cls

    @classmethod
    def selected(cls) -> "type[TrafficArrays]":
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

    def __init__(self) -> None:
        """Create a TrafficArrays node and attach it to the current root.

        The new object registers itself as a child of TrafficArrays.root
        (the traffic object), so that aircraft creation and deletion
        propagate to its registered arrays.
        """
        super().__init__()
        self._parent = TrafficArrays.root
        if self._parent:
            self._parent._children.append(self)
        self._children = []
        self._ArrVars = []
        self._LstVars = []

    def reparent(self, newparent: "TrafficArrays") -> None:
        """Give TrafficArrays object a new parent."""
        # Remove myself from the parent list of children, and add to new parent
        assert self._parent is not None, "reparent() called on a root node"
        self._parent._children.pop(self._parent._children.index(self))
        newparent._children.append(self)
        self._parent = newparent

    def settrafarrays(self) -> RegisterElementParameters:
        """Convenience function for with-style traffic array registration."""
        return RegisterElementParameters(self)

    def _init_trafarrays(self, keys: set[str]) -> None:
        """Register the given attribute names as per-aircraft variables.

        Lists are recorded in _LstVars, numpy arrays in _ArrVars, and
        nested TrafficArrays objects are re-parented to this object. When
        traffic already exists, the new arrays are immediately sized to
        the current number of aircraft.
        """
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
        if TrafficArrays.root is not None and TrafficArrays.root.ntraf:
            self.create(TrafficArrays.root.ntraf)

    def create(self, n: int = 1) -> None:
        """Append n elements (aircraft) to all lists and arrays.

        New elements get a default value based on their element type:
        0 for numeric arrays, False for boolean arrays, and an empty
        string for string lists.

        Args:
            n: Number of aircraft to add (default 1).
        """

        for v in self._LstVars:  # Lists (mostly used for strings)
            lst = self.__dict__[v]  # Not .get() — if v in _LstVars it must exist
            vartype = type(lst[0]).__name__ if lst else "str"
            lst.extend([defaults.get(vartype)] * n)

        for v in self._ArrVars:  # Numpy array
            # Get type without byte length
            vartype = "".join(c for c in str(self.__dict__[v].dtype) if c.isalpha())
            self.__dict__[v] = np.append(self.__dict__[v], [defaults.get(vartype, 0)] * n)

    def istrafarray(self, name: str) -> bool:
        """Returns true if parameter 'name' is a registered traffic array of this object."""
        return name in self._LstVars or name in self._ArrVars

    def create_children(self, n: int = 1) -> None:
        """Call create (aircraft create) recursively on all children.

        Args:
            n: Number of aircraft to add (default 1).
        """
        for child in self._children:
            child.create(n)
            child.create_children(n)

    def delete(self, idx: int | np.ndarray) -> None:
        """Aircraft delete.

        Removes element(s) idx from all registered lists and arrays of
        this object, recursing through its children first, so that all
        per-aircraft data shrinks consistently.

        Args:
            idx: Index or collection of indices of the aircraft to remove.
        """
        # Remove element (aircraft) idx from all lists and arrays
        for child in self._children:
            child.delete(idx)

        for v in self._ArrVars:
            self.__dict__[v] = np.delete(self.__dict__[v], idx)

        if self._LstVars:
            if isinstance(idx, np.ndarray):
                for i in idx[::-1]:
                    for v in self._LstVars:
                        del self.__dict__[v][i]
            else:
                for v in self._LstVars:
                    del self.__dict__[v][idx]

    def reset(self) -> None:
        """Delete all elements from arrays and start at 0 aircraft.

        Recursively empties the registered arrays and lists of this object
        and all of its children, preserving the array dtypes.
        """
        for child in self._children:
            child.reset()

        for v in self._ArrVars:
            self.__dict__[v] = np.array([], dtype=self.__dict__[v].dtype)

        for v in self._LstVars:
            self.__dict__[v] = []
