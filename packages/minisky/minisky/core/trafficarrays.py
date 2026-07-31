"""TrafficArrays: Base class for per-aircraft data arrays.

Classes that derive from TrafficArrays get automated create, delete, and reset
functionality for all registered child arrays. Replaceable implementations are
registered explicitly in a runtime before `SELECTIMPL` can use them.

MiniSky stores aircraft state as parallel numpy arrays and lists, where
index i in every array belongs to the same aircraft. Per-aircraft
parameters are registered by assigning them inside a
`with self.settrafarrays():` block (implemented by
RegisterElementParameters): every list or numpy array created inside the
block is recorded in _LstVars or _ArrVars, and every nested TrafficArrays
instance is re-parented to form a tree rooted at the traffic object.

When aircraft are created, `create(n)` appends n default-valued elements
to every registered list and array; when aircraft are deleted,
`delete(idx)` removes the corresponding elements from all of them, and
`reset()` empties everything back to zero aircraft. Each of these
operations recurses through the tree of children, so all per-aircraft data
in the simulation grows and shrinks in lockstep.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import TYPE_CHECKING

import numpy as np

from minisky.identifiers import normalize_public_name

if TYPE_CHECKING:
    from minisky.stack import Command


defaults = MappingProxyType({"float": 0.0, "int": 0, "uint": 0, "bool": False, "S": "", "str": ""})


@dataclass(frozen=True, slots=True)
class PreparedReplacement:
    """A validated runtime-local replacement entry."""

    base: type[TrafficArrays]
    name: str
    implementation: type[TrafficArrays]


class ReplaceableManager:
    """Own replacement implementations visible to a runtime."""

    def __init__(
        self,
        traffic: TrafficArrays,
        get_command_registry: Callable[[], Mapping[str, Command]],
        *,
        bases: Iterable[type[TrafficArrays]],
        core: Iterable[type[TrafficArrays]] = (),
    ) -> None:
        self.traffic = traffic
        self._get_command_registry = get_command_registry
        self._bases = {base.__name__.upper(): base for base in bases}
        self._implementations: dict[type[TrafficArrays], dict[str, type[TrafficArrays]]] = {
            base: {base.__name__.upper(): base} for base in self._bases.values()
        }
        for implementation in core:
            prepared = self.prepare(implementation)
            self._implementations[prepared.base][prepared.name] = implementation

    def _instance(self, base: type[TrafficArrays]) -> TrafficArrays | None:
        return next(
            (value for value in self.traffic.__dict__.values() if isinstance(value, base)),
            None,
        )

    def prepare(
        self,
        implementation: type[TrafficArrays],
        *,
        base: type[TrafficArrays] | None = None,
        name: str = "",
    ) -> PreparedReplacement:
        if not isinstance(implementation, type) or not issubclass(implementation, TrafficArrays):
            raise TypeError("replacement implementation must inherit TrafficArrays")
        root = base or implementation.replaceable_base()
        if self._bases.get(root.__name__.upper()) is not root:
            raise ValueError(f"unsupported replacement base: {root.__name__}")
        if not issubclass(implementation, root):
            raise TypeError(f"replacement {implementation.__name__} must inherit {root.__name__}")
        public_name = normalize_public_name(name or implementation.__name__)
        if public_name in ("BASE", root.__name__.upper()):
            raise ValueError(f"replacement name is reserved: {public_name}")
        return PreparedReplacement(root, public_name, implementation)

    def validate(self, replacements: tuple[PreparedReplacement, ...]) -> None:
        seen: set[tuple[type[TrafficArrays], str]] = set()
        for replacement in replacements:
            key = (replacement.base, replacement.name)
            if key in seen:
                raise ValueError(f"replacement repeated: {replacement.name}")
            if replacement.name in self._implementations[replacement.base]:
                raise ValueError(f"replacement already registered: {replacement.name}")
            seen.add(key)

    def install(self, replacements: tuple[PreparedReplacement, ...]) -> None:
        for replacement in replacements:
            self._implementations[replacement.base][replacement.name] = replacement.implementation

    def remove(self, replacements: tuple[PreparedReplacement, ...]) -> None:
        registry = self._get_command_registry()
        for replacement in reversed(replacements):
            current = self._instance(replacement.base)
            if current is not None and type(current) is replacement.implementation:
                # NOTE(abraham): replacements are synchronous strategies.
                _replace_instance_on_traf(
                    replacement.base, replacement.base, self.traffic, registry
                )
            implementations = self._implementations[replacement.base]
            if implementations.get(replacement.name) is replacement.implementation:
                del implementations[replacement.name]

    def select(self, basename: str = "", implname: str = "") -> tuple[bool, str]:
        if not basename:
            return True, "Replaceable classes in MiniSky:\n" + ", ".join(sorted(self._bases))

        base = self._bases.get(basename.upper())
        if base is None:
            return False, f"Replaceable {basename} not found."
        implementations = self._implementations[base]
        current_instance = self._instance(base)
        current = type(current_instance) if current_instance is not None else base
        if not implname:
            return True, (
                f"Current implementation for {basename}: {current.__name__}\n"
                f"Available implementations: {', '.join(sorted(implementations))}"
            )

        requested = base.__name__.upper() if implname.upper() == "BASE" else implname.upper()
        implementation = implementations.get(requested)
        if implementation is None:
            return False, f"Implementation {implname} not found for {basename}."
        if current is not implementation:
            replaced = _replace_instance_on_traf(
                base, implementation, self.traffic, self._get_command_registry()
            )
            if not replaced:
                return False, f"No {basename} instance exists on this traffic tree."
        return True, f"Selected {implname} for {basename}"

    def reset(self) -> None:
        registry = self._get_command_registry()
        for base in self._bases.values():
            current = self._instance(base)
            if current is not None and type(current) is not base:
                _replace_instance_on_traf(base, base, self.traffic, registry)


def _replace_instance_on_traf(
    base: type[TrafficArrays],
    impl: type[TrafficArrays],
    traffic: TrafficArrays,
    cmddict: Mapping[str, Command],
) -> bool:
    """Replace an attached implementation while preserving shared state."""
    for attr_name, attr_value in traffic.__dict__.items():
        if isinstance(attr_value, base):
            new_instance = attr_value.new_implementation(impl)
            ntraf = int(getattr(traffic, "ntraf", 0))
            if ntraf:
                new_instance.create(ntraf)
                new_instance.create_children(ntraf)
            if attr_value._parent is not None:
                new_instance.reparent(attr_value._parent)

            for arr_var in attr_value._ArrVars:
                if hasattr(new_instance, arr_var):
                    setattr(new_instance, arr_var, getattr(attr_value, arr_var))
            for lst_var in attr_value._LstVars:
                if hasattr(new_instance, lst_var):
                    setattr(new_instance, lst_var, getattr(attr_value, lst_var))

            setattr(traffic, attr_name, new_instance)
            attr_value.detach()
            _rebind_stack_commands(attr_value, new_instance, cmddict)
            return True
    return False


# TODO(abraham): replace callback rebinding with stable component slots.
def _rebind_stack_commands(
    old_instance: TrafficArrays,
    new_instance: TrafficArrays,
    cmddict: Mapping[str, Command],
) -> None:
    """Rebind stack commands that still target the replaced instance."""
    import inspect

    for command in set(cmddict.values()):
        callback = command.callback
        if inspect.ismethod(callback) and callback.__self__ is old_instance:
            command.callback = getattr(new_instance, callback.__func__.__name__, callback)


class RegisterElementParameters:
    """Context manager that registers per-aircraft parameters on a TrafficArrays object.

    Class to use in 'with'-syntax (through TrafficArrays.settrafarrays()).
    On construction it takes a snapshot of the attributes already present
    on the parent object; on exit it passes all newly created attributes to
    the parent's _init_trafarrays(), which registers lists and numpy arrays
    as per-aircraft variables that automatically grow and shrink with
    aircraft creation and deletion.
    """

    def __init__(self, parent: TrafficArrays) -> None:
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

    Replaceable implementations are discovered from the Python subclass
    hierarchy by `ReplaceableManager`; selection state belongs to an
    individual runtime rather than to this class.

    Attributes:
        _parent: Parent node of this object in the tree.
        _children: Child TrafficArrays objects of this object.
        _ArrVars: Names of the registered numpy-array parameters.
        _LstVars: Names of the registered list parameters.
    """

    @classmethod
    def replaceable_base(cls) -> type[TrafficArrays]:
        """Return the first-level TrafficArrays subclass for this class family.

        A direct subclass such as `Autopilot` is the replaceable base. A plugin
        subclass such as `CustomAutoPilot` resolves to that same base. The
        result is derived from the method-resolution order and therefore does
        not require mutable class or module registration state.
        """
        for candidate in cls.mro():
            if TrafficArrays in candidate.__bases__:
                return candidate
        return cls

    def __init__(self, parent: TrafficArrays | None = None) -> None:
        """Create a TrafficArrays node, optionally attached to `parent`.

        Aircraft creation and deletion propagate through the explicit tree
        of parent and child nodes rooted at the owning traffic object.
        """
        super().__init__()
        self._parent: TrafficArrays | None = None
        self._children: list[TrafficArrays] = []
        self._ArrVars: list[str] = []
        self._LstVars: list[str] = []
        if parent is not None:
            self.reparent(parent)

    def new_implementation(self, implementation: type[TrafficArrays]) -> TrafficArrays:
        """Construct a selected replacement implementation."""
        return implementation()

    def reparent(self, newparent: TrafficArrays) -> None:
        """Give this TrafficArrays object a new parent."""
        if self._parent is newparent:
            return
        if self._parent is not None:
            self._parent._children.remove(self)
        newparent._children.append(self)
        self._parent = newparent

    def detach(self) -> None:
        """Detach this object from its current traffic-array parent."""
        if self._parent is not None:
            self._parent._children.remove(self)
            self._parent = None

    @property
    def tree_root(self) -> TrafficArrays:
        """Return the root node of this object's traffic-array tree."""
        root = self
        while root._parent is not None:
            root = root._parent
        return root

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
        # present. Size traffic arrays accordingly here.
        root = self.tree_root
        ntraf = getattr(root, "ntraf", 0)
        if root is not self and ntraf:
            self.create(ntraf)

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
