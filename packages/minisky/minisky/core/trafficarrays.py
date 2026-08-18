"""TrafficArrays: Base class for per-aircraft data arrays.

Classes that derive from TrafficArrays get automated create, delete, and reset
functionality for all registered child arrays. Replaceable implementations are
registered explicitly in a runtime before `SELECTIMPL` can use them.

MiniSky stores aircraft state as parallel NumPy arrays and lists, where
index i in every array belongs to the same aircraft. Per-aircraft parameters
are registered inside [`TrafficArrays.settrafarrays`][.TrafficArrays.settrafarrays],
implemented by [`RegisterElementParameters`][.RegisterElementParameters]. Every
list or NumPy array created in that context is recorded, and nested
`TrafficArrays` instances are re-parented into a tree rooted at traffic.

When aircraft are created, [`TrafficArrays.create`][.TrafficArrays.create]
appends default-valued elements to every registered list and array;
[`TrafficArrays.delete`][.TrafficArrays.delete] removes corresponding rows, and
[`TrafficArrays.reset`][.TrafficArrays.reset] empties everything back to zero
aircraft. Each operation recurses through the child tree, so per-aircraft data
grows and shrinks in lockstep.
"""

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Generic, TypeVar

import numpy as np

from minisky.command import Keyword, command
from minisky.identifiers import normalize_public_name
from minisky.result import Err, Ok, Result

defaults = MappingProxyType({"float": 0.0, "int": 0, "uint": 0, "bool": False, "S": "", "str": ""})


@dataclass(frozen=True, slots=True)
class PreparedReplacement:
    """A validated runtime-local replacement entry."""

    base: type[TrafficArrays]
    name: str
    implementation: type[TrafficArrays]


@dataclass(slots=True)
class _ComponentSlot:
    """Stable access to a replaceable component attached to the traffic tree."""

    traffic: TrafficArrays
    attribute: str
    base: type[TrafficArrays]

    @property
    def current(self) -> TrafficArrays:
        component = getattr(self.traffic, self.attribute)
        if not isinstance(component, self.base):
            raise TypeError(f"replaceable slot {self.attribute} has an invalid component")
        return component

    def bind(self, callback: Callable[..., Any]) -> Callable[..., Any]:
        method_name = callback.__name__
        signature = inspect.signature(callback)

        def dispatch(*args: Any, **kwargs: Any) -> Any:
            method = getattr(self.current, method_name)
            return method(*args, **kwargs)

        # do not use functools.wraps here: __wrapped__ lets inspect.unwrap bypass
        # this slot dispatch and bind command parsing to the replaced implementation.
        dispatch.__name__ = callback.__name__
        dispatch.__qualname__ = callback.__qualname__
        dispatch.__module__ = callback.__module__
        dispatch.__doc__ = callback.__doc__
        dispatch.__annotations__ = dict(callback.__annotations__)
        dispatch.__signature__ = signature  # type: ignore[attr-defined]
        return dispatch

    def replace(self, implementation: type[TrafficArrays]) -> None:
        previous = self.current
        replacement = previous.new_implementation(implementation)
        if not isinstance(replacement, self.base):
            raise TypeError(
                f"replacement {type(replacement).__name__} must inherit {self.base.__name__}"
            )

        ntraf = int(getattr(self.traffic, "ntraf", 0))
        if ntraf:
            replacement.create(ntraf)
            replacement.create_children(ntraf)
        if previous._parent is not None:
            replacement.reparent(previous._parent)

        for name in previous._ArrVars:
            if hasattr(replacement, name):
                setattr(replacement, name, getattr(previous, name))
        for name in previous._LstVars:
            if hasattr(replacement, name):
                setattr(replacement, name, getattr(previous, name))
        for name in previous._VariantVars:
            if hasattr(replacement, name):
                setattr(replacement, name, getattr(previous, name))
        for name in previous._OptionalVars:
            if hasattr(replacement, name):
                setattr(replacement, name, getattr(previous, name))

        setattr(self.traffic, self.attribute, replacement)
        previous.detach()


class ReplaceableManager:
    """Own replacement implementations visible to a runtime."""

    def __init__(
        self,
        traffic: TrafficArrays,
        *,
        bases: Iterable[type[TrafficArrays]],
        core: Iterable[type[TrafficArrays]] = (),
    ) -> None:
        self.traffic = traffic
        self._bases = {base.__name__.upper(): base for base in bases}
        self._slots = {base: self._find_slot(base) for base in self._bases.values()}
        self._implementations: dict[type[TrafficArrays], dict[str, type[TrafficArrays]]] = {
            base: {base.__name__.upper(): base} for base in self._bases.values()
        }
        for implementation in core:
            prepared = self.prepare(implementation)
            self._implementations[prepared.base][prepared.name] = implementation

    # TODO(abraham): declare replaceable slots during Traffic construction
    # instead of scanning attributes.
    def _find_slot(self, base: type[TrafficArrays]) -> _ComponentSlot:
        matches = [name for name, value in self.traffic.__dict__.items() if isinstance(value, base)]
        if len(matches) != 1:
            raise ValueError(
                f"expected exactly one {base.__name__} component, found {len(matches)}"
            )
        return _ComponentSlot(self.traffic, matches[0], base)

    def bind_callback(self, callback: Callable[..., Any]) -> Callable[..., Any]:
        """Return a callback that follows replacement selection when needed."""
        if not inspect.ismethod(callback):
            return callback
        for slot in self._slots.values():
            if callback.__self__ is slot.current:
                return slot.bind(callback)
        return callback

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
        for replacement in reversed(replacements):
            slot = self._slots[replacement.base]
            if type(slot.current) is replacement.implementation:
                # NOTE(abraham): replacements are synchronous strategies.
                slot.replace(replacement.base)
            implementations = self._implementations[replacement.base]
            if implementations.get(replacement.name) is replacement.implementation:
                del implementations[replacement.name]

    @command(name="SELECTIMPL", aliases=("IMPL", "IMPLEMENTATION", "IMPLEMENT"))
    def list_implementations(self) -> Result[str, str]:
        """List replaceable classes."""
        return Ok(f"Replaceable classes in MiniSky:\n{', '.join(sorted(self._bases))}")

    @command(name="SELECTIMPL")
    def describe_implementations(self, basename: Keyword) -> Result[str, str]:
        """Show the current and available implementations for a replaceable."""
        base = self._bases.get(basename)
        if base is None:
            return Err(f"Replaceable {basename} not found.")
        implementations = self._implementations[base]
        current = type(self._slots[base].current)
        return Ok(
            f"Current implementation for {basename}: {current.__name__}\n"
            f"Available implementations: {', '.join(sorted(implementations))}"
        )

    @command(name="SELECTIMPL")
    def select_implementation(self, basename: Keyword, implname: Keyword) -> Result[str, str]:
        """Select an implementation for a replaceable class."""
        return self.select(basename, implname)

    def select(self, basename: str = "", implname: str = "") -> Result[str, str]:
        """List, describe, or select a replaceable implementation."""
        if not basename:
            return self.list_implementations()
        if not implname:
            return self.describe_implementations(basename.upper())

        base = self._bases.get(basename.upper())
        if base is None:
            return Err(f"Replaceable {basename} not found.")
        implementations = self._implementations[base]
        slot = self._slots[base]
        current = type(slot.current)
        requested = base.__name__.upper() if implname.upper() == "BASE" else implname.upper()
        implementation = implementations.get(requested)
        if implementation is None:
            return Err(f"Implementation {implname} not found for {basename}.")
        if current is not implementation:
            slot.replace(implementation)
        return Ok(f"Selected {implname} for {basename}")

    def reset(self) -> None:
        for base, slot in self._slots.items():
            if type(slot.current) is not base:
                slot.replace(base)


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

    def __exit__(self, exc_type, exc_value, tb) -> None:
        """Register all attributes created inside the with-block as traffic arrays."""
        self._parent._init_trafarrays(set(self._parent.__dict__.keys()) - self.keys0)


# using generics because we want consumers to be able to type with `minisky.quantities`
ArrayValueT = TypeVar("ArrayValueT")


@dataclass(slots=True)
class VariantArray(Generic[ArrayValueT]):
    """Vectorised enum arrays.

    In minisky, we typically need to distinguish between several quantity kinds:
    for example, pressure altitude and MSL altitude are not interchangeable.
    One option is to use a array of structs
    (`Sequence[tuple[float, Discriminant]]`), but to follow the philosphy of
    data-oriented design, we wish to adopt vectorised struct of arrays instead
    (`tuple[Sequence[float], Sequence[Discriminant]]`).

    Conceptually, the internal data layout is the exact same as a
    [numpy masked array](https://numpy.org/doc/stable/reference/maskedarray.html)
    but with the benefit of storing more than two variants. It is also directly
    inspired by [Julia's `Array{Union{String, Nothing}}`](https://julialang.org/blog/2018/06/missing/).

    We intentionally do **not** add complex operator overloads for simplicity.
    Users are expected to handle it themselves.

    !!! note

        [traffic arrays][minisky.core.TrafficArrays] by default zero-fills new
        `kind` lanes, so make sure the '0' value used in your discriminant
        means the default.

    ## Examples

    ```py
    class SpeedVariant(IntEnum):
        CAS = 0
        MACH = 1

    speed = VariantArray(
        values=np.array([0.78, 240]),
        kind=np.array([SpeedVariant.MACH, SpeedVariant.CAS])
    )
    # then some time in the future, to extract all Mach numbers:
    print(speed.values[np.where(speed.kind == SpeedVariant.MACH)])
    ```

    In this case, when traffic arrays creates a new aircraft it will have a
    speed of 0 KCAS.
    """

    values: ArrayValueT
    kind: np.ndarray


ArrayIndex = int | slice | np.ndarray


@dataclass(slots=True)
class OptionalArray(Generic[ArrayValueT]):
    """Vectorised `T | None`. See [minisky.core.trafficarrays.VariantArray][]."""

    values: ArrayValueT
    present: np.ndarray

    def set(self, idx: ArrayIndex, value) -> None:
        self.values[idx] = value  # pyright: ignore[reportIndexIssue]
        self.present[idx] = True

    def clear(self, idx: ArrayIndex) -> None:
        self.present[idx] = False


class TrafficArrays:
    """Parent class to use separate arrays and lists to allow
    vectorizing but still maintain and object like benefits
    for creation and deletion of an element for all parameters.

    TrafficArrays objects form a tree (rooted at the traffic object) in
    which aircraft creation, deletion, and reset propagate recursively, so
    that all registered per-aircraft arrays in the simulation keep the same
    length as the number of aircraft.

    Replaceable implementations are registered explicitly with
    `ReplaceableManager`; selection state belongs to an individual runtime
    rather than to this class.
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
        self._VariantVars: list[str] = []
        self._OptionalVars: list[str] = []
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

        Lists are recorded in _LstVars, numpy arrays in _ArrVars,
        VariantArray objects in _VariantVars, OptionalArray objects in
        _OptionalVars, and nested TrafficArrays objects are re-parented to this
        object. When traffic already exists, the new
        arrays are immediately sized to the current number of aircraft.
        """
        for key in keys:
            if isinstance(self.__dict__[key], list):
                self._LstVars.append(key)
            elif isinstance(self.__dict__[key], np.ndarray):
                self._ArrVars.append(key)
            elif isinstance(self.__dict__[key], VariantArray):
                self._VariantVars.append(key)
            elif isinstance(self.__dict__[key], OptionalArray):
                self._OptionalVars.append(key)
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
        """

        for v in self._LstVars:
            # Deliberately fail if registration is inconsistent: every _LstVars entry must exist.
            lst = self.__dict__[v]
            vartype = type(lst[0]).__name__ if lst else "str"
            lst.extend([defaults.get(vartype)] * n)

        for v in self._ArrVars:
            array = self.__dict__[v]
            # Preserve the declared dtype. Building defaults as a Python list
            # made NumPy promote uncommon dtypes (notably uint64 group masks)
            # through float64 before appending, which can lose high membership bits.
            # Get type without byte length
            vartype = "".join(c for c in str(array.dtype) if c.isalpha())
            extension = np.full(n, defaults.get(vartype, 0), dtype=array.dtype)
            self.__dict__[v] = np.append(array, extension)

        for v in self._VariantVars:
            variant = self.__dict__[v]
            vartype = "".join(c for c in str(variant.values.dtype) if c.isalpha())
            extension = np.full(n, defaults.get(vartype, 0), dtype=variant.values.dtype)
            self.__dict__[v] = VariantArray(
                np.append(variant.values, extension),
                np.append(variant.kind, np.zeros(n, dtype=variant.kind.dtype)),
            )

        for v in self._OptionalVars:
            optional = self.__dict__[v]
            vartype = "".join(c for c in str(optional.values.dtype) if c.isalpha())
            extension = np.full(n, defaults.get(vartype, 0), dtype=optional.values.dtype)
            self.__dict__[v] = OptionalArray(
                np.append(optional.values, extension),
                np.append(optional.present, np.zeros(n, dtype=bool)),
            )

    def istrafarray(self, name: str) -> bool:
        """Returns true if parameter 'name' is a registered traffic array of this object."""
        return (
            name in self._LstVars
            or name in self._ArrVars
            or name in self._VariantVars
            or name in self._OptionalVars
        )

    def create_children(self, n: int = 1) -> None:
        """Call create (aircraft create) recursively on all children."""
        for child in self._children:
            child.create(n)
            child.create_children(n)

    def delete(self, idx: int | np.ndarray) -> None:
        """Aircraft delete.

        Removes element(s) idx from all registered lists and arrays of
        this object, recursing through its children first, so that all
        per-aircraft data shrinks consistently.
        """
        for child in self._children:
            child.delete(idx)

        for v in self._ArrVars:
            array = self.__dict__[v]
            self.__dict__[v] = np.delete(array, idx)

        for v in self._VariantVars:
            variant = self.__dict__[v]
            self.__dict__[v] = VariantArray(
                np.delete(variant.values, idx), np.delete(variant.kind, idx)
            )

        for v in self._OptionalVars:
            optional = self.__dict__[v]
            self.__dict__[v] = OptionalArray(
                np.delete(optional.values, idx), np.delete(optional.present, idx)
            )

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
            array = self.__dict__[v]
            self.__dict__[v] = np.array([], dtype=array.dtype)

        for v in self._VariantVars:
            variant = self.__dict__[v]
            self.__dict__[v] = VariantArray(
                np.array([], dtype=variant.values.dtype),
                np.array([], dtype=variant.kind.dtype),
            )

        for v in self._OptionalVars:
            optional = self.__dict__[v]
            self.__dict__[v] = OptionalArray(
                np.array([], dtype=optional.values.dtype),
                np.array([], dtype=bool),
            )

        for v in self._LstVars:
            self.__dict__[v] = []
