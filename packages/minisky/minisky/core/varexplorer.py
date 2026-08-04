"""BlueSky variable explorer

Provide flexible access to simulation data in BlueSky.

Data sources (by default the simulation and traffic objects) are
registered in a runtime-owned variable list, after which any of their
attributes can be inspected by name or dotted path (optionally with an
index, e.g. `traf.lat[0]`). This backs the LSVAR stack command, which
prints variable type, size, and parent information to the console.
"""

from __future__ import annotations

import re
from collections import OrderedDict
from collections.abc import Collection
from numbers import Number
from typing import Any

import numpy as np

from minisky.command import Token, command
from minisky.core import TrafficArrays
from minisky.result import Err, Ok, Result


class VariableExplorer:
    """Searchable simulation data sources owned by a MiniSky runtime."""

    def __init__(self) -> None:
        # The variable lists and their corresponding sources
        self.varlist: OrderedDict[str, tuple[Any, list[str] | None]] = OrderedDict()

    def init(self, simulation: Any, traffic: Any) -> None:
        """Variable explorer initialization function.

        Registers the default simulation and traffic data sources.
        """
        # Add the default sources to the variable explorer
        self.varlist.update(
            [
                ("sim", (simulation, getvarsfromobj(simulation))),
                ("traf", (traffic, getvarsfromobj(traffic))),
            ]
        )

    def validate_data_parent(self, name: str) -> None:
        """Reject a top-level name already owned by another data source."""
        if name in self.varlist:
            raise ValueError(f"variable parent already registered: {name}")

    def register_data_parent(self, obj: Any, name: str) -> None:
        """Register an object as a searchable data source of the variable explorer.

        Args:
            obj: The object whose attributes should become inspectable.
            name: Top-level name under which the object is registered.
        """
        self.varlist[name] = (obj, getvarsfromobj(obj))

    def unregister_data_parent(self, name: str, *, expected: object | None = None) -> None:
        """Remove a parent only while it still refers to the expected object."""
        current = self.varlist.get(name)
        if current is not None and (expected is None or current[0] is expected):
            del self.varlist[name]

    @command(name="LSVAR")
    def list_variables(self) -> Result[str, str]:
        """List registered variable parents."""
        return Ok(f"\n{', '.join(self.varlist)}")

    @command(name="LSVAR")
    def describe_variable(self, varname: Token) -> Result[str, str]:
        """Show information about a simulation variable."""
        v = self.findvar(varname)
        if v:
            thevar = v.get()  # reference to the actual variable
            # When the variable is an object, get child attributes
            attrs = getvarsfromobj(thevar)
            vartype = v.get_type()  # Type of the variable
            if isinstance(v.parent, TrafficArrays) and v.parent.istrafarray(v.varname):
                vartype += " (TrafficArray)"
            txt = f"Variable:   {v.varname}\n" + f"Type:       {vartype}\n"
            if isinstance(thevar, Collection):
                txt += f"Size:       {len(thevar)}\n"
            txt += f"Parent:     {v.parentname}"
            if attrs:
                txt += f"\nAttributes: {', '.join(attrs)}\n"
            return Ok(f"\n{txt}")
        return Err(f"Variable {varname} not found")

    def findvar(self, varname: str) -> Variable | None:
        """Find a variable and its parent object in the registered varlist set, based
        on varname, as passed by the stack.
        Variables can be searched in two ways:
        By name only: e.g., varname lat returns (traf, lat)
        By object: e.g., varname traf.lat returns (traf, lat)

        An optional integer index may be appended, e.g. `traf.lat[0]`.

        Args:
            varname: Variable name or dotted object path, with optional index.

        Returns:
            Variable: A Variable wrapper object, or None when not found.
        """
        try:
            # Find a string matching 'a.b.c[d]', where everything except a is optional
            varset = re.findall(r"(\w+)(?<=.)*(?:\[(\w+)\])?", varname)
            # The actual variable is always the last
            name, index = varset[-1]
            # is a parent object passed? (e.g., traf.lat instead of just lat)
            if len(varset) > 1:
                obj = None
                # The first object should be in the varlist of Plot
                # As either a top-level object:
                if varset[0][0] in self.varlist:
                    result = self.varlist.get(varset[0][0])
                    obj = result[0] if result is not None else None
                else:
                    for objset in self.varlist.values():
                        if objset[1] is not None and varset[0][0] in objset[1]:
                            obj = getattr(objset[0], varset[0][0])

                # Iterate over objectname,index pairs in varset
                for pair in varset[1:-1]:
                    if obj is None:
                        break
                    obj = getattr(obj, pair[0], None)

                if obj and hasattr(obj, name):
                    return Variable(obj, varset[-2][0], name, index)
            else:
                # A parent object is not passed, we only have a variable name
                # this name should exist in Plot.vlist
                for objname, objset in self.varlist.items():
                    if objset[1] is not None and name in objset[1]:
                        return Variable(objset[0], objname, name, index)
        except (AttributeError, IndexError, KeyError, TypeError, ValueError):
            return None
        return None


class Variable:
    """Wrapper class for variable explorer.
    Keeps reference to parent object, parent name, and variable name.

    Attributes:
        parent: Object that holds the variable.
        parentname: Registered name of the parent object.
        varname: Attribute name of the variable on the parent object.
        index: List of integer indices selected from the variable
            (empty when the whole variable is referenced).
    """

    def __init__(self, parent: Any, parentname: str, varname: str, index: Any) -> None:
        self.parent = parent
        self.parentname = parentname
        self.varname = varname
        try:
            self.index = [int(i) for i in index]
        except ValueError:
            self.index = []

    def is_num(self):
        """py3 replacement of operator.isNumberType."""
        v = getattr(self.parent, self.varname)
        return (
            isinstance(v, Number)
            or (isinstance(v, np.ndarray) and v.dtype.kind not in "OSUV")
            or (
                isinstance(v, (list, np.ndarray))
                and self.index
                and all(isinstance(v[i], Number) for i in self.index)
            )
        )

    def get_type(self) -> str:
        """Return the a string containing the type name of this variable."""
        return self.get().__class__.__name__

    def get(self):
        """Get a reference to the actual variable."""
        if self.index:
            v = getattr(self.parent, self.varname)
            return [v[i] for i in self.index]
        return getattr(self.parent, self.varname)


def getvarsfromobj(obj: Any) -> list[str] | None:
    """Return a list with the names of the variables of the passed object."""
    try:
        # Return attribute names, but exclude private attributes
        return [name for name in vars(obj) if name[0] != "_"]
    except TypeError:
        return None
