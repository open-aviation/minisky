"""Aircraft group administration.

MiniSky accepts the BlueSky GROUP/UNGROUP scenario spelling, but represents the
query and mutation forms as separate command overloads before changing traffic state:

    GROUP   = "GROUP" [ group-name [ area-name | selection { selection } ] ] ;
    UNGROUP = "UNGROUP" group-name selection { selection } ;
    selection = aircraft-id | group-name | "*" | "ALL" ;

`GROUP` with no arguments lists groups. With only a name it lists that
group. With members it creates the group when necessary and adds either an
area's current aircraft or several selections. Area names cannot be mixed
with selections. `*` and `ALL` are read-only virtual selections containing
all aircraft.

BlueSky allowed group names to collide with aircraft and areas. MiniSky keeps
those stores separate and preserves command-specific lookup precedence, while
keeping group identity separate from its member indices.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

from minisky.command import Keyword, command
from minisky.core import TrafficArrays
from minisky.result import Err, Ok, Result

if TYPE_CHECKING:
    from minisky.tools.shapes import Shapes
    from minisky.traffic.traffic import Traffic


class TrafficGroups(TrafficArrays):
    """Runtime-owned groups backed by an unsigned 64-bit mask per aircraft.

    A group receives a bit and each aircraft stores the OR of all groups it
    belongs to. The representation supports exactly 64 stored groups. Virtual
    selections `*` and `ALL` consume no bit.
    """

    _ALL_NAMES = frozenset({"*", "ALL"})

    def __init__(self, traffic: Traffic, shapes: Shapes) -> None:
        super().__init__(traffic)
        self.traffic = traffic
        self.shapes = shapes
        self.groups: dict[str, int] = {}
        self.allmasks = 0
        with self.settrafarrays():
            self.ingroup = np.array([], dtype=np.uint64)

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic and area store."""
        return implementation(self.traffic, self.shapes)

    def __contains__(self, groupname: str) -> bool:
        """Return whether a stored or virtual group exists."""
        return groupname in self.groups or groupname in self._ALL_NAMES

    def reset(self) -> None:
        """Clear both per-aircraft membership and stored group definitions."""
        super().reset()
        self.groups.clear()
        self.allmasks = 0

    def _allocate_mask(self, groupname: str) -> Result[int, str]:
        # Get first unused group mask
        if groupname in self._ALL_NAMES:
            return Err(f"{groupname} is a reserved all-aircraft selection")
        if len(self.groups) >= 64:
            return Err("Maximum number of 64 groups reached")
        for bit in range(64):
            mask = 1 << bit
            if not self.allmasks & mask:
                self.groups[groupname] = mask
                self.allmasks |= mask
                return Ok(mask)
        return Err("No free group mask")

    def _selection(self, name: str) -> Result[np.ndarray, str]:
        """Resolve a formal `selection` using deterministic precedence."""
        if name in self._ALL_NAMES:
            return Ok(np.arange(self.traffic.ntraf, dtype=int))

        if name in self.groups:
            return self.listgroup(name)
        index = self.traffic.idx(name)
        if index is not None:
            return Ok(np.asarray([index], dtype=int))
        return Err(f"Aircraft or group {name} not found")

    def _member_indices(self, members: tuple[str, ...]) -> Result[np.ndarray, str]:
        indices: list[int] = []
        for member in members:
            match self._selection(member):
                case Ok(selection):
                    indices.extend(int(index) for index in selection)
                case Err(error):
                    if member in self.shapes.areas:
                        return Err("Area names cannot be combined with aircraft or groups")
                    return Err(error)
        return Ok(np.unique(np.asarray(indices, dtype=int)))

    @command(name="GROUP")
    def list_groups(self) -> Result[str, str]:
        """List all stored traffic groups."""
        if not self.groups:
            return Ok("There are currently no traffic groups defined.")
        return Ok(f"Defined traffic groups:\n{', '.join(self.groups)}")

    @command(name="GROUP")
    def show_group(self, groupname: Keyword) -> Result[str, str]:
        """List the aircraft in a traffic group."""
        match self.listgroup(groupname):
            case Ok(group):
                callsigns = np.asarray(self.traffic.callsign)[group]
                return Ok(f"Aircraft in group {groupname}:\n{', '.join(callsigns)}")
            case Err(error):
                return Err(error)

    @command(name="GROUP")
    def add_to_group(
        self, groupname: Keyword, first_member: Keyword, *additional_members: Keyword
    ) -> Result[str, str]:
        """Create or extend a traffic group from selections or an area."""
        members = (first_member, *additional_members)
        sole_member = members[0] if len(members) == 1 else None
        sole_is_selection = sole_member is not None and (
            self.traffic.idx(sole_member) is not None or sole_member in self
        )
        area = (
            self.shapes.areas.get(sole_member)
            if sole_member is not None and not sole_is_selection
            else None
        )
        if area is not None:
            inside = area.contains(self.traffic.lat, self.traffic.lon, self.traffic.alt)
            indices = np.flatnonzero(inside)
        else:
            match self._member_indices(members):
                case Ok(value):
                    indices = value
                case Err(error):
                    return Err(error)

        # Resolve every member before allocating a group bit.
        mask = self.groups.get(groupname)
        if mask is None:
            match self._allocate_mask(groupname):
                case Ok(value):
                    mask = value
                case Err(error):
                    return Err(error)

        # Add aircraft to group
        self.ingroup[indices] |= np.uint64(mask)
        callsigns = np.asarray(self.traffic.callsign)[indices]
        return Ok(f"Aircraft added to group {groupname}:\n{', '.join(callsigns)}")

    def delete_group(self, groupname: str) -> Result[None, str]:
        """Delete all members and release a stored group name."""
        # Delete all aircraft in the respective group
        if groupname in self._ALL_NAMES:
            return Err("The all-aircraft selection is not a stored group")
        mask = self.groups.get(groupname)
        if mask is None:
            return Err(f"Group {groupname} doesn't exist")
        match self.listgroup(groupname):
            case Ok(indices):
                self.traffic.delete(indices)
            case Err(error):
                return Err(error)
        self.groups.pop(groupname)
        self.allmasks &= ~mask
        return Ok(None)

    @command(name="UNGROUP")
    def ungroup(
        self, groupname: Keyword, first_member: Keyword, *additional_members: Keyword
    ) -> Result[None, str]:
        """Remove several selections from a stored group."""
        if groupname in self._ALL_NAMES:
            return Err("The all-aircraft selection cannot be modified")
        mask = self.groups.get(groupname)
        if mask is None:
            return Err(f"Group {groupname} doesn't exist")
        match self._member_indices((first_member, *additional_members)):
            case Ok(indices):
                self.ingroup[indices] &= ~np.uint64(mask)
                return Ok(None)
            case Err(error):
                return Err(error)

    def listgroup(self, groupname: str) -> Result[np.ndarray, str]:
        """Return member indices for a stored group or the virtual all group."""
        if groupname in self._ALL_NAMES:
            return Ok(np.arange(self.traffic.ntraf, dtype=int))
        mask = self.groups.get(groupname)
        if mask is None:
            return Err(f"Group {groupname} doesn't exist")
        return Ok(np.flatnonzero((self.ingroup & np.uint64(mask)) != 0))
