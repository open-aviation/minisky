# Logic for group commands
"""Aircraft group administration.

Allows aircraft to be grouped so that stack commands can address several
aircraft at once. Group membership is stored as a 64-bit bitmask per
aircraft (one bit per group, so up to 64 groups can exist simultaneously),
which makes membership tests cheap numpy operations. Implements the GROUP
and UNGROUP stack commands; deleting a whole group is supported through
the DEL command.
"""

import numpy as np

import minisky
from minisky.core import TrafficArrays
from minisky.tools import areafilter


class GroupArray(np.ndarray):
    """Numpy index array that carries the name of the group it represents.

    Returned by TrafficGroups.listgroup(); the extra ``groupname``
    attribute allows commands that receive a group argument (such as DEL)
    to know which group the aircraft indices belong to.
    """

    # Similar to normal numpy arrays, but with the attribute of a groupname
    def __new__(cls, *args, groupname="", **kwargs):
        ret = np.array(*args, **kwargs).view(cls)
        ret.groupname = groupname
        return ret


class TrafficGroups(TrafficArrays):
    """Administration of aircraft groups using per-aircraft bitmasks.

    Each group is assigned one bit of a 64-bit mask; an aircraft's
    ``ingroup`` value is the OR of the masks of all groups it belongs to.
    Available at runtime as ``minisky.traf.groups``. The special group
    name ``*`` refers to all aircraft in the simulation.

    Attributes:
        groups (dict): Mapping of group name to its bitmask (int).
        allmasks (int): OR of all bitmasks currently in use.
        ingroup (ndarray): Per-aircraft group-membership bitmask (int64).
    """

    def __init__(self):
        # Initialize the groups structure
        super().__init__()
        self.groups = dict()
        self.allmasks = 0
        with self.settrafarrays():
            self.ingroup = np.array([], dtype=np.int64)

    def __contains__(self, groupname):
        """Check whether a group with the given name exists ("*" always does)."""
        # Check if a group with a name exists
        return groupname in self.groups or groupname == "*"

    def group(self, groupname="", *args):
        """Add aircraft to a group, list its members, or list all groups.

        Implements the GROUP stack command. Without arguments the existing
        groups are listed; with only a group name its members are listed
        (creating the group when arguments follow). Aircraft can be added
        by index, or by giving the name of an area/shape, in which case all
        aircraft currently inside that area are added.

        Args:
            groupname: Name of the group; empty to list all groups.
            *args: Aircraft indices, or a single area name.

        Returns:
            tuple: (success flag, message).
        """
        # Return list of groups if no groupname is given
        if not groupname:
            if not self.groups:
                return True, "There are currently no traffic groups defined."
            else:
                return True, "Defined traffic groups:\n" + ", ".join(self.groups)
        if len(self.groups) >= 64:
            return False, "Maximum number of 64 groups reached"
        if groupname not in self.groups:
            if not args:
                return False, f"Group {groupname} doesn't exist"
            # Get first unused group mask
            for i in range(64):
                groupmask = 1 << i
                if not self.allmasks & groupmask:
                    self.allmasks |= groupmask
                    self.groups[groupname] = groupmask
                    break

        elif not args:
            acnames = np.array(minisky.traf.callsign)[self.listgroup(groupname)]
            return True, "Aircraft in group {}:\n{}".format(
                groupname, ", ".join(acnames)
            )

        # Add aircraft to group
        if areafilter.has_area(args[0]):
            inside = areafilter.checkInside(
                args[0], minisky.traf.lat, minisky.traf.lon, minisky.traf.alt
            )
            self.ingroup[inside] |= self.groups[groupname]
            acnames = np.array(minisky.traf.callsign)[inside]
        else:
            idx = list(args)
            self.ingroup[idx] |= self.groups[groupname]
            acnames = np.array(minisky.traf.callsign)[idx]
        return True, "Aircraft added to group {}:\n{}".format(
            groupname, ", ".join(acnames)
        )

    def delgroup(self, grouparray):
        """Delete a group, and all aircraft in that group.

        Used by the DEL stack command when it is given a group name. The
        group's bitmask is released for reuse, unless the special group
        "*" (all aircraft) was given.

        Args:
            grouparray: GroupArray with the indices of the group members,
                as returned by listgroup().
        """
        # Delete all aircraft in the respective group
        minisky.traf.delete(grouparray)

        # Remove the group from the group list
        if grouparray.groupname != "*":
            self.allmasks ^= self.groups.pop(grouparray.groupname)

    def ungroup(self, groupname, *args):
        """Remove members from a group by aircraft index.

        Implements the UNGROUP stack command.

        Args:
            groupname: Name of the group.
            *args: Indices of the aircraft to remove from the group.

        Returns:
            tuple or None: (False, error message) when the group does not
            exist.
        """
        groupmask = self.groups.get(groupname, None)
        if groupmask is None:
            return False, f"Group {groupname} doesn't exist"
        self.ingroup[list(args)] ^= groupmask

    def listgroup(self, groupname):
        """Return the aircraft indices of all aircraft in a group.

        When "*" is passed as group name, all aircraft in the simulation
        are returned.

        Args:
            groupname: Name of the group, or "*" for all aircraft.

        Returns:
            GroupArray: Indices of the group members (with the group name
            attached), or (False, error message) when the group does not
            exist.
        """
        if groupname == "*":
            return GroupArray(range(minisky.traf.ntraf), groupname="*")
        groupmask = self.groups.get(groupname, None)
        if groupmask is None:
            return False, f"Group {groupname} doesn't exist"
        return GroupArray(
            np.where((self.ingroup & groupmask) > 0)[0], groupname=groupname
        )
