"""Conditional commands:
KL204 ATSPD 250 KL204 LNAV ON
KL204 ATALT FL100 KL204 SPD 350

Implements the ATALT, ATSPD and ATDIST stack commands: a command line is
stored together with a trigger condition on an aircraft's altitude, speed
or distance to a position. The :class:`Condition` instance owned by
``minisky.traf`` is checked every simulation step; when the monitored
value crosses its target, the stored command is issued on the stack and
the condition is removed.
"""

from typing import Any

import numpy as np

import minisky
from minisky import stack
from minisky.tools.geo import qdrdist

# Enumerated condtion types
alttype, spdtype, postype = 0, 1, 2


class Condition:
    """Administration of pending conditional (ATALT/ATSPD/ATDIST) commands.

    Each condition couples an aircraft to a target value (altitude, speed
    or distance to a reference position) and a command line. A condition
    triggers when the sign of (target - actual) changes compared to the
    previous update, i.e. when the aircraft crosses the target value.
    Triggered and orphaned (deleted-aircraft) conditions are removed.

    Attributes:
        ncond (int): Number of pending conditions.
        id (list): Callsign of the aircraft per condition.
        condtype (ndarray): Condition type (0 = altitude, 1 = speed,
            2 = position/distance).
        target (ndarray): Target value: altitude [m], speed (CAS) [m/s]
            or distance [nm].
        lastdif (ndarray): Difference target - actual at the last update.
        posdata (list): For distance conditions: (lat [deg], lon [deg]) of
            the reference position, else None.
        cmd (list): Command line to stack when the condition triggers.
    """

    def __init__(self) -> None:
        self.ncond = 0  # Number of conditions

        self.id = []  # Id of aircraft of condition
        self.condtype = np.array([], dtype=int)  # Condition type (0=alt,1=spd)
        self.target = np.array([], dtype=float)  # Target value (alt,speed,distance[nm])
        self.lastdif = np.array([], dtype=float)  # Difference during last update
        self.posdata = []  # Data for postype: tuples lat[deg],lon[deg] of ref position
        self.cmd = []  # Commands to be issued

    def update(self) -> None:
        """Check all pending conditions and execute triggered commands.

        Called every simulation step. Conditions of deleted aircraft are
        removed first. Then the actual value (altitude [m], CAS [m/s] or
        distance to the reference position [nm]) is compared with the
        target: when the difference changes sign, the stored command is
        stacked and the condition is deleted.
        """
        if self.ncond == 0:
            return

        # Update indices based on list of id's
        acidxlst = np.array(minisky.traf.idx(self.id))
        if len(acidxlst) > 0:
            idelcond = sorted(np.where(acidxlst < 0)[0])
            for i in idelcond[::-1]:
                del self.id[i]
                self.condtype = np.delete(self.condtype, i)
                self.target = np.delete(self.target, i)
                self.lastdif = np.delete(self.lastdif, i)
                del self.posdata[i]
                del self.cmd[i]

            self.ncond = len(self.id)
            if self.ncond == 0:
                return
            acidxlst = np.array(minisky.traf.idx(self.id))

        # Check condition types
        actdist = (
            np.ones(self.ncond) * 999e9
        )  # Invalid number which never triggers anything is extremely large
        for j in range(self.ncond):
            if self.condtype[j] == postype:
                qdr, dist = qdrdist(
                    minisky.traf.lat[acidxlst[j]],
                    minisky.traf.lon[acidxlst[j]],
                    self.posdata[j][0],
                    self.posdata[j][1],
                )
                actdist[j] = dist  # [nm]

        # Get relevant actual value using index list as index to numpy arrays
        self.actual = (
            (self.condtype == alttype) * minisky.traf.alt[acidxlst]
            + (self.condtype == spdtype) * minisky.traf.cas[acidxlst]
            + (self.condtype == postype) * actdist
        )

        # Compare sign of actual difference with sign of last difference
        actdif = self.target - self.actual

        # Make sorted arrya of indices of true conditions and their conditional commands
        idxtrue = sorted(np.where(actdif * self.lastdif <= 0.0)[0])  # Sign changed
        self.lastdif = actdif
        if idxtrue == None or len(idxtrue) == 0:
            return

        # Execute commands found to have true condition
        for i in idxtrue:
            if i >= 0:
                stack.stack(self.cmd[i])
                # debug
                # stack.stack(" ECHO Conditional command issued: "+self.cmd[i])

        # Delete executed commands to clean up arrays and lists
        # from highest index to lowest for consistency
        for i in idxtrue[::-1]:
            if i >= 0:
                del self.id[i]
                self.condtype = np.delete(self.condtype, i)
                self.target = np.delete(self.target, i)
                self.lastdif = np.delete(self.lastdif, i)
                del self.posdata[i]
                del self.cmd[i]

        # Adjust number of conditions
        self.ncond = len(self.id)

        if self.ncond != len(self.cmd):
            minisky.scr.echo(
                f"delcondition: invalid condition array size (ncond={self.ncond}, cmd={self.cmd})"
            )
        return

    def ataltcmd(self, acidx: int, targalt: float, cmdtxt: str) -> bool:
        """Schedule a command for when an aircraft crosses an altitude.

        Implements the ATALT stack command:
        ``acid ATALT alt cmd`` (e.g. ``KL204 ATALT FL100 KL204 SPD 350``).

        Args:
            acidx: Aircraft index.
            targalt: Trigger altitude [m] (stack input in ft/FL).
            cmdtxt: Command line to stack when the altitude is crossed.

        Returns:
            bool: True (the condition is always added).
        """
        actalt = minisky.traf.alt[acidx]
        self.addcondition(acidx, alttype, targalt, actalt, cmdtxt)
        return True

    def atspdcmd(self, acidx: int, targspd: float, cmdtxt: str) -> bool:
        """Schedule a command for when an aircraft crosses a speed.

        Implements the ATSPD stack command:
        ``acid ATSPD spd cmd`` (e.g. ``KL204 ATSPD 250 KL204 LNAV ON``).

        Args:
            acidx: Aircraft index.
            targspd: Trigger speed, CAS [m/s] (stack input in kts/Mach).
            cmdtxt: Command line to stack when the speed is crossed.

        Returns:
            bool: True (the condition is always added).
        """
        actspd = minisky.traf.cas[acidx]
        self.addcondition(acidx, spdtype, targspd, actspd, cmdtxt)
        return True

    def atdistcmd(self, acidx: int, lat: float, lon: float, targdist: float, cmdtxt: str) -> bool:
        """Schedule a command for a distance from a reference position.

        Implements the ATDIST stack command: ``acid ATDIST lat lon dist
        cmd``. The command triggers when the aircraft's distance to the
        given position crosses the target distance.

        Args:
            acidx: Aircraft index.
            lat: Reference latitude [deg].
            lon: Reference longitude [deg].
            targdist: Trigger distance to the reference position [nm].
            cmdtxt: Command line to stack when the distance is crossed.

        Returns:
            bool: True (the condition is always added).
        """
        qdr, actdist = qdrdist(minisky.traf.lat[acidx], minisky.traf.lon[acidx], lat, lon)
        self.addcondition(acidx, postype, targdist, actdist, cmdtxt, (lat, lon))
        return True

    def addcondition(
        self,
        acidx: int,
        icondtype: int,
        target: float,
        actual: Any,
        cmdtxt: str,
        latlon: Any = None,
    ) -> None:
        """Append a condition to the internal condition arrays.

        Args:
            acidx: Aircraft index; stored as callsign so the condition
                survives index shifts when other aircraft are deleted.
            icondtype: Condition type (0 = altitude, 1 = speed,
                2 = position/distance).
            target: Target value (altitude [m], speed [m/s] or
                distance [nm]).
            actual: Current value, used to initialize the sign of the
                difference.
            cmdtxt: Command line to stack when the condition triggers.
            latlon: Optional (lat [deg], lon [deg]) reference position for
                distance conditions.
        """
        # print ("addcondition:", acidx, icondtype, target, actual, cmdtxt, latlon)

        # Add condition to arrays
        self.id.append(minisky.traf.callsign[acidx])

        self.condtype = np.append(self.condtype, icondtype)
        self.target = np.append(self.target, target)
        self.lastdif = np.append(self.lastdif, target - actual)

        self.posdata.append(latlon)
        self.cmd.append(cmdtxt)

        self.ncond = self.ncond + 1
        # print("addcondition: self.ncond",self.ncond)
        return

    def renameac(self, oldid: str, newid: str) -> None:
        """Update stored callsigns after an aircraft has been renamed.

        Conditions are stored per callsign, so this must be called when an
        aircraft is renamed (e.g. by a future RENAME command) to keep the
        pending conditions attached to the right aircraft.

        Args:
            oldid: Previous callsign.
            newid: New callsign.
        """
        # Continonal commands are stored per id (ac name)
        # When renamed, call this method to update list
        # rename ids in list of ids
        # Call this if RENAME command is implemented
        if self.id.count(oldid) == 0:
            return
        for i in range(len(self.id)):
            if self.id[i] == oldid:
                self.id[i] = newid
        return
