"""Create aircraft trails on the radar display.

Maintains, per aircraft, the history of flown line segments so the GUI can
draw fading trails behind each aircraft. Trails are switched on/off and
colored with the TRAIL stack command. Segments are added at a fixed time
resolution and fade to the "old" color after a configurable time.
"""

from typing import Any

import numpy as np

import minisky
from minisky.core import TrafficArrays


class Trails(TrafficArrays):
    """Data for the aircraft trails shown on the radar display.

    Every ``dt`` seconds of simulation time a line segment (from the last
    recorded position to the current position) is appended per aircraft.
    Segments are kept in a foreground buffer for drawing and can be moved
    to a background buffer with buffer(). Segment colors fade towards the
    "old" color over ``tcol0`` seconds. Available at runtime as
    ``minisky.traf.trails``.

    Attributes:
        active (bool): Whether trails are recorded and shown.
        dt (float): Time resolution of trail segments [s].
        tcol0 (float): Time after which a segment gets the old color [s].
        defcolor (ndarray): Default trail color (RGB, 0-255).
        lat0, lon0 (ndarray): Segment start positions [deg].
        lat1, lon1 (ndarray): Segment end positions [deg].
        time (ndarray): Simulation time at which each segment was added [s].
        col (list): Color per segment (RGB).
        fcol (ndarray): Color fading factor per segment (1.0 = new,
            0.0 = old).
        bglat0, bglon0, bglat1, bglon1, bgtime, bgcol: Background copies of
            the segment data.
        accolor (list): Current trail color per aircraft (RGB).
        lastlat, lastlon (ndarray): Last recorded position per aircraft
            [deg].
        lasttim (ndarray): Simulation time of the last recorded position
            per aircraft [s].

    Created by: Jacco M. Hoekstra
    """

    def __init__(self, dttrail: float = 10.0) -> None:
        super().__init__()
        self.active = False  # Wether or not to show trails
        self.dt = dttrail  # Resolution of trail pieces in time
        self.tcol0 = 60.0  # After how many seconds old colour

        # This list contains some standard colors
        self.colorList = {
            "BLUE": np.array([0, 0, 255]),
            "CYAN": np.array([0, 255, 255]),
            "RED": np.array([255, 0, 0]),
            "YELLOW": np.array([255, 255, 0]),
        }

        # Set default color to Blue
        self.defcolor = self.colorList["CYAN"]

        # Foreground data on line pieces
        self.lat0 = np.array([])
        self.lon0 = np.array([])
        self.lat1 = np.array([])
        self.lon1 = np.array([])
        self.time = np.array([])
        self.col: Any = []
        self.fcol = np.array([])

        # background copy of data
        self.bglat0 = np.array([])
        self.bglon0 = np.array([])
        self.bglat1 = np.array([])
        self.bglon1 = np.array([])
        self.bgtime = np.array([])
        self.bgcol: Any = []
        self.bgacid: list = []

        with self.settrafarrays():
            self.accolor = []
            self.lastlat = np.array([])
            self.lastlon = np.array([])
            self.lasttim = np.array([])

        self.clearnew()

        return

    def create(self, n: int = 1) -> None:
        """Initialize trail data for newly created aircraft.

        Sets the default trail color and records the creation position as
        the starting point of the first trail segment.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)

        self.accolor[-1] = self.defcolor
        self.lastlat[-1] = minisky.traf.lat[-1]
        self.lastlon[-1] = minisky.traf.lon[-1]

    def update(self) -> None:
        """Add new trail segments for aircraft that moved long enough.

        Called every simulation step. When trails are inactive, only the
        last-known positions are refreshed. Otherwise, for each aircraft
        whose last recorded segment is older than ``dt`` seconds, a new
        line segment from the last recorded position to the current
        position is appended to the drawing buffers, and the color fading
        factors of all segments are updated.
        """
        self.acid = minisky.traf.callsign
        if not self.active:
            self.lastlat = minisky.traf.lat
            self.lastlon = minisky.traf.lon
            self.lasttim[:] = minisky.sim.simt
            return
        """Add linepieces for trails based on traffic data"""

        # Use temporary list/array for fast append
        lstlat0 = []
        lstlon0 = []
        lstlat1 = []
        lstlon1 = []
        lsttime = []

        # Check for update
        delta = minisky.sim.simt - self.lasttim
        idxs = np.where(delta > self.dt)[0]

        # Add all a/c which need the update
        # if len(idxs)>0:
        #     print "len(idxs)=",len(idxs)

        for i in idxs:
            # Add to lists
            lstlat0.append(self.lastlat[i])
            lstlon0.append(self.lastlon[i])
            lstlat1.append(minisky.traf.lat[i])
            lstlon1.append(minisky.traf.lon[i])
            lsttime.append(minisky.sim.simt)

            if isinstance(self.col, np.ndarray):
                # print type(trailcol[i])
                # print trailcol[i]
                # print "col type: ",type(self.col)
                self.col = self.col.tolist()

            type(self.col)
            self.col.append(self.accolor[i])

            # Update aircraft record
            self.lastlat[i] = minisky.traf.lat[i]
            self.lastlon[i] = minisky.traf.lon[i]
            self.lasttim[i] = minisky.sim.simt

        # When a/c is no longer part of trail semgment,
        # it is no longer a/c data => add to the GUI send buffer
        self.newlat0.extend(lstlat0)
        self.newlon0.extend(lstlon0)
        self.newlat1.extend(lstlat1)
        self.newlon1.extend(lstlon1)
        # Update colours
        self.fcol = 1.0 - np.minimum(self.tcol0, np.abs(minisky.sim.simt - self.time)) / self.tcol0

        return

    def buffer(self) -> None:
        """Move the current foreground trail segments to the background.

        Background segments keep being drawn (in the old color) but are no
        longer updated; the foreground buffers are cleared afterwards.
        """

        self.bglat0 = np.append(self.bglat0, self.lat0)
        self.bglon0 = np.append(self.bglon0, self.lon0)
        self.bglat1 = np.append(self.bglat1, self.lat1)
        self.bglon1 = np.append(self.bglon1, self.lon1)
        self.bgtime = np.append(self.bgtime, self.time)

        # No color saved: Background: always 'old color' self.col0
        if isinstance(self.bgcol, np.ndarray):
            self.bgcol = self.bgcol.tolist()
        if isinstance(self.col, np.ndarray):
            self.col = self.col.tolist()

        self.bgcol = self.bgcol + self.col
        self.bgacid = self.bgacid + self.acid

        self.clearfg()  # Clear foreground trails
        return

    def clearnew(self) -> None:
        """Clear the pipeline of new line segments used for the QtGL GUI."""
        # Clear new lines pipeline used for QtGL
        self.newlat0 = []
        self.newlon0 = []
        self.newlat1 = []
        self.newlon1 = []

    def clearfg(self) -> None:  # Foreground
        """Clear the foreground trail segment buffers."""
        self.lat0 = np.array([])
        self.lon0 = np.array([])
        self.lat1 = np.array([])
        self.lon1 = np.array([])
        self.time = np.array([])
        self.col = np.array([])
        return

    def clearbg(self) -> None:  # Background
        """Clear the background trail segment buffers."""
        self.bglat0 = np.array([])
        self.bglon0 = np.array([])
        self.bglat1 = np.array([])
        self.bglon1 = np.array([])
        self.bgtime = np.array([])
        self.bgacid = []
        return

    def clear(self) -> None:
        """Clear all trail data: foreground, background and new-line buffers."""
        self.lastlon = np.array([])
        self.lastlat = np.array([])
        self.clearfg()
        self.clearbg()
        self.clearnew()
        return

    def setTrails(self, *args) -> "bool | tuple[bool, str]":
        """Switch trails on/off, or change the trail color of an aircraft.

        Implements the TRAIL stack command:
        ``TRAIL ON/OFF, [dt]`` or ``TRAIL acid color``. Without arguments,
        the current on/off state is reported. Switching trails off clears
        all recorded segments.

        Args:
            *args: Either a bool (on/off) optionally followed by the
                segment time resolution [s], or an aircraft index followed
                by a color name (BLUE/RED/YELLOW).

        Returns:
            bool or tuple: True on success, or (success flag, message).
        """
        if len(args) == 0:
            msg = "TRAIL ON/OFF, [dt] / TRAIL acid color\n"

            msg = msg + "TRAILS ARE ON" if self.active else msg + "TRAILS ARE OFF"

            return True, msg

        # Switch on/off
        elif type(args[0]) == bool:
            # Set trails on/off
            self.active = args[0]
            if len(args) > 1:
                self.dt = args[1]
            if not self.active:
                self.clear()

        # Change color per acid (pygame only)
        else:
            # Change trail color
            if len(args) < 2 or args[1] not in ["BLUE", "RED", "YELLOW"]:
                return (
                    False,
                    "Set aircraft trail color with: TRAIL acid BLUE/RED/YELLOW",
                )
            self.changeTrailColor(args[1], args[0])

        return True

    def changeTrailColor(self, color: str, idx: int) -> None:
        """Change the trail color of one aircraft.

        Args:
            color: Color name; must be a key of colorList
                (BLUE/CYAN/RED/YELLOW).
            idx: Aircraft index.
        """
        self.accolor[idx] = self.colorList[color]
        return

    def reset(self) -> None:
        """Clear all trail data and switch trails off upon simulation reset."""
        # This ensures that the traffic arrays (which size is dynamic)
        # are all reset as well, so all lat,lon,sdp etc but also objects adsb
        super().reset()
        self.clear()
        self.active = False
