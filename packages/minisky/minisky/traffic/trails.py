"""Create aircraft trails on the radar display.

Maintains, per aircraft, the history of flown line segments so the GUI can
draw fading trails behind each aircraft. Trails are switched on/off and
colored with the TRAIL stack command. Segments are added at a fixed time
resolution and fade to the "old" color after a configurable time.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Literal

import numpy as np

from minisky.command import Keyword, OnOff, PositiveFiniteFloat, command
from minisky.core import TrafficArrays
from minisky.result import Err, Ok, Result

if TYPE_CHECKING:
    from minisky.simulation import Simulation
    from minisky.traffic import Traffic


TrailColor = Literal["BLUE", "RED", "YELLOW"]


class Trails(TrafficArrays):
    """Data for the aircraft trails shown on the radar display.

    Every `dt` seconds of simulation time a line segment (from the last
    recorded position to the current position) is appended per aircraft.
    Segments are kept in a foreground buffer for drawing and can be moved
    to a background buffer with buffer(). Segment colors fade towards the
    "old" color over `tcol0` seconds. Available at runtime as
    [`runtime.traffic.trails`][minisky.traffic.trails.Trails].

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

    def __init__(
        self,
        traffic: Traffic,
        get_simulation: Callable[[], Simulation],
        dttrail: float = 10.0,
    ) -> None:
        super().__init__(traffic)
        self.traffic = traffic
        self._get_simulation = get_simulation
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

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic and simulation."""
        return implementation(self.traffic, self._get_simulation)

    def create(self, n: int = 1) -> None:
        """Initialize trail data for newly created aircraft.

        Sets the default trail color and records the creation position as
        the starting point of the first trail segment.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)

        self.accolor[-1] = self.defcolor
        self.lastlat[-1] = self.traffic.lat[-1]
        self.lastlon[-1] = self.traffic.lon[-1]

    def update(self) -> None:
        """Add new trail segments for aircraft that moved long enough.

        Called every simulation step. When trails are inactive, only the
        last-known positions are refreshed. Otherwise, for each aircraft
        whose last recorded segment is older than `dt` seconds, a new
        line segment from the last recorded position to the current
        position is appended to the drawing buffers, and the color fading
        factors of all segments are updated.
        """
        self.acid = self.traffic.callsign
        if not self.active:
            self.lastlat = self.traffic.lat
            self.lastlon = self.traffic.lon
            self.lasttim[:] = self._get_simulation().simt
            return
        """Add linepieces for trails based on traffic data"""

        # Use temporary list/array for fast append
        lstlat0 = []
        lstlon0 = []
        lstlat1 = []
        lstlon1 = []
        lsttime = []

        # Check for update
        delta = self._get_simulation().simt - self.lasttim
        idxs = np.where(delta > self.dt)[0]

        # Add all a/c which need the update
        # if len(idxs)>0:
        #     print "len(idxs)=",len(idxs)

        for i in idxs:
            # Add to lists
            lstlat0.append(self.lastlat[i])
            lstlon0.append(self.lastlon[i])
            lstlat1.append(self.traffic.lat[i])
            lstlon1.append(self.traffic.lon[i])
            lsttime.append(self._get_simulation().simt)

            if isinstance(self.col, np.ndarray):
                # print type(trailcol[i])
                # print trailcol[i]
                # print "col type: ",type(self.col)
                self.col = self.col.tolist()

            type(self.col)
            self.col.append(self.accolor[i])

            # Update aircraft record
            self.lastlat[i] = self.traffic.lat[i]
            self.lastlon[i] = self.traffic.lon[i]
            self.lasttim[i] = self._get_simulation().simt

        # When a/c is no longer part of trail semgment,
        # it is no longer a/c data => add to the GUI send buffer
        self.newlat0.extend(lstlat0)
        self.newlon0.extend(lstlon0)
        self.newlat1.extend(lstlat1)
        self.newlon1.extend(lstlon1)
        # Update colours
        self.fcol = (
            1.0
            - np.minimum(self.tcol0, np.abs(self._get_simulation().simt - self.time)) / self.tcol0
        )

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

    def clearbg(self) -> None:  # Background
        """Clear the background trail segment buffers."""
        self.bglat0 = np.array([])
        self.bglon0 = np.array([])
        self.bglat1 = np.array([])
        self.bglon1 = np.array([])
        self.bgtime = np.array([])
        self.bgacid = []

    def clear(self) -> None:
        """Clear all trail data: foreground, background and new-line buffers."""
        self.lastlon = np.array([])
        self.lastlat = np.array([])
        self.clearfg()
        self.clearbg()
        self.clearnew()

    @command(name="TRAIL", aliases=("TRAILS",))
    def trail_status(self) -> Result[str, str]:
        """Report whether aircraft trails are enabled."""
        message = "TRAIL ON/OFF, [dt] / TRAIL acid color\n"
        message += "TRAILS ARE ON" if self.active else "TRAILS ARE OFF"
        return Ok(message)

    @command(name="TRAIL")
    def set_trail_state(
        self, enabled: OnOff, interval: PositiveFiniteFloat | None = None
    ) -> Result[str, str]:
        """Enable or disable trails, optionally changing the sample interval."""
        self.active = enabled
        if interval is not None:
            self.dt = interval
        if not enabled:
            self.clear()
        return Ok("")

    @command(name="TRAIL")
    def set_trail_color(self, callsign: Keyword, color: TrailColor) -> Result[str, str]:
        """Set the trail color for an aircraft."""
        # Change color per acid (pygame only)
        index = self.traffic.idx(callsign)
        if index < 0:
            return Err(f"Aircraft with callsign {callsign} not found")
        # Change trail color
        self.changeTrailColor(color, index)
        return Ok("")

    def changeTrailColor(self, color: str, idx: int) -> None:
        """Change the trail color of an aircraft.

        Args:
            color: Color name; must be a key of colorList
                (BLUE/CYAN/RED/YELLOW).
            idx: Aircraft index.
        """
        self.accolor[idx] = self.colorList[color]

    def reset(self) -> None:
        """Clear all trail data and switch trails off upon simulation reset."""
        # This ensures that the traffic arrays (which size is dynamic)
        # are all reset as well, so all lat,lon,sdp etc but also objects adsb
        super().reset()
        self.clear()
        self.active = False
