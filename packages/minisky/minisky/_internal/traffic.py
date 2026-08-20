"""Core flight state of every aircraft: position and motion.

Defines the `Traffic`. It holds all per-aircraft state (position, attitude,
speeds, atmosphere, autopilot selections) as numpy arrays, owns the
sub-models (autopilot, performance, conflict detection/resolution, wind,
turbulence, groups), and drives the numerical integration of the
aircraft states each simulation timestep. Each `MiniSky` runtime owns an
instance as [`runtime.traffic`][.Traffic].

Several methods double as stack-command implementations (`CRE`, `MCRE`,
`CRECONFS`, `MOVE`, `POS`, `BANK`, `THR`, `NOISE`, `CRECMD`, ...).
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Iterable
from random import Random
from typing import TYPE_CHECKING, Annotated, Literal, overload

import numpy as np
from annotated_types import Ge, IsFinite, Le, Lt

import minisky.geo as geo  # noqa: PLR0402
from minisky import quantities as q
from minisky._internal.active_waypoint import ActiveWaypoint
from minisky._internal.autopilot import Autopilot
from minisky._internal.command import (
    AcId,
    AcIdSelection,
    CommandField,
    Converter,
    DistanceM,
    Keyword,
    LatLonDeg,
    OnOff,
    ResolvedPositionArg,
    RunwayHeadingRequest,
    RunwayPosition,
    Text,
    TimeS,
    UseRunwayHeading,
    VspdMps,
    command,
)
from minisky._internal.conditions import Condition
from minisky._internal.config import MiniSkyConfig
from minisky._internal.conflict.detection import ConflictDetection
from minisky._internal.conflict.resolution import ConflictResolution
from minisky._internal.convert import latlon2txt
from minisky._internal.groups import TrafficGroups
from minisky._internal.guidance import APorASAS
from minisky._internal.kinematics import Kinematics
from minisky._internal.performance.openap import OpenAP
from minisky._internal.result import Err, Ok, Result
from minisky._internal.shapes import Shapes
from minisky._internal.traffic_arrays import TrafficArrays, VariantArray
from minisky._internal.turbulence import Turbulence
from minisky._internal.uncertainty import SurveillanceUncertainty
from minisky._internal.wind import Wind
from minisky.aero import (
    cas2tas,
    mach2tas,
    tas2cas,
    tas2mach,
    vatmos,
    vcasmach2tas,
    vtas2cas,
    vtas2mach,
)
from minisky.types import (
    AircraftCallsign,
    AircraftIndex,
    AircraftTypeCode,
    AirspeedKind,
    AirwayIdentifier,
    CasMps,
    Ge0,
    Gt0,
    Mach,
    MagneticHeadingDeg,
    StdPressureAltM,
    TrueHeadingDeg,
)

if TYPE_CHECKING:
    from minisky._internal.console import ConsoleIO
    from minisky._internal.navigation import Navdatabase
    from minisky._internal.simulation import Simulation


def _parse_throttle(value: str) -> float:
    factor = 0.01 if value.endswith("%") else 1.0
    number = value.removesuffix("%")
    if "%" in number:
        raise ValueError
    return factor * float(number)


Throttle = Annotated[
    IsFinite[Ge0[float]],
    CommandField(name="throttle", examples=("0.8", "80%")),
    Converter(_parse_throttle),
    Le(1),
]

LatitudeArg = Annotated[q.LatitudeDeg[float], Ge(-90), Le(90)]
LongitudeArg = Annotated[q.LongitudeDeg[float], Ge(-180), Le(180)]
ConflictAngleDeg = q.AngleDeg[IsFinite[float]]
BankLimitDeg = Annotated[q.BankAngleDeg[IsFinite[Gt0[float]]], Lt(90)]


_DEFAULT_ALTITUDE = StdPressureAltM(q.ft_to_m(25000.0))
_DEFAULT_AIRSPEED = CasMps(q.kt_to_mps(300.0))


class Traffic(TrafficArrays):
    """Central traffic module holding the core state of all simulated aircraft.

    Traffic is the top-level
    [`TrafficArrays`][minisky.TrafficArrays] object: all per-aircraft
    arrays registered by its child entities (autopilot, active waypoint data,
    performance model, conflict detection/resolution, etc.) grow and shrink
    together when aircraft are created or deleted.

    Every simulation step, [`update`][.update] advances all aircraft in a
    fixed order:

    1. Atmosphere: pressure, density and temperature at each aircraft's
       altitude.
    2. Surveillance and trajectory noise
       (`runtime.traffic.noise`, surveillance uncertainty).
    3. Autopilot/FMS guidance
       ([`runtime.traffic.ap`][minisky.Autopilot]).
    4. Conflict detection and resolution
       ([`runtime.traffic.cd`][minisky.ConflictDetection],
       [`runtime.traffic.cr`][minisky.ConflictResolution]).
    5. Selection between the autopilot and conflict-resolution commands
       for track, speed, altitude and vertical speed
       ([`runtime.traffic.aporasas`][minisky.APorASAS]).
    6. Performance model update, then limiting of the commanded speed,
       vertical speed and altitude
       ([`runtime.traffic.perf`][minisky.OpenAP]).
    7. Integration of airspeed, heading, vertical speed, ground speed and
       position
       ([`runtime.traffic.kinematics`][minisky.Kinematics]).
    8. Turbulence perturbation of the new positions
       (`runtime.traffic.turbulence`).
    9. Conditional commands triggered by the new state
       (`runtime.traffic.cond`, conditional commands).

    All internal state is kept in SI units; stack commands parse explicit
    units and quantity/reference tags at the boundary.
    """

    def __init__(
        self,
        config: MiniSkyConfig,
        python_random: Random,
        numpy_random: np.random.RandomState,
        shapes: Shapes,
        navigation: Navdatabase,
        console: ConsoleIO,
        get_simulation: Callable[[], Simulation],
        stack_command: Callable[..., None],
        select_implementation: Callable[[str, str], Result[str, str]],
    ) -> None:
        super().__init__()
        self.config = config
        self.python_random = python_random
        self.numpy_random = numpy_random
        self.shapes = shapes
        self.navigation = navigation
        self.console = console
        self._get_simulation = get_simulation
        self.stack_command = stack_command
        self.select_implementation = select_implementation

        self.ntraf = 0
        self.cond = Condition(self, stack_command)
        self.wind = Wind()
        self.wind.reparent(self)
        self.turbulence = Turbulence(self, get_simulation)
        self.translvl: q.PressureAltitudeM[float] = q.ft_to_m(5000.0)  # pyright: ignore[reportGeneralTypeIssues]
        """Transition level used when interpreting and reporting aircraft altitude."""
        self.crecmdlist: list[str] = []
        """Commands issued for every newly created aircraft."""

        with self.settrafarrays():
            self.callsign: list[AircraftCallsign] = []
            self.typecode: list[AircraftTypeCode] = []

            self.lat: q.LatitudeDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.lon: q.LongitudeDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.distflown: q.DistanceM[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.alt: q.PressureAltitudeM[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.hdg: q.TrueHeadingDegrees[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.trk: q.GroundTrackDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.tas: q.TrueAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.gs: q.GroundSpeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.gsnorth: q.GroundSpeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.gseast: q.GroundSpeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.cas: q.CalibratedAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.M: q.MachNumber[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vs: q.VerticalRateMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.p: q.StaticPressurePa[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.rho: q.DensityKgPerM3[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.Temp: q.StaticTemperatureK[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.windnorth: q.WindSpeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.windeast: q.WindSpeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

            self.selected_airspeed: VariantArray[np.ndarray] = VariantArray(
                np.array([]), np.array([], dtype=np.uint8)
            )
            """Selected CAS or Mach command for each aircraft."""
            self.aptas: q.TrueAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """True airspeed used to initialize the autopilot state."""
            self.selalt: q.PressureAltitudeM[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.selvs: q.VerticalRateMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.swlnav = np.array([], dtype=bool)
            """Per-aircraft LNAV enable flags."""
            self.swvnav = np.array([], dtype=bool)
            """Per-aircraft VNAV enable flags."""
            self.swvnavairspeed = np.array([], dtype=bool)
            """Per-aircraft VNAV airspeed-guidance flags."""

            self.cd = ConflictDetection(config, self, stack_command)
            self.cr = ConflictResolution(config, self, select_implementation)
            self.ap = Autopilot(self, get_simulation)
            self.aporasas = APorASAS(self)
            self.noise = SurveillanceUncertainty(self, get_simulation)
            self.actwp = ActiveWaypoint(self)
            self.perf = OpenAP(self)
            self.kinematics = Kinematics(self, get_simulation)

            self.groups = TrafficGroups(self, shapes)

            self.swats = np.array(
                [], dtype=bool
            )  # Switch indicating whether autothrottle system is on/off
            """Per-aircraft autothrottle enable flags."""
            self.thr = np.array([])
            """Fixed throttle fractions used while autothrottle is disabled."""

            self.coslat = np.array([])
            """Cached cosine of latitude used by position integration and turbulence."""
            self.eps = np.array([])
            """Small nonzero values used to guard near-zero divisions."""
            self.work: q.EnergyJ[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Work done by thrust since aircraft creation."""

    @property
    def simulation(self) -> Simulation:
        """Return the simulation that owns this traffic object."""
        return self._get_simulation()

    def reset(self) -> None:
        """Clear all traffic data upon simulation reset.

        Empties all per-aircraft arrays (including those of child entities),
        resets the performance, wind and turbulence models, switches off
        trajectory noise and restores the default transition level.
        """
        # Some child reset functions depend on a correct value of self.ntraf
        self.ntraf = 0
        # This ensures that the traffic arrays (which size is dynamic)
        # are all reset as well, so all lat,lon,sdp etc but also objects adsb
        super().reset()

        self.perf.reset()

        self.wind.clear()
        self.cond.reset()

        self.turbulence.reset()

        self.configure_noise(False)

        self.translvl = q.ft_to_m(5000.0)

    @command(name="CRE", aliases=("CREATE",))
    def command_cre(
        self,
        callsign: Keyword,
        actype: Keyword,
        position: ResolvedPositionArg,
        hdg: TrueHeadingDeg[IsFinite[float]]
        | MagneticHeadingDeg[IsFinite[float]]
        | UseRunwayHeading
        | None = None,
        alt: StdPressureAltM[IsFinite[float]] = _DEFAULT_ALTITUDE,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] = _DEFAULT_AIRSPEED,
    ) -> Result[str, str]:
        """Create an aircraft."""
        if isinstance(position, RunwayPosition):
            coordinates = position.coordinates
            default_heading = position.runway_heading
        else:
            coordinates = position
            default_heading = 45.0

        if isinstance(hdg, RunwayHeadingRequest):
            if not isinstance(position, RunwayPosition):
                return Err("CRE: heading * requires a runway position")
            heading = position.runway_heading
        elif hdg is None:
            heading = default_heading
        elif isinstance(hdg, MagneticHeadingDeg):
            heading = (hdg.value + geo.magdec(coordinates.lat, coordinates.lon)) % 360.0
        else:
            heading = hdg.value

        return self.cre(
            callsign,
            actype,
            coordinates.lat,
            coordinates.lon,
            heading,
            alt,
            airspeed,
        )

    def cre(
        self,
        callsign: Keyword,
        actype: Keyword = "A320",
        lat: q.LatitudeDeg[float] = 53.0,
        lon: q.LongitudeDeg[float] = 4.0,
        hdg: q.TrueHeadingDegrees[float] = 45.0,
        alt: StdPressureAltM[IsFinite[float]] = _DEFAULT_ALTITUDE,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] = _DEFAULT_AIRSPEED,
    ) -> Result[str, str]:
        """Create a single aircraft and add it to the traffic database.

        Callsigns are normalized to upper case and must be unique.
        After creation, any commands stored via `CRECMD` are stacked for the
        new aircraft.
        """

        name_error = self._aircraft_name_collision(callsign)
        if name_error is not None:
            return Err(name_error)

        callsigns = np.array([callsign.upper()])
        actype_ = np.array([actype])
        lat_ = np.array([lat])
        lon_ = np.array([lon])
        alt_ = np.array([alt.value])
        hdg_ = np.array([hdg])
        match airspeed:
            case CasMps(value):
                airspeed_value = value
                airspeed_kind = AirspeedKind.CAS
            case Mach(value):
                airspeed_value = value
                airspeed_kind = AirspeedKind.MACH
        selected_airspeed = VariantArray(
            np.array([airspeed_value]), np.array([airspeed_kind], dtype=np.uint8)
        )

        self.__create_aircraft(callsigns, actype_, lat_, lon_, hdg_, alt_, selected_airspeed)

        return Ok(f"Aircraft {callsign} created")

    @command(name="MCRE")
    def mcre(
        self,
        n: int,
        lat_min: LatitudeArg = 53.0,
        lon_min: LongitudeArg = 0.0,
        lat_max: LatitudeArg = 60.0,
        lon_max: LongitudeArg = 10.0,
        actype: Keyword = "A320",
        acalt: StdPressureAltM[IsFinite[float]] | None = None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
    ) -> Result[str, str]:
        """Create multiple aircraft at random positions in a lat/lon box.

        Implements the `MCRE` stack command. Callsigns are generated randomly
        (two letters plus a sequence number). Heading is drawn uniformly from
        1-360 deg; when not given, altitude is drawn from 2000-39000 ft and
        calibrated airspeed from 250-450 kts. The default area is the North Sea region.
        """

        idtmp = (
            chr(self.python_random.randint(65, 90))
            + chr(self.python_random.randint(65, 90))
            + "{:>03}"
        )
        callsign = [idtmp.format(i) for i in range(n)]
        for name in callsign:
            name_collision = self._aircraft_name_collision(name)
            if name_collision is not None:
                return Err(name_collision)

        actype_ = np.array([actype] * n)

        aclat = self.numpy_random.rand(n) * (lat_max - lat_min) + lat_min
        aclon = self.numpy_random.rand(n) * (lon_max - lon_min) + lon_min
        achdg = self.numpy_random.randint(1, 360, n)
        acalt_ = (
            np.full(n, acalt.value)
            if acalt is not None
            else q.ft_to_m(self.numpy_random.randint(2000, 39000, n))
        )
        if airspeed is None:
            airspeed_value = q.kt_to_mps(self.numpy_random.randint(250, 450, n))
            airspeed_kind = np.full(n, AirspeedKind.CAS, dtype=np.uint8)
        else:
            match airspeed:
                case CasMps(value):
                    airspeed_value = np.full(n, value)
                    airspeed_kind = np.full(n, AirspeedKind.CAS, dtype=np.uint8)
                case Mach(value):
                    airspeed_value = np.full(n, value)
                    airspeed_kind = np.full(n, AirspeedKind.MACH, dtype=np.uint8)

        self.__create_aircraft(
            np.array(callsign),
            actype_,
            aclat,
            aclon,
            achdg,
            acalt_,
            VariantArray(airspeed_value, airspeed_kind),
        )

        return Ok(f"{n} aircraft created")

    def _aircraft_name_collision(self, callsign: AircraftCallsign) -> str | None:
        """Return why a new aircraft name is unavailable, if anything.

        Aircraft identifiers must remain unique. BlueSky allowed the same
        text to name an aircraft, group, and area; command-specific resolution
        keeps that scenario compatibility.
        """
        name = callsign.upper()
        if name in self.callsign:
            return f"aircraft {name} already exists"
        return None

    def __create_aircraft(
        self,
        callsigns: np.ndarray,
        actype: np.ndarray,
        lat: q.LatitudeDeg[np.ndarray],
        lon: q.LongitudeDeg[np.ndarray],
        hdg: q.TrueHeadingDegrees[np.ndarray],
        alt: q.PressureAltitudeM[np.ndarray],
        selected_airspeed: VariantArray[np.ndarray],
    ) -> None:
        """Append one or more aircraft to all traffic arrays.

        Common backend for `cre` and `mcre`: resizes all (child) traffic
        arrays, initializes position, heading, speeds, atmosphere and wind
        for the new aircraft, and stacks any `CRECMD` default commands.
        All array arguments must have the same length. `selected_airspeed.values` contains
        [`CAS` in m/s][minisky.types.CasMps] or [`Mach`][minisky.types.Mach],
        disambiguated by [`AirspeedKind`][minisky.types.AirspeedKind] in `kind`.
        """

        n = len(callsigns)

        super().create(n)
        self.ntraf += n

        lon[lon > 180.0] -= 360.0
        lon[lon < -180.0] += 360.0

        self.callsign[-n:] = callsigns
        self.typecode[-n:] = actype

        self.lat[-n:] = lat
        self.lon[-n:] = lon
        self.alt[-n:] = alt

        self.hdg[-n:] = hdg
        self.trk[-n:] = hdg

        is_mach = selected_airspeed.kind == AirspeedKind.MACH
        tas = vcasmach2tas(selected_airspeed.values, is_mach, alt)
        self.tas[-n:] = tas
        self.cas[-n:] = np.where(is_mach, vtas2cas(tas, alt), selected_airspeed.values)
        self.M[-n:] = np.where(is_mach, selected_airspeed.values, vtas2mach(tas, alt))
        self.gs[-n:] = self.tas[-n:]
        hdgrad = np.radians(hdg)
        self.gsnorth[-n:] = self.tas[-n:] * np.cos(hdgrad)
        self.gseast[-n:] = self.tas[-n:] * np.sin(hdgrad)

        self.p[-n:], self.rho[-n:], self.Temp[-n:] = vatmos(alt)

        if self.wind.has_wind:
            # TODO(abraham): use AGL (see issue #22)
            applywind = self.alt[-n:] > q.ft_to_m(50.0)
            self.windnorth[-n:], self.windeast[-n:] = self.wind.getdata(
                self.lat[-n:], self.lon[-n:], self.alt[-n:]
            )
            self.gsnorth[-n:] = self.gsnorth[-n:] + self.windnorth[-n:] * applywind
            self.gseast[-n:] = self.gseast[-n:] + self.windeast[-n:] * applywind
            self.trk[-n:] = np.logical_not(applywind) * hdg + applywind * np.degrees(
                np.arctan2(self.gseast[-n:], self.gsnorth[-n:])
            )
            self.gs[-n:] = np.sqrt(self.gsnorth[-n:] ** 2 + self.gseast[-n:] ** 2)
        else:
            self.windnorth[-n:] = 0.0
            self.windeast[-n:] = 0.0

        self.selected_airspeed.values[-n:] = selected_airspeed.values
        self.selected_airspeed.kind[-n:] = selected_airspeed.kind
        self.aptas[-n:] = self.tas[-n:]
        self.selalt[-n:] = self.alt[-n:]

        self.coslat[-n:] = np.cos(np.radians(lat))
        self.eps[-n:] = 0.01

        # Finally call create for child TrafficArrays. This only needs to be done
        # manually in Traffic.
        self.create_children(n)

        for j in range(self.ntraf - n, self.ntraf):
            for cmdtxt in self.crecmdlist:
                self.stack_command(self.callsign[j] + " " + cmdtxt)

    @command(name="CRECONFS")
    def creconfs(
        self,
        callsign: Keyword,
        actype: Keyword,
        targetidx: AcId,
        dpsi: ConflictAngleDeg,
        dcpa: DistanceM,
        tlosh: TimeS,
        dH: DistanceM | None = None,
        tlosv: TimeS | None = None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
    ) -> None:
        """Create an aircraft in conflict with a target aircraft.

        Implements the `CRECONFS` stack command. The intruder position, track
        and airspeed are computed such that, relative to the target aircraft,
        separation is lost after the given time with the given distance at
        the closest point of approach. The protected-zone radius and height
        from the config (`asas_pzr`, `asas_pzh`) are taken into account. Omitting
        the vertical offset creates a level conflict; omitted vertical LoS time
        defaults to the horizontal LoS time, and omitted airspeed uses the
        ownship ground speed.

        Args:
            callsign: Callsign of the new intruder.
            actype: Aircraft type of the new intruder.
            targetidx: Ownship aircraft index.
            dpsi: Angle between ownship and intruder tracks.
            dcpa: Requested horizontal separation at CPA.
            tlosh: Time until horizontal loss of separation.
            dH: Initial vertical offset; `None` creates a level conflict.
            tlosv: Time until vertical loss of separation; defaults to `tlosh`.
            airspeed: Explicit intruder CAS/Mach; omitted uses ownship ground speed.
        """
        latref = self.lat[targetidx]
        lonref = self.lon[targetidx]
        altref = self.alt[targetidx]
        trkref = np.radians(self.trk[targetidx])
        gsref = self.gs[targetidx]
        vsref = self.vs[targetidx]
        cpa = dcpa
        pzr = q.nmi_to_m(self.config.asas_pzr)
        pzh = q.ft_to_m(self.config.asas_pzh)
        trk = trkref + np.radians(dpsi)

        if dH is None:
            acalt = altref
            acvs = 0.0
        else:
            acalt = altref + dH
            tlosv = tlosh if tlosv is None else tlosv
            acvs = vsref - np.sign(dH) * (abs(dH) - pzh) / tlosv

        if airspeed is not None:
            # Convert the explicit airspeed command to groundspeed, assuming that
            # wind at intruder position is similar to wind at ownship position.
            tas = (
                cas2tas(airspeed.value, acalt)
                if isinstance(airspeed, CasMps)
                else mach2tas(airspeed.value, acalt)
            )
            tasn, tase = tas * np.cos(trk), tas * np.sin(trk)
            wind_north, wind_east = self.wind.getdata(latref, lonref, acalt)
            gsn, gse = tasn + wind_north, tase + wind_east
        else:
            # Groundspeed is the same as ownship
            gsn, gse = gsref * np.cos(trk), gsref * np.sin(trk)

        # Horizontal relative velocity vector
        vreln, vrele = gsref * np.cos(trkref) - gsn, gsref * np.sin(trkref) - gse
        # Relative velocity magnitude
        vrel = np.sqrt(vreln * vreln + vrele * vrele)
        # Relative travel distance to closest point of approach
        drelcpa = tlosh * vrel + (0 if cpa > pzr else np.sqrt(pzr * pzr - cpa * cpa))
        # Initial intruder distance
        dist = np.sqrt(drelcpa * drelcpa + cpa * cpa)
        # Rotation matrix diagonal and cross elements for distance vector
        rd = drelcpa / dist
        rx = cpa / dist
        # Rotate relative velocity vector to obtain intruder bearing
        brn = np.degrees(np.atan2(-rx * vreln + rd * vrele, rd * vreln + rx * vrele))

        aclat, aclon = geo.kwikpos(latref, lonref, brn, dist)
        aclat_scalar = float(aclat)
        aclon_scalar = float(aclon)
        # convert groundspeed to CAS, and track to heading using actual
        # intruder position
        wind_north, wind_east = self.wind.getdata(aclat_scalar, aclon_scalar, acalt)
        tasn, tase = gsn - wind_north, gse - wind_east
        derived_cas = tas2cas(np.sqrt(tasn * tasn + tase * tase), acalt)
        achdg = np.degrees(np.atan2(tase, tasn))

        # Create and, when necessary, set vertical speed
        self.cre(
            callsign,
            actype,
            aclat_scalar,
            aclon_scalar,
            float(achdg),
            StdPressureAltM(float(acalt)),
            CasMps(float(derived_cas)),
        )
        self.ap.selaltcmd(np.asarray([len(self.lat) - 1]), StdPressureAltM(float(altref)), acvs)
        self.vs[-1] = acvs

    def delete(self, idx: AircraftIndex | np.ndarray) -> bool:  # type: ignore[override]
        """Delete one or more aircraft from the traffic database.

        Removes the corresponding entries from all (child) traffic arrays
        and updates the aircraft count. Used by the `DEL` stack command.
        """
        # If this is a multiple delete, sort first for list delete
        # (which will use list in reverse order to avoid index confusion)
        if isinstance(idx, Collection):
            idx = np.sort(idx)

        super().delete(idx)

        self.ntraf = len(self.lat)
        return True

    def update(self) -> None:
        """Perform one simulation time step for all aircraft.

        Called every step by the simulation loop. In order: updates the
        atmosphere, surveillance noise, autopilot and airborne separation
        assurance (ASAS) guidance, decides per channel between autopilot and
        ASAS commands, updates the performance model and limits the commanded
        speeds accordingly, integrates airspeed/heading/vertical speed,
        ground speed and position, applies turbulence, and triggers conditional
        commands. Does nothing when there is no traffic.
        """
        if self.ntraf == 0:
            return

        self.p, self.rho, self.Temp = vatmos(self.alt)
        self.noise.update()
        self.ap.update()  # Autopilot logic
        self.update_asas()  # Airborne Separation Assurance
        self.aporasas.update()  # Decide to use autopilot or ASAS for commands
        self.perf.update()
        self.aporasas.tas, self.aporasas.vs, self.aporasas.alt = self.perf.limits(
            self.aporasas.tas, self.aporasas.vs, self.aporasas.alt, self.kinematics.ax
        )
        self.kinematics.update()
        self.turbulence.update()
        self.cond.update()

    def update_asas(self) -> None:
        """Run conflict detection and conflict resolution for all aircraft."""
        # Conflict detection and resolution
        self.cd.update(self, self)
        self.cr.update(self.cd, self, self)

    @overload
    def idx(self, callsign: AircraftCallsign) -> AircraftIndex | None: ...
    @overload
    def idx(
        self,
        callsign: list[AircraftCallsign] | tuple[AircraftCallsign, ...] | set[AircraftCallsign],
    ) -> list[AircraftIndex | None]: ...
    def idx(
        self, callsign: AircraftCallsign | Iterable[AircraftCallsign]
    ) -> AircraftIndex | None | list[AircraftIndex | None]:
        """Find the traffic-array index for one or more callsigns.

        Args:
            callsign: A single callsign string, or an iterable of callsigns.
                The special values "*" and "#" refer to the most recently
                created aircraft.

        Returns:
            Index of the aircraft (or list of optional indices when an
            iterable was given); None for callsigns that are not found.
        """
        if not isinstance(callsign, str):
            tmp = {v: i for i, v in enumerate(self.callsign)}
            return [tmp.get(acidi) for acidi in callsign]
        else:
            # FIXME(abraham): we might not want to do this
            # we already handle * explicitly and we should reconsider
            # this implicit behaviour.
            if callsign in ("#", "*"):
                return self.ntraf - 1 if self.ntraf else None

            try:
                return self.callsign.index(callsign.upper())
            except ValueError:
                return None

    @command(name="NOISE")
    def noise_status(self) -> Result[str, str]:
        """Report trajectory-noise state."""
        return Ok(f"Noise is currently {'on' if self.turbulence.active else 'off'}")

    @command(name="NOISE")
    def configure_noise(self, noise: OnOff) -> Result[str, str]:
        """Enable or disable trajectory and surveillance noise."""
        self.turbulence.setnoise(noise)
        self.noise.setnoise(noise)
        return Ok("")

    def engchange(self, acidx: AircraftIndex, engid: str) -> None:
        """Change the engine type of an aircraft in the performance model."""
        self.perf.engchange(acidx, engid)  # type: ignore[attr-defined]

    @command(name="MOVE")
    def move(
        self,
        idx: AcId,
        position: LatLonDeg,
        alt: StdPressureAltM[IsFinite[float]] | None = None,
        hdg: TrueHeadingDeg[IsFinite[float]] | MagneticHeadingDeg[IsFinite[float]] | None = None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
        vspd: VspdMps | None = None,
    ) -> None:
        """Instantaneously move an aircraft to a new position/state.

        Optional state values are left unchanged when omitted.
        Setting a vertical speed disengages VNAV.

        Args:
            idx: Aircraft to move.
            position: New aircraft position.
            alt: New altitude; also becomes the selected altitude.
            hdg: New heading; also becomes the autopilot track command.
            airspeed: New CAS or Mach command.
            vspd: New vertical speed; setting it disengages VNAV.
        """
        self.lat[idx] = position.lat
        self.lon[idx] = position.lon

        if alt is not None:
            self.alt[idx] = alt.value
            self.selalt[idx] = alt.value

        if hdg is not None:
            heading = (
                (hdg.value + geo.magdec(position.lat, position.lon)) % 360.0
                if isinstance(hdg, MagneticHeadingDeg)
                else hdg.value
            )
            self.hdg[idx] = heading
            self.ap.trk[idx] = heading

        if airspeed is not None:
            h = alt.value if alt is not None else float(self.alt[idx])
            if isinstance(airspeed, CasMps):
                tas = cas2tas(airspeed.value, h)
                cas = airspeed.value
                mach = tas2mach(tas, h)
            else:
                tas = mach2tas(airspeed.value, h)
                cas = tas2cas(tas, h)
                mach = airspeed.value
            self.tas[idx] = tas
            self.cas[idx] = cas
            self.M[idx] = mach
            self.selected_airspeed.values[idx] = airspeed.value
            self.selected_airspeed.kind[idx] = (
                AirspeedKind.CAS if isinstance(airspeed, CasMps) else AirspeedKind.MACH
            )

        if vspd is not None:
            self.vs[idx] = vspd
            self.swvnav[idx] = False

    @command(name="POS", aliases=("AWY", "AIRPORT", "RUNWAYS", "AIRWAY", "AIRWAYS"))
    def position(self, name: Keyword) -> Result[str, str]:
        """Show information on an aircraft, airport, waypoint or navaid."""
        index = self.idx(name)
        if index is not None:
            return self.position_aircraft(index)
        return self.position_by_name(name)

    def position_aircraft(self, idx: AircraftIndex) -> Result[str, str]:
        """Generate a position report for a single aircraft.

        The report includes position, heading/track [deg], altitude [ft],
        vertical speed [fpm], CAS/TAS/GS [kts], Mach, active FMS modes
        (LNAV/VNAV) with the active waypoint, and origin/destination.
        """

        callsign = self.callsign[idx]

        actype = self.typecode[idx]
        latlon = latlon2txt(self.lat[idx], self.lon[idx])
        alt = round(q.m_to_ft(self.alt[idx]))
        hdg = round(self.hdg[idx])
        trk = round(self.trk[idx])
        cas = round(q.mps_to_kt(self.cas[idx]))
        tas = round(q.mps_to_kt(self.tas[idx]))
        gs = round(q.mps_to_kt(self.gs[idx]))
        M = self.M[idx]
        VS = round(q.mps_to_fpm(self.vs[idx]))
        route = self.ap.route[idx]

        # Position report
        info = (
            f"Information on aircraft {callsign} (index: {idx})\n"
            f"Aircraft typecde: {actype} \n"
            f"Position: {latlon}\n"
            f"Hdg: {hdg:03d} \tTrk: {trk:03d}\n"
            f"Alt: {alt} ft\tV/S: {VS} fpm\n"
            f"CAS/TAS/GS: {cas}/{tas}/{gs} kts   M: {M:.3f}\n"
        )

        # FMS AP modes
        if self.swlnav[idx] and route.wpname and (active_idx := route.iactwp) is not None:
            if self.swvnav[idx]:
                if self.swvnavairspeed[idx]:
                    info = info + "VNAV (incl.VNAVSPD), "
                else:
                    info = info + "VNAV (NOT VNAVSPD), "

            info += "LNAV to " + route.wpname[active_idx] + "\n"

        # Flight info: Destination and origin
        if self.ap.orig[idx] != "" or self.ap.dest[idx] != "":
            info = info + "Flying"

            if self.ap.orig[idx] != "":
                info = info + " from " + self.ap.orig[idx]

            if self.ap.dest[idx] != "":
                info = info + " to " + self.ap.dest[idx]

        return Ok(info)

    def position_by_name(self, name: str) -> Result[str, str]:
        """Look up a name and generate an information report for it.

        Searches, in order: airports, aircraft callsigns, waypoints/navaids,
        and airways in the navigation database. Airport reports include
        position, elevation [ft] and runways; navaid reports include type,
        frequency and airway connections.

        Args:
            name: Name/identifier to look up (case-insensitive).
        """
        name = name.upper()

        lines = "Information on " + name + ":\n"

        idx_airport = self.navigation.getaptidx(name)
        if idx_airport is not None:
            airport_size = self.navigation.apsize[idx_airport].name.lower()

            aptname = self.navigation.aptname[idx_airport]
            aptlat = self.navigation.aptlat[idx_airport]
            aptlon = self.navigation.aptlon[idx_airport]
            aptelev = self.navigation.aptelev[idx_airport]

            idx_cc = self.navigation.cocode2.index(self.navigation.aptco[idx_airport].upper())
            country_name = self.navigation.coname[idx_cc].upper()
            country_code = self.navigation.aptco[idx_airport]

            lines += (
                f"{aptname} is a {airport_size} airport in {country_name} ({country_code}):\n"
                f"Position: {latlon2txt(aptlat, aptlon)}\n"
                f"Elevation: {round(q.m_to_ft(aptelev))} ft \n"
            )

            if self.navigation.aptid[idx_airport] in self.navigation.rwythresholds:
                runways = self.navigation.rwythresholds[self.navigation.aptid[idx_airport]].keys()
                if runways:
                    lines += f"Runways: {', '.join(runways)}\n"

            return Ok(lines)

        idx_ac = self.idx(name)
        if idx_ac is not None:
            return self.position_aircraft(idx_ac)

        else:
            idx_waypoints = self.navigation.getwpindices(name)
            if idx_waypoints:
                typetxt = ""
                desctxt = ""
                lastdesc = "XXXXXXXX"
                for i in idx_waypoints:
                    if typetxt == "":
                        typetxt = typetxt + self.navigation.wptype[i]
                    else:
                        typetxt = typetxt + " and " + self.navigation.wptype[i]

                    samedesc = self.navigation.wpdesc[i] == lastdesc
                    if desctxt == "":
                        desctxt = desctxt + self.navigation.wpdesc[i]
                        lastdesc = self.navigation.wpdesc[i]
                    elif not samedesc:
                        desctxt = desctxt + "\n" + self.navigation.wpdesc[i]
                        lastdesc = self.navigation.wpdesc[i]

                    if self.navigation.wptype[i] in ["VOR", "DME", "TACAN"] and not samedesc:
                        desctxt = desctxt + " " + str(self.navigation.wpfreq[i]) + " MHz"
                    elif self.navigation.wptype[i] == "NDB" and not samedesc:
                        desctxt = desctxt + " " + str(self.navigation.wpfreq[i]) + " kHz"

                iwp = idx_waypoints[0]

                lines += (
                    f"{name} is a {typetxt} with \n"
                    f"Position: {latlon2txt(self.navigation.wplat[iwp], self.navigation.wplon[iwp])}\n"
                )

                if len(desctxt) > 0:
                    lines += f"{desctxt}\n"

                if self.navigation.wptype[iwp] == "VOR":
                    lines += f"Variation: {self.navigation.wpvar[iwp]} deg\n"

                n_other = self.navigation.wpid.count(name) - len(idx_waypoints)
                if n_other > 0:
                    lines += f"Attention: {n_other} other waypoint(s) also has name {name}\n"

                connect = self.navigation.listconnections(
                    name, self.navigation.wplat[iwp], self.navigation.wplon[iwp]
                )
                if len(connect) > 0:
                    awset = set()
                    for c in connect:
                        awset.add(c.airway)

                    lines += f"Connected to airways: {'-'.join(awset)}\n"

                return Ok(lines)

            else:  # airway
                awid: AirwayIdentifier = name
                airway = self.navigation.listairway(awid)
                if len(airway) > 0:
                    lines = ""
                    for segment in airway:
                        lines += f"Airway {awid}: {' - '.join(segment)}\n"
                    return Ok(lines)

        return Err(f"{name} not found as aircraft, airport, navaid, or waypoint")

    def settrans(self, alt: StdPressureAltM[IsFinite[float]] | None = None) -> Result[str, str]:
        """Set or show the transition level."""
        if alt is not None:
            if alt.value > 0.0:
                self.translvl = alt.value
                return Ok("")
            return Err("Transition level needs to be ft/FL and larger than zero")

        tlvl = round(q.m_to_ft(self.translvl))
        return Ok(f"Transition level = {tlvl}/FL{round(tlvl / 100.0)}")

    @command(name="BANK", aliases=("BANKLIM",))
    def bank_limit_status(self, idx: AcIdSelection) -> Result[str, str]:
        """Show the bank-angle limit for an aircraft or selection."""
        return Ok(
            "\n".join(
                f"Banklimit of {self.callsign[index]} is "
                f"{int(np.degrees(self.ap.bankdef[index]))} deg"
                for index in idx
            )
        )

    @command(name="BANK")
    def set_bank_limit(self, idx: AcIdSelection, bankangle: BankLimitDeg) -> Result[str, str]:
        """Set the bank-angle limit for an aircraft or selection."""
        self.ap.bankdef[idx] = np.radians(bankangle)
        return Ok("")

    @command(name="THR")
    def throttle_status(self, idx: AcId) -> Result[str, str]:
        """Report autothrottle state and fixed throttle when applicable."""
        if self.swats[idx]:
            return Ok("ATS of " + self.callsign[idx] + " is ON")
        return Ok("ATS of " + self.callsign[idx] + " is OFF. THR is " + str(self.thr[idx]))

    @command(name="THR")
    def enable_autothrottle(self, idx: AcId, _mode: Literal["AUTO", "OFF"]) -> Result[str, str]:
        """Enable autothrottle."""
        self.swats[idx] = True
        return Ok("")

    @command(name="THR")
    def set_idle_throttle(self, idx: AcId, _mode: Literal["IDLE"]) -> Result[str, str]:
        """Disable autothrottle and select idle thrust."""
        self.swats[idx] = False
        self.thr[idx] = 0.0
        return Ok("")

    @command(name="THR")
    def set_throttle(self, idx: AcId, throttle: Throttle) -> Result[str, str]:
        """Disable autothrottle and set a fixed throttle fraction."""
        self.swats[idx] = False
        self.thr[idx] = throttle
        return Ok("")

    def _crecmd_status(self) -> Result[str, str]:
        if self.crecmdlist:
            commands = "; ".join(f"[acid] {text}" for text in self.crecmdlist)
            return Ok(f"CRECMD list: {commands}")
        return Ok("CRECMD will add a/c specific commands to an aircraft after creation")

    @command(name="CRECMD")
    def crecmd_status(self) -> Result[str, str]:
        """Show commands issued for every newly created aircraft."""
        return self._crecmd_status()

    @command(name="CRECMD")
    def crecmd_status_explicit(self, _query: Literal["?"]) -> Result[str, str]:
        """Show commands issued for every newly created aircraft."""
        return self._crecmd_status()

    @command(name="CRECMD")
    def add_crecmd(self, cmdline: Text) -> Result[str, str]:
        """Add a command to issue for every newly created aircraft."""
        self.crecmdlist.append(cmdline)
        return Ok("")

    @command(name="CLRCRECMD")
    def clrcrecmd(self) -> Result[str, str]:
        """Clear the list of commands issued for newly created aircraft.

        Implements the `CLRCRECMD` stack command, removing all command lines
        previously added with `CRECMD`.
        """
        ncrecmd = len(self.crecmdlist)
        if ncrecmd == 0:
            return Ok("CLRCRECMD deletes all commands on clears command")
        else:
            self.crecmdlist = []
            return Ok(f"All {ncrecmd} crecmd commands deleted.")
