"""Conditional commands triggered by altitude, speed, or distance crossings."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, TypeAlias

from minisky import quantities as q
from minisky.command import (
    AcId,
    AltM,
    LatLonDeg,
    LatLonDegrees,
    NonNegativeFiniteFloat,
    SpeedMpsOrMach,
    Text,
    command,
)
from minisky.tools.geo import qdrdist

if TYPE_CHECKING:
    from minisky.traffic import Traffic


@dataclass(slots=True)
class AltitudeCondition:
    """Pending crossing of an altitude target."""

    callsign: str
    target: q.PressureAltitudeM[float]
    last_difference: q.VerticalDistanceM[float]
    command: str


@dataclass(slots=True)
class SpeedCondition:
    """Pending crossing of a legacy CAS-or-Mach speed target."""

    callsign: str
    # TODO(abraham): #40 must split CAS and Mach command values at runtime.
    target: q.MachNumber[float] | q.CalibratedAirspeedMps[float]
    last_difference: float
    command: str


@dataclass(slots=True)
class DistanceCondition:
    """Pending crossing of a distance from a geographic reference point."""

    callsign: str
    target: q.DistanceM[float]
    last_difference: q.DistanceM[float]
    command: str
    reference: LatLonDegrees


PendingCondition: TypeAlias = AltitudeCondition | SpeedCondition | DistanceCondition


class Condition:
    """Administration of pending ATALT, ATSPD, and ATDIST commands.

    Each pending condition is one typed record, so a distance reference cannot
    be attached to an altitude/speed condition and values with different units
    cannot share one numeric array.
    """

    def __init__(self, traffic: Traffic, stack_command: Callable[..., None]) -> None:
        self.traffic = traffic
        self.stack_command = stack_command
        self.conditions: list[PendingCondition] = []

    @property
    def ncond(self) -> int:
        return len(self.conditions)

    def reset(self) -> None:
        """Clear all pending conditional commands."""
        self.conditions.clear()

    def _actual(self, condition: PendingCondition, acidx: int) -> float:
        if isinstance(condition, AltitudeCondition):
            return float(self.traffic.alt[acidx])
        if isinstance(condition, SpeedCondition):
            return float(self.traffic.cas[acidx])
        _bearing, distance = qdrdist(
            self.traffic.lat[acidx],
            self.traffic.lon[acidx],
            condition.reference.lat,
            condition.reference.lon,
        )
        return float(distance)

    def update(self) -> None:
        """Execute conditions whose target value was crossed since the last update."""
        remaining: list[PendingCondition] = []
        for condition in self.conditions:
            acidx = self.traffic.idx(condition.callsign)
            if acidx is None:
                continue
            actual = self._actual(condition, acidx)
            difference = condition.target - actual
            if difference * condition.last_difference <= 0.0:
                self.stack_command(condition.command)
                continue
            condition.last_difference = difference
            remaining.append(condition)
        self.conditions = remaining

    @command(name="ATALT")
    def ataltcmd(self, acidx: AcId, targalt: AltM, cmdtxt: Text) -> bool:
        """Schedule a command for when an aircraft crosses an altitude."""
        callsign = self.traffic.callsign[acidx]
        actual = float(self.traffic.alt[acidx])
        self.conditions.append(AltitudeCondition(callsign, targalt, targalt - actual, cmdtxt))
        return True

    @command(name="ATSPD")
    def atspdcmd(self, acidx: AcId, targspd: SpeedMpsOrMach, cmdtxt: Text) -> bool:
        """Schedule a command for when an aircraft crosses a speed."""
        # TODO(abraham): #40 should make ATSPD explicitly CAS or Mach. Until
        # then the parser still returns the legacy threshold-encoded float.
        callsign = self.traffic.callsign[acidx]
        actual: q.CalibratedAirspeedMps[float] = float(self.traffic.cas[acidx])
        self.conditions.append(SpeedCondition(callsign, targspd, targspd - actual, cmdtxt))
        return True

    @command(name="ATDIST")
    def atdistcmd(
        self,
        acidx: AcId,
        position: LatLonDeg,
        targdist: q.DistanceNM[NonNegativeFiniteFloat],
        cmdtxt: Text,
    ) -> bool:
        """Schedule a command for crossing a distance given in nautical miles."""
        target: q.DistanceM[float] = q.nmi_to_m(targdist)
        _bearing, actual_distance = qdrdist(
            self.traffic.lat[acidx], self.traffic.lon[acidx], position.lat, position.lon
        )
        actual = float(actual_distance)
        self.conditions.append(
            DistanceCondition(
                self.traffic.callsign[acidx],
                target,
                target - actual,
                cmdtxt,
                position,
            )
        )
        return True

    def renameac(self, oldid: str, newid: str) -> None:
        """Retarget pending conditions after an aircraft callsign changes."""
        for condition in self.conditions:
            if condition.callsign == oldid:
                condition.callsign = newid
