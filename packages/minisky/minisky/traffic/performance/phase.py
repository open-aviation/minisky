"""Flight-phase identification for the OpenAP performance model."""

from enum import IntEnum

import numpy as np

from minisky import quantities as q

from .coeff import LiftType


class FlightPhase(IntEnum):
    UNKNOWN = 0
    GROUND = 1
    INITIAL_CLIMB = 2
    CLIMB = 3
    CRUISE = 4
    DESCENT = 5
    APPROACH = 6


def readable_phase(phase: FlightPhase) -> str:
    """Return the human-readable name of a flight phase."""
    return {
        FlightPhase.UNKNOWN: "Unknown phase",
        FlightPhase.GROUND: "Ground",
        FlightPhase.INITIAL_CLIMB: "Initial climb",
        FlightPhase.CLIMB: "Climb",
        FlightPhase.CRUISE: "Cruise",
        FlightPhase.DESCENT: "Descent",
        FlightPhase.APPROACH: "Approach",
    }[phase]


_GROUND_MAX = q.ft_to_m(75.0)
_TERMINAL_MAX = q.ft_to_m(1000.0)
_CRUISE_MIN = q.ft_to_m(10000.0)
_PHASE_RATE = q.fpm_to_mps(150.0)


def get(
    lifttype: np.ndarray,
    roc: q.VerticalRateMps[np.ndarray],
    alt: q.PressureAltitudeM[np.ndarray],
) -> np.ndarray:
    """Return the flight phase for each aircraft from SI state arrays."""
    phases = np.full(len(lifttype), FlightPhase.UNKNOWN.value, dtype=int)
    fixed_wing = lifttype == LiftType.FIXED_WING
    phases[fixed_wing] = get_fixwing(roc[fixed_wing], alt[fixed_wing])
    return phases


def get_fixwing(
    roc: q.VerticalRateMps[np.ndarray], alt: q.PressureAltitudeM[np.ndarray]
) -> np.ndarray:
    """Classify fixed-wing flight phase from SI vertical rate and altitude."""
    # TODO(abraham): #22 should provide AGL height here. These thresholds are
    # near-ground phase thresholds, but MiniSky currently only has pressure
    # altitude, so high-elevation airports are classified against the wrong reference.
    phases = np.full(len(alt), FlightPhase.UNKNOWN.value, dtype=int)

    phases[alt <= _GROUND_MAX] = FlightPhase.GROUND.value
    terminal = (alt > _GROUND_MAX) & (alt <= _TERMINAL_MAX)
    phases[terminal & (roc >= _PHASE_RATE)] = FlightPhase.INITIAL_CLIMB.value
    phases[terminal & (roc <= -_PHASE_RATE)] = FlightPhase.APPROACH.value
    phases[(alt > _TERMINAL_MAX) & (roc >= _PHASE_RATE)] = FlightPhase.CLIMB.value
    phases[(alt > _TERMINAL_MAX) & (roc <= -_PHASE_RATE)] = FlightPhase.DESCENT.value
    phases[(alt >= _CRUISE_MIN) & (roc < _PHASE_RATE) & (roc > -_PHASE_RATE)] = (
        FlightPhase.CRUISE.value
    )
    return phases
