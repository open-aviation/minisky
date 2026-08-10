"""Flight-phase identification for the OpenAP performance model.

Infers the phase of flight of each aircraft from its speed, vertical rate,
and altitude. `FlightPhase` values are used by the performance model to select the applicable drag polar,
thrust model, speed limits, and bank angle. Fixed-wing phases are determined
with simple altitude/vertical-rate thresholds; rotorcraft are always
classified as `FlightPhase.UNKNOWN`.
"""

from enum import IntEnum

import numpy as np

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


def get(
    lifttype: np.ndarray,
    spd: np.ndarray,
    roc: np.ndarray,
    alt: np.ndarray,
    unit: str = "SI",
) -> np.ndarray:
    """Get the flight phase for all aircraft, dispatching on lift type.

    Fixed-wing aircraft are classified with :func:`get_fixwing`, rotorcraft
    with :func:`get_rotor`.

    Args:
        lifttype (1D array): Lift type per aircraft.
        spd (1D array): Aircraft speed(s); [m/s] for unit "SI", [kts] for "EP".
        roc (1D array): Vertical rate(s); [m/s] for "SI", [fpm] for "EP".
        alt (1D array): Altitude(s); [m] for "SI", [ft] for "EP".
        unit (str): Unit convention of the inputs, "SI" (default) or "EP".

    Returns:
        1D array: `FlightPhase` values per aircraft.
    """
    ph = np.full(len(spd), FlightPhase.UNKNOWN.value, dtype=int)

    # phase for fixwings
    ph = np.where(lifttype == LiftType.FIXED_WING, get_fixwing(spd, roc, alt, unit), ph)

    # phase for rotors
    ph = np.where(lifttype == LiftType.ROTORCRAFT, get_rotor(spd, roc, alt, unit), ph)
    return ph


def get_fixwing(spd: np.ndarray, roc: np.ndarray, alt: np.ndarray, unit: str = "SI") -> np.ndarray:
    """Get the phase of flight base on aircraft state data

    Classifies fixed-wing aircraft with altitude and vertical-rate
    thresholds (altitudes in ft, rates in fpm after unit conversion):
    ground below 75 ft; initial climb / approach between 75 and 1000 ft
    when climbing / descending faster than 150 fpm; climb / descent above
    1000 ft; cruise above 10000 ft when the vertical rate is within
    +/-150 fpm.

    Args:
        spd (float or 1D array): aircraft speed(s); [m/s] for unit "SI",
            [kts] for "EP".
        roc (float or 1D array): aircraft vertical rate(s); [m/s] for "SI",
            [fpm] for "EP".
        alt (float or 1D array): aricraft altitude(s); [m] for "SI",
            [ft] for "EP".
        unit (String):  unit, default 'SI', option 'EP'

    Returns:
        1D array: `FlightPhase` values.

    Raises:
        RuntimeError: If ``unit`` is not "SI" or "EP".
    """

    if unit not in ["SI", "EP"]:
        raise RuntimeError("wrong unit type")

    if unit == "SI":
        spd = spd / 0.514444
        roc = roc / 0.00508
        alt = alt / 0.3048

    ph = np.full(len(spd), FlightPhase.UNKNOWN.value, dtype=int)

    ph[alt <= 75] = FlightPhase.GROUND.value
    ph[(alt > 75) & (alt <= 1000) & (roc >= 150)] = FlightPhase.INITIAL_CLIMB.value
    ph[(alt > 75) & (alt <= 1000) & (roc <= -150)] = FlightPhase.APPROACH.value
    ph[(alt > 1000) & (roc >= 150)] = FlightPhase.CLIMB.value
    ph[(alt > 1000) & (roc <= -150)] = FlightPhase.DESCENT.value
    ph[(alt >= 10000) & (roc < 150) & (roc > -150)] = FlightPhase.CRUISE.value

    return ph


def get_rotor(spd: np.ndarray, roc: np.ndarray, alt: np.ndarray, unit: str = "SI") -> np.ndarray:
    """Get the flight phase for rotorcraft (always unknown).

    Rotorcraft phase identification is not implemented; all rotorcraft are
    classified as `FlightPhase.UNKNOWN`.

    Args:
        spd (float or 1D array): aircraft speed(s) (unused).
        roc (float or 1D array): aircraft vertical rate(s) (unused).
        alt (float or 1D array): aircraft altitude(s) (unused).
        unit (str): unit convention, "SI" or "EP" (unused).

    Returns:
        1D array: `FlightPhase.UNKNOWN` for every aircraft.
    """
    ph = np.full(len(spd), FlightPhase.UNKNOWN.value, dtype=int)
    return ph
