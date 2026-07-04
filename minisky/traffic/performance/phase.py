"""Flight-phase identification for the OpenAP performance model.

Infers the phase of flight of each aircraft from its speed, vertical rate,
and altitude. The phase identifiers defined here (NA, GD, IC, CL, CR, DE, AP)
are used by the performance model to select the applicable drag polar,
thrust model, speed limits, and bank angle. Fixed-wing phases are determined
with simple altitude/vertical-rate thresholds; rotorcraft are always
classified as NA (unknown).
"""

import numpy as np

from .coeff import *

NA = 0  # Unknown phase
GD = 1  # Ground
IC = 2  # Initial climb
CL = 3  # Climb
CR = 4  # Cruise
DE = 5  # Descent
AP = 6  # Approach


def readable_phase(ph: int) -> str:
    """Return the human-readable name of a flight-phase identifier.

    Args:
        ph (int): Phase identifier (0-6, i.e. NA, GD, IC, CL, CR, DE, AP).

    Returns:
        str: Phase name, e.g. "Cruise".
    """
    phases = {
        0: "Unknown phase",
        1: "Ground",
        2: "Initial climb",
        3: "Climb",
        4: "Cruise",
        5: "Descent",
        6: "Approach",
    }
    return phases[ph]


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
        lifttype (1D array): Lift type per aircraft, LIFT_FIXWING (1) or
            LIFT_ROTOR (2).
        spd (1D array): Aircraft speed(s); [m/s] for unit "SI", [kts] for "EP".
        roc (1D array): Vertical rate(s); [m/s] for "SI", [fpm] for "EP".
        alt (1D array): Altitude(s); [m] for "SI", [ft] for "EP".
        unit (str): Unit convention of the inputs, "SI" (default) or "EP".

    Returns:
        1D array: Phase identifier per aircraft (NA, GD, IC, CL, CR, DE, AP).
    """
    ph = np.zeros(len(spd))

    # phase for fixwings
    ph = np.where(lifttype == LIFT_FIXWING, get_fixwing(spd, roc, alt, unit), ph)

    # phase for rotors
    ph = np.where(lifttype == LIFT_ROTOR, get_rotor(spd, roc, alt, unit), ph)
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
        int or 1D array: phase indentifier (NA, GD, IC, CL, CR, DE, AP)

    Raises:
        RuntimeError: If ``unit`` is not "SI" or "EP".
    """

    if unit not in ["SI", "EP"]:
        raise RuntimeError("wrong unit type")

    if unit == "SI":
        spd = spd / 0.514444
        roc = roc / 0.00508
        alt = alt / 0.3048

    ph = np.zeros(len(spd), dtype=int)

    ph[(alt <= 75)] = GD
    ph[(alt >= 75) & (alt <= 1000) & (roc >= 150)] = IC
    ph[(alt >= 75) & (alt <= 1000) & (roc <= -150)] = AP
    ph[(alt >= 1000) & (roc >= 150)] = CL
    ph[(alt >= 1000) & (roc <= -150)] = DE
    ph[(alt >= 10000) & (roc <= 150) & (roc >= -150)] = CR

    return ph


def get_rotor(spd: np.ndarray, roc: np.ndarray, alt: np.ndarray, unit: str = "SI") -> np.ndarray:
    """Get the flight phase for rotorcraft (always NA).

    Rotorcraft phase identification is not implemented; all rotorcraft are
    classified as NA (unknown phase).

    Args:
        spd (float or 1D array): aircraft speed(s) (unused).
        roc (float or 1D array): aircraft vertical rate(s) (unused).
        alt (float or 1D array): aircraft altitude(s) (unused).
        unit (str): unit convention, "SI" or "EP" (unused).

    Returns:
        1D array: NA phase identifier for every aircraft.
    """
    ph = np.ones(len(spd)) * NA
    return ph
