"""This module defines a set of standard aerodynamic functions and constants.

The aeronautics conversion and atmosphere library of MiniSky. It provides:

- Unit conversion constants (kts, ft, fpm, inch, sqft, nm, lbs) and ISA
  constants (g0, R, p0, rho0, T0, gamma, ...), all in SI units.
- The International Standard Atmosphere (ISA): pressure [Pa], density
  [kg/m3], temperature [K], and speed of sound [m/s] as a function of
  altitude [m].
- Conversions between calibrated airspeed (CAS), equivalent airspeed
  (EAS), true airspeed (TAS) - all in [m/s] - and Mach number [-] at a
  given altitude [m].

Functions prefixed with a "v" are vectorized and accept numpy arrays;
these use a simplified two-layer ISA (troposphere and lower stratosphere,
valid up to approximately 22 km). The scalar variants without prefix use
the full multi-layer ISA table. The casormach* functions interpret a
single speed input as either CAS or Mach number, depending on the
CAS/Mach threshold that can be set with the CASMACHTHR command.
"""

import numpy as np

# International standard atmpshere only up to 72000 ft / 22 km

#
# Constants Aeronautics
#
kts = 0.514444  # m/s  of 1 knot
ft = 0.3048  # m    of 1 foot
fpm = ft / 60.0  # feet per minute
inch = 0.0254  # m    of 1 inch
sqft = 0.09290304  # 1sqft
nm = 1852.0  # m    of 1 nautical mile
lbs = 0.453592  # kg   of 1 pound mass
g0 = 9.80665  # m/s2    Sea level gravity constant
R = 287.05287  # Used in wikipedia table: checked with 11000 m
p0 = 101325.0  # Pa     Sea level pressure ISA
rho0 = 1.225  # kg/m3  Sea level density ISA
T0 = 288.15  # K   Sea level temperature ISA
Tstrat = 216.65  # K Stratosphere temperature (until alt=22km)
gamma = 1.40  # cp/cv: adiabatic index for air
gamma1 = 0.2  # (gamma-1)/2 for air
gamma2 = 3.5  # gamma/(gamma-1) for air
beta = -0.0065  # [K/m] ISA temp gradient below tropopause
Rearth = 6371000.0  # m  Average earth radius
a0 = np.sqrt(gamma * R * T0)  # sea level speed of sound ISA
casmach_thr = 2  # Threshold below which speeds should
# be considered as Mach numbers in casormach* functions


def casmachthr(threshold: float | None = None) -> tuple[bool, str]:
    """CASMACHTHR threshold

    Set a threshold below which speeds should be considered as Mach numbers
    in CRE(ATE), ADDWPT, and SPD commands. Set to zero if speeds should
    never be considered as Mach number (e.g., when simulating drones).

    Argument:
    - threshold: CAS speed threshold [m/s]
    """
    if threshold is None:
        return (
            True,
            f"CASMACHTHR: The current CAS/Mach threshold is {casmach_thr} m/s ({casmach_thr / kts} kts",
        )

    globals()["casmach_thr"] = threshold
    return True, f"CASMACHTHR: Set CAS/Mach threshold to {threshold}"


#
# Functions for aeronautics in this module
#  - physical quantities always in SI units
#  - lat,lon,course and heading in degrees
#
#  International Standard Atmosphere up to 22 km
#
#   p,rho,T = vatmos(h)    # atmos as function of geopotential altitude h [m]
#   a = vvsound(h)         # speed of sound [m/s] as function of h[m]
#   p = vpressure(h)       # calls atmos but retruns only pressure [Pa]
#   T = vtemperature(h)    # calculates temperature [K] (saves time rel to atmos)
#   rho = vdensity(h)      # calls atmos but retruns only pressure [Pa]
#
#  Speed conversion at altitude h[m] in ISA:
#
# M   = vtas2mach(tas,h)  # true airspeed (tas) to mach number conversion
# tas = vmach2tas(M,h)    # true airspeed (tas) to mach number conversion
# tas = veas2tas(eas,h)   # equivalent airspeed to true airspeed, h in [m]
# eas = vtas2eas(tas,h)   # true airspeed to equivent airspeed, h in [m]
# tas = vcas2tas(cas,h)   # cas  to tas conversion both m/s, h in [m]
# cas = vtas2cas(tas,h)   # tas to cas conversion both m/s, h in [m]
# cas = vmach2cas(M,h)    # Mach to cas conversion cas in m/s, h in [m]
# M   = vcas2mach(cas,h)   # cas to mach copnversion cas in m/s, h in [m]

# Atmosphere up to 22 km (72178 ft)


# ------------------------------------------------------------------------------
# Vectorized aero functions
# ------------------------------------------------------------------------------
def vatmos(h: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate atmospheric pressure, density, and temperature for a given altitude.

    Arguments:
    - h: Altitude [m]

    Returns:
    - p: Pressure [Pa]
    - rho: Density [kg / m3]
    - T: Temperature [K]
    """
    # Temp
    T = vtemp(h)

    # Density
    rhotrop = 1.225 * (T / 288.15) ** 4.256848030018761
    dhstrat = np.maximum(0.0, h - 11000.0)
    rho = rhotrop * np.exp(-dhstrat / 6341.552161)  # = *g0/(287.05*216.65))

    # Pressure
    p = rho * R * T

    return p, rho, T


def vtemp(h: np.ndarray) -> np.ndarray:
    """Calculate atmospheric temperature for a given altitude.

    Arguments:
    - h: Altitude [m]

    Returns:
    - T: Temperature [K]
    """
    T = np.maximum(288.15 - 0.0065 * h, Tstrat)
    return T


# Atmos wrappings:
def vpressure(h: np.ndarray) -> np.ndarray:
    """Calculate atmospheric pressure for a given altitude.

    Arguments:
    - h: Altitude [m]

    Returns:
    - p: Pressure [Pa]
    """
    p, _, _ = vatmos(h)
    return p


def vdensity(h: np.ndarray) -> np.ndarray:
    """Calculate atmospheric density for a given altitude.

    Arguments:
    - h: Altitude [m]

    Returns:
    - rho: Density [kg / m3]
    """
    _, r, _ = vatmos(h)
    return r


def vvsound(h: np.ndarray) -> np.ndarray:
    """Calculate the speed of sound for a given altitude.

    Arguments:
    - h: Altitude [m]

    Returns:
    - a: Speed of sound [m/s]
    """
    T = vtemp(h)
    a = np.sqrt(gamma * R * T)
    return a


# ---------Speed conversions---h in [m]------------------
def vtas2mach(tas: np.ndarray, h: np.ndarray) -> np.ndarray:
    """True airspeed (tas) to mach number conversion for numpy arrays.

    Arguments:
    - tas: True airspeed [m/s]
    - h: Altitude [m]

    Returns:
    - M: Mach number [-]
    """
    a = vvsound(h)
    mach = tas / a
    return mach


def vmach2tas(mach: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Mach number to True airspeed (tas) conversion for numpy arrays.

    Arguments:
    - mach: Mach number [-]
    - h: Altitude [m]

    Returns:
    - tas: True airspeed [m/s]
    """
    a = vvsound(h)
    tas = mach * a
    return tas


def veas2tas(eas: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Equivalent airspeed to true airspeed conversion for numpy arrays.

    Arguments:
    - eas: Equivalent airspeed [m/s]
    - h: Altitude [m]

    Returns:
    - tas: True airspeed [m/s]
    """
    rho = vdensity(h)
    tas = eas * np.sqrt(rho0 / rho)
    return tas


def vtas2eas(tas: np.ndarray, h: np.ndarray) -> np.ndarray:
    """True airspeed to equivalent airspeed conversion for numpy arrays.

    Arguments:
    - tas: True airspeed [m/s]
    - h: Altitude [m]

    Returns:
    - eas: Equivalent airspeed [m/s]
    """
    rho = vdensity(h)
    eas = tas * np.sqrt(rho / rho0)
    return eas


def vcas2tas(cas: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Calibrated to true airspeed conversion for numpy arrays.

    Arguments:
    - cas: Calibrated airspeed [m/s]
    - h: Altitude [m]

    Returns:
    - tas: True airspeed [m/s]
    """
    p, rho, _ = vatmos(h)
    qdyn = p0 * ((1.0 + rho0 * cas * cas / (7.0 * p0)) ** 3.5 - 1.0)
    tas = np.sqrt(7.0 * p / rho * ((1.0 + qdyn / p) ** (2.0 / 7.0) - 1.0))

    # cope with negative speed
    tas = np.where(cas < 0, -1 * tas, tas)
    return tas


def vtas2cas(tas: np.ndarray, h: np.ndarray) -> np.ndarray:
    """True to calibrated airspeed conversion for numpy arrays.

    Arguments:
    - tas: True airspeed [m/s]
    - h: Altitude [m]

    Returns:
    cas: Calibrated airspeed [m/s]
    """
    p, rho, _ = vatmos(h)
    qdyn = p * ((1.0 + rho * tas * tas / (7.0 * p)) ** 3.5 - 1.0)
    cas = np.sqrt(7.0 * p0 / rho0 * ((qdyn / p0 + 1.0) ** (2.0 / 7.0) - 1.0))

    # cope with negative speed
    cas = np.where(tas < 0, -1 * cas, cas)
    return cas


def vmach2cas(mach: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Mach to calibrated airspeed conversion for numpy arrays.

    Arguments:
    - mach: Mach number [-]
    - h: Altitude [m]

    Returns:
    - cas: Calibrated airspeed [m/s]
    """
    tas = vmach2tas(mach, h)
    cas = vtas2cas(tas, h)
    return cas


def vcas2mach(cas: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Calibrated airspeed to Mach conversion for numpy arrays.

    Arguments:
    - cas: Calibrated airspeed [m/s]
    - h: Altitude [m]

    Returns:
    - mach: Mach number [-]
    """
    tas = vcas2tas(cas, h)
    M = vtas2mach(tas, h)
    return M


def vcasormach(spd: np.ndarray, h: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpret input speed as either CAS or a Mach number, and return TAS, CAS, and Mach.

    Arguments:
    - spd: Airspeed. Interpreted as Mach number [-] when its value is below the
           CAS/Mach threshold. Otherwise interpreted as CAS [m/s].
    - h: Altitude [m]

    Returns:
    - tas: True airspeed [m/s]
    - cas: Calibrated airspeed [m/s]
    - mach: Mach number [-]
    """
    ismach = np.logical_and(spd > 0.1, spd < casmach_thr)
    tas = np.where(ismach, vmach2tas(spd, h), vcas2tas(spd, h))
    cas = np.where(ismach, vtas2cas(tas, h), spd)
    mach = np.where(ismach, spd, vtas2mach(tas, h))
    return tas, cas, mach


def vcasormach2tas(spd: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Interpret input speed as either CAS or a Mach number, and return TAS.

    Arguments:
    - spd: Airspeed. Interpreted as Mach number [-] when its value is below the
           CAS/Mach threshold. Otherwise interpreted as CAS [m/s].
    - h: Altitude [m]

    Returns:
    - tas: True airspeed [m/s]
    """
    ismach = np.logical_and(spd > 0.1, spd < casmach_thr)
    return np.where(ismach, vmach2tas(spd, h), vcas2tas(spd, h))


def crossoveralt(cas: float, mach: float) -> float:
    """Calculate crossover altitude for given CAS and Mach number.

    Calculates the altitude where the given CAS and Mach values
    correspond to the same true airspeed.

    (BADA User Manual 3.12, p. 12)

    Arguments:
    - cas: Calibrated airspeed [m/s]
    - mach: Mach number [-]

    Returns:
    - Altitude [m].
    """
    # Delta: pressure ratio at the transition altitude
    delta = ((1.0 + 0.5 * (gamma - 1.0) * (cas / a0) ** 2) ** (gamma / (gamma - 1.0)) - 1.0) / (
        (1.0 + 0.5 * (gamma - 1.0) * mach**2) ** (gamma / (gamma - 1.0)) - 1.0
    )
    # Theta: Temperature ratio at the transition altitude
    theta = delta ** (-beta * R / g0)
    return 1000.0 / 6.5 * T0 * (1.0 - theta)


# ------------------------------------------------------------------------------
# Scalar aero functions
# ------------------------------------------------------------------------------
def atmos(h: float) -> tuple[float, float, float]:
    """International Standard Atmosphere calculator (scalar version).

    Uses the full multi-layer ISA table up to the mesosphere, with base
    values corrected to avoid small discontinuities at the layer borders.
    Isothermal layers use an exponential pressure decay; gradient layers
    use the standard lapse-rate relation.

    Args:
        h: Altitude [m], 0.0 < h < 84852.0 (clipped when outside range,
            integer input allowed).

    Returns:
        tuple: (p, rho, T): pressure [Pa], density [kg/m3], and
        temperature [K].
    """

    # Constants

    # Base values and gradient in table from hand-out
    # (but corrected to avoid small discontinuities at borders of layers)
    h0 = [0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0, 86852.0]

    p0 = [
        101325.0,  # Sea level
        22631.7009099,  # 11 km
        5474.71768857,  # 20 km
        867.974468302,  # 32 km
        110.898214043,  # 47 km
        66.939,  # 51 km
        3.9564,
    ]  # 71 km

    T0 = [
        288.15,  # Sea level
        216.65,  # 11 km
        216.65,  # 20 km
        228.65,  # 32 km
        270.65,  # 47 km
        270.65,  # 51 km
        214.65,
    ]  # 71 km

    # a = lapse rate (temp gradient)
    # integer 0 indicates isothermic layer!
    a = [
        -0.0065,  # 0-11 km
        0,  # 11-20 km
        0.001,  # 20-32 km
        0.0028,  # 32-47 km
        0,  # 47-51 km
        -0.0028,  # 51-71 km
        -0.002,
    ]  # 71-   km

    # Clip altitude to maximum!
    h = max(0.0, min(float(h), h0[-1]))

    # Find correct layer
    i = 0
    while h > h0[i + 1] and i < len(h0) - 2:
        i = i + 1

    # Calculate if sothermic layer
    if a[i] == 0:
        T = T0[i]
        p = p0[i] * exp(-g0 / (R * T) * (h - h0[i]))
        rho = p / (R * T)

    # Calculate for temperature gradient
    else:
        T = T0[i] + a[i] * (h - h0[i])
        p = p0[i] * ((T / T0[i]) ** (-g0 / (a[i] * R)))
        rho = p / (R * T)

    return p, rho, T


def temp(h: float) -> float:
    """Temperature-only version of the ISA atmosphere (scalar).

    Saves time relative to atmos() when only the temperature is needed.

    Args:
        h: Altitude [m], 0.0 < h < 84852.0 (clipped when outside range,
            integer input allowed).

    Returns:
        Temperature [K].
    """

    # Base values and gradient in table from hand-out
    # (but corrected to avoid small discontinuities at borders of layers)
    h0 = [0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0, 86852.0]

    T0 = [
        288.15,  # Sea level
        216.65,  # 11 km
        216.65,  # 20 km
        228.65,  # 32 km
        270.65,  # 47 km
        270.65,  # 51 km
        214.65,
    ]  # 71 km

    # a = lapse rate (temp gradient)
    # integer 0 indicates isothermic layer!
    a = [
        -0.0065,  # 0-11 km
        0,  # 11-20 km
        0.001,  # 20-32 km
        0.0028,  # 32-47 km
        0,  # 47-51 km
        -0.0028,  # 51-71 km
        -0.002,
    ]  # 71-   km

    # Clip altitude to maximum!
    h = max(0.0, min(float(h), h0[-1]))

    # Find correct layer
    i = 0
    while h > h0[i + 1] and i < len(h0) - 2:
        i = i + 1

    # Isothermic layer has constant temperature, otherwise apply the gradient
    T = T0[i] if a[i] == 0 else T0[i] + a[i] * (h - h0[i])

    return T


# Atmos wrappings:
def pressure(h: float) -> float:  # h [m]
    """Calculate ISA atmospheric pressure for a given altitude (scalar).

    Args:
        h: Altitude [m].

    Returns:
        Pressure [Pa].
    """
    p, r, T = atmos(h)
    return p


def density(h: float) -> float:  # air density at given altitude h [m]
    """Calculate ISA atmospheric density for a given altitude (scalar).

    Args:
        h: Altitude [m].

    Returns:
        Density [kg/m3].
    """
    p, r, T = atmos(h)
    return r


def vsound(h: float) -> float:  # Speed of sound for given altitude h [m]
    """Calculate the ISA speed of sound for a given altitude (scalar).

    a = sqrt(gamma * R * T)

    Args:
        h: Altitude [m].

    Returns:
        Speed of sound [m/s].
    """
    T = temp(h)
    a = np.sqrt(gamma * R * T)
    return a


# ---------Speed conversions---h in [m]------------------
def tas2mach(tas: float, h: float) -> float:
    """True airspeed (tas) to mach number conversion (scalar).

    Args:
        tas: True airspeed [m/s].
        h: Altitude [m].

    Returns:
        Mach number [-].
    """
    a = vsound(h)
    M = tas / a
    return M


def mach2tas(M: float, h: float) -> float:
    """Mach number to true airspeed (tas) conversion (scalar).

    Args:
        M: Mach number [-].
        h: Altitude [m].

    Returns:
        True airspeed [m/s].
    """
    a = vsound(h)
    tas = M * a
    return tas


def eas2tas(eas: float, h: float) -> float:
    """Equivalent airspeed to true airspeed conversion (scalar).

    tas = eas * sqrt(rho0 / rho(h))

    Args:
        eas: Equivalent airspeed [m/s].
        h: Altitude [m].

    Returns:
        True airspeed [m/s].
    """
    rho = density(h)
    tas = eas * np.sqrt(rho0 / rho)
    return tas


def tas2eas(tas: float, h: float) -> float:
    """True airspeed to equivalent airspeed conversion (scalar).

    eas = tas * sqrt(rho(h) / rho0)

    Args:
        tas: True airspeed [m/s].
        h: Altitude [m].

    Returns:
        Equivalent airspeed [m/s].
    """
    rho = density(h)
    eas = tas * np.sqrt(rho / rho0)
    return eas


def cas2tas(cas: float, h: float) -> float:
    """Calibrated airspeed to true airspeed conversion (scalar).

    Uses the compressible-flow relation: the impact pressure that would be
    measured at sea level for the given CAS is converted back to TAS using
    pressure and density at the given ISA altitude. Negative input speeds
    yield negative output speeds.

    Args:
        cas: Calibrated airspeed [m/s].
        h: Altitude [m].

    Returns:
        True airspeed [m/s].
    """
    p, rho, T = atmos(h)
    qdyn = p0 * ((1.0 + rho0 * cas * cas / (7.0 * p0)) ** 3.5 - 1.0)
    tas = np.sqrt(7.0 * p / rho * ((1.0 + qdyn / p) ** (2.0 / 7.0) - 1.0))
    tas = -1 * tas if cas < 0 else tas
    return tas


def tas2cas(tas: float, h: float) -> float:
    """True airspeed to calibrated airspeed conversion (scalar).

    Inverse of cas2tas(), using the compressible-flow relation at the
    given ISA altitude. Negative input speeds yield negative output
    speeds.

    Args:
        tas: True airspeed [m/s].
        h: Altitude [m].

    Returns:
        Calibrated airspeed [m/s].
    """
    p, rho, T = atmos(h)
    qdyn = p * ((1.0 + rho * tas * tas / (7.0 * p)) ** 3.5 - 1.0)
    cas = np.sqrt(7.0 * p0 / rho0 * ((qdyn / p0 + 1.0) ** (2.0 / 7.0) - 1.0))
    cas = -1 * cas if tas < 0 else cas
    return cas


def mach2cas(M: float, h: float) -> float:
    """Mach number to calibrated airspeed conversion (scalar).

    Args:
        M: Mach number [-].
        h: Altitude [m].

    Returns:
        Calibrated airspeed [m/s].
    """
    tas = mach2tas(M, h)
    cas = tas2cas(tas, h)
    return cas


def cas2mach(cas: float, h: float) -> float:
    """Calibrated airspeed to Mach number conversion (scalar).

    Args:
        cas: Calibrated airspeed [m/s].
        h: Altitude [m].

    Returns:
        Mach number [-].
    """
    tas = cas2tas(cas, h)
    M = tas2mach(tas, h)
    return M


def casormach(spd: float, h: float) -> tuple[float, float, float]:
    """Interpret input speed as either CAS or a Mach number (scalar version).

    The speed is treated as a Mach number when 0.1 < spd < casmach_thr
    (settable with the CASMACHTHR command), and as CAS otherwise.

    Args:
        spd: Airspeed: Mach number [-] or calibrated airspeed [m/s].
        h: Altitude [m].

    Returns:
        tuple: (tas, cas, m): true airspeed [m/s], calibrated airspeed
        [m/s], and Mach number [-].
    """
    if 0.1 < spd < casmach_thr:
        # Interpret spd as Mach number
        tas = mach2tas(spd, h)
        cas = mach2cas(spd, h)
        m = spd
    else:
        # Interpret spd as CAS
        tas = cas2tas(spd, h)
        cas = spd
        m = cas2mach(spd, h)
    return tas, cas, m


def casormach2tas(spd: float, h: float) -> float:
    """Interpret input speed as either CAS or Mach, and return TAS (scalar version).

    Args:
        spd: Airspeed: Mach number [-] when 0.1 < spd < casmach_thr,
            otherwise calibrated airspeed [m/s].
        h: Altitude [m].

    Returns:
        True airspeed [m/s].
    """
    # Interpret spd as Mach number inside the threshold band, otherwise as CAS
    tas = mach2tas(spd, h) if 0.1 < spd < casmach_thr else cas2tas(spd, h)
    return tas


def metres_to_feet_rounded(metres: float) -> int:
    """
    Converts metres to feet.
    Returns feet as rounded integer.
    """
    return int(round(metres / ft))


def metric_spd_to_knots_rounded(speed: float) -> int:
    """
    Converts speed in m/s to knots.
    Returns knots as rounded integer.
    """
    return int(round(speed / kts))
