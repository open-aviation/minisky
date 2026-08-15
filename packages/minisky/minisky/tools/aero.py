"""This module defines a set of standard aerodynamic functions and constants.

The aeronautics conversion and atmosphere library of MiniSky. It provides:

- International Standard Atmosphere constants and aerodynamic relations in SI units.
- The International Standard Atmosphere (ISA): pressure [Pa], density
  [kg/m3], temperature [K], and speed of sound [m/s] as a function of
  altitude [m].
- Conversions between calibrated airspeed (CAS), equivalent airspeed
  (EAS), true airspeed (TAS) - all in [m/s] - and Mach number [-] at a
  given altitude [m].

Functions prefixed with a "v" are vectorized and accept numpy arrays;
these use a simplified two-layer ISA (troposphere and lower stratosphere,
valid up to approximately 22 km). The scalar variants without prefix use the
full multi-layer ISA table.
"""

from typing import NamedTuple

import numpy as np

from minisky import quantities as q

# International standard atmpshere only up to 72000 ft / 22 km


#
# Constants Aeronautics
#
g0: q.GravitationalAccelerationMps2[float] = 9.80665  # m/s2    Sea level gravity constant
R: q.SpecificGasConstantJPerKgK[float] = 287.05287  # Used in wikipedia table: checked with 11000 m
p0: q.StaticPressurePa[float] = 101325.0  # Pa     Sea level pressure ISA
rho0: q.DensityKgPerM3[float] = 1.225  # kg/m3  Sea level density ISA
T0: q.StaticTemperatureK[float] = 288.15  # K   Sea level temperature ISA
Tstrat: q.StaticTemperatureK[float] = 216.65  # K Stratosphere temperature (until alt=22km)
gamma = 1.40  # cp/cv: adiabatic index for air
gamma1 = 0.2  # (gamma-1)/2 for air
gamma2 = 3.5  # gamma/(gamma-1) for air
beta: q.TemperatureGradientKPerM[float] = -0.0065  # [K/m] ISA temp gradient below tropopause
Rearth: q.LengthM[float] = 6371000.0  # m  Average earth radius
a0: q.SpeedOfSoundMps = np.sqrt(gamma * R * T0)  # sea level speed of sound ISA


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
class VectorAtmosphere(NamedTuple):
    pressure: q.StaticPressurePa
    density: q.DensityKgPerM3
    temperature: q.StaticTemperatureK


def vatmos(h: q.PressureAltitudeM) -> VectorAtmosphere:
    """Calculate atmospheric pressure, density, and temperature for a given altitude."""
    # Temp
    T = vtemp(h)

    # Density
    rhotrop = 1.225 * (T / 288.15) ** 4.256848030018761
    dhstrat = np.maximum(0.0, h - 11000.0)
    rho = rhotrop * np.exp(-dhstrat / 6341.552161)  # = *g0/(287.05*216.65))

    # Pressure
    p = rho * R * T

    return VectorAtmosphere(p, rho, T)


def vtemp(h: q.PressureAltitudeM) -> q.StaticTemperatureK:
    """Calculate atmospheric temperature for a given altitude."""
    T = np.maximum(288.15 - 0.0065 * h, Tstrat)
    return T


# Atmos wrappings:
def vpressure(h: q.PressureAltitudeM) -> q.StaticPressurePa:
    """Calculate atmospheric pressure for a given altitude."""
    p, _, _ = vatmos(h)
    return p


def vdensity(h: q.PressureAltitudeM) -> q.DensityKgPerM3:
    """Calculate atmospheric density for a given altitude."""
    _, r, _ = vatmos(h)
    return r


def vvsound(h: q.PressureAltitudeM) -> q.SpeedOfSoundMps:
    """Calculate the speed of sound for a given altitude."""
    T = vtemp(h)
    a = np.sqrt(gamma * R * T)
    return a


# ---------Speed conversions---h in [m]------------------
def vtas2mach(tas: q.TrueAirspeedMps, h: q.PressureAltitudeM) -> q.MachNumber:
    """True airspeed (tas) to mach number conversion for numpy arrays."""
    a = vvsound(h)
    mach = tas / a
    return mach


def vmach2tas(mach: q.MachNumber, h: q.PressureAltitudeM) -> q.TrueAirspeedMps:
    """Mach number to True airspeed (tas) conversion for numpy arrays."""
    a = vvsound(h)
    tas = mach * a
    return tas


def veas2tas(eas: q.EquivalentAirspeedMps, h: q.PressureAltitudeM) -> q.TrueAirspeedMps:
    """Equivalent airspeed to true airspeed conversion for numpy arrays."""
    rho = vdensity(h)
    tas = eas * np.sqrt(rho0 / rho)
    return tas


def vtas2eas(tas: q.TrueAirspeedMps, h: q.PressureAltitudeM) -> q.EquivalentAirspeedMps:
    """True airspeed to equivalent airspeed conversion for numpy arrays."""
    rho = vdensity(h)
    eas = tas * np.sqrt(rho / rho0)
    return eas


def vcas2tas(cas: q.CalibratedAirspeedMps, h: q.PressureAltitudeM) -> q.TrueAirspeedMps:
    """Calibrated to true airspeed conversion for numpy arrays."""
    p, rho, _ = vatmos(h)
    qdyn = p0 * ((1.0 + rho0 * cas * cas / (7.0 * p0)) ** 3.5 - 1.0)
    tas = np.sqrt(7.0 * p / rho * ((1.0 + qdyn / p) ** (2.0 / 7.0) - 1.0))

    # cope with negative speed
    tas = np.where(cas < 0, -1 * tas, tas)
    return tas


def vcasmach2tas(
    value: np.ndarray, is_mach: np.ndarray, h: q.PressureAltitudeM[np.ndarray]
) -> q.TrueAirspeedMps[np.ndarray]:
    """Convert mixed per-lane [`CAS` in m/s][minisky.values.CasMps] /
    [`Mach`][minisky.values.Mach] values to TAS."""
    tas = np.empty_like(value, dtype=float)
    tas[is_mach] = vmach2tas(value[is_mach], h[is_mach])
    is_cas = ~is_mach
    tas[is_cas] = vcas2tas(value[is_cas], h[is_cas])
    return tas


def vtas2cas(tas: q.TrueAirspeedMps, h: q.PressureAltitudeM) -> q.CalibratedAirspeedMps:
    """True to calibrated airspeed conversion for numpy arrays."""
    p, rho, _ = vatmos(h)
    qdyn = p * ((1.0 + rho * tas * tas / (7.0 * p)) ** 3.5 - 1.0)
    cas = np.sqrt(7.0 * p0 / rho0 * ((qdyn / p0 + 1.0) ** (2.0 / 7.0) - 1.0))

    # cope with negative speed
    cas = np.where(tas < 0, -1 * cas, cas)
    return cas


def vmach2cas(mach: q.MachNumber, h: q.PressureAltitudeM) -> q.CalibratedAirspeedMps:
    """Mach to calibrated airspeed conversion for numpy arrays."""
    tas = vmach2tas(mach, h)
    cas = vtas2cas(tas, h)
    return cas


def vcas2mach(cas: q.CalibratedAirspeedMps, h: q.PressureAltitudeM) -> q.MachNumber:
    """Calibrated airspeed to Mach conversion for numpy arrays."""
    tas = vcas2tas(cas, h)
    M = vtas2mach(tas, h)
    return M


def crossoveralt(
    cas: q.CalibratedAirspeedMps[float], mach: q.MachNumber[float]
) -> q.PressureAltitudeM[float]:
    """Calculate crossover altitude for given CAS and Mach number.

    Calculates the altitude where the given CAS and Mach values
    correspond to the same true airspeed.

    (BADA User Manual 3.12, p. 12)
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
class ScalarAtmosphere(NamedTuple):
    pressure: q.StaticPressurePa[float]
    density: q.DensityKgPerM3[float]
    temperature: q.StaticTemperatureK[float]


def atmos(h: q.PressureAltitudeM[float]) -> ScalarAtmosphere:
    """International Standard Atmosphere calculator (scalar version).

    Uses the full multi-layer ISA table up to the mesosphere, with base
    values corrected to avoid small discontinuities at the layer borders.
    Isothermal layers use an exponential pressure decay; gradient layers
    use the standard lapse-rate relation.

    Args:
        h: 0.0 < h < 84852.0 (clipped when outside range).
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
        p = p0[i] * np.exp(-g0 / (R * T) * (h - h0[i]))
        rho = p / (R * T)

    # Calculate for temperature gradient
    else:
        T = T0[i] + a[i] * (h - h0[i])
        p = p0[i] * ((T / T0[i]) ** (-g0 / (a[i] * R)))
        rho = p / (R * T)

    return ScalarAtmosphere(p, rho, T)


def temp(h: q.PressureAltitudeM[float]) -> q.StaticTemperatureK[float]:
    """Temperature-only version of the ISA atmosphere (scalar).

    Saves time relative to atmos() when only the temperature is needed.

    Args:
        h: 0.0 < h < 84852.0 (clipped when outside range).
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
def pressure(h: q.PressureAltitudeM[float]) -> q.StaticPressurePa[float]:
    """Calculate ISA atmospheric pressure for a given altitude (scalar)."""
    p, _r, _T = atmos(h)
    return p


def density(h: q.PressureAltitudeM[float]) -> q.DensityKgPerM3[float]:
    """Calculate ISA atmospheric density for a given altitude (scalar)."""
    _p, r, _T = atmos(h)
    return r


def vsound(h: q.PressureAltitudeM[float]) -> q.SpeedOfSoundMps[float]:
    """Calculate the ISA speed of sound for a given altitude (scalar).

    a = sqrt(gamma * R * T)
    """
    T = temp(h)
    a = np.sqrt(gamma * R * T)
    return a


# ---------Speed conversions---h in [m]------------------
def tas2mach(tas: q.TrueAirspeedMps[float], h: q.PressureAltitudeM[float]) -> q.MachNumber[float]:
    """True airspeed (tas) to mach number conversion (scalar)."""
    a = vsound(h)
    M = tas / a
    return M


def mach2tas(M: q.MachNumber[float], h: q.PressureAltitudeM[float]) -> q.TrueAirspeedMps[float]:
    """Mach number to true airspeed (tas) conversion (scalar)."""
    a = vsound(h)
    tas = M * a
    return tas


def eas2tas(
    eas: q.EquivalentAirspeedMps[float], h: q.PressureAltitudeM[float]
) -> q.TrueAirspeedMps[float]:
    """Equivalent airspeed to true airspeed conversion (scalar).

    tas = eas * sqrt(rho0 / rho(h))
    """
    rho = density(h)
    tas = eas * np.sqrt(rho0 / rho)
    return tas


def tas2eas(
    tas: q.TrueAirspeedMps[float], h: q.PressureAltitudeM[float]
) -> q.EquivalentAirspeedMps[float]:
    """True airspeed to equivalent airspeed conversion (scalar).

    eas = tas * sqrt(rho(h) / rho0)
    """
    rho = density(h)
    eas = tas * np.sqrt(rho / rho0)
    return eas


def cas2tas(
    cas: q.CalibratedAirspeedMps[float], h: q.PressureAltitudeM[float]
) -> q.TrueAirspeedMps[float]:
    """Calibrated airspeed to true airspeed conversion (scalar).

    Uses the compressible-flow relation: the impact pressure that would be
    measured at sea level for the given CAS is converted back to TAS using
    pressure and density at the given ISA altitude. Negative input speeds
    yield negative output speeds.
    """
    p, rho, _T = atmos(h)
    qdyn = p0 * ((1.0 + rho0 * cas * cas / (7.0 * p0)) ** 3.5 - 1.0)
    tas = np.sqrt(7.0 * p / rho * ((1.0 + qdyn / p) ** (2.0 / 7.0) - 1.0))
    tas = -1 * tas if cas < 0 else tas
    return tas


def tas2cas(
    tas: q.TrueAirspeedMps[float], h: q.PressureAltitudeM[float]
) -> q.CalibratedAirspeedMps[float]:
    """True airspeed to calibrated airspeed conversion (scalar).

    Inverse of cas2tas(), using the compressible-flow relation at the
    given ISA altitude. Negative input speeds yield negative output
    speeds.
    """
    p, rho, _T = atmos(h)
    qdyn = p * ((1.0 + rho * tas * tas / (7.0 * p)) ** 3.5 - 1.0)
    cas = np.sqrt(7.0 * p0 / rho0 * ((qdyn / p0 + 1.0) ** (2.0 / 7.0) - 1.0))
    cas = -1 * cas if tas < 0 else cas
    return cas


def mach2cas(
    M: q.MachNumber[float], h: q.PressureAltitudeM[float]
) -> q.CalibratedAirspeedMps[float]:
    """Mach number to calibrated airspeed conversion (scalar)."""
    tas = mach2tas(M, h)
    cas = tas2cas(tas, h)
    return cas


def cas2mach(
    cas: q.CalibratedAirspeedMps[float], h: q.PressureAltitudeM[float]
) -> q.MachNumber[float]:
    """Calibrated airspeed to Mach number conversion (scalar)."""
    tas = cas2tas(cas, h)
    M = tas2mach(tas, h)
    return M
