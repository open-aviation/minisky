"""Physical quantity annotations and unit conversions used by MiniSky.

minisky uses SI units internally. To convert from imperial for example:

```pycon
>>> from minisky import quantities as q
>>> q.ft_to_m(1300)
396.24
```

Furthermore, we recommend annotating your types with quantity kind and unit
metadata, for example:

```pycon
>>> from minisky import quantities as q
>>> def _(alt: q.PressureAltitudeM): ...
>>> from dataclasses import dataclass
>>> @dataclass
... class Container:
...     alt: q.PressureAltitudeM
...
```

Use **unconstrained type variables** where possible. Do not unnecessarily
strictly type inputs like `np.ndarray | float`, they destroy type information.

For more information, see: <https://github.com/open-aviation/minisky/issues/38#issuecomment-5156669559>
"""

from __future__ import annotations

from fractions import Fraction
from typing import Annotated, TypeVar

import isqx
import isqx.usc
from isqx import aerospace

_T = TypeVar("_T")

#
# coords and angles
#

AngleDeg = Annotated[_T, isqx.ANGLE(isqx.DEG)]
AngleRad = Annotated[_T, isqx.ANGLE(isqx.RAD)]
LatitudeDeg = Annotated[_T, isqx.LATITUDE(isqx.DEG)]
LongitudeDeg = Annotated[_T, isqx.LONGITUDE(isqx.DEG)]
BearingDeg = Annotated[_T, isqx.ANGLE["bearing", "true"](isqx.DEG)]
TrueHeadingDegrees = Annotated[_T, aerospace.HEADING_TRUE(isqx.DEG)]
MagneticHeadingDegrees = Annotated[_T, aerospace.HEADING_MAG(isqx.DEG)]
GroundTrackDeg = Annotated[_T, aerospace.GROUND_TRACK(isqx.DEG)]
BankAngleDeg = Annotated[_T, aerospace.BANK_ANGLE(isqx.DEG)]
BankAngleRad = Annotated[_T, aerospace.BANK_ANGLE(isqx.RAD)]
TurnRateDegPerS = Annotated[_T, aerospace.TURN_RATE(isqx.DEG * isqx.S**-1)]
YawRateDegPerS = Annotated[_T, aerospace.YAW_RATE(isqx.DEG * isqx.S**-1)]
WindDirectionDeg = Annotated[_T, aerospace.HEADING_TRUE_WIND(isqx.DEG)]


#
# distance/length
#

DistanceM = Annotated[_T, isqx.DISTANCE(isqx.M)]
DistanceNM = Annotated[_T, isqx.DISTANCE(isqx.usc.NMI)]
VerticalDistanceM = Annotated[_T, isqx.DISTANCE["vertical"](isqx.M)]
VerticalDistanceFt = Annotated[_T, isqx.DISTANCE["vertical"](isqx.usc.FT)]
TurnRadiusM = Annotated[_T, aerospace.TURN_RADIUS(isqx.M)]
LengthM = Annotated[_T, isqx.LENGTH(isqx.M)]

#
# altitude/height
#

PressureAltitudeM = Annotated[_T, aerospace.PRESSURE_ALTITUDE(isqx.M)]
PressureAltitudeFt = Annotated[_T, aerospace.PRESSURE_ALTITUDE(isqx.usc.FT)]
MslAltitudeM = Annotated[_T, isqx.ALTITUDE["above_mean_sea_level"](isqx.M)]
BarometricHeightM = Annotated[_T, isqx.HEIGHT["barometric"](isqx.M)]
AglHeightM = Annotated[_T, aerospace.HEIGHT_ABOVE_GROUND_LEVEL(isqx.M)]

#
# speed/rate
#

SpeedMps = Annotated[_T, isqx.SPEED(isqx.M_PERS)]
AirspeedMps = Annotated[_T, aerospace.AIRSPEED(isqx.M_PERS)]
CalibratedAirspeedMps = Annotated[_T, aerospace.CALIBRATED_AIRSPEED(isqx.M_PERS)]
CalibratedAirspeedKt = Annotated[_T, aerospace.CALIBRATED_AIRSPEED(isqx.usc.KNOT)]
EquivalentAirspeedMps = Annotated[_T, aerospace.EQUIVALENT_AIRSPEED(isqx.M_PERS)]
TrueAirspeedMps = Annotated[_T, aerospace.TRUE_AIRSPEED(isqx.M_PERS)]
TrueAirspeedKt = Annotated[_T, aerospace.TRUE_AIRSPEED(isqx.usc.KNOT)]
GroundSpeedMps = Annotated[_T, aerospace.GROUND_SPEED(isqx.M_PERS)]
GroundSpeedKt = Annotated[_T, aerospace.GROUND_SPEED(isqx.usc.KNOT)]
WindSpeedMps = Annotated[_T, aerospace.WIND_SPEED(isqx.M_PERS)]
WindSpeedKt = Annotated[_T, aerospace.WIND_SPEED(isqx.usc.KNOT)]
VerticalRateMps = Annotated[_T, aerospace.VERTICAL_RATE(isqx.M * isqx.S**-1)]
VerticalRateFpm = Annotated[_T, aerospace.VERTICAL_RATE(isqx.usc.FT * isqx.MIN**-1)]
MachNumber = Annotated[_T, isqx.MACH_NUMBER]
VelocityMps = Annotated[_T, isqx.VELOCITY(isqx.M_PERS)]
AccelerationMps2 = Annotated[_T, isqx.ACCELERATION(isqx.M * isqx.S**-2)]
GravitationalAccelerationMps2 = Annotated[_T, isqx.ACCELERATION_OF_FREE_FALL(isqx.M_PERS2)]

#
# time
#

DurationS = Annotated[_T, isqx.DURATION(isqx.S)]
FrequencyHz = Annotated[_T, isqx.FREQUENCY(isqx.HZ)]
SimulationTimeS = Annotated[_T, isqx.TIME["simulation"](isqx.S)]
WallClockTimeS = Annotated[_T, isqx.TIME["wall_clock"](isqx.S)]

#
# atmosphere and performance
#

SpecificGasConstantJPerKgK = Annotated[
    _T, isqx.SPECIFIC_GAS_CONSTANT(isqx.J * isqx.KG**-1 * isqx.K**-1)
]
_TEMPERATURE_GRADIENT = isqx.QtyKind(isqx.K * isqx.M**-1, ("temperature_gradient",))
TemperatureGradientKPerM = Annotated[_T, _TEMPERATURE_GRADIENT(isqx.K * isqx.M**-1)]
_POSITION_DIFFUSION = isqx.QtyKind(isqx.M * isqx.S ** Fraction(-1, 2), ("position_diffusion",))
PositionDiffusionMPerSqrtS = Annotated[_T, _POSITION_DIFFUSION(isqx.M * isqx.S ** Fraction(-1, 2))]

StaticTemperatureK = Annotated[_T, aerospace.STATIC_TEMPERATURE(isqx.K)]
StaticPressurePa = Annotated[_T, isqx.STATIC_PRESSURE(isqx.PA)]
DensityKgPerM3 = Annotated[_T, isqx.DENSITY(isqx.KG * isqx.M**-3)]
SpeedOfSoundMps = Annotated[_T, isqx.SPEED_OF_SOUND(isqx.M_PERS)]
MassKg = Annotated[_T, isqx.MASS(isqx.KG)]
MtowKg = Annotated[_T, aerospace.MAXIMUM_TAKEOFF_WEIGHT(isqx.KG)]
OewKg = Annotated[_T, aerospace.OPERATING_EMPTY_WEIGHT(isqx.KG)]
ForceN = Annotated[_T, isqx.FORCE(isqx.N)]
PowerW = Annotated[_T, isqx.POWER(isqx.W)]
EnergyJ = Annotated[_T, isqx.ENERGY(isqx.J)]
EnergyWh = Annotated[_T, isqx.ENERGY(isqx.W * isqx.HOUR)]
MassFlowKgPerS = Annotated[_T, isqx.MASS_FLOW_RATE(isqx.KG * isqx.S**-1)]
AreaM2 = Annotated[_T, isqx.AREA(isqx.M**2)]
DragCoefficient = Annotated[_T, isqx.DRAG_COEFFICIENT]
ZeroLiftDragCoefficient = Annotated[_T, aerospace.ZERO_LIFT_DRAG_COEFFICIENT]
LiftInducedDragCoefficient = Annotated[_T, aerospace.LIFT_INDUCED_DRAG_COEFFICIENT]
InducedDragFactor = Annotated[_T, isqx.Dimensionless("induced_drag_factor")]
OswaldEfficiency = Annotated[_T, aerospace.OSWALD_EFFICIENCY]
BypassRatio = Annotated[_T, aerospace.BYPASS_RATIO]

#
# unit conversions
#
_ARCMINUTE = (isqx.DEG / 60).alias("arcminute")
_ARCSECOND = (_ARCMINUTE / 60).alias("arcsecond")

deg_to_arcmin = isqx.convert(isqx.DEG, _ARCMINUTE)
arcmin_to_arcsec = isqx.convert(_ARCMINUTE, _ARCSECOND)
ft_to_m = isqx.convert(isqx.usc.FT, isqx.M)
km_to_m = isqx.convert(isqx.KILO * isqx.M, isqx.M)
m_to_ft = isqx.convert(isqx.M, isqx.usc.FT)
nmi_to_m = isqx.convert(isqx.usc.NMI, isqx.M)
m_to_nmi = isqx.convert(isqx.M, isqx.usc.NMI)
kt_to_mps = isqx.convert(isqx.usc.KNOT, isqx.M_PERS)
kmh_to_mps = isqx.convert(isqx.KILO * isqx.M / isqx.HOUR, isqx.M_PERS)
mps_to_kt = isqx.convert(isqx.M_PERS, isqx.usc.KNOT)
fpm_to_mps = isqx.convert(isqx.usc.FT * isqx.MIN**-1, isqx.M * isqx.S**-1)
fpm_per_s_to_mps2 = isqx.convert(isqx.usc.FT * isqx.MIN**-1 * isqx.S**-1, isqx.M * isqx.S**-2)
mps_to_fpm = isqx.convert(isqx.M * isqx.S**-1, isqx.usc.FT * isqx.MIN**-1)
kw_to_w = isqx.convert(isqx.KILO * isqx.W, isqx.W)
wh_to_j = isqx.convert(isqx.W * isqx.HOUR, isqx.J)
j_to_wh = isqx.convert(isqx.J, isqx.W * isqx.HOUR)
min_to_s = isqx.convert(isqx.MIN, isqx.S)
s_to_min = isqx.convert(isqx.S, isqx.MIN)
hour_to_s = isqx.convert(isqx.HOUR, isqx.S)
n_to_kn = isqx.convert(isqx.N, isqx.KILO * isqx.N)
n_to_lbf = isqx.convert(isqx.N, isqx.usc.LBF)
lbf_to_n = isqx.convert(isqx.usc.LBF, isqx.N)
