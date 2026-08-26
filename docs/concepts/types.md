# Types, Quantities and Units

Aerospace uses both imperial and metric units, which are easy to mix up. In 1983, [Air Canada Flight 143 ("Gimli Glider")](https://en.wikipedia.org/wiki/Gimli_Glider) ran out of fuel midflight. It was later determined to be caused by a confusion between pounds and kilograms when refuelling. Unit confusion continues to be a problem, such as a Bluesky bug where the [time allocated to an RTA leg was miscalculated due to a confusion between nautical miles and meters](https://github.com/open-aviation/minisky/commit/b0fedb5). Minisky mitigates this risk by:

1. internally using SI units everywhere
2. providing developers with explicit conversion functions to convert between imperial and metric units (e.g. [`q.ft_to_m`][minisky.quantities.ft_to_m])
3. requiring users to *always* specify the unit when issuing a [command](./commands.md) (for example, `30000FT[MSL]` instead of `30000`)
4. requiring developers to *always* use newtype wrappers at method callback boundaries (for example, [`minisky.types.MslAltM`][] instead of a bare float)
5. encouraging developers to use unit/quantity type annotations in [`minisky.quantities`][]

Minisky also directly addresses several shortcomings in BlueSky. minisky removes an implicit CAS/Mach threshold hack needed to distinguish between the two[^casmach_threshold] and internally represents various forms of altitude (QNH, QNE, QFE) distinctly[^altitudes].

## Quantity Kinds

In addition to units, minisky also distinguishes between various **quantity kinds**.

Calibrated airspeed, true airspeed and ground airspeed can all be expressed in `m/s`, but are not interchangeable.

Likewise, standard pressure altitude, altitude above MSL, height above ground level all have the canonical SI unit of `m` but refer to different references/geoids.

Minisky provides developers with the [`minisky.quantities`][] and [`minisky.types`][] modules to help distinguish them.

### Guide for Users

Always specify the value, unit and quantity kind instead of just the value. For example, to express a height of 13000 feet [above mean sea level](https://en.wikipedia.org/wiki/Height_above_mean_sea_level):

```
13000FT[MSL]
```

The quantity kind (`[MSL]`) can sometimes be omitted depending on the command.

Here are some commonly used quantity kinds:

- Altitude
    - [`StdPressureAltM`][minisky.types.StdPressureAltM] is pressure altitude on the standard-pressure reference (QNE), e.g. `FL350`, or `35000FT[STD]`, or `10000M[STD]`
    - [`MslAltM`][minisky.types.MslAltM] is altitude above mean sea level (QNH), e.g. `13000FT[MSL]`, or `4000M[MSL]`
    - [`q.AglHeightM`][minisky.quantities.AglHeightM] is the height above the terrain. minisky does not properly support parsing from a string.
- Speed
    - [Calibrated airspeed (CAS)][minisky.types.CasMps] is the indicated airspeed (IAS) corrected for instrument and position errors, for example, `130KT[CAS]`, `67M/S[CAS]`.
    - [Mach][minisky.types.Mach] is the [true airspeed (TAS)][minisky.quantities.TrueAirspeedMps] divided by the local [speed of sound][minisky.quantities.SpeedOfSoundMps]. For example, `M.78`, `M0.78`.

See the [API reference](../api/types.md) for the full list.

### Guide for developers

For commands that take in a scalar value, minisky follows the [newtype idiom in Rust](https://doc.rust-lang.org/rust-by-example/generics/new_types.html) to prevent mixing up different quantity kinds.

For performance-critical internal logic, we strongly recommend using [`typing.Annotated`][] to embed [`minisky.quantities`][] metadata into the type instead of newtypes.

#### Commands

When defining your own [plugin command](../developer-guide/commands.md), simply annotate the arguments of your method:

```py
from minisky import command
from minisky.types import CasMps, Mach

@command
def set_speed(self, speed: CasMps | Mach):
    match speed:
        case CasMps(value):
            # handle cas...
        case Mach(value):
            # handle mach...
```

Here, the [`@command` decorator][minisky.command] internally extracts the annotation of the `speed` argument and understands how to parse both forms (e.g. `130KT[CAS]` or `M.78`).

Conceptually, [`CasMps`][minisky.types.CasMps] and [`Mach`][minisky.types.Mach] are just simple wrappers over a [`float`][]:

```py
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class CasMps:
    value: float


@dataclass(frozen=True, slots=True)
class Mach:
    value: float
```

so they can be [pattern matched](https://peps.python.org/pep-0636/) easily.

#### Internal functions

In many cases though, we do not recommend using newtypes defined in [`minisky.types`][] because they incur a runtime cost and create friction for downstream consumers[^isqx_annotated].

Instead, minisky follows the [FastAPI convention of embedding metadata into types](https://fastapi.tiangolo.com/python-types/). Use type aliases under [`minisky.quantities`][], for example:

```py
from dataclasses import dataclass
from minisky import quantities as q


# use in data structures:
@dataclass
class GasState:
    temperature: q.StaticTemperatureK
    pressure: q.StaticPressurePa
    density: q.DensityKgPerM3


# use in functions:
def mach(tas: q.TrueAirspeedMps, a: q.SpeedOfSoundMps):
    return tas / a
```

Conceptually, these type aliases are just:

```py
from typing import TypeAlias, TypeVar, Annotated
import isqx
from isqx import aerospace

_T = TypeVar("_T")
StaticTemperature: TypeAlias = Annotated[_T, aerospace.STATIC_TEMPERATURE(isqx.K)]
```

If you wish, you can further constrain the type, for example using `q.StaticTemperature[np.ndarray]`, however we recommend leaving them unconstrained since many Python libraries employ duck typing[^jax_duck_typing].

#### Unit conversions

Bluesky makes heavy use of manual conversion factors[^bluesky_manual_conversion], which are prone to mistakes. Instead, minisky provides conversion functions like [`minisky.quantities.ft_to_m`][]:

```pycon
>>> from minisky import quantities as q
>>> q.ft_to_m(1300)
396.24
```

To learn how to define your own units and conversions, visit the [`isqx` documentation](https://abc8747.github.io/isqx/examples/#unit-conversion).

[^casmach_threshold]: See: <https://github.com/open-aviation/minisky/issues/40>
[^altitudes]: See: <https://github.com/open-aviation/minisky/issues/22>
[^isqx_annotated]: See <https://abc8747.github.io/isqx/design/#problem-1-the-friction-of-newtypes> for a detailed explanation of why we discourage using newtypes for performance-critical code.
[^jax_duck_typing]: See <https://docs.jax.dev/en/latest/jep/12049-type-annotations.html#challenge-2-array-duck-typing> for why we don't recommend excessively annotating `np.ndarray` or `float` unless absolutely necessary.
[^bluesky_manual_conversion]: See <https://github.com/TUDelft-CNS-ATM/bluesky/blob/22fdf9e/bluesky/tools/aero.py#L15-L19>