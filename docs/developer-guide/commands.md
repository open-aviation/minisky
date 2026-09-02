# Commands

!!! note "TLDR"

    Use the [`@command` decorator][minisky.command] to register a new minisky [stack command](../concepts/commands.md). You should feel right at home if you have experience in [creating a new route in `fastapi`](https://fastapi.tiangolo.com/python-types/), [creating a new command in `typer`](https://typer.tiangolo.com/tutorial/), or [using `discord.py`](https://discordpy.readthedocs.io/en/stable/ext/commands/commands.html)!

    minisky also extracts Python annotations for runtime validation, much like [`pydantic`](https://pydantic.dev/docs/validation/latest/concepts/types/). It natively understands [custom constraints](#custom-constraints) (via [`annotated-types`](https://github.com/annotated-types/annotated-types)), [literals](#literals), [optional values](#optional-values) (`T | None`), [sum types](#sum-types) (`A | B`), [product types](#product-types) (`NamedTuple(a, b)`), [variadics](#variadic-parameters) (`*args`), [overloads](#overloads) and [custom parsers](#custom-parsers). [Document arguments](#documentation) with [`annotated_doc.Doc`](https://github.com/fastapi/annotated-doc/).

    minisky defines a few [built-in types](#built-in-types), such as [`AcId`][minisky.AcId]/[`AcIdSelection`][minisky.AcIdSelection], as well as those in the [`minisky.types`][] module (e.g. [`CasMps`][minisky.types.CasMps], [`Mach`][minisky.types.Mach]).

## Basic Example

In your [plugin class](./plugins.md), add a method decorated with [`@command`][minisky.command]:

```python
from minisky import Ok, Result, command


class Example:
    def __init__(self) -> None:
        self.speed = 0

    @command
    def set_speed(self, speed: int) -> Result[str, str]:
        """Set the current speed."""
        self.speed = speed
        return Ok(f"speed set to {speed}")
```

Users can then run your command in the console:

```command title="minisky console"
> SET_SPEED 250
speed set to 250
> HELP SET_SPEED
Set the current speed.

SET_SPEED <speed>

Args:
    speed(int)
> SET_SPEED garbage
error: argument `speed`: expected a value, but got 'garbage'
 --> <command>:1:11
  |
1 | SET_SPEED garbage
  |           ^^^^^^^
```

The [`int`][] annotation instructs minisky to reject any input that cannot be converted into a valid integer.

??? question "How does it work internally?"

    The [`command` decorator][minisky.command] extracts the method name (`set_speed`), argument (`speed`), type annotation ([`int`][]) and the docstrings into an intermediate representation (IR). This IR is then used to create a parser, build the [command reference](../reference/commands.md) and the [`HELP` command][command.HELP].

    You can inspect the IR through the REST API:

    ```command
    $ minisky server
    $ curl -s localhost:8000/commands | jq '.[] | .commands.SET_SPEED.forms[]? | {parameters, doc}'
    {
      "parameters": [
        {
          "name": "speed",
          "variants": [
            {
                "input": {"kind": "field"},
                "values": [{"ref": "int"}]
            }
          ]
        }
      ],
      "doc": "Set the current speed."
    }
    ```

??? note "Comparison against Bluesky"

    Bluesky defines commands with a separate parser-spec DSL, such as `"txt,int"` or `"txt,[int]"`, which "teaches" the command interpreter how to parse the command. Minisky instead reads the Python signature directly, eliminating the need for plugin authors to learn and maintain a separate DSL.

You can also override the name of the command, provide aliases and command-level examples. When returning values, use [`Ok`][minisky.Ok] for a successful output and [`Err`][minisky.Err] for a command error[^result_types].

<!-- NOTE(abraham): we should really use structured error messages and Ok(value) where value implements __str__. not creating a dedicated page for now. -->

## Custom constraints

Suppose you want to also ensure that users do not pass a negative speed or values that are too large. A tempting approach is to validate the input inside the callback:

!!! warning "Do not do this"

    ```python
    from minisky import Err


    @command
    def set_speed(self, speed: int) -> Result[str, str]:
        if speed <= 0 or speed > 600:
            return Err("speed must be between 1 and 600")

        self.speed = speed
        return Ok(f"speed set to {speed}")
    ```

Instead, *encode* the constraints in the parameter type itself:

```python
from typing import Annotated, TypeAlias

from annotated_types import Gt, Le


MySpeed: TypeAlias = Annotated[int, Gt(0), Le(600)]


@command
def set_speed(self, speed: MySpeed) -> Result[str, str]:
    """Set the current speed."""
    self.speed = speed
    return Ok(f"speed set to {speed!r} ({type(speed).__name__})")
```

Here, [`Annotated`][typing.Annotated] attaches metadata objects ([`Gt`, `Le`](https://github.com/annotated-types/annotated-types#gt-ge-lt-le)) to the runtime type (`int`). These metadata objects are used by minisky to *validate* the user input before calling the method:

```command title="minisky console"
> SET_SPEED 250
speed set to 250 (int)
> SET_SPEED -10
error: argument `speed`: expected a value greater than 0, but got '-10'
 --> <command>:1:11
  |
1 | SET_SPEED -10
  |           ^^^
> SET_SPEED 601
error: argument `speed`: expected a value less than or equal to 600, but got '601'
 --> <command>:1:11
  |
1 | SET_SPEED 601
  |           ^^^
> HELP SET_SPEED
Set the current speed.

SET_SPEED <speed>

Args:
    speed(int[> 0, <= 600])
```

Notice that the [`HELP` command][command.HELP] here also displays the constraints nicely!

For custom constraints, use [`annotated_types.Predicate()`](https://github.com/annotated-types/annotated-types#predicate).

## Literals

For a fixed set of values, use [`Literal`][typing.Literal]:

```python
from typing import Literal


@command(name="MODE")
def set_mode(self, mode: Literal["AUTO", "MANUAL"]) -> Result[str, str]:
    return Ok(f"mode set to {mode!r}")
```

```command title="minisky console"
> MODE AUTO
mode set to 'AUTO'
> MODE CRUISE
error: argument `mode`: expected AUTO or MANUAL, but got 'CRUISE'
 --> <command>:1:6
  |
1 | MODE CRUISE
  |      ^^^^^^
> HELP MODE
MODE <mode>

Args:
    mode(str): AUTO, MANUAL
```

## Optional values

To mark an argument as optional, use [`T | None`][typing.Optional]:

```python
@command
def note(self, text: str | None = None) -> Result[str, str]:
    self.note = text
    return Ok(f"note set to {text!r} ({type(text).__name__})")
```

```command title="minisky console"
> NOTE hello
note set to 'hello' (str)
> NOTE
note set to None (NoneType)
> HELP NOTE
NOTE [<text>]

Args:
    text(str | None)
```

## Sum types

Minisky also understands [unions][typing.Union]:

```python
@command(name="RECORD")
def set_recording(self, target: bool | str) -> Result[str, str]:
    self.recording = target
    return Ok(f"recording target set to {target!r} ({type(target).__name__})")
```

Internally, minisky tries each branch from left-to-right. In this case, it first tries to cast the user input with [`bool`][], and if it fails, falls back to [`str`][]:

```command title="minisky console"
> RECORD OFF
recording target set to False (bool)
> RECORD flight.csv
recording target set to 'flight.csv' (str)
> HELP RECORD
RECORD <target>

Args:
    target(bool | str)
        One of:
            bool: True: TRUE, YES, Y, 1, ON; False: FALSE, NO, N, 0, OFF
            str
```

## Product types

In cases where you want to accept multiple fields, for example, repeated `(latitude, longitude)` pairs, use a [`NamedTuple`][typing.NamedTuple]:

```python
from typing import NamedTuple


class Window(NamedTuple):
    start: int
    end: int


@command(name="WINDOW")
def set_window(self, window: Window) -> Result[str, str]:
    self.window = window
    return Ok(f"window set to {window!r}")
```

```command title="minisky console"
> WINDOW 10,20
window set to Window(start=10, end=20)
> HELP WINDOW
WINDOW <window>

Args:
    window(Window): All of: start, end
```

At this time, only `NamedTuple` containers are supported. If you wish to support a custom class, see the section on [custom parsers](#custom-parsers).

## Variadic parameters

To accept a parameter zero or more times, use standard variadic parameters:

```python
@command(name="TAGS")
def set_tags(self, *tags: str) -> Result[str, str]:
    self.tags = tags
    return Ok(f"tags set to {tags!r}")
```

```command title="minisky console"
> TAGS HEAVY,PRIORITY
tags set to ('HEAVY', 'PRIORITY')
> HELP TAGS
TAGS [<tags>...]

Args:
    tags(str)
```

The same rule works for [product types](#product-types) too! For example, you can use `*windows: Window`.

## Overloads

In the case where you want one command to serve multiple purposes, you can simply use multiple [`@command` decorators][minisky.command] with the same `name=`:

```python
@command(name="MASS")
def set_mass(self, value: float) -> Result[str, str]:
    """Set the current mass."""
    self.mass = value
    return Ok(f"mass set to {value!r}")


@command(name="MASS")
def get_mass(self) -> Result[str, str]:
    """Show the current mass."""
    return Ok(f"mass is {self.mass!r}")
```

Minisky will handle the dispatching automatically.

```command title="minisky console"
> MASS 42000
mass set to 42000.0
> MASS
mass is 42000.0
> HELP MASS
1. Set the current mass.

   MASS <value>

   Args:
     value(float)

2. Show the current mass.

   MASS
```

## Custom parsers

We have now covered many ordinary Python types, which should be sufficient for expressing 90% of commands you need.

But for cases where you need extra control, minisky has [`Converter`][minisky.Converter]. It is analogous to Pydantic's [`BeforeValidator`](https://pydantic.dev/docs/validation/dev/concepts/validators/#field-after-validator), allowing you to provide a custom function that converts [`str`][] to your type:

```python
from typing import Annotated

from minisky import CommandField, Converter


def parse_percentage(value: str) -> float:
    if not value.endswith("%"):
        raise ValueError
    return float(value[:-1]) / 100


@command(name="FACTOR")
def set_factor(
    self,
    factor: Annotated[
        float,
        CommandField(examples=("80%",)),
        Converter(parse_percentage),
    ],
) -> Result[str, str]:
    self.factor = factor
    return Ok(f"factor set to {factor!r} ({type(factor).__name__})")
```

Here, we also use a [`CommandField`][minisky.CommandField] to add examples to the documentation.

```command title="minisky console"
> FACTOR 80%
factor set to 0.8 (float)
> HELP FACTOR
FACTOR <factor>

Args:
    factor(float) (e.g. 80%)
```

For even more fine-grained control, use [`CmdParser`][minisky.CmdParser].

!!! warning

    To support unions and command overloads, minisky often tries multiple branches, fail, and backtrack the internal cursor.

    Make sure your custom parsers are pure (free of side-effects) to avoid corruption. For example, you can *read* the [Traffic][minisky.Traffic] object to fetch an aircraft but you must not mutate it.

## Documentation

To document an argument, use [`annotated_doc.Doc`](https://github.com/fastapi/annotated-doc) metadata object inside [`Annotated`][typing.Annotated]:

```python
from typing import Annotated, TypeAlias

from annotated_doc import Doc

AirportIcao: TypeAlias = Annotated[str, Doc("Destination airport ICAO identifier.")]


@command(name="DIVERT")
def divert(self, airport: AirportIcao) -> Result[str, str]:
    return Ok(f"diverting to {airport!r}")
```

The advantage with this approach is the reusability of `AirportIcao` across multiple methods, without the need to duplicate docstrings everywhere. It also makes the documentation available in the [`HELP` command][command.HELP]:

```command title="minisky console"
> DIVERT EHAM
diverting to 'EHAM'
> HELP DIVERT
DIVERT <airport>

Args:
    airport(str): Destination airport ICAO identifier.
```

## Built-in types

For general usage, minisky provides many reusable aviation-specific command types.

### Aircraft Index

To refer to a particular aircraft callsign, use [`AcId`][minisky.AcId], which is the index to the internal [aircraft traffic arrays](../concepts/basics.md#state).

```python
from minisky import AcId


@command(name="AIRCRAFTINDEX")
def aircraft_index(self, idx: AcId) -> Result[str, str]:
    """Show the resolved aircraft index."""
    return Ok(f"aircraft index is {idx} ({type(idx).__name__})")
```

Internally, minisky checks that the aircraft callsign actually exists in the [traffic arrays][minisky.TrafficArrays].

```command title="minisky console"
> CRE KL204,A320,52,4,90,FL100,250KT[CAS]
Aircraft KL204 created
> AIRCRAFTINDEX KL204
aircraft index is 0 (int)
> AIRCRAFTINDEX NOSUCH
error: argument `idx`: expected an existing aircraft, but got 'NOSUCH'
 --> <command>:1:15
  |
1 | AIRCRAFTINDEX NOSUCH
  |               ^^^^^^
> HELP AIRCRAFTINDEX
Show the resolved aircraft index.

AIRCRAFTINDEX <idx>

Args:
    idx(int): An existing aircraft callsign.
```

If you need to accept a traffic group, or `*`/ALL, use [`AcIdSelection`][minisky.AcIdSelection] instead.

### Airspeed

To distinguish between various units and quantity kinds, minisky provides several useful **runtime newtypes** in the [minisky.types][] module. Background information can be found in the [types, quantities and units guide](../concepts/types.md)

To distinguish between [calibrated airspeed][minisky.types.CasMps] and [Mach][minisky.types.Mach] commands for example:

```python
from annotated_types import IsFinite

from minisky.types import CasMps, Ge0, Gt0, Mach


@command(name="TARGETSPD")
def set_target_speed(
    self,
    speed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]],
) -> Result[str, str]:
    match speed:
        case CasMps(value):
            self.target_speed = value
            return Ok(f"CAS set to {value!r} m/s")
        case Mach(value):
            self.target_speed = value
            return Ok(f"Mach set to {value!r}")
```

```command title="minisky console"
> TARGETSPD 250KT[CAS]
CAS set to 128.61111111111111 m/s
> TARGETSPD M0.78
Mach set to 0.78
> TARGETSPD M0.0
error: argument `speed`: expected a value greater than 0, but got '0.0'
 --> <command>:1:11
  |
1 | TARGETSPD M0.0
  |           ^^^^
> HELP TARGETSPD
TARGETSPD <speed>

Args:
    speed(CasMps[meter · second⁻¹, >= 0, finite] | Mach[> 0, finite])
        One of:
            CasMps: Calibrated airspeed normalized to metres per second. (e.g. 250KT[CAS], 128MPS[CAS])
            Mach: Mach number. (e.g. M0.78, M.78)
```

### Altitude

Likewise, to distinguish between [barometric pressure altitude on the standard pressure reference][minisky.types.StdPressureAltM] and the [altitude above mean sea level][minisky.types.MslAltM]:

```python
from annotated_types import IsFinite

from minisky.types import MslAltM, StdPressureAltM


@command(name="ALTITUDE")
def set_altitude(
    self,
    altitude: StdPressureAltM[IsFinite[float]] | MslAltM[IsFinite[float]],
) -> Result[str, str]:
    match altitude:
        case StdPressureAltM(value):
            self.altitude = value
            return Ok(f"pressure altitude set to {value!r} m")
        case MslAltM(value):
            self.altitude = value
            return Ok(f"MSL altitude set to {value!r} m")
```

```command title="minisky console"
> ALTITUDE FL100
pressure altitude set to 3048.0 m
> ALTITUDE 10000FT[MSL]
MSL altitude set to 3048.0 m
> ALTITUDE -100FT[MSL]
MSL altitude set to -30.48 m
> ALTITUDE inf
error: argument `altitude`: expected MSL altitude such as 10000FT[MSL] or 3048M[MSL], but got 'inf'
 --> <command>:1:10
  |
1 | ALTITUDE inf
  |          ^^^
> HELP ALTITUDE
ALTITUDE <altitude>

Args:
    altitude(StdPressureAltM[meter, finite] | MslAltM[meter, finite])
        One of:
            StdPressureAltM: Barometric pressure altitude on the standard-pressure reference. (e.g. FL100, 10000FT[STD], 3048M[STD])
            MslAltM: Altitude above mean sea level. (e.g. 10000FT[MSL], 3048M[MSL])
```

[^result_types]: This follows the same idea as [Rust's `Result`](https://jellis18.github.io/post/2021-12-13-python-exceptions-rust-go/) type.
