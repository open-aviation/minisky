# Commands

minisky commands are the text interface used by the console, the REST `stack/` endpoint, and scenario (`.scn`) files. A scenario file is just a list of commands with simulation timestamps.

Suppose you would like to add a new `GREET` command that accepts a string parameter:

```text
00:00:10.00>GREET Alice
```

You can model this in Python with [`@command`][minisky.command]:

```python
from minisky import Ok, Result, command


@command
def greet(self, name: str) -> Result[str, str]:
    """Greet someone from the stack."""
    return Ok(f"hello {name}")
```

Here, the [`command`][minisky.command] decorator instructs minisky to *extract* the method's name, arguments, type annotations and docstrings to build an internal parser. It also constructs the necessary infrastructure for intellisense.

??? note "Comparison against Bluesky"

    To define a new command in Bluesky, you had to use a separate parser-spec DSL, such as `"txt,int"` or `"txt,[int]"`. This is then used to "teach" the command interpreter how to parse the command. The problems with this are 1) plugin authors have learn this small language, 2) it is easy to forget to keep the DSL in sync with the Python side, and 3) it is opaque to modern static analysers like Ruff. Minisky removes this completely and instead relies on the Python signature to understand your command.

Here are some tips for writing minisky commands:

### Put validation in the type

A common pattern is to put defensive checks inside the method:

```python
from minisky import Err, Result, command


@command(name="PASSENGERS")
def set_passenger_count(self, callsign: str, count: int) -> Result[str, str]:
    idx = self.traffic.idx(callsign)
    if idx is None:
        return Err(f"aircraft {callsign} not found")
    if count < 0 or count > 500:
        return Err("passenger count must be between 0 and 500")

    self.passengers[idx] = count
    return Ok(f"passenger count set to {count}")
```

This works, but is ugly when we have to duplicate it across multiple commands. We highly recommend encoding the invariants into the parameter types themselves:

```python
from typing import Annotated

from annotated_types import Ge, Gt, Le
from minisky import AcId, command


@command(name="PASSENGERS", aliases=("PAX",))
def set_passenger_count(self, idx: AcId, count: Annotated[int, Ge(0), Le(500)]) -> Result[str, str]:
    self.passengers[idx] = count
    return Ok(f"passenger count set to {count}")
```

Here, annotating the first parameter as [`AcId`][minisky.AcId] effectively asks minisky to help us convert the user input (say, `KL204`) into a valid integer index into the traffic arrays *before* the method is even called. This ensures that we always get a valid pointer to the aircraft indices inside the method body.

Similarly, encoding range constraints with [`annotated-types`](https://github.com/annotated-types/annotated-types) asks minisky to give us a value within the declared range.

If the user passes an invalid count or an aircraft that does not exist, the method will not be called.

Minisky follows the practice of [type-driven design](https://lexi-lambda.github.io/blog/2019/11/05/parse-don-t-validate/). It parses once at the boundary and allows the inner code rely on the result.

!!! tip
    If you want to target an aircraft *or* a traffic group, use [`AcIdSelection`][minisky.AcIdSelection], which resolves the input to an array of aircraft indices.

Minisky provides many useful built-in types and we recommend using them instead of manually interpreting strings inside your method. For example, [`StdPressureAltM`][minisky.types.StdPressureAltM] accepts user input such as `FL250`, `25000FT[STD]` or `7620M[STD]`, and internally transforms that into an altitude in meters.

```python
from minisky.types import StdPressureAltM


@command(name="TARGETALT")
def set_target_altitude(self, idx: AcId, altitude: StdPressureAltM) -> Result[str, str]:
    self.target_altitude[idx] = altitude.value
    return Ok("")
```

See the [types guide](types.md) for the built-in altitude, airspeed and heading types, and for the distinction between runtime values in `minisky.types` and physical quantity annotations in `minisky.quantities`.

### Optional values

To mark a trailing value as optional, use Python default `= None`:

```python
@command
def note(self, idx: AcId, text: str | None = None) -> Result[str, str]:
    # `NOTE KL204` gives text=None
    # `NOTE KL204 hello` gives text="hello"
```

For an explicitly empty positional field, use `T | None`:

```python
from minisky.types import CasMps, Mach, StdPressureAltM


def route(self, idx: AcId, altitude: StdPressureAltM | None, airspeed: CasMps | Mach) -> None:
    # `ROUTE KL204,,250KT[CAS]` gives altitude=None
```

## Add command overloads

In the case where you want one command to serve multiple purposes, say:

```sh
# set the aircraft mass
MASS aircraft,value
# get the aircraft mass
MASS aircraft
```

Define two methods with the same `name`:

```python
@command(name="MASS")
def set_mass(self, idx: AcId, value: Annotated[float, Gt(0)]) -> Result[str, str]: ...


@command(name="MASS")
def get_mass(self, idx: AcId) -> Result[float, str]: ...
```

## Advanced usage

For custom syntax, refer to [`CmdParser`][minisky.CmdParser]. In most cases you should not have to use it!

!!! warning

    Minisky deals with unions (`str | int`) and overloads, so parsers can run and fail. **Avoid side-effects for custom parsers!**
