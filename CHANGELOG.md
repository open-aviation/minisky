# Changelog

<!-- markdownlint-disable MD024 MD033 -->

## `minisky` v0.0.2

This first release of minisky focuses on *internal structural changes* and comes with lots of breaking changes. Highlights:

- To support multiple independent simulators in the same process, all simulator state is now explicitly owned by the `MiniSky` runtime.
- More generally, hidden global dependencies have been replaced with explicit passing and pure functions where possible, to make minisky easier to understand and debug in isolation.
- To make plugins easier to develop and distribute, the plugin system has been completely rebuilt with independent Python packages, entry points, and explicit lifecycles, following the approach used in [tangram](https://mode-s.org/tangram).
- Plugin authors can now use standard Python signatures to declaratively define stack commands in a single location. The `@command` decorator compiles each definition into a JSON-serialisable IR used by the parser, help menu, documentation, and future consumers such as syntax highlighting.
- To support mixed airspace design, different aircraft types need different quantity kinds: for example, CAS/Mach for conventional fixed-wing aircraft and TAS/GS for AAM. minisky now requires quantity kinds/units to be specified at command boundaries to reduce confusion.
- Navigation data (airports, waypoints etc.) are no longer included in the core. To use X-Plane 11 data, install the `minisky-xplane-navdata` package (licensed under the GPL) alongside the core. Users can also provide their own proprietary data if they wish.

Note: this version of minisky is derived from [bluesky `849d76f` (2024-06-12)](https://github.com/TUDelft-CNS-ATM/bluesky/commit/849d76fd44880f8d17a69aefa0bd37208f2b2fbb). Changes in upstream will be synced here in future releases.

### Migration Guide

#### Runtime and Python API

Minisky no longer uses process-global singletons (see #28, 63efb08 to d768c09). Create a self-contained `minisky.MiniSky` runtime. Eventually, minisky will move towards a side-effect-free, pure architecture. See the [runtime and state guide](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/concepts/basics.md#L9-L29), PR #28, 63efb08 to d768c09.

<details markdown>
<summary>bluesky to minisky migration</summary>

This list is not exhaustive.

| BlueSky | MiniSky |
| --- | --- |
| `bs.traf` | `runtime.traffic` |
| `bs.sim` | `runtime.simulation` |
| `bs.scr` | `runtime.console` |
| `bs.stack` | `runtime.commands` |
| `bs.stack.stack(...)` | `runtime.commands.stack(...)` |
| `bs.settings` | `runtime.config` |
| `bluesky.core.plugin` | `runtime.plugins` |
| `bluesky.core.varexplorer` | `runtime.variables` |
| `bluesky.tools.areafilter` | `runtime.shapes` |
| `bluesky.core.select_implementation(...)` | `runtime.replaceables.select(...)` |
| `bs.navdb` | split across `runtime.waypoints`, `runtime.airports`, `runtime.airways`, `runtime.firs`, `runtime.countries`, and `runtime.runway_thresholds` |
| `bs.net` | removed |
| `bs.server` | removed |
| `bs.{INIT,HOLD,OP,END}` | `SimulationState.{INIT,HOLD,OP,END}` |
| `bs.navdb.wpid` | `runtime.waypoints.identifiers` |
| `bs.navdb.wplat` | `runtime.waypoints.latitudes` |
| `bs.navdb.wplon` | `runtime.waypoints.longitudes` |
| `bs.navdb.wptype` | `runtime.waypoints.categories` |
| `bs.navdb.wpelev` | `runtime.waypoints.elevations` |
| `bs.navdb.wpvar` | `runtime.waypoints.magnetic_variations` |
| `bs.navdb.wpfreq` | `runtime.waypoints.frequencies` |
| `bs.navdb.wpdesc` | `runtime.waypoints.descriptions` |
| `bs.navdb.aptid` | `runtime.airports.identifiers` |
| `bs.navdb.aptname` | `runtime.airports.names` |
| `bs.navdb.aptlat` | `runtime.airports.latitudes` |
| `bs.navdb.aptlon` | `runtime.airports.longitudes` |
| `bs.navdb.aptmaxrwy` | `runtime.airports.max_runway_lengths` |
| `bs.navdb.aptype` | `runtime.airports.sizes` |
| `bs.navdb.aptco` | `runtime.airports.countries` |
| `bs.navdb.aptelev` | `runtime.airports.elevations` |
| `bs.navdb.awid` | `runtime.airways.identifiers` |
| `bs.navdb.awfromwpid` | `runtime.airways.from_waypoints` |
| `bs.navdb.awfromlat` | `runtime.airways.from_latitudes` |
| `bs.navdb.awfromlon` | `runtime.airways.from_longitudes` |
| `bs.navdb.awtowpid` | `runtime.airways.to_waypoints` |
| `bs.navdb.awtolat` | `runtime.airways.to_latitudes` |
| `bs.navdb.awtolon` | `runtime.airways.to_longitudes` |
| `bs.navdb.awndir` | `runtime.airways.directions` |
| `bs.navdb.awlowfl` | `runtime.airways.lower_altitudes` |
| `bs.navdb.awupfl` | `runtime.airways.upper_altitudes` |
| `bs.navdb.fir` | `runtime.firs.boundaries` |
| `bs.navdb.firlat0` | `runtime.firs.segment_start_latitudes` |
| `bs.navdb.firlon0` | `runtime.firs.segment_start_longitudes` |
| `bs.navdb.firlat1` | `runtime.firs.segment_end_latitudes` |
| `bs.navdb.firlon1` | `runtime.firs.segment_end_longitudes` |
| `bs.navdb.coname` | `runtime.countries.names` |
| `bs.navdb.cocode2` | `runtime.countries.codes2` |
| `bs.navdb.cocode3` | `runtime.countries.codes3` |
| `bs.navdb.conr` | `runtime.countries.numbers` |
| `bs.navdb.rwythresholds` | `runtime.runway_thresholds` |

</details>

```py
# bluesky
import bluesky as bs

print(bs.traf.id)

# minisky synchronous (basic usage)
from minisky import MiniSky

with MiniSky() as runtime:
    print(runtime.traffic.callsign)

# minisky asynchronous (suitable when plugins use async build/teardown)
async with MiniSky() as runtime:
    await runtime.run()
```

To encourage determinism, the Python and numpy RNG controlled by `SEED` stack command are now scoped to the runtime. (PR #28, d4aceee)

If you are a plugin developer, most public objects are re-exported and available through the top-level `minisky` package. More utils are also available under `minisky.aero`, `minisky.geo`, `minisky.types` and `minisky.quantities`. For example:

```py
from minisky import MiniSky, Ok, Plugin, Result, SimulationState, command
```

See PR #48, d472123, f79e2f8 for more information.

#### Scenarios and stack commands

Interactive commands now require units and quantity kinds to be explicitly specified. See [Commands](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/concepts/commands.md#L1-L41) and [Types, Quantities and Units](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/concepts/types.md#L1-L43) for the rationale. See also: issue #38, issue #40, PR #47, PR #50, 2152928 to 1871e8b

```text
# bluesky
CRE KL204 B744 52 4 90 FL300 250

# minisky
CRE KL204 B744 52 4 90 FL300 250KT[CAS]
```

This supersedes the implicit "CAS-or-Mach thresholding" approach. For the Python API, use `minisky.quantities` and `minisky.types`.

In an effort to remove implicit behaviour, minisky now requires the aircraft target to be specified inside the `AT ... DO` grammar (PR #44):

```text
# bluesky
KL204 AT LOPIK DO HDG 270

# minisky
KL204 AT LOPIK DO KL204 HDG 270
```

Define new commands by decorating a typed Python function with `@command`. See the [command developer guide](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/developer-guide/commands.md#L1-L81), issue #37, PR #44, 83cd4ba to 6da7fe7. For example:

```py
from typing import Annotated
from annotated_types import Ge, Le
from minisky import AcId, command, Result

@command(name="PASSENGERS")
def set_passenger_count(
    self,
    idx: AcId,
    count: Annotated[int, Ge(0), Le(500)],
) -> Result[str, str]:
    ...
```

Here, command callbacks should use the Rust-like `Result` type, a Union of `Ok` and `Err`. Migrate the Go-like `(ok, value)` tuples to `Result`. See: PR #39, PR #44, e22da85, b31e81f, 438612e. Consumers can then easily pattern match with:

```py
match result:
    case Ok(value):
        ...
    case Err(error):
        ...
```

#### Configuration

`settings.cfg` is replaced with TOML. On minisky startup, it reads the TOML configuration and validates them with Pydantic via `MiniSkyConfig`. See the [configuration guide](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/user-guide/configuration.md#L1-L66). (PR #28, PR #36, e5bafa5, 8fa50fe, 22ef260)

#### Plugins

A MiniSky plugin is now an independently installable Python package. Create a new standard package (e.g. `uv init --lib`), replace the `init_plugin()` with a `Plugin` export and register that export through the `minisky.plugins` entry-point group. See [Writing plugins](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/developer-guide/plugins.md#L1-L85), issue #24, PR #36. For example:

```py
from minisky import Plugin, PluginContext, PluginSpec


def build(context: PluginContext) -> PluginSpec:
    context.mount(MyPlugin())
    return context.finish()


plugin = Plugin(build=build)
```

```toml
[project.entry-points."minisky.plugins"]
example = "minisky_example:plugin"
```

In the `build` function, you can add commands, hooks, custom per-aircraft state, replacements to internal implementations and manage asynchronous resources. See the [plugin developer guide](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/developer-guide/plugins.md#L89-L230), PR #36, 768c0d7, 92e7ebf, 19686b8, 9fa800e.

To accept plugin-specific configuration, set `Plugin.config_class` to a Pydantic model. This instructs minisky to parse/validate the table under `[plugins.<id>]` before calling `build()`. See [Using plugins](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/user-guide/plugins.md#L1-L60). (PR #36, 8fa50fe, 22ef260)

```toml
[plugins.example]
message = "hello"
```

The CLI and server load configured plugins automatically. When constructing `MiniSky()`, call `await runtime.plugins.load_configured()` to load any `[plugins.<id>]` entries. (PR #36, 19686b8, 22ef260)

#### Navigation data

Previously, bluesky depended on `bluesky-simdata`. Minisky core now starts with empty navigation data and requires navigation data to be installed separately, for example:

```sh
uv add minisky minisky-xplane-navdata
```

You can also use your own navigation data by constructing `NavData` and passing it into `MiniSky`. See the [navigation-data guide](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/user-guide/navdata.md#L1-L40), PR #53, bb14710, b8e8fcf.

Navigation columns are numpy arrays. Frequencies use Hz, and airway altitude bounds use pressure-altitude metres. (PR #53, 6e5d4a1, e62ed82, ac8b260, 375ddd4)

#### Geography and shapes

`geo.qdrdist*`, `geo.kwikdist*`, and `geo.kwikqdrdist*` now return metres. `geo.qdrpos` and `geo.kwikpos` now accept metres. See: issue #38, PR #47, 66a8c93.

`AreaFilter` / `areafilter` is now `Shapes`. Areas and graphical lines are now stored separately. `checkInside` is now `contains`. See: issue #32, PR #42, 6bce3bd to 1bff065.

#### Optional values and enums

Sentinel values such as `-999`, `-1`, `999999`, or large finite placeholders should be replaced with the corresponding `None`, `VariantArray`, or `OptionalArray`. (issue #40, PR #46, PR #50, 800b226 to b078247, dc76114)

Important migrations (non-exhaustive):

- Route waypoint altitude, speed, and RTA constraints use `None` when absent.
- `Route.iactwp` uses `None` when no waypoint is active.
- Route and navigation lookup helpers return `None` on failure.
- `Traffic.idx()` returns `None` for an unknown callsign; iterable lookups contain `None` for missing aircraft.
- `getwpindices()` returns an empty list when there are no matches.
- Navigation references and unbounded area altitude sides use explicit optional values.
- `getnextturnwp()` returns `NextTurn | None`.

Numeric state constants for waypoint/condition types, flight phase, lift type, wind-field kind, ASAS priority, and position-resolution state are now enums. (PR #46, 4ff4fbe to b078247)

### Added

- An experimental REST API and WebSocket stream for simulator state. (PR #10, 88935ca, 7d853c3, 2627f53 to 85fa646)
- A headless `minisky` CLI based on `typer`, capable of running scenarios and attaching an interactive console to a server. See the [server, console, and streaming guide](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/user-guide/server.md#L1-L58). (1ecc000, PR #39, 9542c43)

  ```sh
  minisky run --scenario example.scn
  minisky server
  minisky console
  minisky stream
  ```

- Stack commands may now be executed asynchronously. (PR #36, ddb6823)

### Changed

- Changes to runtime ownership, configuration, plugins, commands, physical quantities, navigation data, geography, and optional values are covered in the migration guide above.
- The default simulation timestep is now 1 second. Separate performance and ASAS update intervals are removed. (4ab00ae, PR #28, ff6ea7b)
- `IC` now requires an absolute scenario path. (4ab00ae, PR #44, 44f78ac)
- `MCRE` now takes explicit geographic bounds instead of the radar viewport, defaults to `A320`, and generates three-digit callsign suffixes. (4ab00ae, 17b0de8, PR #47, b0b76b7)
- Conflict detection now uses `scipy.spatial.cKDTree` for candidate selection. (issue #15, PR #16, 17c26b2)
- Waypoint passing and route switching are vectorized across aircraft. (PR #12, PR #13, a830b2c, 44d3099)
- Navigation storage and lookup now use numpy arrays. See the migration guide above. (PR #53, 6e5d4a1, e62ed82)
- OpenAP is now a direct dependency of minisky. Note that the performance API is being redesigned and OpenAP will be moved to its own plugin soon. ([bluesky `c7f84e2`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/c7f84e2bbd79270e52805efa35b2361a2407e18b), PR #2, ef074c7)
- Polygon containment now uses Shapely. Polygons crossing the antimeridian or winding around a pole are rejected. (issue #32, PR #42, 581a526, 1bff065)

### Removed

- Minisky is headless: Qt/OpenGL and pygame interfaces, graphical resources, mouse/radar interaction, and display commands (`PAN`, `ZOOM`, `FILTERALT`, `TRAILS`, `PLOT`, `SYMBOL`, and `SWRAD`) are removed. A modern (and optional) graphical interface is currently being developed in the highly experimental Tangram plugin. (4ab00ae)
- The distributed node/client architecture is removed, including `ADDNODES`, the distributed `BATCH` workflow, node discovery, and the networking code. (4ab00ae)
- The work-directory/resource-root mechanism is removed, including `--workdir` option, `CD` command and local resource/plugin lookup. See the configuration and plugin migration above. (4ab00ae)
- The `REALTIME` / `RT` variable-timestep catch-up mode is removed. Use `DTMULT` to control wall-clock pacing. (PR #28, e800163)
- NLR/NASA Traffic Manager (TMX) compatibility, including `FIXDT`, is removed. (4ab00ae)
- BADA 3.x and BlueSky's legacy aircraft-performance backends, including the `PERF` stack command and runtime `OPENAP`/`BADA`/`LEGACY`/`OFF` selectors, are temporarily removed. BADA3.x will be reintroduced in the future. (4ab00ae, PR #1, c5eefaa)
- BlueSky's default plugin collection, including the OpenSky/ADS-B feeds, geofence/geovector, metrics, traffic-generation, ECMWF/GFS wind, and EBY/SSD/SWARM ASAS plugins, is removed. A selection of them may be re-introduced as independently-installable plugins. (4ab00ae, PR #36)
- Application and workflow commands, including `BENCHMARK`, `CALC`, `DIST`, `DOC`, `DT`, `FF`, `SAVE`, `PCALL`, `SAVEIC`, `DUMPRTE`, `CRELOG`, `IMPORT`, are removed. (4ab00ae)
- `ADDWAYPOINTS` is removed. Use repeated `ADDWPT` commands instead. (4ab00ae)
- Directory-scanned plugins/`init_plugin()`, command parser-spec strings, and `CASMACHTHR` are removed. See the migration guide above. (PR #36, PR #44, PR #50, 768c0d7, 83cd4ba, d4aceee)
- The legacy compiled `cgeo` geography module and C++ state-based conflict detector are removed for simplicity. (issue #27, PR #35, 843cb3d, 2ab2990)

### Fixed

- `AT <waypoint> SPD ...` now updates the waypoint speed constraint instead of its altitude. (PR #7, f3d5a68)
- `ATSPD` now uses CAS consistently. (PR #7, f3d5a68)
- Deleting a custom navigation waypoint now removes its complete record. (PR #7, f3d5a68)
- `DELWPT` now removes all associated waypoint metadata. ([bluesky `928b66d`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/928b66d3563183df66c8a13080966091247f4bd1), f2ed001)
- Removing a wind point now removes the corresponding longitude entry. (PR #7, f3d5a68)
- Surveillance noise now generates a separate sample for each updated aircraft. (PR #7, f3d5a68)
- Multi-aircraft creation now initializes active-waypoint state for every aircraft. (PR #7, f3d5a68)
- Valid short scenario lines are no longer discarded. (PR #7, f3d5a68)
- Scenario commands are stably ordered by timestamp. ([bluesky `98915c7`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/98915c73a8edd2adb9094b72bcbf37700547304f), PR #44, 44f78ac)
- `SWTOD <aircraft>` now reports the Top-of-Descent switch instead of the Top-of-Climb switch. (PR #7, f3d5a68)
- Fly-turn guidance preserves the previous turn speed and handles a turn at route index 0. ([bluesky `ac45bd3`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/ac45bd330da183f5d09eaf746b6dc19020abf564), bc9a00c, PR #46, b6bdd26)
- Latitude/longitude text conversion now selects the correct hemisphere. ([bluesky `f7c326f`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/f7c326f79fe12260c2c090eaa1870009a2db3ab5), ea00cf9)
- Geographic matrix calculations use the midpoint latitude, and `latlondist_matrix()` returns metres. (PR #7, PR #9, f3d5a68, 530acaf)
- Fixed-speed RTA calculations now use consistent distance/speed units. ([bluesky `1a1acd5`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/1a1acd545d105e7a70975aee7b0b368269b2fcbd), PR #46, PR #47, b0fedb5, 66a8c93)
- OpenAP speed limits now use the aircraft's current altitude. ([bluesky `86a1528`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/86a15289aae6a905bf361e040d0ca82f59d76d9d), PR #52, c93774a)
- MVP vertical resolution now sends equal-vertical-rate conflict pairs in opposite vertical directions. ([bluesky PR #661](https://github.com/TUDelft-CNS-ATM/bluesky/pull/661), PR #57)

## `minisky-xplane-navdata` v0.0.1

### Added

Provides the X-Plane 11 navigation dataset used by MiniSky, including waypoints/navaids, airways, and runway thresholds. When the `minisky-xplane-navdata` package is installed, MiniSky will discover it automatically when constructing the default runtime. See the [navigation-data guide](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/user-guide/navdata.md#L1-L40). (PR #53, b8e8fcf)

## `minisky-multicopter` v0.0.1

### Added

Multicopter support is highly experimental. This package adds small electric multirotors, including hover/yaw/delivery behavior, battery/work state, and a sample delivery scenario. Note that the performance/energy model will change significantly. See the [multicopter guide](https://github.com/open-aviation/minisky/blob/b8e8fcf169ef238ac97d0601caff564ef8f6c0e9/docs/user-guide/multicopters.md#L1-L31). (PR #17, PR #45, ad0a349, 70ba725 to fa4f785)
