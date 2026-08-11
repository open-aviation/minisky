# Flying multicopters

The `MULTICOPTER` plugin (in `packages/minisky-multicopter/`) makes MiniSky fly
small electric multirotors — DJI MAVIC/M600-class camera drones and
Amazon/Matternet-style delivery drones — with multicopter behaviour that the
fixed-wing core cannot express:

- **Hover and yaw at zero speed.** A multicopter can stop (`SPD 0`), hold
  position, and rotate its nose in place at a yaw-rate limit instead of the
  bank-angle turn rate (which degenerates as speed approaches zero).
- **Velocity decoupled from heading.** The direction of travel follows the
  *track* commanded by LNAV or conflict resolution, while the nose points
  wherever `YAW` put it — the aircraft can strafe, and course changes at
  waypoints are thrust redirections with no turn radius.
- **Electric performance.** Thrust-based power draw, a battery state of
  charge, and an envelope that tightens when the battery runs low.

Fixed-wing aircraft in the same simulation are untouched, and helicopters are
deliberately out of scope (the `EC35` keeps its default envelope-only
behaviour).

## Loading the plugin

Add an (empty) plugin table to your [config file](configuration.md):

```toml
[plugins.multicopter]
```

or load it at runtime — from a scenario or the console with
`PLUGINS LOAD MULTICOPTER`, or from Python with
`await runtime.plugins.load("MULTICOPTER")`.

The plugin table accepts optional, validated settings (defaults shown):

```toml
[plugins.multicopter]
capture_radius = 10.0                       # waypoint capture radius [m]
performance_path = "~/my/multicopter.toml"  # performance table override
soc_low = 0.2               # state of charge below which the envelope shrinks
lowbatt_spd_factor = 0.6    # max-speed factor below that threshold
lowbatt_vs_factor = 0.5     # max-climb-rate factor below that threshold
gs_hover = 0.1              # "stopped" ground speed for hover holds [m/s]
alt_capture = 0.5           # altitude tolerance for hover holds [m]
cruise_speed_fraction = 0.8 # cruise fraction of v_max for the energy fallback
```

`performance_path` points at a performance TOML (see
[Adding a new multicopter type](#adding-a-new-multicopter-type)); when
omitted, a `multicopter.toml` in the platform cache directory is read if it
exists.

On the first simulation step after loading, the plugin selects its
implementations of five [replaceable traffic components](../architecture.md#replaceable-components)
(kinematics, pilot logic, autopilot, waypoint capture, and performance), and
re-selects them after every `RESET`. A manual `SELECTIMPL` afterwards is
respected until the next reset.

## Which aircraft count as multicopters

Membership follows the aircraft type: creating any typecode listed in the
performance table produces a multicopter. The built-in table covers

```
MAVIC  PHAN4  M100  M200  M600  MNET  AMZN  HORSEFLY
```

and a user performance TOML extends it (see
[Adding a new multicopter type](#adding-a-new-multicopter-type)).
Everything else — including the `EC35` helicopter —
keeps stock behaviour. Query or override per aircraft with `MCOPT`:

```
MCOPT DRONE1          -> MCOPT DRONE1: ON
MCOPT DRONE1 OFF      -> back to fixed-wing kinematics
```

## Commands

| Command | What it does |
| --- | --- |
| `MCOPT acid [ON/OFF]` | Query or set whether an aircraft is flown as a multicopter |
| `YAW acid hdg` | Point the nose at a body heading; the velocity vector is unaffected |
| `YAWRATE acid [rate]` | Query or set the maximum yaw rate (default 90 deg/s) |
| `HOVER acid [time] [alt]` | Hold position — optionally for `time` seconds, optionally moving vertically to `alt` |
| `BATT acid` | Report battery state of charge, power draw, and endurance |

By default the nose follows the direction of travel, like any aircraft. The
first `YAW` decouples them: the nose stays where you put it while LNAV,
conflict resolution, and waypoint corners steer the velocity vector
underneath. For a multicopter, `HDG` is an alias of `YAW` — there is no
command that couples heading back to track short of `MCOPT acid OFF`.

`HOVER` is composable rather than a scripted manoeuvre:

- `HOVER DRONE1` brakes to a stop and holds position until `LNAV DRONE1 ON`
  resumes the route.
- `HOVER DRONE1 30` holds for 30 seconds — counted while actually stopped at
  the selected altitude — then restores the saved LNAV/VNAV/speed state.
- `HOVER DRONE1 30 200` also moves vertically to 200 ft while holding
  position; the route resumes at the hover altitude.
- Re-issuing `HOVER` while hovering updates the hold time and altitude, and a
  plain `ALT` changes the hover altitude too.

A delivery profile is therefore just scenario vocabulary: fly a route, `HOVER`
over the drop point at a descent altitude, climb back with `ALT`, `LNAV ON`
to fly home. See `packages/minisky-multicopter/scenarios/multicopter_delivery.scn`
for a complete example:

```bash
uv run minisky run --scenario ../minisky-multicopter/scenarios/multicopter_delivery.scn --speed 10
```

(scenario paths resolve relative to the `packages/minisky/` package root).

Multicopters fly point-to-point: their waypoints default to fly-over with a
fixed capture radius (10 m unless `capture_radius` is configured), and
`SPD 0` is valid (the rotor envelopes have a
negative minimum speed). One thing to keep in mind: LNAV commands only the
track, so a multicopter created at rest also needs a speed source — a `SPD`
command or waypoint speed constraints — before it starts moving.

## The battery model

Multicopter rows replace fuel flow with an electric model, updated every
simulation step:

- **Required thrust** supports the weight and overcomes flat-plate parasite
  drag:

    $$T = \sqrt{L^2 + D^2}, \qquad
    L = m\sqrt{g^2 + a_z^2}, \qquad
    D = \tfrac{1}{2}\rho v^2 C_D S$$

- **Electrical power** follows a momentum-theory scaling anchored to the
  installed power from the OpenAP rotor coefficients, with
  $T_\text{max} = \mathrm{TWR} \cdot m g$ and a default thrust-to-weight
  ratio of 2:

    $$P = P_\text{max} \left(\frac{T}{T_\text{max}}\right)^{1.5}$$

- **State of charge** integrates that power against a usable pack energy —
  an ideal energy tank, with no terminal-voltage or current modelling.
- **Envelope feedback**: below 20% charge (`soc_low`) the maximum speed
  shrinks to 60% and the maximum climb rate to 50% (`lowbatt_spd_factor`,
  `lowbatt_vs_factor`). Descent stays unrestricted — a low battery should
  not keep an aircraft airborne.

`BATT` reports the live state:

```
BATT DRONE1 -> BATT DRONE1: 50%, drawing 1022 W, endurance 18 min
```

The absolute forward-flight power is approximate (momentum-theory shape, not
measured propeller data), but hover figures and the qualitative trends are
sound — hover endurance for the MAVIC comes out around 28 minutes.

## Adding a new multicopter type

All multicopter performance data lives in a TOML table, keyed on the
ICAO-style typecode: the plugin ships built-in entries and merges them with
an optional user file, validated on plugin load — no source edits needed.
The user file is `multicopter.toml` in the platform cache directory (e.g.
`~/Library/Caches/minisky/` on macOS, `~/.cache/minisky/` on Linux), or any
path set as `performance_path` under `[plugins.multicopter]`. A user entry
for a built-in typecode replaces it entirely; a new typecode extends the
membership set.

An entry has two parts:

1. **Electric model** — the usable pack energy `battery_wh` (in Wh; the one
   datum the rotor database cannot carry — its `mfc` fuel field is unused
   for electric types), plus optional `cds` (flat-plate drag area, m²,
   default 0.01) and `twr` (thrust-to-weight ratio, default 2). Without
   `battery_wh` the pack energy is derived from the envelope's
   `d_range_max` flown at cruise speed, like the delivery-drone types that
   have no public pack spec.
2. **Airframe block** — only for typecodes unknown to the shipped rotor
   database, given all together: masses, motor count and power, and the
   flight envelope. It becomes a full rotor entry at runtime.

```toml
[types.MYDRONE]
battery_wh = 100.0 # usable pack energy [Wh]
cds = 0.02         # flat-plate drag area [m2] (optional)
twr = 2.0          # thrust-to-weight ratio (optional)
# Airframe block, required for types the rotor database does not know:
oew = 5.0          # empty mass [kg]
mtow = 8.0         # maximum take-off mass [kg]
n_engines = 4      # number of motors
engine_kw = 0.5    # maximum power per motor [kW]
v_min = -10.0      # minimum speed [m/s] — negative so the aircraft may stop
v_max = 20.0       # maximum speed [m/s]
vs_min = -5.0      # maximum descent rate [m/s]
vs_max = 5.0       # maximum climb rate [m/s]
h_max = 3000.0     # ceiling [m]
d_range_max = 20.0 # maximum range [km] (optional, for the energy fallback)
```
