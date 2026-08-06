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

On the first simulation step after loading, the plugin selects its
implementations of five [replaceable traffic components](../architecture.md#replaceable-components)
(kinematics, pilot logic, autopilot, waypoint capture, and performance), and
re-selects them after every `RESET`. A manual `SELECTIMPL` afterwards is
respected until the next reset.

## Which aircraft count as multicopters

Membership follows the aircraft type: creating any of

```
MAVIC  PHAN4  M100  M200  M600  MNET  AMZN  HORSEFLY
```

produces a multicopter. Everything else — including the `EC35` helicopter —
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
fixed 10 m capture radius, and `SPD 0` is valid (the rotor envelopes have a
negative minimum speed). One thing to keep in mind: LNAV commands only the
track, so a multicopter created at rest also needs a speed source — a `SPD`
command or waypoint speed constraints — before it starts moving.

## The battery model

Multicopter rows replace fuel flow with an electric model, updated every
simulation step:

- **Required thrust** supports the weight and overcomes flat-plate parasite
  drag: `T = hypot(m * sqrt(g^2 + az^2), 0.5 * rho * v^2 * CdS)`.
- **Electrical power** follows a momentum-theory scaling anchored to the
  installed power from the OpenAP rotor coefficients
  (`P = P_max * (T / T_max)^1.5`, with `T_max = TWR * m * g` and a default
  thrust-to-weight ratio of 2).
- **State of charge** integrates that power against a usable pack energy —
  an ideal energy tank, with no terminal-voltage or current modelling.
- **Envelope feedback**: below 20% charge the maximum speed shrinks to 60%
  and the maximum climb rate to 50%. Descent stays unrestricted — a low
  battery should not keep an aircraft airborne.

`BATT` reports the live state:

```
BATT DRONE1 -> BATT DRONE1: 50%, drawing 1022 W, endurance 18 min
```

The absolute forward-flight power is approximate (momentum-theory shape, not
measured propeller data), but hover figures and the qualitative trends are
sound — hover endurance for the MAVIC comes out around 28 minutes. A
measured-data upgrade path is sketched in the plan document
(`docs/multicopter-plan.md`).

## Adding a new multicopter type

Three places, all keyed on the ICAO-style typecode:

1. **Performance data** — add a rotor entry to
   `packages/minisky/minisky/data/performance/openap/rotor/aircraft.json`
   with the masses (`oew`, `mtow`, in kg), `n_engines`, per-engine power
   (`engines`, in kW), and the flight envelope (`v_min`/`v_max`,
   `vs_min`/`vs_max`, `h_max`, `d_range_max`). Give `v_min` a negative value
   so the aircraft may stop.
2. **Membership** — add the typecode to `MULTICOPTER_TYPES` in
   `minisky_multicopter/entity.py`.
3. **Battery capacity** — add a `CONSTANTS` entry in
   `minisky_multicopter/perf.py` with the usable pack energy `battery_wh`
   (the one datum the rotor `aircraft.json` cannot carry — its `mfc` fuel
   field is unused for electric types), plus optional `cds` (flat-plate drag
   area, m²) and `twr` (thrust-to-weight ratio) overrides. Without an entry
   the pack energy is derived from the envelope's `d_range_max` flown at
   cruise speed, like the delivery-drone types that have no public pack spec.
