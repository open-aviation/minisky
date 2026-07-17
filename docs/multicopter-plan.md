# Multicopter support plan

Status: **proposal** — nothing in this document is implemented yet.

## Goal

Make MiniSky able to simulate small electric multirotors ("multicopters": DJI MAVIC/M600/PHAN4-class,
Amazon/Matternet-style delivery drones) with realistic behaviour:

1. **Hover and yaw** — change heading at zero ground/airspeed, limited by a yaw rate instead of a
   bank-angle turn rate.
2. **Decoupled track and heading** — change direction of travel without rotating the body. A
   multicopter redirects thrust; its velocity vector (track) is independent of where the nose
   points (heading). Course changes at waypoints are immediate, with no turn radius.
3. **Electric performance** — battery state of charge, power draw as a function of speed and
   required thrust, and a flight envelope that degrades as the battery sags.

Everything lands as a **plugin** plus one small, behaviour-preserving core refactor. This follows
the project direction: keep the core minimal, make behaviour hackable from outside.

### Explicit non-goals

- **Helicopters.** OpenAP's rotor list includes the EC35 (a crewed helicopter). It is deliberately
  *not* covered: it keeps today's envelope-only performance and bank-to-turn kinematics. This is
  why the feature is named *multicopter*, not *drone* (wrong axis: describes crew, not lift type)
  and not *rotorcraft* (would promise helicopter support).
- **Aeroelastic / attitude-level dynamics.** We stay at the kinematic point-mass level of the rest
  of the simulator; "heading" is the only attitude state.
- **A runtime PyThrust dependency.** We use PyThrust's *data* (see Phase 3), never its code at
  runtime, and it is not added to `pyproject.toml`.

## What the codebase already provides

The exploration that produced this plan found MiniSky closer to multicopter-ready than expected:

- **Rotor performance path exists.** `minisky/traffic/performance/perfoap.py` distinguishes
  `LIFT_FIXWING` from `LIFT_ROTOR`. Creating an aircraft with a rotor typecode (`CRE D1 MAVIC ...`)
  gets envelope-only performance: no drag polar, fixed `axmax = 3.5 m/s²`, static limits.
  Shipped rotor typecodes: `EC35, M600, AMZN, MNET, PHAN4, M100, M200, MAVIC, HORSEFLY`.
- **Zero speed already passes the performance clamp.** Rotor envelopes have *negative* `vmin`
  (e.g. M600: −18 m/s), and `OpenAP.limits()` clamps rotor TAS directly against `[vmin, vmax]`,
  so `SPD D1 0` survives. Fixed-wing aircraft are clamped to stall speed and cannot do this.
- **The replaceable pattern.** Every first-level `TrafficArrays` subclass auto-registers for
  `SELECTIMPL` (`minisky/core/trafficarrays.py`), and `_replace_instance_on_traf()` hot-swaps the
  instance on `traf`, carrying per-aircraft arrays over and rebinding stack commands. `Autopilot`,
  `OpenAP`, `APorASAS`, `ConflictDetection`, `ConflictResolution` are all swappable today.
  `example_plugins/customautopilot.py` demonstrates the pattern.
- **Plugin machinery.** Timed `preupdate`/`update`/`reset` hooks, `plugin.Entity` +
  `settrafarrays()` for per-aircraft state that grows/shrinks with the fleet, and
  `@stack.command` for new commands.

## What blocks the two manoeuvring behaviours

Both live in `Traffic` (`minisky/traffic/traffic.py`), in ~100 lines of kinematics:

1. **`update_airspeed()`** derives turn rate from the bank-angle triangle,
   `ω = g·tan(φ)/max(tas, eps)` with `eps = 0.01`. At TAS → 0 this *explodes* (≈26 000 °/s), so
   heading snaps instantly — hover-yaw "works" by numerical accident, with no physical yaw-rate
   limit.
2. **`update_groundspeed()`** hard-couples `trk = hdg` and points the velocity vector along the
   heading. The aircraft must fly where its nose points. Upstream, `APorASAS.update()`
   (`minisky/traffic/aporasas.py`) converts the desired *track* into a desired *heading* (with wind
   correction), baking the same coupling into the command path.

Everything downstream is already agnostic: conflict detection/resolution, the stream snapshot and
LNAV all work off `trk`/`gs`, which remain well-defined when decoupled from `hdg`.

`Traffic` is technically registered as a replaceable (`SELECTIMPL TRAFFIC ...` lists it), but the
hot-swap helper only swaps instances found *on* `traf` — it cannot replace the root object itself,
and in CLI runs plugins load after `minisky.init()` has constructed `traf`. Hence Phase 1.

---

## Phase 1 — core refactor: extract `Kinematics` as a replaceable entity

**The only core change in this plan.** Behaviour-preserving.

Move `update_airspeed()`, `update_groundspeed()`, `update_pos()` and the state they own
(`ax`, `az`, `swhdgsel`, `swaltsel`) out of `Traffic` into a new first-level class:

```python
# minisky/traffic/kinematics.py
class Kinematics(TrafficArrays):
    """Integrates airspeed, heading, vertical speed and position each step.

    Replaceable via SELECTIMPL KINEMATICS <IMPL>; plugins may subclass to
    change how (a subset of) aircraft fly.
    """
    def update(self):
        self.update_airspeed()
        self.update_groundspeed()
        self.update_pos()
```

- Instantiated as `self.kinematics = Kinematics()` inside `Traffic.__init__`'s
  `settrafarrays()` block; `Traffic.update()` calls `self.kinematics.update()` in place of the
  three method calls.
- Because it is a first-level `TrafficArrays` subclass it **auto-registers** as replaceable —
  `SELECTIMPL KINEMATICS MULTICOPTERKINEMATICS` then hot-swaps mid-simulation exactly like the
  custom-autopilot example, with no further core support needed.
- Keep thin delegating properties on `Traffic` only if anything external reads `traf.ax` etc.
  (grep first; `streaming.py` and `perfoap.py` read `traf.ax` — either keep `ax` on `Traffic` and
  have `Kinematics` write it, or add a property. Decide during implementation; prefer keeping the
  arrays registered on `Kinematics` and exposing properties on `Traffic`.)

**Acceptance:** entire existing test suite passes unchanged; `SELECTIMPL KINEMATICS` lists the
base implementation; a trivial subclass registered from a test can be selected and reverts on
reset (mirror the existing `tests/integration/test_plugin.py` replaceable test).

## Phase 2 — the `multicopter` plugin: membership + kinematics

New file `plugins/multicopter.py` (plugin name `MULTICOPTER`), no core changes.

### Membership

Selection must **not** be `traf.perf.lifttype == LIFT_ROTOR` — that would sweep in the EC35.

- Module constant `MULTICOPTER_TYPES = {"MAVIC", "PHAN4", "M100", "M200", "M600", "MNET", "AMZN",
  "HORSEFLY"}` (the OpenAP rotor list minus helicopters).
- A `plugin.Entity` subclass holding per-aircraft arrays registered via `settrafarrays()`:
  - `ismulticopter` (bool) — set in `create()` from the typecode, manual override via a
    `MCOPT acid ON/OFF` stack command for custom typecodes;
  - `selhdg` (deg) — commanded body heading, decoupled from track;
  - `yawrate` (deg/s) — default ≈ 90 °/s, settable per aircraft (`YAWRATE acid 120`).

### `MulticopterKinematics(Kinematics)`

Selected with `SELECTIMPL KINEMATICS MULTICOPTERKINEMATICS` (the plugin issues this on load /
documents it). Calls `super().update()` for the whole fleet, then re-integrates the multicopter
rows (mask `m`):

```python
dt = minisky.sim.simdt
# 1. Yaw at a fixed rate — valid at tas = 0 (hover-yaw)
delhdg = (mc.selhdg[m] - traf.hdg[m] + 180) % 360 - 180
traf.hdg[m] += np.clip(delhdg, -mc.yawrate[m] * dt, mc.yawrate[m] * dt)
traf.hdg[m] %= 360
# 2. Velocity vector follows the commanded *track* (LNAV/ASAS), not the heading
trkcmd = np.radians(traf.aporasas.trk[m])
traf.gsnorth[m] = traf.tas[m] * np.cos(trkcmd) + traf.windnorth[m] * airborne
traf.gseast[m]  = traf.tas[m] * np.sin(trkcmd) + traf.windeast[m] * airborne
traf.gs[m]  = np.hypot(traf.gsnorth[m], traf.gseast[m])
traf.trk[m] = np.degrees(np.arctan2(traf.gseast[m], traf.gsnorth[m])) % 360
# 3. Re-integrate lat/lon for these rows (base class integrated with the wrong velocity)
```

Implementation notes:

- The base class integrates position before the override, so either re-integrate lat/lon for the
  masked rows from the stored previous position, or (cleaner) restructure `Kinematics.update()`
  into `update_airspeed / update_groundspeed / update_pos` calls so the subclass overrides the
  first two and lets `update_pos()` run once, after. Prefer the latter — it is exactly what the
  Phase 1 split enables.
- Heading no longer follows track for these rows, so also subclass or bypass the
  `APorASAS` trk→hdg coupling: `MulticopterAPorASAS(APorASAS)` that, after `super().update()`,
  overwrites `self.hdg[m]` with `mc.selhdg[m]`. (`SELECTIMPL APORASAS MULTICOPTERAPORASAS`.)
- `HDG` (stack) semantics for multicopters: route the existing `HDG` command value into
  `mc.selhdg` (nose) and add `YAW acid 45` as an explicit alias; the FMS/LNAV track command
  continues to steer the velocity vector. Default behaviour when no `selhdg` was ever set:
  follow the track (nose-along-course), so routes look natural without extra commands.
- Turn-anticipation in the FMS assumes a turn radius; multicopters fly point-to-point. Keep it
  simple first: the immediate-course-capture behaviour falls out of step 2 automatically because
  `aporasas.trk` snaps to the new leg bearing at waypoint switch.

### `MulticopterAutopilot(Autopilot)` — thin, mission-level

A full autopilot rewrite is **not** needed: LNAV already outputs a *track* command
(`ap.trk = qdr2wp`), which is exactly what the decoupled kinematics consumes; fly-over waypoints
already exist (`ADDWPTMODE FLYOVER`); the vertical channel (`ALT`/`selvs`) is speed-independent,
so hover-climb/descend works with the plain `ALT` command; and turn-speed deceleration only
activates for `FLYTURN` waypoints, which multicopters won't use.

A thin subclass (`SELECTIMPL AUTOPILOT MULTICOPTERAUTOPILOT`) covers what the stock FMS cannot:

- **Mission primitives** the FMS has no concept of:
  - `HOVER acid [time]` — suspend LNAV, hold position (commanded gs = 0), auto-resume the route
    after the optional duration. The conditional-command machinery (ATALT/ATDIST) cannot express
    "hold for 90 s".
  - `DELIVER acid alt [time]` — at the current position: vertical descent to `alt`, dwell, climb
    back, continue the route. Implemented as a small per-aircraft state machine on top of
    `super().update()`.
- **Low-speed guards**: `calcturn()` and the turn-distance/deceleration formulas are bank- and
  speed-based; clamp `actwp.turndist` for multicopter rows to a fixed capture radius (~5–10 m)
  so waypoint switching stays sane at creeping speeds and at hover on top of a waypoint.
- **Route defaults**: set fly-over + capture radius automatically for `ismulticopter` aircraft
  when waypoints are added, so scenario authors need no extra commands.

With this, the plugin issues three swaps on load — `KINEMATICS`, `APORASAS`, `AUTOPILOT` — each
subclass calling `super()` and adjusting only the masked multicopter rows.

**Acceptance (integration tests, driven through the stack like `test_stack.py`):**

- `CRE D1 MAVIC ... ; SPD D1 0` → ground speed reaches 0 and stays; aircraft holds position.
- At `gs == 0`, `HDG D1 90` → heading slews at `yawrate`, position unchanged.
- In cruise, `YAW D1 0` while flying track 090 → `trk` stays 090, `hdg` goes to 0.
- Waypoint passage: course changes leg-to-leg with no overshoot arc.
- `HOVER D1 90` mid-route → position frozen for 90 s of sim time, then the route resumes.
- `DELIVER D1 50 30` → vertical descent to 50 ft, 30 s dwell, climb back, route resumes;
  lat/lon unchanged throughout.
- A fixed-wing aircraft in the same simulation behaves byte-identically to `main` (regression
  guard for the fleet-wide hooks).

## Phase 3 — `MulticopterPerf`: electric performance from PyThrust *data*

`class MulticopterPerf(OpenAP)`, selected with `SELECTIMPL OPENAP MULTICOPTERPERF`. Fixed-wing
rows keep `super()` behaviour untouched; multicopter rows get an electric model. This fills the
long-standing `# TODO: implement thrust computation for rotor aircraft` in `perfoap.py`.

### Data pipeline (no new runtime dependency — decided)

[PyThrust](https://github.com/Setuav/PyThrust) (Apache 2.0) ships everything needed as plain
data:

- **Propeller tables** (`data/propellers/apc_202602/*.csv`, 441 APC props): full performance
  grids with `rpm, speed_mps, thrust_n, power_w, torque_nm, ct, cp, ...` — thrust *and* shaft
  power are already tabulated, so the inverse question a perf model asks
  ("required thrust at this airspeed → power") is pure interpolation. No solver needed.
- **Motor specs** (`data/motors/*.json`): `kv`, `resistance`, `io`, `max_current` — shaft-to-
  electrical conversion is a few lines of textbook motor algebra.
- **Battery curves** (`data/batteries/*.json`): open-circuit voltage and internal resistance vs
  depth-of-discharge — `np.interp` territory. (Their example cell is synthetic; real types
  should get measured curves eventually.)

Pipeline, following the existing regen conventions (navdb parquet, `minisky commands docs`):

1. `scripts/gen_multicopter_perf.py` — **self-contained** (numpy only, no pythrust import):
   reads vendored prop CSV + motor JSON per multicopter typecode (config mapping typecode →
   {prop, motor, cell, series/parallel, n_rotors, mass}), and emits one small artifact per type:
   a grid `(airspeed, thrust) → (power_w, current_a, feasible)` (~30 KB float32 npz/parquet)
   plus the battery curves.
2. Artifacts are **checked in** next to the plugin (`plugins/data/multicopter/`). The handful of
   vendored source CSV/JSONs (~1 MB) live under `plugins/data/multicopter/pythrust/` together
   with PyThrust's LICENSE and an attribution note (the prop tables are repackaged APC published
   performance data).
3. Runtime: `MulticopterPerf` loads the artifacts at plugin load and evaluates with vectorised
   `np.interp`/`RegularGridInterpolator`. Zero per-step Python loops, zero new dependencies.

### Runtime model (multicopter rows)

- **Required thrust:** per rotor, `T = m·√(g² + a²)/n_rotors` in hover/climb, plus a flat-plate
  parasite term `½ρv²·CdS` in translation (edgewise-flow caveat below).
- **Power/current:** from the per-type map at `(tas, T)`; write `self.thrust` (total) and expose
  `battery_power` as the electric analogue of `fuelflow`.
- **Battery:** per-aircraft `soc` array; integrate `soc -= I·dt / capacity`; terminal voltage
  from OCV/R curves.
- **Envelope feedback:** where the map is infeasible at current battery voltage (sag at low SoC),
  tighten `vmax`/`vsmax` in `limits()` — performance genuinely degrades as the battery empties.
- **Stack commands:** `BATT acid` (report SoC/power/endurance estimate), optional auto-RTH/land
  threshold via the conditional-command machinery later.

**Fidelity caveat (documented in the plugin):** APC coefficients are axial-flow; a translating
multicopter has edgewise inflow, so forward-flight power is approximate. Hover figures and the
qualitative trends (power vs speed, voltage sag) are sound — the right level for a traffic
simulator.

**Acceptance:** hover endurance for a MAVIC-class config lands within sanity bounds (~20–35 min);
`BATT` reports monotonically decreasing SoC; envelope shrinks below a SoC threshold; unit tests
for the map interpolation against a few hand-computed points from the source CSV.

## Phase 4 — docs, scenarios, cleanup

- New guide `docs/guides/multicopters.md`: creating multicopters, hover/yaw commands, battery
  model, how to add a new type (config + regen script).
- Update `docs/architecture.md` with the `Kinematics` entity and the replaceable list.
- Example scenario `scenarios/multicopter_delivery.scn`: create, fly a route, hover at a
  delivery point, yaw for "camera", return; exercises everything above.
- Regenerate `docs/reference/commands.md` (`uv run minisky commands docs`) after adding the
  stack commands (`MCOPT`, `YAW`, `YAWRATE`, `HOVER`, `DELIVER`, `BATT`).
- `ruff`, `pyright`, full test suite green at every phase boundary.

## Sequencing and effort

| Phase | Scope | Risk | Depends on |
|---|---|---|---|
| 1 | Extract `Kinematics` (core, behaviour-preserving) | Low — mechanical move guarded by existing tests | — |
| 2 | Plugin: membership + kinematics + commands | Medium — command semantics for HDG/YAW need care | 1 |
| 3 | Perf: data vendoring, gen script, `MulticopterPerf`, battery | Medium — model calibration/sanity | 2 (usable after 1) |
| 4 | Docs, scenario, polish | Low | 2, 3 |

Each phase is a separately reviewable PR; phase 1 is intentionally the only one touching
`minisky/`.

## Decision log

| Decision | Choice | Why |
|---|---|---|
| Name | **multicopter** (not drone/rotorcraft) | Names the lift/control type actually modelled; scope excludes helicopters (EC35) |
| Where behaviour lives | Plugin + replaceable subclasses | Matches "minimal core, hack from outside"; hot-swappable via `SELECTIMPL`; reverts on reset |
| Kinematics override mechanism | New first-level `Kinematics` entity (Phase 1) | `Traffic` itself can't be hot-swapped (root object); post-hoc plugin-hook correction would double-integrate state |
| Custom autopilot | Thin `MulticopterAutopilot` for mission primitives (HOVER/DELIVER), capture-radius clamp and fly-over defaults only | LNAV's track output already suits decoupled kinematics; no guidance rewrite needed |
| Membership predicate | Plugin-owned typecode set + `ismulticopter` array | `LIFT_ROTOR` includes helicopters |
| PyThrust | Data only, vendored with attribution; self-contained gen script; nothing at runtime | Prop CSVs already tabulate thrust & power; keeps dependency tree untouched (Apache 2.0 permits) |
| Perf evaluation | Precomputed per-type maps, vectorised interp | Keeps the numpy discipline; fleet-size independent; regen convention already exists in repo |
