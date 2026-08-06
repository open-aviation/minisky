# Multicopter support plan

Status: **complete** — all four phases are implemented on this branch.

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
- **PyThrust, for now.** The measured-prop-data pipeline is deferred entirely to future work
  (see the last section). Phase 3 uses only the OpenAP rotor coefficients already shipped in
  `data/performance/openap/rotor/aircraft.json` plus a few spec-sheet constants. If the pipeline
  is ever built, it uses PyThrust's *data*, never its code at runtime, and it is not added to
  `pyproject.toml`.

## What the codebase already provides

The exploration that produced this plan found MiniSky closer to multicopter-ready than expected:

- **Rotor performance path exists.** `minisky/traffic/performance/perfoap.py` distinguishes
  `LIFT_FIXWING` from `LIFT_ROTOR`. Creating an aircraft with a rotor typecode (`CRE D1 MAVIC ...`)
  gets envelope-only performance: no drag polar, fixed `axmax = 3.5 m/s²`, static limits.
  Shipped rotor typecodes: `EC35, M600, AMZN, MNET, PHAN4, M100, M200, MAVIC, HORSEFLY`.
- **Zero speed already passes the performance clamp.** Rotor envelopes have *negative* `vmin`
  (e.g. M600: −18 m/s), and `OpenAP.limits()` clamps rotor TAS directly against `[vmin, vmax]`,
  so `SPD D1 0` survives. Fixed-wing aircraft are clamped to stall speed and cannot do this.
- **The replaceable pattern.** `ReplaceableManager` (`minisky/core/trafficarrays.py`) owns a
  curated set of replaceable bases per runtime (registered in `MiniSky.__init__`) and hot-swaps
  the instance on `traf` via `SELECTIMPL`, carrying per-aircraft arrays over and dispatching
  stack commands through the current instance. Plugins register implementations runtime-locally
  with `@plugin.replacement` + `context.finish(replacements=...)`;
  `packages/minisky-example-customautopilot` demonstrates the pattern.
- **Plugin machinery.** Plugins are packages exposing a `Plugin` declaration through the
  `minisky.plugins` entry-point group. Timed `preupdate`/`update`/`reset` hooks via
  `@plugin.hook`, `plugin.Entity` + `settrafarrays()` for per-aircraft state that grows/shrinks
  with the fleet, and `@plugin.command` for new commands.

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
- Registered as a replaceable base in the runtime's `ReplaceableManager` (`minisky/runtime.py`,
  together with `APorASAS` and `ActiveWaypoint`, which Phase 2 also swaps) —
  `SELECTIMPL KINEMATICS MULTICOPTERKINEMATICS` then hot-swaps mid-simulation exactly like the
  custom-autopilot example, with no further core support needed.
- Keep thin delegating properties on `Traffic` only if anything external reads `traf.ax` etc.
  (grep first; `streaming.py` and `perfoap.py` read `traf.ax` — either keep `ax` on `Traffic` and
  have `Kinematics` write it, or add a property. Decide during implementation; prefer keeping the
  arrays registered on `Kinematics` and exposing properties on `Traffic`.)

**Acceptance:** entire existing test suite passes unchanged; `SELECTIMPL KINEMATICS` lists the
base implementation; a trivial subclass registered from a test can be selected and reverts on
reset (mirror the existing `tests/integration/test_plugin.py` replaceable test).

### Phase 1 checklist

- [x] Create `minisky/traffic/kinematics.py`: `Kinematics(TrafficArrays)` with
      `update_airspeed` / `update_groundspeed` / `update_pos` and the `ax`, `az`, `swhdgsel`,
      `swaltsel` arrays moved over from `Traffic` (all four registered in `settrafarrays` and
      seeded in `create()`; `az`/`swaltsel` were previously undeclared attributes materialised
      by `update_airspeed`)
- [x] Instantiate as `self.kinematics = Kinematics()` inside `Traffic.__init__`'s
      `settrafarrays()` block; `Traffic.update()` calls `self.kinematics.update()`
- [x] Grep external readers of the moved arrays: only `perfoap.py` reads `ax`
      (`streaming.py` does not); pointed it at `traf.kinematics.ax` (no property needed)
- [x] Register `Kinematics` (plus `APorASAS` and `ActiveWaypoint` for Phase 2) as replaceable
      bases in `MiniSky.__init__`; verify `SELECTIMPL KINEMATICS` lists the base implementation
- [x] Test: install a trivial subclass runtime-locally, select it, verify it takes effect and
      reverts on reset (`tests/integration/test_kinematics.py`)
- [x] `uv run pytest`, `uv run ruff check .`, `uv run pyright` all green

## Phase 2 — the `multicopter` plugin: membership + kinematics

New workspace package `packages/minisky-multicopter/` (plugin ID `multicopter`), no core changes
beyond the Phase 1 base registration. One module per class, so each piece stays small and
readable:

```
packages/minisky-multicopter/
├── pyproject.toml           # workspace member; minisky.plugins entry point "multicopter"
└── src/minisky_multicopter/
    ├── __init__.py     # Plugin declaration: build() mounts the entity, registers replacements
    ├── entity.py       # MULTICOPTER_TYPES + Multicopter Entity (ismulticopter, selhdg, yawrate),
    │                   # its stack commands (MCOPT, YAW, YAWRATE, HOVER) and the selection hooks
    ├── kinematics.py   # MulticopterKinematics(Kinematics)
    ├── aporasas.py     # MulticopterAPorASAS(APorASAS)
    ├── autopilot.py    # MulticopterAutopilot(Autopilot): hover primitive, fly-over defaults
    ├── activewp.py     # MulticopterActiveWaypoint(ActiveWaypoint): fixed capture radius
    ├── perf.py         # MulticopterPerf(OpenAP) + BATT              (Phase 3)
    └── data/           # generated perf maps + vendored PyThrust data (Phase 3)
```

Loader notes: the plugin manager discovers installed packages through the `minisky.plugins`
entry-point group without importing them; `__init__.py` exports the `Plugin` declaration and
imports the class modules. Replacements are registered runtime-locally when the plugin loads
(`@plugin.replacement` classes passed to `context.finish(replacements=...)`) and removed again
on shutdown. Selection is *not* automatic on load: the entity's `preupdate` hook selects the
four implementations on the first step after loading (via `traffic.select_implementation`), and
its `reset` hook re-selects them after every reset, which reverts all replaceables to their core
defaults.

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

Selected with `SELECTIMPL KINEMATICS MULTICOPTERKINEMATICS` (the plugin's hooks keep this
selected). Calls `super().update()` for the whole fleet, then re-integrates the multicopter
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

- **A hover primitive** the FMS has no concept of — deliberately *composable*, not a scripted
  manoeuvre (a "delivery" is written in the scenario from `HOVER` + `ALT` + `LNAV`):
  - `HOVER acid [time] [alt]` — suspend LNAV/VNAV, hold position (commanded gs = 0), optionally
    at a commanded altitude (moved to vertically, at a fixed position). With a `time` the route
    auto-resumes once position and altitude have been held that long (the conditional-command
    machinery cannot express "hold for 90 s"); without one the aircraft hovers until LNAV is
    re-engaged. Repeating `HOVER` while hovering updates the hold; a plain `ALT` changes the
    hover altitude too.
- **`HDG` semantics**: for multicopter rows `HDG` becomes an alias of `YAW` — it rotates the
  nose only and leaves LNAV engaged.
- **Route defaults**: fly-over waypoints automatically for multicopter aircraft, so scenario
  authors need no extra commands.
- **Low-speed capture (in `MulticopterActiveWaypoint`)**: `calcturn()` and the turn-distance
  formulas are bank- and speed-based and degenerate at multicopter speeds; multicopter rows use
  a fixed capture radius (10 m) instead. This must live in an `ActiveWaypoint` subclass, because
  `ActiveWaypoint.reached()` recomputes `turndist` every step — clamping it from the autopilot
  update would be overwritten before it is ever used.

With this, the plugin registers and keeps selected four swaps — `KINEMATICS`, `APORASAS`,
`AUTOPILOT`, `ACTIVEWAYPOINT` — each subclass calling `super()` and adjusting only the masked
multicopter rows.

**Acceptance (integration tests, driven through the stack like `test_stack.py`):**

- `CRE D1 MAVIC ... ; SPD D1 0` → ground speed reaches 0 and stays; aircraft holds position.
- At `gs == 0`, `HDG D1 90` → heading slews at `yawrate`, position unchanged.
- In cruise, `YAW D1 0` while flying track 090 → `trk` stays 090, `hdg` goes to 0.
- Waypoint passage: course changes leg-to-leg with no overshoot arc.
- `HOVER D1 90` mid-route → position frozen for 90 s of sim time, then the route resumes.
- `HOVER D1 30 100` mid-route → vertical descent to 100 ft at a fixed position, 30 s hold,
  route resumes at the hover altitude; a delivery profile composes from `HOVER`, `ALT` and
  `LNAV ON` with lat/lon unchanged throughout.
- A fixed-wing aircraft in the same simulation behaves byte-identically to `main` (regression
  guard for the fleet-wide hooks).

### Phase 2 checklist

- [x] `packages/minisky-multicopter/` workspace package with a `minisky.plugins` entry point
      (`multicopter = "minisky_multicopter:plugin"`) and the `Plugin` declaration in
      `__init__.py`
- [x] `entity.py`: `MULTICOPTER_TYPES` set + `Entity` with `ismulticopter`, `selhdg`,
      `yawrate` arrays, auto-set from typecode in `create()`; stack commands `MCOPT`,
      `YAW`, `YAWRATE`, `HOVER` (declared with `@plugin.command`)
- [x] `kinematics.py`: `MulticopterKinematics(Kinematics)` — yaw-rate-limited heading,
      track-driven velocity vector, single `update_pos()` pass
- [x] `aporasas.py`: `MulticopterAPorASAS(APorASAS)` — skip trk→hdg coupling for
      multicopter rows
- [x] `autopilot.py`: `MulticopterAutopilot(Autopilot)` — composable `HOVER [time] [alt]`
      (the planned `DELIVER` was dropped as too use-case specific), `HDG`-yaws-the-nose,
      fly-over route defaults; `activewp.py`: `MulticopterActiveWaypoint` fixed capture
      radius
- [x] Plugin registers the four replacements on load; the entity's `preupdate` hook selects
      them on the first step, and its `reset` hook re-selects after every reset (which
      reverts all replaceables to the core defaults)
- [x] Integration tests: hover-hold, yaw at gs = 0, strafe (fixed nose, moving track),
      leg-to-leg course capture, `HOVER` (timed, at altitude, composed with `ALT`/`LNAV`),
      fixed-wing regression guard
- [x] `uv run pytest`, `uv run ruff check .`, `uv run pyright` all green

## Phase 3 — `MulticopterPerf`: analytic electric model from the OpenAP rotor data

`class MulticopterPerf(OpenAP)`, selected with `SELECTIMPL OPENAP MULTICOPTERPERF`. Fixed-wing
rows keep `super()` behaviour untouched; multicopter rows get an electric model. This fills the
long-standing `# TODO: implement thrust computation for rotor aircraft` in `perfoap.py`.

### Data: what ships already, and the one gap

`data/performance/openap/rotor/aircraft.json` (already loaded by `OpenAP.create()`) provides,
per rotor typecode: mass (`oew`/`mtow` → `traf.perf.mass`), `n_engines` (`engnum`), per-engine
max power in kW (`engpower`), and the flight envelope. That is enough for an analytic power
model with **no new data pipeline and no PyThrust anything**:

- **Installed power** `P_max = engnum · engpower` is the model's anchor.
- **Battery capacity is the one thing the json lacks** (`mfc` is 0 for every rotor type). A
  small hand-written per-typecode dict in `perf.py` supplies spec-sheet watt-hours (e.g. MAVIC
  43.6 Wh, PHAN4 81.3 Wh, M600 6×99.9 Wh), with a fallback that derives energy from
  `d_range_max` at cruise speed for unlisted types. The same dict optionally carries `CdS`
  (flat-plate area) and a thrust-to-weight ratio where the default is wrong.

### Runtime model (multicopter rows)

- **Required thrust:** `T = m·√(g² + a_z²)`, plus a flat-plate parasite term `½ρv²·CdS` in
  translation (small default `CdS`).
- **Power:** momentum-theory scaling referenced to installed power,
  `P = P_max · (T / T_max)^1.5`, with `T_max = TWR · m·g` and a default thrust-to-weight ratio
  of 2 (typical for camera/delivery multirotors). Sanity anchor: MAVIC installed power is
  4 × 66.9 W ≈ 268 W, giving ≈ 130 W in hover — matching published figures. Write `self.thrust`
  and expose `battery_power` as the electric analogue of `fuelflow`.
- **Battery:** per-aircraft `soc` array, ideal-energy-tank integration
  `soc -= P·dt / E_batt`. No terminal-voltage/current modelling — that needs the electrical
  data (motor kv/resistance, OCV/R curves) deferred with PyThrust.
- **Envelope feedback:** below an SoC threshold, tighten `vmax`/`vsmax` in `limits()` —
  keyed on SoC directly rather than physical voltage sag.
- **Stack commands:** `BATT acid` (report SoC/power/endurance estimate), optional auto-RTH/land
  threshold via the conditional-command machinery later.

**Fidelity caveat (documented in the plugin):** the power curve is momentum-theory shape, not
measured prop data — absolute forward-flight power is approximate, and the model deliberately
ignores the induced-power *drop* in fast translation (power is monotone in required thrust
here). Hover figures and the qualitative trends (power vs thrust, endurance, envelope shrink at
low battery) are sound — the right level for a traffic simulator, upgradeable later without API
changes (see the future-work section).

**Acceptance:** hover endurance for a MAVIC-class config lands within sanity bounds (~20–35 min);
`BATT` reports monotonically decreasing SoC; envelope shrinks below a SoC threshold; unit tests
for the power model against a few hand-computed points.

### Phase 3 checklist

- [x] Per-typecode constants dict in `perf.py`: `{battery_wh, cds?, twr?}` from public spec
      sheets (MAVIC, PHAN4, M100, M200, M600), `d_range_max`-derived fallback for unlisted
      rotor types (MNET, AMZN, HORSEFLY)
- [x] `perf.py`: `MulticopterPerf(OpenAP)` — required-thrust model, momentum-theory power from
      `engnum · engpower` (stored in kW — converted at the model boundary), per-aircraft SoC
      integration, envelope feedback in `limits()` (descent deliberately unrestricted); the
      `BATT` command lives on the Multicopter entity and delegates at call time, like `HOVER`,
      so it survives the reset double-swap; fifth entry in the plugin's `IMPLEMENTATIONS`
- [x] Unit tests: power model vs hand-computed points; SoC monotonically decreasing;
      envelope tightens below SoC threshold (`tests/integration/test_perf.py`)
- [x] Sanity: MAVIC-class hover endurance in the 20–35 min range (≈27.7 min analytic)
- [x] `uv run pytest`, `uv run ruff check .`, `uv run pyright` all green

## Phase 4 — docs, scenarios, cleanup

- New guide `docs/guides/multicopters.md`: creating multicopters, hover/yaw commands, battery
  model, how to add a new type (rotor `aircraft.json` entry + constants dict).
- Update `docs/architecture.md` with the `Kinematics` entity and the replaceable list.
- Example scenario `packages/minisky-multicopter/scenarios/multicopter_delivery.scn`: create,
  fly a route, hover at a delivery point, yaw for "camera", return; exercises everything
  above. Lives in the plugin package because it depends entirely on the plugin.
- Regenerate `docs/reference/commands.md` (`uv run minisky commands docs`) after adding the
  stack commands (`MCOPT`, `YAW`, `YAWRATE`, `HOVER`, `BATT`).
- `ruff`, `pyright`, full test suite green at every phase boundary.

### Phase 4 checklist

- [x] `docs/guides/multicopters.md` (usage, commands, battery model, adding a new type)
- [x] Update `docs/architecture.md`: `Kinematics` entity + replaceable list
- [x] `packages/minisky-multicopter/scenarios/multicopter_delivery.scn` exercising
      create → route → hover-delivery (`HOVER` + `ALT` + `LNAV ON`) → return
- [x] Verify `docs/reference/commands.md` regeneration — the reference is now built by the
      `command_docs()` macro at site build (`just docs-build`), so there is no separate gen
      step; the table is core-only by design and the plugin commands are documented in
      `docs/guides/multicopters.md`
- [x] Final sweep: `uv run pytest`, `uv run ruff check .`, `uv run pyright`

## Sequencing and effort

| Phase | Scope | Risk | Depends on |
|---|---|---|---|
| 1 | Extract `Kinematics` (core, behaviour-preserving) | Low — mechanical move guarded by existing tests | — |
| 2 | Plugin: membership + kinematics + commands | Medium — command semantics for HDG/YAW need care | 1 |
| 3 | Perf: analytic `MulticopterPerf` + battery from shipped OpenAP data | Low–medium — model calibration/sanity | 2 (usable after 1) |
| 4 | Docs, scenario, polish | Low | 2, 3 |

Implementation lands on this branch phase by phase, checking off the checklists above as items
complete; phase 1 is intentionally the only one touching `minisky/`.

## Decision log

| Decision | Choice | Why |
|---|---|---|
| Name | **multicopter** (not drone/rotorcraft) | Names the lift/control type actually modelled; scope excludes helicopters (EC35) |
| Where behaviour lives | Plugin + replaceable subclasses | Matches "minimal core, hack from outside"; hot-swappable via `SELECTIMPL`; reverts on reset |
| Kinematics override mechanism | New first-level `Kinematics` entity (Phase 1) | `Traffic` itself can't be hot-swapped (root object); post-hoc plugin-hook correction would double-integrate state |
| Custom autopilot | Thin `MulticopterAutopilot` for the hover primitive, HDG semantics and fly-over defaults only | LNAV's track output already suits decoupled kinematics; no guidance rewrite needed |
| Mission primitive | One composable `HOVER acid [time] [alt]`; no `DELIVER` | Delivery choreography belongs in scenarios (`HOVER` + `ALT` + `LNAV ON`); keeps the primitive abstract |
| Capture radius | `MulticopterActiveWaypoint` subclass (fourth swap) | `ActiveWaypoint.reached()` recomputes `turndist` every step, so clamping it from the autopilot is overwritten before use |
| Membership predicate | Plugin-owned typecode set + `ismulticopter` array | `LIFT_ROTOR` includes helicopters |
| PyThrust | **Deferred entirely to future work** (2026-08-02; was: vendor its data + gen script) | The shipped OpenAP rotor json (mass, engine power, envelope) supports an analytic model; only battery Wh needs a small constants dict. Avoids ~1 MB vendored data and a gen script until the fidelity is actually needed |
| Perf evaluation | Analytic momentum-theory scaling, pure numpy | Keeps the numpy discipline; fleet-size independent; no artifacts to regenerate |

## Future work — PyThrust-data fidelity upgrade (deferred)

If measured-data fidelity is ever needed, the original Phase 3 design still applies and slots in
behind the same `MulticopterPerf` API (only the power/current evaluation changes):

[PyThrust](https://github.com/Setuav/PyThrust) (Apache 2.0) ships everything needed as plain
data — APC propeller performance grids (`rpm, speed_mps, thrust_n, power_w, ...`; thrust *and*
shaft power tabulated, so "required thrust at this airspeed → power" is pure interpolation),
motor specs (`kv`, `resistance`, `io` — shaft-to-electrical is textbook motor algebra), and
battery OCV/internal-resistance-vs-DoD curves. The pipeline: vendor the handful of needed
CSV/JSONs (~1 MB) under `packages/minisky-multicopter/src/minisky_multicopter/data/pythrust/`
with PyThrust's LICENSE and an attribution note; a **self-contained** numpy-only
`scripts/gen_multicopter_perf.py` (regen convention like navdb parquet) emits one ~30 KB
`(airspeed, thrust) → (power_w, current_a, feasible)` grid per typecode plus battery curves,
checked in next to the plugin; runtime evaluates with vectorised interpolation. Never a runtime
PyThrust dependency. This upgrade adds what the analytic model cannot do: current draw,
terminal-voltage sag, and envelope infeasibility driven by physical voltage collapse rather
than an SoC threshold. Fidelity caveat regardless: APC coefficients are axial-flow, so
forward-flight power in edgewise translation stays approximate.
