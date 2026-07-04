# Known issues

Latent bugs found while documenting the codebase (July 2026) were fixed in a dedicated
bug-fix pass (July 2026). Each fix is covered by a regression test in `tests/` where the
behavior is observable. This page records what was fixed, plus the small number of
remaining quirks.

## Fixed: crashes

- `tools/aero.py` — scalar `atmos()` called bare `exp(...)` in the isothermal-layer
  branch (`NameError` for altitudes in isothermal layers). Now `np.exp`.
- `traffic/route.py` — `at_wpt()` and `delwpt()` called `acrte.direct(...)`, but
  `direct` is a module-level function, not a `Route` method. Same bug existed in
  `autopilot.py` (`setLNAV` re-engage) and `asas/resolution.py` (`resumenav()`); all
  call sites now use the module-level function.
- `traffic/route.py` — `direct()` used bare `pi` (not imported) in the heading-rate
  branch. Now `math.pi`.
- `traffic/route.py` — `addwpt_before()`/`addwpt_after()` named their literal-keyword
  parameter `addwpt`, shadowing the `addwpt()` function they call (`'str' object is not
  callable`). Parameter renamed.
- `traffic/trails.py` — `update()` referenced `self.pygame` (pygame was removed) and
  `buffer()` used `self.bgacid` before `clearbg()` had run. Dead branch removed,
  `bgacid` initialised in `__init__`.
- `traffic/conditional.py` — `renameac()` referenced undefined `old.id`.
- `traffic/autopilot.py` — `setVNAV` query path referenced nonexistent
  `minisky.traf.id` and had an operator-precedence bug in the ON/OFF suffix.
- `traffic/asas/detection.py` — `sethpz()`/`setrpz()` with no argument formatted the
  whole per-aircraft array instead of the scalar default (`TypeError`).
- `stack/__init__.py` — `showhelp` `HELP >filename` branch referenced nonexistent
  `obj.parsers` and clobbered `fname` inside its loop.
- `tools/navdata.py` — `defwpt` had `lon.upper == "DELETE"` (missing call parentheses);
  `delwpt` searched `wpid` with the raw name after uppercasing only the existence check
  (`ValueError` for lowercase input).
- `traffic/turbulence.py` — a fresh `Turbulence()` had an empty `sd` array, so
  `NOISE ON` crashed after any `sim.reset()` re-instantiated it.
- `traffic/traffic.py` — `clrcrecmd()`'s `str("All", ...)` `TypeError` (already fixed
  in commit 4d144bd; regression test added).
- `traffic/asas/resolution.py` — `setmethod("OFF")` calling `switch` unbound (already
  fixed in commit e76a1a1; regression test added).

## Fixed: wrong results (silent)

- `stack/__init__.py` — the argument-spec tokenizer did not strip whitespace, so any
  registration written as `"callsign, wpt"` silently dropped parameters. This made a
  raft of commands unusable or partially usable from the stack: `AT`, `DIRECT`,
  `AFTER`, `BEFORE`, `HELP`, `LISTRTE`, `SCENARIO`, `DTLOOK`, `DTNOLOOK`, and the
  per-aircraft forms of `RESOOFF`, `NORESO`, `ZONER`, `ZONEDH`. The tokenizer now
  strips tokens, and all broken specs were repaired
  (`tests/integration/test_stack.py::TestArgumentSpecs` audits every registration).
- `stack/commands.py` — commands bound to `minisky.traf.cr` methods went stale after
  `RESO MVP` replaced the instance; they now dispatch through module-level wrappers in
  `asas/resolution.py` that resolve the current instance at call time.
- `stack/commands.py` — the `WIND` spec ran the first value through the altitude parser
  (ft→m) while `Wind.add()` expects raw values: the two-element form mangled the wind
  direction and the altitude form double-converted the first altitude. The spec now
  passes raw floats (and accepts `WIND lat,lon,DEL`).
- `tools/navdata.py` — `delwpt` discarded the results of `np.delete`, so waypoint
  coordinates were never actually removed.
- `traffic/route.py` — `at_wpt()` alt/spd branch wrote the parsed speed into the
  altitude-constraint array.
- `traffic/activewpdata.py` — `create()` had `self.nextaltco[-n] = -999.0` (missing
  colon), initialising one element instead of the slice.
- `traffic/wind.py` — `Windfield.remove()` deleted from `lat` and assigned to `lon`;
  `Wind.add()`'s DEL branch was unreachable for 3+ arguments.
- `traffic/conditional.py` — `atspdcmd` seeded the condition with TAS while `update()`
  compares CAS (the command means CAS).
- `traffic/uncertainty.py` — `update()` used `len(np.where(...))` (always 1), so one
  noise sample was broadcast to all due aircraft.
- `traffic/traffic.py` — `cre()` defaults were meant as 25000 ft / 300 kts but used as
  SI; now `25000 * ft` / `300 * kts`.
- `tools/geo.py` — `latlondist_matrix` returned NM while scalar `latlondist` returns
  metres; both now return metres (`qdrdist`/`qdrdist_matrix` return NM). Also fixed a
  wrong-shape `np.zeros` for non-square inputs.
- `traffic/asas/detection.py` — `reset()` restored a different `hpz_def` than
  `__init__` (`(asas_pzh - 1) * ft` vs `asas_pzh * ft`).
- `core/varexplorer.py` — `Variable.get()` returned `None` when no index was given.
- `stack/__init__.py` — `readscn` skipped any line shorter than 12 characters, not just
  empty/comment lines.

## Fixed: cosmetic / dead code / deprecations

- `traffic/asas/mvp.py` — `setresometh`/`setresometv` success paths returned `None`
  instead of `(True, msg)`; `setresooff` help text said "NORESO".
- `traffic/asas/resolution.py` — `f"Current CR method: "` interpolated nothing.
- `traffic/asas/detection.py` and `tools/geo.py` — migrated off deprecated
  `np.asmatrix` to plain ndarray broadcasting.
- `traffic/performance/perfoap.py` — duplicated rotorcraft assignments in `create()`
  and dead `axmax` allocation removed (the unused `Drag`/`FuelFlow` imports were
  already gone as of commit 4d144bd).
- `traffic/performance/phase.py` — boundary conditions at exactly 75 ft / 1000 ft and
  ±150 fpm now assign exactly one phase; `get()` returns an int array consistently.
- `traffic/turbulence.py` — `__init__` now calls `super().__init__()`.
- `traffic/autopilot.py` — `setswtod` status now reads `swtod` (was `swtoc`); dead
  `dist2accel` removed.
- `stack/argparser.py` — duplicate `"RIGHT"` removed from the `PandirArg` directions.
- `stack/commands.py` — dead synonyms (`ADDAWY`, `COLOUR`, `CALC`, `SWRAD`, `DTMULT`)
  removed; `AIRWAY`/`AIRWAYS` repointed to `POS`.
- `core/settings.py` — the module-level dict shadowed by the `data()` function was
  renamed to `_settings`.
- `plugin/plugin.py` — deprecated `ast` attribute `.s` replaced with
  `isinstance(node, ast.Constant)` / `.value`.
- `simulation/simulation.py` — deprecated `datetime.utcnow()` replaced with
  `datetime.now(datetime.UTC)` (kept naive, matching the rest of the module).
- Top-level `minisky/route.py` (empty, unreferenced) deleted.

## Still open (minor)

- `tools/geo.py` — the matrix variants evaluate the earth radius at `lat1 + lat2`
  while the scalar functions use `0.5 * (lat1 + lat2)`; negligible except at extreme
  latitude combinations.
- `traffic/asas/mvp.py` — `setresometh`/`setresometv` (RMETHH/RMETHV) exist only on
  the MVP subclass and are not registered as stack commands.
