# Known issues

Latent bugs found while documenting the codebase (July 2026). None of these were fixed
during the documentation pass — docstrings only. Listed here so they can be picked off
one by one.

## Crashes waiting to happen

- `tools/aero.py` — scalar `atmos()` calls bare `exp(...)` in the isothermal-layer
  branch, but only `numpy as np` is imported → `NameError` for altitudes in isothermal
  layers (e.g. 11–20 km).
- `traffic/route.py` — `addwpt()` with a string callsign calls
  `minisky.traf.callsign.idx(ac)`; lists have no `.idx` (would raise `AttributeError`).
- `traffic/route.py` — `at_wpt()` and `delwpt()` call `acrte.direct(...)`, but `direct`
  is a module-level function, not a `Route` method.
- `traffic/route.py` — `direct()` uses bare `pi` (not imported) in the heading-rate
  branch.
- `traffic/trails.py` — `update()` references `self.pygame`, which is never defined
  (pygame was removed); `buffer()` uses `self.bgacid`, which only exists after
  `clearbg()` has run.
- `traffic/traffic.py` — `clrcrecmd()` calls `str("All", ncrecmd, "...")`, a `TypeError`
  when the pending-creation list is non-empty.
- `traffic/conditional.py` — `renameac()` references undefined `old.id`.
- `traffic/autopilot.py` — `setVNAV` query path references nonexistent
  `minisky.traf.id` (should be `callsign`).
- `traffic/asas/resolution.py` — `setmethod("OFF")` calls
  `ConflictResolution.switch(False)`, passing `False` as `self` → `AttributeError`.
- `traffic/asas/detection.py` — `sethpz()` with no argument formats the whole per-aircraft
  array instead of `hpz_def`.
- `stack/__init__.py` — `showhelp` `HELP >filename` branch references nonexistent
  `obj.parsers` (should be `params`) and clobbers `fname` inside its loop.
- `tools/navdata.py` — `defwpt` has `lon.upper == "DELETE"` (missing call parentheses).

## Wrong results (silent)

- `tools/navdata.py` — `delwpt` does not assign the results of
  `np.delete(self.wplat, idx)` / `np.delete(self.wplon, idx)`, so waypoint coordinates
  are never actually removed.
- `traffic/route.py` — `at_wpt()` alt/spd branch assigns
  `acrte.wpalt[wpidx] = txt2spd(spdtxt)`: a speed written into the altitude constraint.
- `traffic/activewpdata.py` — `create()` has `self.nextaltco[-n] = -999.0` (missing
  colon), initialising one element instead of the slice.
- `traffic/wind.py` — `Windfield.remove()` has `self.lon = np.delete(self.lat, idx)`:
  deletes from `lat` and assigns to `lon`.
- `traffic/conditional.py` — `atspdcmd` seeds the condition with TAS while `update()`
  compares against CAS.
- `traffic/uncertainty.py` — `update()` sets `nup = len(up)` where `up` is the tuple
  returned by `np.where` (always 1), so one noise value is broadcast to all due aircraft.
- `traffic/traffic.py` — `cre()` defaults `alt=25000, spd=300` look like ft/kts but are
  used as SI (m, m/s).
- `tools/geo.py` — scalar `latlondist` returns metres while `latlondist_matrix` returns
  NM.
- `traffic/asas/detection.py` — `reset()` uses `hpz_def = (asas_pzh - 1) * ft` while
  `__init__` uses `asas_pzh * ft`: inconsistent defaults.
- `core/varexplorer.py` — `Variable.get()` returns `None` when no index is given.
- `stack/__init__.py` — `readscn` skips any line shorter than 12 characters, not just
  empty/comment lines.

## Cosmetic / dead code

- `traffic/asas/mvp.py` — `setresometh`/`setresometv` success paths return `None`
  instead of a `(True, msg)` tuple; `setresooff` help text says "NORESO".
- `traffic/asas/resolution.py` — `f"Current CR method: "` interpolates nothing.
- `traffic/performance/perfoap.py` — unused `Drag`, `FuelFlow` imports; duplicated
  rotorcraft assignments in `create()`; dead `axmax = np.zeros(...)` in `calc_axmax()`.
- `traffic/performance/phase.py` — overlapping boundary conditions at exactly 75 ft /
  1000 ft in `get_fixwing()`; `get()` returns floats where ints are produced upstream.
- `traffic/turbulence.py` — `__init__` does not call `super().__init__()`.
- `traffic/autopilot.py` — `setswtod` status output reads `self.swtoc`; `dist2accel` is
  initialised but never computed or used.
- `traffic/wind.py` — `Wind.add()`'s `"DEL"` branch is unreachable for the documented
  3+ argument form.
- `stack/argparser.py` — `PandirArg` valid-directions tuple contains `"RIGHT"` twice.
- `stack/commands.py` — several synonym keys (`ADDAWY`, `AIRWAY`, `COLOUR`, `CALC`,
  `SWRAD`, `DTMULT`) point at commands that don't exist (lookups just miss).
- `core/settings.py` — the `data()` function shadows the module-level `data` dict.
- `tools/convert.py` — `degto180` and `deg180` are duplicate implementations.
- `tools/geo.py` — `qdrdist_matrix`/`latlondist_matrix` use deprecated `np.asmatrix`.
- `plugin/plugin.py` — uses deprecated `ast` attribute `.s` (Python 3.14 removal
  warning).
- Top-level `minisky/route.py` is an empty file.
