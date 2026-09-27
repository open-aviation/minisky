# Upstream decisions

MiniSky is a fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) that
moves in the opposite direction: toward a bare minimum. Upstream changes are
adopted selectively. This page is a full history of every upstream PR and
mainline commit since the fork — not just the ones needing a decision — so the
question isn't reopened every time someone diffs against upstream.

This page only tracks upstream changes **after the fork baseline** — BlueSky
`849d76fd` (2024-06-12), the latest revision consistent with MiniSky's initial
source. Changes merged before that are inherited into the baseline, not
porting decisions, so they don't belong here.

When evaluating a new upstream change, check here first. Entries are grouped
into **Pending** (still needs a change here), **Resolved** (evaluated in
depth, no further action), and a **Full history log** (everything else —
one line each, oldest first). Note that BlueSky merges PRs out of number
order, so PR numbers in any section are not sequential.

If a resolved change becomes relevant again (e.g. upstream lands a follow-up
with actual new behaviour), add a new entry rather than editing the old one.

## Pending

Upstream fixes — or MiniSky-specific bugs on the same boundary — that still
require a change here.

### PR [#470](https://github.com/TUDelft-CNS-ATM/bluesky/pull/470) — improved turning / turn bank / cruise-speed restoration (merged 2024-11-18)

Upstream reworked turns: a turn-bank state (`TURNBANK`/`TURNPHI`), derived turn
parameters, and post-turn cruise-speed restoration. Partial here: MiniSky
models turn radius, turn CAS, and heading rate as typed `TurnParameters` and
**deliberately rejects** `TURNBANK`/`TURNPHI` (`route.py:778`); it has no
`CRUISESPD` post-turn speed restoration. **Decision needed:** evaluate
turn-bank state and cruise-speed policy as separate navigation features with
scenarios if wanted; do not port wholesale.

### PR [#541](https://github.com/TUDelft-CNS-ATM/bluesky/pull/541) — relicense BlueSky to MIT (merged 2025-03-05)

Upstream relicensed from GPL-3.0 to MIT. MiniSky's copied source predates the
relicensing, so it remains GPL-3.0. **Decision needed:** only with a deliberate
legal/provenance review — never treat this as a normal upstream sync.

### commit `f53ee9a` — richer heading references (committed 2025-12-19)

Upstream's heading parser gained navaid, airport, and aircraft references plus
additional leading-zero forms. MiniSky's heading grammar accepts numeric
headings with `T`/`M`/`TRK` suffixes and the `*` runway-heading sentinel, but
not navaid/airport/aircraft-as-heading references. **Decision needed:** low
priority; if added, implement through MiniSky's `CommandParseContext` rather
than a parser-global dependency.

### PR [#619](https://github.com/TUDelft-CNS-ATM/bluesky/pull/619) — graceful Windows SIGBREAK termination (merged 2026-01-15)

Upstream added Windows `SIGBREAK` handling for graceful shutdown. MiniSky's CLI
catches only `KeyboardInterrupt` (`cli.py:301`) and has a different asyncio
runner/server lifecycle. **Decision needed:** evaluate only if Windows is a
supported platform, and implement against MiniSky's lifecycle rather than
copying the patch.

### PR [#629](https://github.com/TUDelft-CNS-ATM/bluesky/pull/629) — scalar static wind fields in addpointvne (merged 2026-02-24)

Upstream normalized scalar/static inputs in `addpointvne()`. MiniSky's
`Windfield.addpointvne()` still assumes array-shaped `lat`/`lon`/`vnorth`/
`veast` — it calls `len(lat)` and indexes `vnorth[0]`/`vnorth[-1]` (`wind.py:90`)
— so scalar use can still fail. **Fix:** normalize scalar and 1-D inputs, with
tests, if this method remains supported API.

### PR [#630](https://github.com/TUDelft-CNS-ATM/bluesky/pull/630) — zero relative vertical-speed conflict sign (merged 2026-02-24)

Upstream corrected the conflict sign when relative vertical speed is exactly
zero. MiniSky's KD-tree detector floors `|dvs|` to `+1e-6` irrespective of its
sign (`conflict/detection.py:514-522`), which makes level pairs at the vertical
boundary direction-asymmetric. **Fix:** remove the positive-sign bias in the
detector; add symmetric boundary regression tests at `|dalt| == hpz` and level
flight.

### PR [#654](https://github.com/TUDelft-CNS-ATM/bluesky/pull/654) — UNKNOWN-phase minimum speed (merged 2026-08-27)

The phase-condition half of this PR is already handled: MiniSky's
`_construct_v_limits` uses an explicit `CLIMB | CRUISE | DESCENT` mask, so the
always-true `(phase >= CL) | (phase <= DE)` bug is absent. What remains is the
`FlightPhase.UNKNOWN` fallback: MiniSky sets `vmin = 0`
(`performance/openap.py:437`), while upstream falls back to the en-route minimum
to preserve stall protection. **Decision needed:** adopt `vminer` for UNKNOWN
(stall protection) or keep `vmin = 0` as a deliberate permissive choice. If
changed, add a low-altitude level-flight regression case.

### OpenAP batch typecode — MiniSky-specific, no upstream PR

`OpenAP.create(n)` applies the last aircraft's typecode to the whole appended
batch: `performance/openap.py:134` reads `self.traffic.typecode[-1].upper()`,
and `:130` carries an explicit `TODO`. This is distinct from PR #601 (case
mismatch, already handled here) but sits on the same correctness boundary.
**Fix:** per-row type initialization in `create()`.

## Resolved

Upstream PRs — and MiniSky-specific equivalents — that need no further action:
already present, superseded by MiniSky's design, deliberately rejected, or out
of scope. Each is tagged with its disposition.

### PR [#644](https://github.com/TUDelft-CNS-ATM/bluesky/pull/644) — ResumeNavigation as a replaceable class (merged 2026-07-21) — rejected

**What it does upstream:** moves `ConflictResolution.resumenav()` into a
replaceable-`Entity` `ResumeNavigation` class (past-CPA as a `PastCPA`
subclass, swapped via a new `RESNAV` command), for research extensibility.

**Why not here:** zero behavioural change — MiniSky already has the identical
past-CPA algorithm in `ConflictResolution.resumenav()` — and the seam depends
on the replaceable-`Entity` registry MiniSky deliberately removed.

**If the need arises:** `resumenav()` is already an overridable method — a CR
subclass or plugin can replace the resume policy with no new infrastructure.
If upstream's FTR algorithm becomes useful, port *that algorithm* as a
`resumenav` override, not the class scaffolding.

### PR [#650](https://github.com/TUDelft-CNS-ATM/bluesky/pull/650) — C geo module fixes (merged 2026-07-21) — out of scope

Fixes to the compiled C geo extension. The extension was removed from MiniSky;
current Python pair geometry normalizes bearings and wraps longitude
differences. Do not reintroduce native geo solely for parity.

### PR [#653](https://github.com/TUDelft-CNS-ATM/bluesky/pull/653) — vsmin from descent envelopes (merged 2026-08-26) — present

Upstream derived `vsmin` from *climb* vertical-speed envelopes; the fix uses
the descent envelopes. MiniSky's `coefficients.py` already does (its extra
`initclimb_vs` term is a positive climb rate that never wins the `min()`).

### PR [#656](https://github.com/TUDelft-CNS-ATM/bluesky/pull/656) — Free-to-Revert (FTR) resume navigation (merged 2026-08-26) — rejected

The FTR follow-up anticipated in the #644 entry above.

**What it does upstream:** adds an `FTR` subclass of `ResumeNavigation`: revert
to autopilot only when the forward CPA of the ownship's desired velocity
against the intruder clears the protected zone, with a second intent-based
criterion selected via a new `FTRINTENT` command (`OFF`/`ASSUMED`/`DECLARED`).
Opt-in via `RESNAV FTR`; upstream's default stays past-CPA.

**Why not here:** research policy, not a bug fix — no shared behaviour changes
— and it hangs off the replaceable-`ResumeNavigation` registry rejected with
#644.

**If the need arises:** the algorithm (forward-CPA `clears()` test,
wind-triangle desired velocity, assumed-intent dict) is self-contained and
ports cleanly as a `resumenav()` override in a plugin package.

### PR [#647](https://github.com/TUDelft-CNS-ATM/bluesky/pull/647) — Qt clipboard paste (merged 2026-08-27) — out of scope

Qt clipboard-paste handling. The Qt console was removed from MiniSky; there is
nothing to apply it to.

### PR [#658](https://github.com/TUDelft-CNS-ATM/bluesky/pull/658) — missing nm conversion in RTA leg time (merged 2026-08-28) — superseded

Upstream divided a distance in nautical miles (`wpdistto`) by a speed in m/s
(`legtas`) when computing `legtime` for an RTA-constrained leg. Not applicable:
MiniSky's `route.py` stores `wpdistto` in metres throughout (SI units), so the
same division is already dimensionally correct.

### PR [#659](https://github.com/TUDelft-CNS-ATM/bluesky/pull/659) — OpenAP speed limits at intended altitude (merged 2026-08-29) — present

Upstream evaluated the CAS/Mach speed envelope at the *intended* altitude
(`allow_h`) instead of the current one. MiniSky's `OpenAP.limits()` already
evaluates the envelope at `self.traffic.alt` (current altitude).

### PR [#661](https://github.com/TUDelft-CNS-ATM/bluesky/pull/661) — MVP vertical resolution direction (merged 2026-09-01) — present

`MVP.MVP()`'s vertical resolution direction was symmetric (not dependent on
`drel`/`vrel`) when both aircraft of a pair shared the same vertical rate, so a
co-altitude conflict pair could be pushed the same way instead of apart. Fixed
in `mvp.py` (`mvp.py:440-446`) to fall back to relative altitude, then aircraft
index, as an antisymmetric tiebreaker, matching upstream's fix. (Upstream's
other fix in the same PR — an `asasalttemp` sentinel blowing up for
non-conflicting aircraft — was already avoided here via the
`has_resolution_time` mask in `MVP.resolve()`.)

### PR [#663](https://github.com/TUDelft-CNS-ATM/bluesky/pull/663) — geo backend consistency (merged 2026-09-24) — present

Most of this PR doesn't apply: the compiled C geo extension and the
`prefer_compiled`/backend-selection machinery it fixes were already removed
from MiniSky, same as #650.

The Python-only accuracy fixes were real, independently-reproduced bugs in
`geo.py` and were ported directly:
- `qdrdist` used the law-of-cosines distance formula, which loses all
  precision at short range (two points 10 cm apart came out 0 m apart).
  Switched to the haversine formula already used by `qdrdist_matrix`.
- `latlondist`'s different-hemisphere branch divided by
  `abs(lat1) + abs(lat2)` with no floor, so two points on the equator (both
  latitudes 0) produced `nan`. Added the same `np.maximum(0.000001, ...)`
  floor `qdrdist`'s equivalent branch already had.
- `kwikdist_matrix`/`kwikqdrdist_matrix` never got the `atleast_2d`
  row/column broadcasting treatment `qdrdist_matrix`/`latlondist_matrix` did,
  so two differently-sized position vectors silently produced a
  wrong-shaped, elementwise result instead of an `(n1, n2)` matrix.
  `kwikdist_matrix` also averaged the wrong pair of latitudes
  (`lata + latb.T` instead of `lata.T + latb`); both fixed to build 2-D row
  arrays first, matching upstream's fix.

Upstream's other fixes (matrix earth radius at mean vs. sum of latitudes,
the different-hemisphere divisor guard in the matrix variants) were already
correct here, in the same vein as #653.

### PR [#664](https://github.com/TUDelft-CNS-ATM/bluesky/pull/664) — wrap aircraft positions at the poles and antimeridian (merged 2026-09-24) — present

`Kinematics.update_pos` integrated latitude/longitude with a flat Euler step
and never normalised the result, so longitude drifted unbounded past ±180° and
latitude past ±90° (division by zero at the poles, broken guidance near them;
independently found in
[minisky#14](https://github.com/open-aviation/minisky/issues/14)). Fixed to
wrap longitude into `[-180, 180)` every step, and past a pole mirror latitude
and flip heading, track, autopilot track, active leg direction and ground speed
components 180°, since local north/east point the other way there, matching
upstream's fix.

### PR [#665](https://github.com/TUDelft-CNS-ATM/bluesky/pull/665) — remove unused SSD.detect (merged 2026-09-24) — out of scope

Nothing to apply it to: the SSD/ASAS plugin was already removed from MiniSky
entirely, same bucket as #647/#650.

## Full history log

Everything else since the fork baseline — no decision needed, one line each,
oldest first. PRs and mainline commits already covered above (Pending or
Resolved) are omitted here to avoid duplicating their reasoning.

| Date | BlueSky | Change | Status |
| --- | --- | --- | --- |
| 2024-07-25 | [#518](https://github.com/TUDelft-CNS-ATM/bluesky/pull/518) | Strip inline scenario comments | improved — `readscn()` already strips comments + sorts stably |
| 2024-09-02 | `56ae5de` | NumPy 2.0 GUI/dependency compat | no action |
| 2024-09-02 | `c9c797e` | Reorganize geo module, selective compiled backend | superseded — C-geo removed |
| 2024-09-04 | [#526](https://github.com/TUDelft-CNS-ATM/bluesky/pull/526) | GitHub Actions artifact update | no action — independent CI |
| 2024-12-03 | [#535](https://github.com/TUDelft-CNS-ATM/bluesky/pull/535) | Remove debug print from new turn impl | out of scope — only relevant to #470 |
| 2024-12-17 | [#538](https://github.com/TUDelft-CNS-ATM/bluesky/pull/538) | Switch build backend to Hatch | improved — own uv workspace + Hatchling |
| 2024-12-17 | `093b033`, `17ca78a` | Wheel-build workflow setup/updates | no action |
| 2024-12-18 | [#516](https://github.com/TUDelft-CNS-ATM/bluesky/pull/516) | Qt settings-window ESC fix | out of scope — Qt removed |
| 2024-12-18 | `fdf2f23` | Dynamic versioning | no action |
| 2025-02-10 | `b04f0d7` | Remove explicit NumPy version dependency | no action |
| 2025-02-10 | `3bf6438` | Remove Qt5 compatibility | out of scope |
| 2025-02-17 | `f5f2368` | README update | no action |
| 2025-03-07 | [#546](https://github.com/TUDelft-CNS-ATM/bluesky/pull/546) | BlueSky network overhaul | improved — own runtime ownership + REST/WS/Tangram boundaries |
| 2025-03-07 | `00ecfba`, `798f782`, `f89c6a2` | Wheel architecture/build-target changes | no action |
| 2025-03-07 | `a023ecb` | `.gitignore` update | no action |
| 2025-03-10 | `4dd4a6c` | Print client traceback to terminal | out of scope / improved — own result/console/server error paths |
| 2025-03-10 | `3f9735d`, `98f82db` | Version BlueSky data packages | improved — own `minisky-xplane-navdata` package |
| 2025-03-10/11 | `7fcd4d6`, `263827a` | Client/Node restructuring | out of scope |
| 2025-03-11 | `b61e01f` | Package metadata | no action |
| 2025-03-13 | [#553](https://github.com/TUDelft-CNS-ATM/bluesky/pull/553) | Windows Qt dark/light mode | out of scope |
| 2025-03-14 | `3c0502f` | Subscription construction fix | out of scope — shared-state/network removed |
| 2025-03-14 | `baaaf6b` | Print exceptions from stack processing | improved — typed `Result`/parser diagnostics |
| 2025-03-14 | `9555bfe`, `3ede7a6` | Client example/shared-state action fixes | out of scope |
| 2025-03-14 | `ee4d270` | Accept flight-level altitude in WIND profiles | improved — typed `WindLevel` already accepts `FL...`/`StdPressureAltM` |
| 2025-03-16 | `4a598f2` | Fix function unwrap typo | superseded — command/decorator impl replaced |
| 2025-03-19 | `b48f2ee` | Detect malformed `init_plugin()` | improved — scanning removed; entry points + `Plugin` contracts |
| 2025-03-19 | `bc227d2` | Add diagnostics to old function-object stub | superseded |
| 2025-03-24 | `f76deee` | ADS-B plugin refactor | out of scope |
| 2025-03-24 | `04d53ff` | Add optimization plugin | out of scope / evaluate — only with a concrete research use case |
| 2025-03-25/04-09 | `9455731`, `71e47e4`, `2a68109` | README updates | no action |
| 2025-04-10 | [#561](https://github.com/TUDelft-CNS-ATM/bluesky/pull/561) | BlueSky client-mode bug fix | out of scope |
| 2025-04-12 | `c53aa47` | `stackcheck` bundled-plugin update | out of scope |
| 2025-04-12 | `1a0e124` | TODO update | no action |
| 2025-04-12 | `8c96fa5` | `trafgen` bundled-plugin update | out of scope |
| 2025-04-12 | `46e9442`, `79bc50c` | Python geo lint/cleanup | superseded — geo code diverged substantially |
| 2025-04-14 | `1db5a58` | README update | no action |
| 2025-05-09 | `d16a758` | Distributed batch-mode fix | out of scope |
| 2025-05-09/12 | `c856458`, `ecee293`, `eea5978` | Network/client typing and subscription fixes | out of scope |
| 2025-05-12 | `a22c6bf` | Simulation timer cleanup | improved — own runtime/plugin hook scheduling |
| 2025-05-16 | `f983aff` | Avoid GUI import errors | out of scope |
| 2025-05-18 | `054aab3` | Union command-argument specifications | improved — derives alternatives from Python unions/`Annotated` |
| 2025-05-18 | `718ee09` | Reintroduce `SWRAD` | out of scope — display command belongs to removed GUI |
| 2025-05-19 | `8ca4d67`, `6abdddb` | Shared-state and Qt zoom fixes | out of scope |
| 2025-05-19 | `15cff5f` | Reduce BADA warnings | out of scope |
| 2025-05-22 | [#567](https://github.com/TUDelft-CNS-ATM/bluesky/pull/567) | Aircraft synonym fixes | improved — reads from maintained `openap` package |
| 2025-05-22 | `ca27273` | Phase out `bs.scr` | improved — already separates console/output from GUI state |
| 2025-06-16 | `9886114`, `b527bc1` | Data-package README updates | no action |
| 2025-06-17 | [#571](https://github.com/TUDelft-CNS-ATM/bluesky/pull/571) | `np.frombuffer` network codec fix | out of scope — network codec removed |
| 2025-07-06 | [#577](https://github.com/TUDelft-CNS-ATM/bluesky/pull/577) | Correct pathfinder workdir type | out of scope — node launcher removed |
| 2025-07-06 | [#578](https://github.com/TUDelft-CNS-ATM/bluesky/pull/578) | Fix NumPy truth-value conversion in areafilter | improved — own `Shapes` + Shapely geometry |
| 2025-07-06 | [#579](https://github.com/TUDelft-CNS-ATM/bluesky/pull/579) | Set node workdir from server | out of scope |
| 2025-07-06 | [#580](https://github.com/TUDelft-CNS-ATM/bluesky/pull/580) | Qt stylesheet resource path | out of scope |
| 2025-08-11 | [#584](https://github.com/TUDelft-CNS-ATM/bluesky/pull/584) | Fix old turn-speed bookkeeping | implemented — same fix already in `bc9a00c` |
| 2025-08-11 | [#587](https://github.com/TUDelft-CNS-ATM/bluesky/pull/587) | Fix lat/lon text conversion | implemented — already in `ea00cf9` |
| 2025-08-14 | [#585](https://github.com/TUDelft-CNS-ATM/bluesky/pull/585) | Read OpenAP coefficients from `openap` package | improved — own OpenAP dependency |
| 2025-08-18 | [#589](https://github.com/TUDelft-CNS-ATM/bluesky/pull/589) | Delete all waypoint metadata in `DELWPT` | implemented — already in `f2ed001` |
| 2025-08-25 | [#592](https://github.com/TUDelft-CNS-ATM/bluesky/pull/592) | Optional speed constraint in `DEST` | implemented — already in `85314fe` |
| 2025-08-27 | [#594](https://github.com/TUDelft-CNS-ATM/bluesky/pull/594) | Remove self/cls from Qt console hints | out of scope |
| 2025-08-27 | [#595](https://github.com/TUDelft-CNS-ATM/bluesky/pull/595) | Deselect routes in GUI | out of scope — GUI concern |
| 2025-10-02 | [#601](https://github.com/TUDelft-CNS-ATM/bluesky/pull/601) | Stop valid fixed-wing types falling back to B744 on case mismatch | implemented, but exposed a distinct MiniSky bug — see OpenAP batch typecode in Pending |
| 2025-10-02 | [#603](https://github.com/TUDelft-CNS-ATM/bluesky/pull/603) | Remove copied OpenAP fixed-wing resources | improved — `openap` package is authoritative |
| 2025-10-02 | [#604](https://github.com/TUDelft-CNS-ATM/bluesky/pull/604) | Fix PERFSTATS float formatting | improved — rewritten around current perf/result system |
| 2025-10-11 | [#606](https://github.com/TUDelft-CNS-ATM/bluesky/pull/606) | Stable timestamp ordering for scenarios | implemented |
| 2025-10-11 | [#608](https://github.com/TUDelft-CNS-ATM/bluesky/pull/608) | Use min vertical-rate envelope when limiting descent | implemented |
| 2025-10-13 | [#610](https://github.com/TUDelft-CNS-ATM/bluesky/pull/610) | Replace deprecated `np.mat` | implemented / superseded |
| 2025-12-18 | `7ded459` | Python 3.14 `ast.Constant.value` scanner fix | superseded — old plugin scanner gone |
| 2025-12-18 | `6590c3c` | Merge accumulated upstream history onto first-parent | no independent semantic change |
| 2025-12-19 | `3436767` | Windows setup batch script | no action |
| 2026-01-16 | [#620](https://github.com/TUDelft-CNS-ATM/bluesky/pull/620) | Separate shape classes from areafilter | improved — already split into runtime-owned `Shapes` |
| 2026-01-26 | `4b46602` | Windows setup package-install update | no action |
| 2026-01-27 | `27d2842` | Network config-template update | out of scope |
| 2026-02-03 | [#622](https://github.com/TUDelft-CNS-ATM/bluesky/pull/622) | Geofence plugin fix | out of scope — geofence plugin not shipped |
| 2026-02-21 | [#623](https://github.com/TUDelft-CNS-ATM/bluesky/pull/623) | State-based conflict and C-geo fixes | partial/superseded — C-geo & old statebased removed; remaining semantics tracked by PR #630 (Pending) |
| 2026-03-04 | [#632](https://github.com/TUDelft-CNS-ATM/bluesky/pull/632) | NumPy dependency constraint | no action |
| 2026-03-09 | [#635](https://github.com/TUDelft-CNS-ATM/bluesky/pull/635) | Qt console Ctrl+V handling | out of scope |
| 2026-03-18 | [#638](https://github.com/TUDelft-CNS-ATM/bluesky/pull/638) | BADA parser fix | out of scope — BADA not in core; re-evaluate if it returns as a plugin |
| 2026-04-08 | [#645](https://github.com/TUDelft-CNS-ATM/bluesky/pull/645) | `DCT` alias for `DIRECT` | implemented — already in `6852818` |
| 2026-05-18 | `f343a98` | Wind support in BlueSky optimize plugin | out of scope / evaluate — only if an optimization plugin is built |
| 2026-07-03 | `af637ca`, `7cd5dd4` | Add then revert Read the Docs config | no action — net zero |
