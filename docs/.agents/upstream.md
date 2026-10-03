# Upstream log

MiniSky is a fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) that
moves in the opposite direction: toward a bare minimum. Upstream changes are
adopted selectively. This page is a chronological log of **every** upstream PR
and mainline commit since the fork, each with its MiniSky status, so the
question isn't reopened every time someone diffs against upstream.

- **Baseline:** BlueSky `849d76fd` (2024-06-12), the latest revision consistent
  with MiniSky's initial source. Anything merged before it is inherited, not a
  porting decision, and doesn't belong here.
- **Synced through:** BlueSky `1a0886f` (2026-09-28).

## Status

| | Meaning |
| --- | --- |
| 🔴 | Needs triage, a decision, or a fix that hasn't landed in MiniSky yet. |
| 🟢 | Resolved in MiniSky: ported, already present, or made moot by MiniSky's own implementation of the same component. The MiniSky column names the PR/commit. |
| 🔵 | Not applicable: the code it touches doesn't exist in MiniSky (Qt GUI, networking/nodes, C++ geo, BADA, bundled plugins, upstream CI/packaging/docs), or it was deliberately declined. |

## Maintaining the log

- One row per upstream PR merge or direct commit to `master`, newest first;
  add new rows at the top of the current year's table and bump **Synced
  through**. Commits inside a PR branch are covered by the PR's row.
- Rows are ordered by the date a change landed upstream, not by PR number.
- BlueSky column: `#N` is a BlueSky PR, a hash is a BlueSky commit. MiniSky
  column: `#N` is a MiniSky PR, a hash is a MiniSky commit on `main`.
- When a 🔴 row is resolved, flip it to 🟢 or 🔵 and fill in the MiniSky
  column. If a resolved change becomes relevant again (e.g. upstream lands a
  follow-up with new behaviour), that follow-up gets its own row; don't reopen
  the old one.
- File/symbol references in notes rot quickly; name the symbol, not the line.

## 2026

| | Date | BlueSky | Change | MiniSky | Notes |
| --- | --- | --- | --- | --- | --- |
| 🔵 | 2026-09-28 | [#668](https://github.com/TUDelft-CNS-ATM/bluesky/pull/668) | Fix `CIRCLE12` scenario: enable conflict detection and resolution | — | Scenario not shipped. |
| 🟢 | 2026-09-24 | [#663](https://github.com/TUDelft-CNS-ATM/bluesky/pull/663) | Geo backend consistency and Python geo accuracy fixes | [#63](https://github.com/open-aviation/minisky/pull/63) `f74224e` | Ported the Python fixes: haversine in `qdrdist` (short-range precision), equator `nan` floor in `latlondist`, `(n1, n2)` broadcasting and latitude averaging in `kwikdist_matrix`/`kwikqdrdist_matrix`. The backend-selection half doesn't apply (no compiled geo, [#35](https://github.com/open-aviation/minisky/pull/35)); the matrix earth-radius and hemisphere-divisor fixes were already correct. |
| 🟢 | 2026-09-24 | [#664](https://github.com/TUDelft-CNS-ATM/bluesky/pull/664) | Wrap aircraft positions at the poles and antimeridian | [#63](https://github.com/open-aviation/minisky/pull/63) `2dbfd4a` | Ported: `Kinematics.update_pos` wraps longitude into `[-180, 180)` every step and, past a pole, mirrors latitude and flips heading, track, autopilot track, active leg direction and ground-speed components. Closes MiniSky issue [#14](https://github.com/open-aviation/minisky/issues/14). |
| 🔵 | 2026-09-24 | [#665](https://github.com/TUDelft-CNS-ATM/bluesky/pull/665) | Remove unused `SSD.detect` | — | No SSD resolver. |
| 🟢 | 2026-09-01 | [#661](https://github.com/TUDelft-CNS-ATM/bluesky/pull/661) | MVP vertical resolution direction for co-altitude pairs with equal vertical rate | [#57](https://github.com/open-aviation/minisky/pull/57) `66a0e38`, `55e91dc` | Ported: falls back to relative altitude, then aircraft index, as an antisymmetric tiebreaker. The PR's other fix (`asasalttemp` sentinel for non-conflicting aircraft) was already avoided by the `has_resolution_time` mask in `MVP.resolve()`. |
| 🟢 | 2026-08-29 | [#659](https://github.com/TUDelft-CNS-ATM/bluesky/pull/659) | Evaluate OpenAP speed limits at the current altitude instead of the intended one | [#52](https://github.com/open-aviation/minisky/pull/52) `c93774a` | Same fix; closed MiniSky issue [#33](https://github.com/open-aviation/minisky/issues/33). |
| 🟢 | 2026-08-28 | [#658](https://github.com/TUDelft-CNS-ATM/bluesky/pull/658) | Missing nm → m conversion in RTA leg time | [#47](https://github.com/open-aviation/minisky/pull/47) `66a8c93` | Bug absent: route distances (`wpdistto`) are stored in metres throughout, so the division by a speed in m/s is dimensionally correct. |
| 🔴 | 2026-08-27 | [#654](https://github.com/TUDelft-CNS-ATM/bluesky/pull/654) | OpenAP phase envelope: fix always-true en-route phase condition; `UNKNOWN` phase falls back to the en-route minimum speed | — | The phase-condition half is already handled (explicit `CLIMB \| CRUISE \| DESCENT` mask). Open: for `FlightPhase.UNKNOWN` MiniSky sets `vmin = 0` while upstream uses `vminer` to keep stall protection. Decision needed: adopt `vminer` or keep `0` as a deliberate permissive choice; if changed, add a low-altitude level-flight regression case. |
| 🔵 | 2026-08-27 | [#647](https://github.com/TUDelft-CNS-ATM/bluesky/pull/647) | Qt clipboard paste | — | No Qt GUI. Evaluation recorded in [#51](https://github.com/open-aviation/minisky/pull/51). |
| 🟢 | 2026-08-26 | [#653](https://github.com/TUDelft-CNS-ATM/bluesky/pull/653) | Derive `vsmin` from descent (not climb) vertical-speed envelopes | [#2](https://github.com/open-aviation/minisky/pull/2) `ef074c7` | Already present: the coefficient loader uses the descent envelopes. Evaluation recorded in [#51](https://github.com/open-aviation/minisky/pull/51). |
| 🔵 | 2026-08-26 | [#656](https://github.com/TUDelft-CNS-ATM/bluesky/pull/656) | Free-to-Revert (FTR) resume navigation with `FTRINTENT` | [#51](https://github.com/open-aviation/minisky/pull/51) | Declined. Opt-in research policy (upstream's default stays past-CPA) built on the #644 scaffolding. If needed, the algorithm (forward-CPA `clears()` test, wind-triangle desired velocity, assumed-intent dict) is self-contained and ports as a `resumenav()` override in a plugin package. |
| 🔵 | 2026-07-21 | [#644](https://github.com/TUDelft-CNS-ATM/bluesky/pull/644) | `ResumeNavigation` as a replaceable class (`PastCPA` subclass, `RESNAV` command) | [#21](https://github.com/open-aviation/minisky/pull/21) | Declined. Zero behavioural change (MiniSky has the identical past-CPA algorithm in `ConflictResolution.resumenav()`), and it hangs off upstream's replaceable-`Entity` registry. `resumenav()` is already overridable, so a CR subclass or plugin can swap the resume policy without new infrastructure. |
| 🔵 | 2026-07-21 | [#650](https://github.com/TUDelft-CNS-ATM/bluesky/pull/650) | C geo module fixes | [#35](https://github.com/open-aviation/minisky/pull/35) | Compiled geo extension removed; Python pair geometry already normalises bearings and wraps longitude differences. Don't reintroduce native geo for parity. Evaluation recorded in [#51](https://github.com/open-aviation/minisky/pull/51). |
| 🔵 | 2026-07-03 | `7cd5dd4` | Revert the Read the Docs config | — | Upstream docs; net zero with `af637ca`. |
| 🔵 | 2026-07-03 | `af637ca` | Add Read the Docs config | — | Upstream docs; reverted by `7cd5dd4`. |
| 🔵 | 2026-05-18 | `f343a98` | Wind support in the optimization plugin | — | Bundled plugin not shipped (see `04d53ff`). |
| 🟢 | 2026-04-08 | [#645](https://github.com/TUDelft-CNS-ATM/bluesky/pull/645) | `DCT` alias for `DIRECT` | [#44](https://github.com/open-aviation/minisky/pull/44) `6852818` | Present. |
| 🔵 | 2026-03-18 | [#638](https://github.com/TUDelft-CNS-ATM/bluesky/pull/638) | BADA parser fix | — | No BADA in core; re-evaluate if it returns as a plugin. |
| 🔵 | 2026-03-09 | [#635](https://github.com/TUDelft-CNS-ATM/bluesky/pull/635) | Qt console Ctrl+V handling | — | No Qt GUI. |
| 🔵 | 2026-03-04 | [#632](https://github.com/TUDelft-CNS-ATM/bluesky/pull/632) | NumPy dependency constraint | — | Upstream dependency metadata. |
| 🔴 | 2026-02-24 | [#630](https://github.com/TUDelft-CNS-ATM/bluesky/pull/630) | Conflict sign when relative vertical speed is exactly zero | — | MiniSky's KD-tree detector floors `\|dvs\|` to `+1e-6` irrespective of sign (`vertical_interval` in `ConflictDetection`), which makes level pairs at the vertical boundary direction-asymmetric. Fix: remove the positive-sign bias; add symmetric boundary regression tests at `\|dalt\| == hpz` in level flight. |
| 🔴 | 2026-02-24 | [#629](https://github.com/TUDelft-CNS-ATM/bluesky/pull/629) | Scalar static wind fields in `addpointvne()` | — | `Windfield.addpointvne()` still assumes array-shaped `lat`/`lon`/`vnorth`/`veast` (it takes `len(windalt)` and indexes `vnorth[0]`/`vnorth[-1]`), so scalar use can fail. Fix: normalise scalar and 1-D inputs, with tests, if this stays supported API. |
| 🔴 | 2026-02-21 | [#623](https://github.com/TUDelft-CNS-ATM/bluesky/pull/623) | State-based detection: no vertical crossing for zero relative vertical speed; C++ `kwikqdr` reverse bearing | — | The C++ geo half doesn't apply ([#35](https://github.com/open-aviation/minisky/pull/35)). The vertical-crossing half is the same boundary #630 reworked; resolve the two together. |
| 🔵 | 2026-02-03 | [#622](https://github.com/TUDelft-CNS-ATM/bluesky/pull/622) | Geofence plugin fix | — | Bundled plugin not shipped. |
| 🔵 | 2026-01-27 | `27d2842` | Bring config template in line with network changes | — | No networking. |
| 🔵 | 2026-01-26 | `4b46602` | Windows setup script: package installation update | — | Upstream tooling. |
| 🟢 | 2026-01-16 | [#620](https://github.com/TUDelft-CNS-ATM/bluesky/pull/620) | Separate shape classes from areafilter | [#42](https://github.com/open-aviation/minisky/pull/42) `9a8bf24` | Runtime-owned `Shapes` with separate areas and lines. |
| 🔴 | 2026-01-15 | [#619](https://github.com/TUDelft-CNS-ATM/bluesky/pull/619) | Graceful termination on Windows `SIGBREAK` | — | MiniSky's CLI only catches `KeyboardInterrupt` and has a different asyncio runner/server lifecycle. Decision needed: evaluate only if Windows is a supported platform, and implement against MiniSky's lifecycle rather than copying the patch. |

## 2025

| | Date | BlueSky | Change | MiniSky | Notes |
| --- | --- | --- | --- | --- | --- |
| 🔵 | 2025-12-19 | `3436767` | Windows setup batch script | — | Upstream tooling. |
| 🔴 | 2025-12-19 | `f53ee9a` | Heading arguments accept navaid, airport and aircraft references, plus leading zeroes | — | MiniSky's heading grammar accepts numeric headings with `T`/`M`/`TRK` suffixes and the `*` runway-heading request, but no navaid/airport/aircraft-as-heading references. Decision needed, low priority; if added, implement through `CommandParseContext` rather than parser-global state. |
| 🔵 | 2025-12-18 | `6590c3c` | Merge of the long-lived side branch into `master` | — | Merge commit, no change of its own; its contents are the 2025-06-17 to 2025-11-20 rows below. |
| 🟢 | 2025-12-18 | `7ded459` | Python 3.14: `ast.Constant.s` → `.value` in the plugin scanner | [#36](https://github.com/open-aviation/minisky/pull/36) `768c0d7` | Moot: AST plugin scanner replaced by entry points. |
| 🔵 | 2025-11-20 | `ad63ee4` | Don't sort scenario lines when loading batch files (fix for #606) | — | No batch-file mode, so the sort from #606 has nothing to break. |
| 🔵 | 2025-11-20 | `df3b107` | `sharedstate.py` update | — | No networking/shared state. |
| 🔵 | 2025-11-20 | `dfa24d9` | Doc edit for a stack function argument in the Qt main window | — | No Qt GUI. |
| 🟢 | 2025-10-13 | [#610](https://github.com/TUDelft-CNS-ATM/bluesky/pull/610) | Replace deprecated `np.mat` | `11b112c` | Ported; matrix types have since been removed entirely. |
| 🟢 | 2025-10-11 | [#606](https://github.com/TUDelft-CNS-ATM/bluesky/pull/606) | Order scenario commands by timestamp (stable) | [#44](https://github.com/open-aviation/minisky/pull/44) `44f78ac` | Scenario commands are stably sorted by time. |
| 🟢 | 2025-10-11 | [#608](https://github.com/TUDelft-CNS-ATM/bluesky/pull/608) | Use the minimum vertical-rate envelope when limiting descent | [#1](https://github.com/open-aviation/minisky/pull/1) `c5eefaa` | Ported (`vs_min_with_acc`). |
| 🟢 | 2025-10-02 | [#601](https://github.com/TUDelft-CNS-ATM/bluesky/pull/601) | Stop valid fixed-wing types falling back to B744 on a case mismatch | [#2](https://github.com/open-aviation/minisky/pull/2) `ef074c7` | Ported; type codes are now normalised to upper case. A distinct MiniSky-only bug on the same boundary is still open and has no upstream counterpart: `OpenAP.create(n)` applies the last aircraft's typecode to the whole appended batch (see the `TODO` there). |
| 🟢 | 2025-10-02 | [#603](https://github.com/TUDelft-CNS-ATM/bluesky/pull/603) | Remove copied OpenAP fixed-wing resources | [#2](https://github.com/open-aviation/minisky/pull/2) `ef074c7` | The `openap` package is authoritative. |
| 🟢 | 2025-10-02 | [#604](https://github.com/TUDelft-CNS-ATM/bluesky/pull/604) | Fix `PERFSTATS` float formatting | [#1](https://github.com/open-aviation/minisky/pull/1) `c5eefaa` | Ported; since rewritten around quantity helpers. |
| 🔵 | 2025-09-19 | `99d34bd` | `detached.py` cleanup | — | No networking/nodes. |
| 🔵 | 2025-09-19 | `7a1b5f8` | Process `QUIT` in detached node (follow-up) | — | No networking/nodes. |
| 🔵 | 2025-09-19 | `dd67a9f` | Process `QUIT` in detached node | — | No networking/nodes. |
| 🔵 | 2025-09-19 | `c1e88cd` | Remove obsolete `MANIFEST.in` and `Makefile` | — | Upstream packaging. |
| 🔵 | 2025-09-19 | `cc70f44` | TODO update | — | Upstream docs. |
| 🔵 | 2025-09-12 | `9b86ca2` | Example GUI plugin that opens a Qt window | — | No Qt GUI. |
| 🔵 | 2025-09-01 | `d56eb35` | Merge of `master` into the side branch | — | Merge commit, no change of its own. |
| 🔵 | 2025-09-01 | `c8d33c4` | Remove obsolete stack annotation in the Qt main window | — | No Qt GUI. |
| 🔵 | 2025-08-27 | [#594](https://github.com/TUDelft-CNS-ATM/bluesky/pull/594) | Hide `self`/`cls` in Qt console hints | — | No Qt GUI. |
| 🔵 | 2025-08-27 | [#595](https://github.com/TUDelft-CNS-ATM/bluesky/pull/595) | Deselect routes in the GUI | — | No Qt GUI. |
| 🟢 | 2025-08-25 | [#592](https://github.com/TUDelft-CNS-ATM/bluesky/pull/592) | Optional speed constraint in `DEST` | `85314fe` | Ported. |
| 🟢 | 2025-08-25 | `3e254a1` | Fix `RESET` clearing function objects bound to instance methods | [#36](https://github.com/open-aviation/minisky/pull/36) | Moot: same `Replaceable`/`FuncObject` machinery as `c5118e6`, replaced by runtime-local replacement declarations. |
| 🟢 | 2025-08-18 | [#589](https://github.com/TUDelft-CNS-ATM/bluesky/pull/589) | Delete all waypoint metadata in `DELWPT` | `f2ed001` | Ported. |
| 🟢 | 2025-08-14 | [#585](https://github.com/TUDelft-CNS-ATM/bluesky/pull/585) | Read OpenAP coefficients from the `openap` package | [#2](https://github.com/open-aviation/minisky/pull/2) `ef074c7` | MiniSky depends on `openap` directly. |
| 🟢 | 2025-08-11 | [#584](https://github.com/TUDelft-CNS-ATM/bluesky/pull/584) | Fix old turn-speed bookkeeping | `bc9a00c` | Ported. |
| 🟢 | 2025-08-11 | [#587](https://github.com/TUDelft-CNS-ATM/bluesky/pull/587) | Fix lat/lon to text conversion | `ea00cf9` | Ported. |
| 🔵 | 2025-08-11 | `fc12a1f` | Client-side forward sent to all nodes instead of the active one | — | No networking. |
| 🟢 | 2025-07-15 | `c5118e6` | Fix re-implementing timed and stack methods in `Replaceable` classes | [#36](https://github.com/open-aviation/minisky/pull/36) | Moot: `FuncObject`-based selection replaced by runtime-local replacement declarations (`da30fae`, `7d0012d`). |
| 🔵 | 2025-07-06 | [#579](https://github.com/TUDelft-CNS-ATM/bluesky/pull/579) | Set node workdir from the server | — | No networking/nodes. |
| 🔵 | 2025-07-06 | [#577](https://github.com/TUDelft-CNS-ATM/bluesky/pull/577) | Correct pathfinder workdir type | — | No node launcher. |
| 🟢 | 2025-07-06 | [#578](https://github.com/TUDelft-CNS-ATM/bluesky/pull/578) | Fix NumPy truth-value conversion in areafilter | [#42](https://github.com/open-aviation/minisky/pull/42) `6bce3bd` | Moot: areafilter rewritten as `Shapes` with Shapely containment. |
| 🔵 | 2025-07-06 | [#580](https://github.com/TUDelft-CNS-ATM/bluesky/pull/580) | Qt stylesheet resource path | — | No Qt GUI. |
| 🔵 | 2025-06-17 | `c0550c9` | Move SIGINT/SIGTERM hookup into `Simulation.run()` | — | Node-process signal handling; MiniSky's CLI/runner owns shutdown (see #619 for the open Windows question). |
| 🟢 | 2025-06-17 | `22e387f` | Typing update in `cmdparser` | [#44](https://github.com/open-aviation/minisky/pull/44) | Moot: old command parser replaced. |
| 🔵 | 2025-06-17 | [#571](https://github.com/TUDelft-CNS-ATM/bluesky/pull/571) | `np.frombuffer` fix in the network codec | — | No network codec. |
| 🔵 | 2025-06-16 | `b527bc1` | Data-package README update | — | Upstream docs. |
| 🔵 | 2025-06-16 | `9886114` | Data-package README update | — | Upstream docs. |
| 🟢 | 2025-05-22 | [#567](https://github.com/TUDelft-CNS-ATM/bluesky/pull/567) | Aircraft synonym fixes (`synonym.dat`) | [#2](https://github.com/open-aviation/minisky/pull/2) `ef074c7` | Synonyms come from the maintained `openap` package; the copied `synonym.dat` is gone. |
| 🟢 | 2025-05-22 | `ca27273` | Phase out the `bs.scr` global | [#28](https://github.com/open-aviation/minisky/pull/28), [#61](https://github.com/open-aviation/minisky/pull/61) | No process-wide globals; console output is runtime-owned structured events. |
| 🔵 | 2025-05-19 | `15cff5f` | Reduce BADA warnings | — | No BADA. |
| 🔵 | 2025-05-19 | `6abdddb` | Fix Qt zoom with `-`/`=`/`+` keys | — | No Qt GUI. |
| 🔵 | 2025-05-19 | `8ca4d67` | Shared-state messages that originate client-side | — | No networking/shared state. |
| 🔵 | 2025-05-18 | `718ee09` | Reintroduce `SWRAD` | — | Display command for the removed GUI. |
| 🟢 | 2025-05-18 | `054aab3` | Allow union argument-type specifications for stack commands | [#44](https://github.com/open-aviation/minisky/pull/44) | The typed command parser derives alternatives from Python unions / `Annotated` schemas. |
| 🔵 | 2025-05-16 | `f983aff` | Avoid GUI import errors | — | No Qt GUI. |
| 🟢 | 2025-05-12 | `a22c6bf` | Clean up simulation `Timer`/timed functions | [#36](https://github.com/open-aviation/minisky/pull/36) `26c9a77` | Moot: timed functions are superseded by runtime-owned plugin hook scheduling. |
| 🔵 | 2025-05-12 | `eea5978` | Fix subscription for active node only | — | No networking. |
| 🔵 | 2025-05-12 | `ecee293` | `client.py` update | — | No networking. |
| 🔵 | 2025-05-09 | `c856458` | Network typing edits | — | No networking. |
| 🔵 | 2025-05-09 | `d16a758` | Fix distributed batch mode | — | No server/node batch mode. |
| 🔵 | 2025-04-14 | `1db5a58` | README update | — | Upstream docs. |
| 🔵 | 2025-04-12 | `79bc50c` | Python geo cleanup (`_geo.py`) | — | Cosmetic; MiniSky's `geo.py` has diverged. |
| 🔵 | 2025-04-12 | `46e9442` | Fix linter error in the geo package | — | Cosmetic; MiniSky's `geo.py` has diverged. |
| 🔵 | 2025-04-12 | `8c96fa5` | `trafgen` plugin update | — | Bundled plugin not shipped. |
| 🔵 | 2025-04-12 | `1a0e124` | TODO update | — | Upstream docs. |
| 🔵 | 2025-04-12 | `c53aa47` | `stackcheck` plugin update | — | Bundled plugin not shipped. |
| 🔵 | 2025-04-10 | [#561](https://github.com/TUDelft-CNS-ATM/bluesky/pull/561) | Client-mode bug fix | — | No client mode. |
| 🔵 | 2025-04-09 | `2a68109` | README update | — | Upstream docs. |
| 🔵 | 2025-04-09 | `71e47e4` | README update | — | Upstream docs. |
| 🔵 | 2025-03-25 | `9455731` | README update | — | Upstream docs. |
| 🔵 | 2025-03-24 | `04d53ff` | Add optimization plugin | — | Bundled plugin not shipped. Revisit only with a concrete research use case. |
| 🔵 | 2025-03-24 | `f76deee` | ADS-B feed plugin refactor | — | Bundled plugin not shipped. |
| 🟢 | 2025-03-19 | `bc227d2` | Add diagnostics to the function-object not-implemented stub | [#44](https://github.com/open-aviation/minisky/pull/44) | Moot: `FuncObject`/old `cmdparser` machinery replaced. |
| 🟢 | 2025-03-19 | `b48f2ee` | Detect malformed `init_plugin()` functions | [#36](https://github.com/open-aviation/minisky/pull/36) `768c0d7` | Moot: directory scanning and `init_plugin()` replaced by entry points and typed `Plugin` contracts. |
| 🟢 | 2025-03-16 | `4a598f2` | Fix typo in the command decorators' `inspect.unwrap` stop condition | [#44](https://github.com/open-aviation/minisky/pull/44) | Moot: command/decorator implementation replaced by the typed command parser. |
| 🟢 | 2025-03-14 | `ee4d270` | Accept flight-level altitudes in `WIND` profiles | [#44](https://github.com/open-aviation/minisky/pull/44) `bb134fc` | Typed `WindLevel` parsing accepts `FL...` and explicit altitude units. |
| 🔵 | 2025-03-14 | `3ede7a6` | Fix shared-state action type processing | — | No networking/shared state. |
| 🔵 | 2025-03-14 | `9555bfe` | Update client example to new network API | — | No networking. |
| 🟢 | 2025-03-14 | `baaaf6b` | Print exceptions raised during stack processing | [#61](https://github.com/open-aviation/minisky/pull/61) `b344ba3` | Command callback exceptions are caught and reported with their traceback as structured runtime events; parse failures are typed `Result` diagnostics. |
| 🔵 | 2025-03-14 | `3c0502f` | Fix `Signal` → `Subscription` promotion | — | No networking/shared state. |
| 🔵 | 2025-03-13 | [#553](https://github.com/TUDelft-CNS-ATM/bluesky/pull/553) | Windows Qt dark/light mode | — | No Qt GUI. |
| 🔵 | 2025-03-11 | `b61e01f` | Package metadata update | — | Upstream packaging. |
| 🔵 | 2025-03-11 | `263827a` | Client/Node cleanup and relocation | — | No networking. |
| 🔵 | 2025-03-10 | `7fcd4d6` | Make `Client` a subclass of `Node` | — | No networking. |
| 🔵 | 2025-03-10 | `98f82db` | Versioning of data packages | [#53](https://github.com/open-aviation/minisky/pull/53) | Upstream packaging; MiniSky ships navigation data as its own `minisky-xplane-navdata` package. |
| 🔵 | 2025-03-10 | `3f9735d` | Version the graphics/navdata resource packages | — | Upstream packaging; no graphics resources. |
| 🔵 | 2025-03-10 | `4dd4a6c` | Print client-stack traceback to terminal | — | No client stack. |
| 🔵 | 2025-03-07 | `a023ecb` | `.gitignore` update | — | No action. |
| 🔵 | 2025-03-07 | `f89c6a2` | Update wheel-build workflow | — | Independent CI/release. |
| 🔵 | 2025-03-07 | `798f782` | Remove 32-bit Windows wheel build | — | Independent CI/release. |
| 🔵 | 2025-03-07 | `00ecfba` | Reduce Linux wheel architectures | — | Independent CI/release. |
| 🔵 | 2025-03-07 | [#546](https://github.com/TUDelft-CNS-ATM/bluesky/pull/546) | Network overhaul | — | No networking/multi-node layer; MiniSky exposes REST + WebSocket streaming from a single runtime. |
| 🟢 | 2025-03-05 | [#541](https://github.com/TUDelft-CNS-ATM/bluesky/pull/541) | Relicense BlueSky from GPL-3.0 to MIT | [#58](https://github.com/open-aviation/minisky/pull/58) `83628cd` | MiniSky relicensed to MIT as well. |
| 🔵 | 2025-02-17 | `f5f2368` | README update | — | Upstream docs. |
| 🔵 | 2025-02-10 | `3bf6438` | Remove Qt5 backwards compatibility | — | No Qt GUI. |
| 🔵 | 2025-02-10 | `b04f0d7` | Remove explicit NumPy version pin | — | Upstream dependency metadata. |

## 2024

| | Date | BlueSky | Change | MiniSky | Notes |
| --- | --- | --- | --- | --- | --- |
| 🔵 | 2024-12-18 | [#516](https://github.com/TUDelft-CNS-ATM/bluesky/pull/516) | Qt settings window: ESC key fix | — | No Qt GUI. |
| 🔵 | 2024-12-18 | `fdf2f23` | Dynamic versioning | — | Upstream packaging. |
| 🔵 | 2024-12-17 | [#538](https://github.com/TUDelft-CNS-ATM/bluesky/pull/538) | Switch build backend to Hatch | — | Upstream packaging; MiniSky has its own uv workspace + Hatchling build. |
| 🔵 | 2024-12-17 | `17ca78a` | Update wheel-build workflow | — | Independent CI/release. |
| 🔵 | 2024-12-17 | `093b033` | Create wheel-build workflow | — | Independent CI/release. |
| 🔵 | 2024-12-03 | [#535](https://github.com/TUDelft-CNS-ATM/bluesky/pull/535) | Remove a debug print from the #470 turn implementation | — | Only touches #470 code that isn't in MiniSky. |
| 🔴 | 2024-11-18 | [#470](https://github.com/TUDelft-CNS-ATM/bluesky/pull/470) | Improved turning: turn-bank state (`TURNBANK`/`TURNPHI`), derived turn parameters, post-turn cruise-speed restoration | — | Partial. MiniSky models turn radius, turn speed and heading rate as typed `TurnParameters`, deliberately rejects the `TURNBANK`/`TURNPHI` waypoint syntax (see the comment in `route.py`), and has no `CRUISESPD` restoration. Decision needed: evaluate turn-bank state and cruise-speed policy as separate navigation features with scenarios; don't port wholesale. |
| 🔵 | 2024-09-04 | [#526](https://github.com/TUDelft-CNS-ATM/bluesky/pull/526) | Dependabot bump of the download-artifact action | — | Independent CI. |
| 🔵 | 2024-09-02 | `c9c797e` | Conditional loading of compiled/Python geo functions | [#35](https://github.com/open-aviation/minisky/pull/35) | Compiled geo backend removed; Python geo only. |
| 🔵 | 2024-09-02 | `56ae5de` | NumPy 2.0 compatibility (Qt GL code, requirements files) | — | Touches GUI code and requirements files MiniSky doesn't have. |
| 🟢 | 2024-07-25 | [#518](https://github.com/TUDelft-CNS-ATM/bluesky/pull/518) | Strip inline `#` comments from scenario lines | [#44](https://github.com/open-aviation/minisky/pull/44) `44f78ac` | The typed scenario parser strips inline comments. |
