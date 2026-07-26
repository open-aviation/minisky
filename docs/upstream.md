# Upstream decisions

MiniSky is a fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) that
moves in the opposite direction: toward a bare minimum. Upstream changes are
adopted selectively. This page records upstream PRs and features that were
evaluated and **deliberately not adopted**, so the question isn't reopened
every time someone diffs against upstream.

When evaluating a new upstream change, check here first. If a rejected change
becomes relevant later (e.g. upstream lands a follow-up with actual new
behaviour), add a new entry rather than editing the old one.

## Rejected

### PR [#644](https://github.com/TUDelft-CNS-ATM/bluesky/pull/644) — ResumeNavigation as a replaceable class (rejected 2026-07-22)

**What it does upstream:** moves `ConflictResolution.resumenav()` verbatim into
a new `ResumeNavigation` base class built on BlueSky's replaceable-`Entity`
registry, with the existing past-CPA algorithm as a `PastCPA` subclass, a
`RESNAV` stack command to swap implementations, and a `traf.resnav` singleton
updated after `cr.update()`. Motivation is research extensibility (a
"dual-criteria FTR" resume method is planned upstream).

**Why not here:**

- Zero behavioural change — MiniSky already has the identical past-CPA
  algorithm in `ConflictResolution.resumenav()`
  (`minisky/traffic/asas/resolution.py`), with slightly more robust variable
  initialisation than the upstream version.
- The seam depends on the replaceable-`Entity` registry
  (`select()`/`selected()`/`derived()`), machinery MiniSky deliberately
  removed. Porting it would reintroduce indirection the fork exists to strip.
- It adds stateful surface (resume-nav selection independent of CR selection,
  `RESO OFF` silently disabling resume-nav) with no functional gain.

**If the need arises:** `resumenav()` is already an overridable method — a
custom CR subclass or plugin can replace the resume policy today with no new
infrastructure. If upstream's dual-criteria FTR algorithm becomes useful, port
*that algorithm* as a `resumenav` override, not the class scaffolding.
