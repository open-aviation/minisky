# Upstream decisions

MiniSky is a fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) that
moves in the opposite direction: toward a bare minimum. Upstream changes are
adopted selectively. This page records upstream PRs and features that were
evaluated and **deliberately not adopted**, so the question isn't reopened
every time someone diffs against upstream.

When evaluating a new upstream change, check here first. If a rejected change
becomes relevant later (e.g. upstream lands a follow-up with actual new
behaviour), add a new entry rather than editing the existing entry.

## Rejected

### PR [#644](https://github.com/TUDelft-CNS-ATM/bluesky/pull/644) — ResumeNavigation as a replaceable class (rejected 2026-07-22)

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

### PR [#656](https://github.com/TUDelft-CNS-ATM/bluesky/pull/656) — Free-to-Revert (FTR) resume navigation (rejected 2026-08-27)

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

## Not adopted — already fixed or not applicable

### PR [#653](https://github.com/TUDelft-CNS-ATM/bluesky/pull/653) — vsmin from descent envelopes (2026-08-27)

Upstream derived `vsmin` from *climb* vertical-speed envelopes; the fix uses
the descent envelopes. MiniSky's `coefficients.py` already does (its extra
`initclimb_vs` term is a positive climb rate that never wins the `min()`).

### PR [#654](https://github.com/TUDelft-CNS-ATM/bluesky/pull/654) — speed-envelope phase condition (2026-08-27)

Fixes `(phase >= CL) | (phase <= DE)`, an always-true condition that let
en-route limits clobber IC/AP limits. MiniSky's `_construct_v_limits` already
uses an explicit `enroute = CLIMB | CRUISE | DESCENT` mask. The rewrite's side
effect (UNKNOWN `vmin`: 0 → `vminer`) is not adopted; `vmin = 0` for an
unclassifiable phase is the deliberate, more permissive choice.

### PRs [#647](https://github.com/TUDelft-CNS-ATM/bluesky/pull/647) (Qt clipboard paste) and [#650](https://github.com/TUDelft-CNS-ATM/bluesky/pull/650) (C geo module) (2026-08-27)

The Qt console and the compiled C geo extension were removed from MiniSky;
there is nothing to apply these fixes to.

### PR [#659](https://github.com/TUDelft-CNS-ATM/bluesky/pull/659) — OpenAP speed limits evaluated at intended altitude (2026-09-01)

Upstream evaluated the CAS/Mach speed envelope at the *intended* altitude
(`allow_h`) instead of the current one, letting a descent-with-altitude-change
command apply the wrong phase's speed limit. MiniSky's `OpenAP.limits()`
already evaluates the envelope at `self.traffic.alt` (current altitude).

### PR [#658](https://github.com/TUDelft-CNS-ATM/bluesky/pull/658) — missing nm conversion in RTA leg time (2026-09-01)

Upstream divided a distance in nautical miles (`wpdistto`) by a speed in m/s
(`legtas`) when computing `legtime` for an RTA-constrained leg with an
airspeed constraint. Not applicable: MiniSky's `route.py` stores `wpdistto`
in metres throughout (SI units), so the same division is already correct.

### PR [#661](https://github.com/TUDelft-CNS-ATM/bluesky/pull/661) — MVP vertical resolution direction (2026-09-01)

Ported directly rather than recorded here: `MVP.MVP()`'s vertical resolution
direction was symmetric (not dependent on `drel`/`vrel`) when both aircraft
of a pair shared the same vertical rate, so a co-altitude conflict pair could
be pushed the same way instead of apart. Fixed in `mvp.py` to fall back to
relative altitude, then aircraft index, as an antisymmetric tiebreaker,
matching upstream's fix. (Upstream's other fix in the same PR — an
`asasalttemp` sentinel blowing up for non-conflicting aircraft — was already
avoided here via the `has_resolution_time` mask in `MVP.resolve()`.)
