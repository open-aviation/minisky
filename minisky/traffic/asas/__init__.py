"""Airborne Separation Assurance System (ASAS) package.

This package bundles MiniSky's conflict detection and resolution (CD&R)
functionality:

- ``detection``: pairwise state-based conflict detection (:class:`ConflictDetection`),
  which linearly extrapolates aircraft states to find protected-zone intrusions
  within a lookahead time.
- ``resolution``: the conflict resolution base class (:class:`ConflictResolution`),
  which manages resolution state and navigation recovery after conflicts.
- ``mvp``: the Modified Voltage Potential (:class:`MVP`) resolution algorithm.

The active detection and resolution instances live on the traffic object as
``minisky.traf.cd`` and ``minisky.traf.cr``.
"""

# isort: off
# Import order matters: MVP subclasses ConflictResolution, so resolution must
# be importable before mvp to avoid a partially-initialised circular import.
from .detection import ConflictDetection
from .resolution import ConflictResolution
from .mvp import MVP

# isort: on
