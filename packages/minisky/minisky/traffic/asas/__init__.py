"""Airborne Separation Assurance System package.

This package bundles MiniSky's conflict detection and resolution:

- `detection`: pairwise state-based
  [`ConflictDetection`][minisky.traffic.asas.detection.ConflictDetection].
- `resolution`: shared
  [`ConflictResolution`][minisky.traffic.asas.resolution.ConflictResolution]
  state and navigation recovery.
- `mvp`: the Modified Voltage Potential
  [`MVP`][minisky.traffic.asas.mvp.MVP] resolution algorithm.

The active instances are [`runtime.traffic.cd`][minisky.traffic.asas.detection.ConflictDetection] and [`runtime.traffic.cr`][minisky.traffic.asas.resolution.ConflictResolution].
"""

# isort: off
# Import order matters: MVP subclasses ConflictResolution, so resolution must
# be importable before mvp to avoid a partially-initialised circular import.
from .detection import ConflictDetection
from .resolution import ConflictResolution
from .mvp import MVP

# isort: on
