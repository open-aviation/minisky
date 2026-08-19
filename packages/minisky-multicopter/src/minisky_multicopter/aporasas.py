"""Multicopter pilot-logic override.

The core [`APorASAS`][minisky.traffic.aporasas.APorASAS] derives the desired
*heading* from the desired *track* (with a wind-drift correction), baking
the fixed-wing assumption "the aircraft flies where its nose points" into
the command path. A multicopter redirects thrust instead, so for multicopter
rows the desired heading is the commanded body heading and the desired track
is left to the FMS / conflict resolution.

Selected with `SELECTIMPL APORASAS MULTICOPTERAPORASAS`.
"""

from __future__ import annotations

import numpy as np
from minisky import replacement
from minisky.traffic.aporasas import APorASAS

from minisky_multicopter.entity import get_multicopter


@replacement
class MulticopterAPorASAS(APorASAS):
    """Skip the track-to-heading coupling for multicopter rows."""

    def update(self) -> None:
        """Select the desired states, then decouple heading from track.

        Runs the base selection for the whole fleet and afterwards
        overwrites `self.hdg` on the multicopter rows with the commanded
        body heading, leaving `self.trk` (which the kinematics now flies)
        untouched. Where no body heading was ever commanded the nose follows
        the track — without the wind-drift correction, since a multicopter
        does not need to point its nose into the relative wind.
        """
        super().update()
        mc = get_multicopter(self.traffic)
        if mc is None or not mc.ismulticopter.any():
            return

        m = mc.ismulticopter
        self.hdg[m] = np.where(mc.swselhdg[m], mc.selhdg[m], self.trk[m]) % 360.0
