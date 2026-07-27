"""Multicopter pilot-logic override.

The core :class:`APorASAS` derives the desired *heading* from the desired
*track* (with a wind-drift correction), baking the fixed-wing assumption
"the aircraft flies where its nose points" into the command path. A
multicopter redirects thrust instead, so for multicopter rows the desired
heading is the commanded body heading and the desired track is left to the
FMS / conflict resolution.

Selected with ``SELECTIMPL APORASAS MULTICOPTERAPORASAS``.
"""

from __future__ import annotations

from minisky.traffic.aporasas import APorASAS


class MulticopterAPorASAS(APorASAS):
    """Skip the track-to-heading coupling for multicopter rows."""

    def update(self) -> None:
        """Select the desired states, then decouple heading from track.

        Runs the base selection for the whole fleet and afterwards
        overwrites ``self.hdg`` on the multicopter rows with the commanded
        body heading, leaving ``self.trk`` (which the kinematics now flies)
        untouched.
        """
        super().update()
        # TODO: self.hdg[m] = commanded body heading, falling back to
        # self.trk[m] where no body heading was ever commanded.
