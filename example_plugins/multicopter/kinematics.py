"""Multicopter flight integration.

Replaces the bank-to-turn kinematics of the core :class:`Kinematics` entity
for multicopter rows: heading slews at a fixed yaw rate (valid at zero
airspeed, so hover-yaw works), and the velocity vector follows the
*commanded track* rather than the heading, so track and heading are
decoupled.

Selected with ``SELECTIMPL KINEMATICS MULTICOPTERKINEMATICS``; fixed-wing
rows keep the base-class behaviour untouched.
"""

from __future__ import annotations

from minisky.traffic.kinematics import Kinematics


class MulticopterKinematics(Kinematics):
    """Yaw-rate-limited, track-driven integration for multicopter rows.

    Only :meth:`update_airspeed` and :meth:`update_groundspeed` are
    overridden; the inherited :meth:`Kinematics.update` still runs a single
    :meth:`Kinematics.update_pos` pass afterwards, so position is integrated
    exactly once from the corrected velocity.
    """

    def update_airspeed(self) -> None:
        """Integrate TAS, heading and vertical speed one step.

        Runs the base implementation for the whole fleet, then re-integrates
        the heading of multicopter rows at their yaw rate instead of the
        bank-angle turn rate (which explodes as TAS approaches zero).
        """
        super().update_airspeed()
        # TODO: slew traf.hdg[m] towards the commanded body heading,
        # clipped to yawrate * simdt.

    def update_groundspeed(self) -> None:
        """Compute ground speed and track from the velocity vector.

        Runs the base implementation for the whole fleet, then rebuilds the
        ground-speed components of multicopter rows from the *commanded
        track* (``traf.aporasas.trk``) plus wind, and derives ``gs``/``trk``
        from them.
        """
        super().update_groundspeed()
        # TODO: recompute traf.gsnorth/gseast/gs/trk for the multicopter
        # rows from aporasas.trk instead of traf.hdg.
