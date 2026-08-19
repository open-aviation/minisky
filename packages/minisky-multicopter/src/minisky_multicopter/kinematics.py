"""Multicopter flight integration.

Replaces the bank-to-turn kinematics of the core
[`Kinematics`][minisky.traffic.kinematics.Kinematics] entity for multicopter
rows: heading slews at a fixed yaw rate (valid at zero airspeed, so
hover-yaw works), and the velocity vector follows the *commanded track*
rather than the heading, so track and heading are decoupled.

Selected with `SELECTIMPL KINEMATICS MULTICOPTERKINEMATICS`; fixed-wing
rows keep the base-class behaviour untouched.
"""

from __future__ import annotations

import numpy as np
from minisky import quantities as q
from minisky import replacement
from minisky.traffic.kinematics import Kinematics

from minisky_multicopter.entity import get_multicopter


@replacement
class MulticopterKinematics(Kinematics):
    """Yaw-rate-limited, track-driven integration for multicopter rows.

    Only `update_airspeed` and `update_groundspeed` are overridden; the
    inherited [`Kinematics.update`][minisky.traffic.kinematics.Kinematics.update]
    still runs a single `update_pos` pass afterwards, so position is
    integrated exactly once from the corrected velocity.
    """

    def update_airspeed(self) -> None:
        """Integrate TAS, heading and vertical speed one step.

        Runs the base implementation for the whole fleet, then re-integrates
        the heading of multicopter rows at their yaw rate instead of the
        bank-angle turn rate (which explodes as TAS approaches zero).
        """
        # NOTE(abraham): the base method mutates shared traffic state for
        # every row before we overwrite the multicopter subset.
        # in the future we should redesign the core so it only dispatches the
        # relevant rows directly.
        traf = self.traffic
        mc = get_multicopter(traf)
        if mc is None or not mc.ismulticopter.any():
            super().update_airspeed()
            return

        m = mc.ismulticopter
        hdg0 = traf.hdg[m]  # fancy indexing copies: heading before this step
        super().update_airspeed()

        # Yaw at a fixed rate towards the desired body heading, from the
        # pre-update heading (the base class snapped it, because its
        # bank-angle turn rate goes to infinity at tas -> 0).
        simdt = self._get_simulation().simdt
        delhdg = (traf.aporasas.hdg[m] - hdg0 + 180.0) % 360.0 - 180.0
        maxdel = mc.yawrate[m] * simdt
        turning = np.abs(delhdg) > maxdel
        traf.hdg[m] = (
            np.where(turning, hdg0 + np.sign(delhdg) * maxdel, traf.aporasas.hdg[m]) % 360.0
        )
        self.swhdgsel[m] = turning

    def update_groundspeed(self) -> None:
        """Compute ground speed and track from the velocity vector.

        Runs the base implementation for the whole fleet, then rebuilds the
        ground-speed components of multicopter rows from the *commanded
        track* (`traf.aporasas.trk`) plus wind, and derives `gs`/`trk` from
        them: thrust is redirected without rotating the body, and course
        changes have no turn radius. The work accumulated by the base class
        along its heading-driven ground speed is corrected to the rebuilt
        velocity.
        """
        traf = self.traffic
        super().update_groundspeed()
        mc = get_multicopter(traf)
        if mc is None or not mc.ismulticopter.any():
            return

        m = mc.ismulticopter
        gsbase = traf.gs[m]
        trkcmd = np.radians(traf.aporasas.trk)
        airborne = traf.alt > q.ft_to_m(50.0)  # windnorth/east are zero without wind
        traf.gsnorth[m] = (traf.tas * np.cos(trkcmd) + traf.windnorth * airborne)[m]
        traf.gseast[m] = (traf.tas * np.sin(trkcmd) + traf.windeast * airborne)[m]
        # In the no-wind branch the base class aliases traf.gs to traf.tas
        # and traf.trk to traf.hdg (plain assignment of the same ndarray), so
        # writing them in place would corrupt tas and hdg. Rebuild instead.
        gs = np.hypot(traf.gsnorth, traf.gseast)
        traf.gs = np.where(m, gs, traf.gs)
        # The track angle is undefined at hover; hold the commanded track.
        trk = np.where(
            gs > 0.01,
            np.degrees(np.arctan2(traf.gseast, traf.gsnorth)) % 360.0,
            traf.aporasas.trk % 360.0,
        )
        traf.trk = np.where(m, trk, traf.trk)

        # The base class accumulated traf.work along its heading-driven
        # ground speed; replace that increment with one along the rebuilt
        # velocity for multicopter rows.
        simdt = self._get_simulation().simdt
        vs2 = traf.vs[m] ** 2
        traf.work[m] += (
            traf.perf.thrust[m]
            * simdt
            * (np.sqrt(traf.gs[m] ** 2 + vs2) - np.sqrt(gsbase**2 + vs2))
        )
