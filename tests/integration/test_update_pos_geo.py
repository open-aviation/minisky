"""Regression tests for Traffic.update_pos near poles and antimeridian."""

from __future__ import annotations

import numpy as np

from minisky.tools.convert import degto180


def _prepare_update_pos(traf) -> None:
    # update_pos reads swaltsel, which is only set inside update_airspeed.
    traf.swaltsel = np.zeros(traf.ntraf, dtype=bool)


def test_update_pos_wraps_longitude_across_antimeridian(bs, sim, monkeypatch):
    ok, _ = bs.traf.cre("KL180", "A320", lat=0.0, lon=179.9, hdg=90, alt=10000, spd=250)
    assert ok
    bs.traf.gs[:] = 250.0
    bs.traf.gsnorth[:] = 0.0
    bs.traf.gseast[:] = 250.0
    bs.traf.trk[:] = 90.0
    _prepare_update_pos(bs.traf)

    monkeypatch.setattr(bs.sim, "simdt", 60.0)
    bs.traf.update_pos()

    assert -180.0 <= bs.traf.lon[0] <= 180.0
    assert abs(bs.traf.lat[0]) <= 90.0
    assert np.isfinite(bs.traf.lon[0])
    # Eastbound from just west of the date line should cross into western longitudes.
    assert bs.traf.lon[0] < 0.0


def test_update_pos_crosses_pole_to_far_side(bs, sim, monkeypatch):
    start_lat = 89.9
    start_lon = 10.0
    ok, _ = bs.traf.cre(
        "KL090", "A320", lat=start_lat, lon=start_lon, hdg=0, alt=10000, spd=250
    )
    assert ok
    bs.traf.gs[:] = 250.0
    bs.traf.gsnorth[:] = 250.0
    bs.traf.gseast[:] = 0.0
    bs.traf.trk[:] = 0.0
    _prepare_update_pos(bs.traf)

    # ~30 km north from 89.9° crosses the pole under great-circle stepping.
    monkeypatch.setattr(bs.sim, "simdt", 120.0)
    bs.traf.update_pos()

    assert np.isfinite(bs.traf.lon[0])
    assert np.isfinite(bs.traf.coslat[0])
    assert abs(bs.traf.lat[0]) < 90.0
    # Far side of the pole: longitude shifts by about 180°.
    assert abs(abs(degto180(bs.traf.lon[0] - start_lon)) - 180.0) < 1.0
    assert bs.traf.lat[0] < start_lat
