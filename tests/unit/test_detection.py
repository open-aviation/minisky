"""Regression tests for the KD-tree conflict detection against the previous
all-pairs (dense N x N) algorithm.

``detect_reference`` below is the pre-KD-tree implementation of
``ConflictDetection.detect()``, kept verbatim as the behavioural reference.
The current implementation must produce identical conflict pairs, LoS pairs,
per-aircraft flags, and per-conflict geometry (up to float noise from
mathematically equivalent formulations) on randomized traffic states.

This includes a subtle edge case that the dense algorithm exhibits: because
|dvs| is floored to +1e-6 irrespective of sign, level aircraft pairs at
exactly |dalt| == hpz (adjacent flight levels) flag a conflict in one
direction only.
"""

from types import SimpleNamespace

import numpy as np
import pytest

from minisky.tools import geo
from minisky.tools.aero import ft, nm
from minisky.traffic.asas.detection import ConflictDetection


def detect_reference(ownship, intruder, rpz, hpz, dtlookahead):
    """The previous dense all-pairs detect(), kept as behavioural reference."""
    eye = np.eye(ownship.ntraf)

    qdr, dist = geo.kwikqdrdist_matrix(
        np.atleast_2d(ownship.lat),
        np.atleast_2d(ownship.lon),
        np.atleast_2d(intruder.lat),
        np.atleast_2d(intruder.lon),
    )
    qdr = np.asarray(qdr)
    dist = np.asarray(dist) * nm + 1e9 * eye

    qdrrad = np.radians(qdr)
    dx = dist * np.sin(qdrrad)
    dy = dist * np.cos(qdrrad)

    owntrkrad = np.radians(ownship.trk)
    ownu = ownship.gs * np.sin(owntrkrad).reshape((1, ownship.ntraf))
    ownv = ownship.gs * np.cos(owntrkrad).reshape((1, ownship.ntraf))
    inttrkrad = np.radians(intruder.trk)
    intu = intruder.gs * np.sin(inttrkrad).reshape((1, ownship.ntraf))
    intv = intruder.gs * np.cos(inttrkrad).reshape((1, ownship.ntraf))

    du = ownu - intu.T
    dv = ownv - intv.T

    dv2 = du * du + dv * dv
    dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)
    vrel = np.sqrt(dv2)

    tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * eye
    dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

    rpz = np.maximum(rpz[np.newaxis, :], rpz[:, np.newaxis])
    R2 = rpz * rpz
    swhorconf = dcpa2 < R2

    dxinhor = np.sqrt(np.maximum(0.0, R2 - dcpa2))
    dtinhor = dxinhor / vrel
    tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)
    touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)

    dalt = (
        ownship.alt.reshape((1, ownship.ntraf))
        - intruder.alt.reshape((1, ownship.ntraf)).T
        + 1e9 * eye
    )
    dvs = ownship.vs.reshape(1, ownship.ntraf) - intruder.vs.reshape(1, ownship.ntraf).T
    dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)

    hpz = np.maximum(hpz[np.newaxis, :], hpz[:, np.newaxis])
    tcrosshi = (dalt + hpz) / -dvs
    tcrosslo = (dalt - hpz) / -dvs
    tinver = np.minimum(tcrosshi, tcrosslo)
    toutver = np.maximum(tcrosshi, tcrosslo)

    tinconf = np.maximum(tinver, tinhor)
    toutconf = np.minimum(toutver, touthor)

    swconfl = np.array(
        swhorconf
        * (tinconf <= toutconf)
        * (toutconf > 0.0)
        * (tinconf < np.asarray(dtlookahead)[:, np.newaxis])
        * (1.0 - eye),
        dtype=bool,
    )

    inconf = np.any(swconfl, 1)
    tcpamax = np.max(tcpa * swconfl, 1)
    confpairs = [
        (ownship.callsign[i], ownship.callsign[j]) for i, j in zip(*np.where(swconfl), strict=False)
    ]
    swlos = (dist < rpz) * (np.abs(dalt) < hpz)
    lospairs = [
        (ownship.callsign[i], ownship.callsign[j]) for i, j in zip(*np.where(swlos), strict=False)
    ]

    return (
        confpairs,
        lospairs,
        inconf,
        tcpamax,
        qdr[swconfl],
        dist[swconfl],
        np.sqrt(dcpa2[swconfl]),
        tcpa[swconfl],
        tinconf[swconfl],
        dalt[swconfl],
    )


def make_traffic(
    n,
    seed,
    latspan=(51.8, 52.6),
    lonspan=(3.6, 4.9),
    level_fraction=1.0,
):
    """Random traffic state on an exact flight-level grid (SI units)."""
    rng = np.random.default_rng(seed)
    vs = np.where(
        rng.random(n) < level_fraction,
        0.0,
        rng.uniform(-15.0, 15.0, n),  # climbing/descending up to ~3000 fpm
    )
    return SimpleNamespace(
        ntraf=n,
        lat=rng.uniform(*latspan, n),
        lon=rng.uniform(*lonspan, n),
        trk=rng.uniform(0.0, 360.0, n),
        gs=rng.uniform(150.0, 260.0, n),
        alt=rng.integers(10, 37, n) * 1000.0 * ft,  # FL100..FL360 grid
        vs=vs,
        callsign=[f"AC{i:03d}" for i in range(n)],
    )


def assert_detect_equal(traf, rpz, hpz, dtlookahead):
    """Assert current detect() output matches the dense reference exactly."""
    # detect() uses no instance state, so a bare instance suffices
    cd = ConflictDetection.__new__(ConflictDetection)
    ref = detect_reference(traf, traf, rpz, hpz, dtlookahead)
    new = cd.detect(traf, traf, rpz, hpz, dtlookahead)

    # Directed pair lists, identical content and order
    assert new[0] == ref[0], "confpairs differ"
    assert new[1] == ref[1], "lospairs differ"
    # Per-aircraft arrays
    np.testing.assert_array_equal(new[2], ref[2], err_msg="inconf differs")
    np.testing.assert_allclose(new[3], ref[3], rtol=1e-9, atol=1e-9, err_msg="tcpamax differs")
    # Per-conflict geometry (equivalent formulations -> tiny float noise)
    names = ["qdr", "dist", "dcpa", "tcpa", "tinconf", "dalt"]
    for k, name in enumerate(names):
        np.testing.assert_allclose(
            new[4 + k], ref[4 + k], rtol=1e-6, atol=1e-6, err_msg=f"{name} differs"
        )


DEFAULTS = {"rpz": 5.0 * nm, "hpz": 1000.0 * ft, "dtlookahead": 300.0}


def default_params(n):
    return (
        np.full(n, DEFAULTS["rpz"]),
        np.full(n, DEFAULTS["hpz"]),
        np.full(n, DEFAULTS["dtlookahead"]),
    )


class TestDetectMatchesDenseReference:
    @pytest.mark.parametrize("seed", [0, 1, 2])
    def test_dense_area_level_traffic(self, seed):
        # Everything level on an exact FL grid: exercises the
        # one-direction-only conflicts at |dalt| == hpz
        traf = make_traffic(150, seed)
        assert_detect_equal(traf, *default_params(150))

    @pytest.mark.parametrize("seed", [3, 4, 5])
    def test_dense_area_mixed_vertical_speeds(self, seed):
        traf = make_traffic(150, seed, level_fraction=0.6)
        assert_detect_equal(traf, *default_params(150))

    @pytest.mark.parametrize("seed", [6, 7])
    def test_spread_area_prunes_candidates(self, seed):
        # Continental spread: most pairs are pruned by the KD-tree query
        traf = make_traffic(150, seed, latspan=(46.0, 58.0), lonspan=(-4.0, 12.0))
        assert_detect_equal(traf, *default_params(150))

    def test_per_aircraft_separation_minima(self):
        rng = np.random.default_rng(8)
        n = 100
        traf = make_traffic(n, 8, level_fraction=0.7)
        rpz = rng.uniform(3.0, 8.0, n) * nm
        hpz = rng.uniform(500.0, 1500.0, n) * ft
        dtlookahead = rng.uniform(120.0, 600.0, n)
        assert_detect_equal(traf, rpz, hpz, dtlookahead)

    def test_adjacent_flight_levels_conflict_one_direction(self):
        # Head-on pair exactly 1000 ft apart, both level: the dense
        # algorithm flags this in one direction only (dvs floored to +1e-6
        # makes the higher aircraft's fictitious drift enter the band)
        traf = SimpleNamespace(
            ntraf=2,
            lat=np.array([52.0, 52.0]),
            lon=np.array([4.0, 4.2]),
            trk=np.array([90.0, 270.0]),
            gs=np.array([200.0, 200.0]),
            alt=np.array([25000.0 * ft, 26000.0 * ft]),
            vs=np.array([0.0, 0.0]),
            callsign=["OWN", "INT"],
        )
        rpz, hpz, dtlookahead = default_params(2)
        cd = ConflictDetection.__new__(ConflictDetection)
        result = cd.detect(traf, traf, rpz, hpz, dtlookahead)
        assert result[0] == [("INT", "OWN")]
        assert result[1] == []  # exactly hpz apart is not yet LoS
        assert_detect_equal(traf, rpz, hpz, dtlookahead)

    def test_no_traffic(self):
        traf = make_traffic(0, 9)
        cd = ConflictDetection.__new__(ConflictDetection)
        result = cd.detect(traf, traf, *default_params(0))
        assert result[0] == [] and result[1] == []
        assert len(result[2]) == 0 and len(result[3]) == 0

    def test_single_aircraft(self):
        traf = make_traffic(1, 10)
        cd = ConflictDetection.__new__(ConflictDetection)
        result = cd.detect(traf, traf, *default_params(1))
        assert result[0] == [] and result[1] == []
        assert not result[2].any()
