"""Unit tests for the pure functions of the tangram bridge plugin."""

import pytest

from example_plugins.tangram import convert_snapshot, extract_command


def make_snapshot() -> dict:
    return {
        "siminfo": {
            "speed": 5.0,
            "simdt": 1.0,
            "simt": 42.0,
            "simutc": "2026-01-01T00:00:00",
            "ntraf": 1,
            "state": 2,
            "scenname": "kl204",
        },
        "acdata": {
            "callsign": ["KL204"],
            "lat": [52.0],
            "lon": [4.0],
            "alt": [3048.0],  # 10000 ft
            "trk": [90.0],
            "vs": [5.08],  # 1000 fpm
            "tas": [102.8888],  # 200 kt
            "cas": [51.4444],  # 100 kt
            "gs": [154.3332],  # 300 kt
            "typecode": ["B744"],
            "inconf": [False],
            "tcpamax": [0.0],
            "nconf_cur": 0,
            "nconf_tot": 0,
            "nlos_cur": 0,
            "nlos_tot": 0,
        },
    }


def test_convert_snapshot_units_and_fields():
    payload = convert_snapshot(make_snapshot())

    assert payload["count"] == 1
    siminfo = payload["siminfo"]
    assert siminfo["state"] == 2
    assert siminfo["state_name"] == "OP"
    assert siminfo["scenname"] == "kl204"
    assert siminfo["speed"] == 5.0

    (ac,) = payload["aircraft"]
    assert ac["id"] == "KL204"
    assert ac["callsign"] == "KL204"
    assert ac["typecode"] == "B744"
    assert ac["latitude"] == 52.0
    assert ac["longitude"] == 4.0
    assert ac["altitude"] == 10000
    assert ac["vertical_rate"] == 1000
    assert ac["tas"] == pytest.approx(200, abs=0.1)
    assert ac["ias"] == pytest.approx(100, abs=0.1)
    assert ac["groundspeed"] == pytest.approx(300, abs=0.1)
    assert ac["track"] == 90.0
    assert ac["inconf"] is False
    # simutc converted to an epoch timestamp usable as a trajectory point time
    assert ac["timestamp"] == pytest.approx(1767225600.0, abs=86400)


def test_convert_snapshot_empty():
    payload = convert_snapshot({"siminfo": {}, "acdata": {}})
    assert payload == {
        "aircraft": [],
        "count": 0,
        "siminfo": {
            "simt": 0.0,
            "simdt": 0.0,
            "simutc": None,
            "speed": 1.0,
            "ntraf": 0,
            "state": 0,
            "state_name": "INIT",
            "scenname": None,
            "nconf_cur": 0,
            "nlos_cur": 0,
        },
    }


def test_convert_snapshot_bad_simutc():
    snapshot = make_snapshot()
    snapshot["siminfo"]["simutc"] = "not a date"
    payload = convert_snapshot(snapshot)
    assert payload["aircraft"][0]["timestamp"] is None


@pytest.mark.parametrize(
    ("payload", "expected"),
    [
        ('{"command": "OP"}', "OP"),
        ('{"command": " DTMULT 5 "}', "DTMULT 5"),
        (b'{"command": "HOLD"}', "HOLD"),
        ('"RESET"', "RESET"),
        ("ECHO hello", "ECHO hello"),  # bare string for redis-cli convenience
        ('{"command": ""}', None),
        ('{"other": "x"}', None),
        ('{"command": null}', None),
        ("", None),
        ("   ", None),
        ("[1, 2]", None),
    ],
)
def test_extract_command(payload, expected):
    assert extract_command(payload) == expected
