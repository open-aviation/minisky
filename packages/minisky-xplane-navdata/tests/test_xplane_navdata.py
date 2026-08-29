import numpy as np
from minisky import MiniSky
from minisky import quantities as q
from minisky_xplane_navdata import load, load_airways, load_waypoints


def test_xplane_navdata_units() -> None:
    waypoints = load_waypoints()
    vor = int(np.flatnonzero(waypoints.identifiers == "AAL")[0])
    ndb = int(np.flatnonzero(waypoints.identifiers == "APH")[0])

    airways = load_airways()
    airway = int(np.flatnonzero(airways.identifiers == "R464")[0])

    assert np.isclose(waypoints.frequencies[vor], 116_700_000.0)
    assert np.isclose(waypoints.frequencies[ndb], 396_000.0)
    assert np.isclose(airways.lower_altitudes[airway], q.ft_to_m(1200.0))
    assert np.isclose(airways.upper_altitudes[airway], q.ft_to_m(46000.0))


def test_xplane_navdata_runtime() -> None:
    runtime = MiniSky(navdata=load())
    try:
        assert isinstance(runtime.waypoints.latitudes, np.ndarray)
        assert "SPY" in runtime.waypoints.identifiers
        assert "EHAM" in runtime.airports.identifiers
        assert "NL" in runtime.countries.codes2
        assert "EHAM" in runtime.runway_thresholds
        assert "116.7 MHz" in runtime.traffic.position_by_name("AAL").unwrap()
        assert "396.0 kHz" in runtime.traffic.position_by_name("APH").unwrap()
    finally:
        runtime.close()
