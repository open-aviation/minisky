"""Unit tests for minisky.tools.convert (text parsers).

Units: txt2alt returns meters, txt2spd returns m/s (or Mach if < 1),
txt2tim returns seconds.
"""

import pytest

from minisky.tools import convert as cv

FT = 0.3048
KTS = 0.514444


class TestAltitude:
    def test_flight_level(self):
        assert cv.txt2alt("FL300") == pytest.approx(30000 * FT)

    def test_plain_feet(self):
        assert cv.txt2alt("2500") == pytest.approx(2500 * FT)

    def test_invalid_raises(self):
        with pytest.raises(ValueError):
            cv.txt2alt("NOTANALT")


class TestTime:
    def test_txt2tim_hms(self):
        assert cv.txt2tim("00:01:30") == pytest.approx(90.0)

    def test_txt2tim_seconds(self):
        assert cv.txt2tim("45") == pytest.approx(45.0)

    def test_tim2txt_format(self):
        assert cv.tim2txt(90) == "00:01:30.00"

    def test_roundtrip(self):
        assert cv.txt2tim(cv.tim2txt(3725.0)) == pytest.approx(3725.0)


class TestLatLon:
    def test_decimal_lat(self):
        assert cv.txt2lat("52.3") == pytest.approx(52.3)

    def test_decimal_lon(self):
        assert cv.txt2lon("4.5") == pytest.approx(4.5)

    def test_negative_lat(self):
        assert cv.txt2lat("-33.9") == pytest.approx(-33.9)

    def test_hemisphere_lat(self):
        # N52'18'00 == 52.3 degrees
        assert cv.txt2lat("N52'18'0") == pytest.approx(52.3, abs=1e-6)

    def test_hemisphere_south_is_negative(self):
        assert cv.txt2lat("S52'18'0") == pytest.approx(-52.3, abs=1e-6)


class TestSpeed:
    def test_knots_to_ms(self):
        assert cv.txt2spd("250") == pytest.approx(250 * KTS, rel=1e-3)

    def test_mach_passthrough(self):
        assert cv.txt2spd(".8") == pytest.approx(0.8)

    def test_invalid_raises(self):
        with pytest.raises(ValueError):
            cv.txt2spd("FAST")


class TestAngles:
    @pytest.mark.parametrize(
        "angle,expected",
        [(190.0, -170.0), (-190.0, 170.0), (180.0, -180.0), (0.0, 0.0), (359.0, -1.0)],
    )
    def test_degto180_wraps(self, angle, expected):
        assert cv.degto180(angle) == pytest.approx(expected)

    def test_deg180_is_alias_of_degto180(self):
        # Regression: deg180 and degto180 were duplicate implementations
        assert cv.deg180 is cv.degto180
        assert cv.deg180(190.0) == pytest.approx(-170.0)


class TestBool:
    @pytest.mark.parametrize("txt", ["ON", "TRUE", "YES", "1"])
    def test_truthy(self, txt):
        assert cv.txt2bool(txt) is True

    @pytest.mark.parametrize("txt", ["OFF", "FALSE", "NO", "0"])
    def test_falsy(self, txt):
        assert cv.txt2bool(txt) is False
