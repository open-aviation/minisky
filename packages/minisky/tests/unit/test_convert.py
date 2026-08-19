"""Unit tests for minisky._internal.convert (text parsers)."""

import minisky._internal.convert as cv
import pytest


class TestTime:
    def test_txt2tim_hms(self) -> None:
        assert cv.txt2tim("00:01:30") == pytest.approx(90.0)

    def test_txt2tim_seconds(self) -> None:
        assert cv.txt2tim("45") == pytest.approx(45.0)

    def test_tim2txt_format(self) -> None:
        assert cv.tim2txt(90) == "00:01:30.00"

    def test_roundtrip(self) -> None:
        assert cv.txt2tim(cv.tim2txt(3725.0)) == pytest.approx(3725.0)


class TestLatLon:
    def test_decimal_lat(self) -> None:
        assert cv.txt2lat("52.3") == pytest.approx(52.3)

    def test_decimal_lon(self) -> None:
        assert cv.txt2lon("4.5") == pytest.approx(4.5)

    def test_negative_lat(self) -> None:
        assert cv.txt2lat("-33.9") == pytest.approx(-33.9)

    def test_hemisphere_lat(self) -> None:
        # N52'18'00 == 52.3 degrees
        assert cv.txt2lat("N52'18'0") == pytest.approx(52.3, abs=1e-6)

    def test_hemisphere_south_is_negative(self) -> None:
        assert cv.txt2lat("S52'18'0") == pytest.approx(-52.3, abs=1e-6)


class TestAngles:
    @pytest.mark.parametrize(
        ("angle", "expected"),
        [(190.0, -170.0), (-190.0, 170.0), (180.0, -180.0), (0.0, 0.0), (359.0, -1.0)],
    )
    def test_degto180_wraps(self, angle: float, expected: float) -> None:
        assert cv.degto180(angle) == pytest.approx(expected)


class TestBool:
    @pytest.mark.parametrize("txt", ["ON", "TRUE", "YES", "1"])
    def test_truthy(self, txt: str) -> None:
        assert cv.txt2bool(txt) is True

    @pytest.mark.parametrize("txt", ["OFF", "FALSE", "NO", "0"])
    def test_falsy(self, txt: str) -> None:
        assert cv.txt2bool(txt) is False
