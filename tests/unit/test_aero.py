"""Unit tests for minisky.tools.aero (ISA atmosphere and speed conversions).

All aero functions use SI units: altitude in meters, speed in m/s.
"""

import numpy as np
import pytest

from minisky.tools import aero


class TestAtmosphere:
    def test_isa_sea_level(self):
        p, rho, temp = aero.vatmos(0.0)
        assert p == pytest.approx(101325.0, rel=1e-3)
        assert rho == pytest.approx(1.225, rel=1e-3)
        assert temp == pytest.approx(288.15, rel=1e-4)

    def test_isa_tropopause_temperature(self):
        assert aero.vtemp(11000.0) == pytest.approx(216.65, rel=1e-3)

    def test_temperature_constant_above_tropopause(self):
        assert aero.vtemp(12000.0) == pytest.approx(aero.vtemp(15000.0), rel=1e-6)

    def test_pressure_decreases_with_altitude(self):
        alts = np.array([0.0, 3000.0, 6000.0, 9000.0, 12000.0])
        p, _, _ = aero.vatmos(alts)
        assert np.all(np.diff(p) < 0)

    def test_density_decreases_with_altitude(self):
        alts = np.array([0.0, 3000.0, 6000.0, 9000.0, 12000.0])
        _, rho, _ = aero.vatmos(alts)
        assert np.all(np.diff(rho) < 0)

    def test_scalar_atmos_isothermal_layer(self):
        # Regression: atmos() raised NameError (bare exp) in isothermal layers
        p, rho, temp = aero.atmos(15000.0)  # 11-20 km layer
        assert temp == pytest.approx(216.65, rel=1e-6)
        assert p == pytest.approx(12045.0, rel=1e-3)  # ISA at 15 km
        assert rho == pytest.approx(p / (aero.R * temp), rel=1e-6)

    def test_scalar_atmos_upper_isothermal_layer(self):
        # 47-51 km layer is also isothermal
        p, rho, temp = aero.atmos(49000.0)
        assert temp == pytest.approx(270.65, rel=1e-6)
        assert p > 0.0
        assert rho > 0.0

    def test_scalar_atmos_matches_vectorized_in_stratosphere(self):
        # vatmos uses a simplified two-layer ISA, valid up to ~22 km
        for h in (12000.0, 15000.0, 19000.0):
            p, rho, temp = aero.atmos(h)
            p_v, rho_v, t_v = aero.vatmos(h)
            assert p == pytest.approx(float(p_v), rel=1e-3)
            assert rho == pytest.approx(float(rho_v), rel=1e-3)
            assert temp == pytest.approx(float(t_v), rel=1e-3)

    def test_vatmos_vectorized_matches_scalar(self):
        alts = np.array([0.0, 5000.0, 10000.0])
        p_vec, rho_vec, t_vec = aero.vatmos(alts)
        for i, h in enumerate(alts):
            p, rho, t = aero.vatmos(h)
            assert p_vec[i] == pytest.approx(p)
            assert rho_vec[i] == pytest.approx(rho)
            assert t_vec[i] == pytest.approx(t)


class TestSpeedConversions:
    @pytest.mark.parametrize("h", [0.0, 3000.0, 8000.0, 11000.0])
    def test_tas_mach_roundtrip(self, h):
        tas = 200.0
        mach = aero.vtas2mach(tas, h)
        assert aero.vmach2tas(mach, h) == pytest.approx(tas, rel=1e-6)

    @pytest.mark.parametrize("h", [0.0, 3000.0, 8000.0, 11000.0])
    def test_cas_tas_roundtrip(self, h):
        cas = 130.0
        tas = aero.vcas2tas(cas, h)
        assert aero.vtas2cas(tas, h) == pytest.approx(cas, rel=1e-6)

    def test_cas_equals_tas_at_sea_level(self):
        assert aero.vcas2tas(150.0, 0.0) == pytest.approx(150.0, rel=1e-6)

    def test_tas_exceeds_cas_at_altitude(self):
        assert aero.vcas2tas(150.0, 10000.0) > 150.0

    def test_mach_increases_with_altitude_at_constant_tas(self):
        # Speed of sound drops with temperature, so Mach rises for fixed TAS
        assert aero.vtas2mach(200.0, 10000.0) > aero.vtas2mach(200.0, 0.0)

    def test_vcasormach_interprets_small_value_as_mach(self):
        tas, cas, mach = aero.vcasormach(0.8, 10000.0)
        assert mach == pytest.approx(0.8, rel=1e-6)
        assert tas > 200.0

    def test_vcasormach_interprets_large_value_as_cas(self):
        tas, cas, mach = aero.vcasormach(150.0, 5000.0)
        assert cas == pytest.approx(150.0, rel=1e-6)
        assert tas > cas


class TestCrossoverAltitude:
    def test_crossover_alt_typical_values(self):
        # 150 m/s CAS (~290 kts) / M0.8 crosses over near FL320
        alt = aero.crossoveralt(150.0, 0.8)
        assert 8000.0 < alt < 12000.0

    def test_crossover_alt_consistency(self):
        # At the crossover altitude, CAS and Mach describe the same TAS
        cas, mach = 150.0, 0.8
        alt = aero.crossoveralt(cas, mach)
        tas_from_cas = aero.vcas2tas(cas, alt)
        tas_from_mach = aero.vmach2tas(mach, alt)
        assert tas_from_cas == pytest.approx(tas_from_mach, rel=1e-2)


class TestUnitConstants:
    def test_unit_constants(self):
        assert aero.ft == pytest.approx(0.3048)
        assert aero.kts == pytest.approx(0.514444, rel=1e-4)
        assert aero.nm == pytest.approx(1852.0)
        assert aero.fpm == pytest.approx(0.3048 / 60.0, rel=1e-6)
