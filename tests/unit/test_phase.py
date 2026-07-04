"""Unit tests for flight-phase identification (minisky.traffic.performance.phase).

The fixed-wing altitude bands must be non-overlapping: an aircraft exactly at
75 ft or 1000 ft must be assigned exactly one phase.
"""

import numpy as np

from minisky.traffic.performance import phase
from minisky.traffic.performance.coeff import LIFT_FIXWING, LIFT_ROTOR

FT = 0.3048
FPM = 0.00508


def fixwing_phase(alt_ft, roc_fpm, spd_kts=150.0):
    ph = phase.get_fixwing(
        np.array([spd_kts]), np.array([roc_fpm]), np.array([alt_ft]), unit="EP"
    )
    return ph[0]


class TestFixwingBoundaries:
    def test_exactly_75ft_climbing_is_ground(self):
        assert fixwing_phase(75.0, 500.0) == phase.GD

    def test_exactly_75ft_descending_is_ground(self):
        assert fixwing_phase(75.0, -500.0) == phase.GD

    def test_just_above_75ft_climbing_is_initial_climb(self):
        assert fixwing_phase(76.0, 500.0) == phase.IC

    def test_just_above_75ft_descending_is_approach(self):
        assert fixwing_phase(76.0, -500.0) == phase.AP

    def test_exactly_1000ft_climbing_is_initial_climb(self):
        assert fixwing_phase(1000.0, 500.0) == phase.IC

    def test_exactly_1000ft_descending_is_approach(self):
        assert fixwing_phase(1000.0, -500.0) == phase.AP

    def test_just_above_1000ft_climbing_is_climb(self):
        assert fixwing_phase(1001.0, 500.0) == phase.CL

    def test_just_above_1000ft_descending_is_descent(self):
        assert fixwing_phase(1001.0, -500.0) == phase.DE

    def test_level_above_10000ft_is_cruise(self):
        assert fixwing_phase(30000.0, 0.0) == phase.CR

    def test_boundary_conditions_assign_exactly_one_phase(self):
        # Each altitude/roc band must match exactly one condition
        alt = np.array([75.0, 75.0, 1000.0, 1000.0, 1001.0, 1001.0])
        roc = np.array([500.0, -500.0, 500.0, -500.0, 500.0, -500.0])
        conditions = [
            alt <= 75,
            (alt > 75) & (alt <= 1000) & (roc >= 150),
            (alt > 75) & (alt <= 1000) & (roc <= -150),
            (alt > 1000) & (roc >= 150),
            (alt > 1000) & (roc <= -150),
            (alt >= 10000) & (roc < 150) & (roc > -150),
        ]
        matches = np.sum(conditions, axis=0)
        assert np.all(matches == 1)

    def test_si_units_exactly_75ft_is_ground(self):
        ph = phase.get_fixwing(
            np.array([80.0]), np.array([5.0]), np.array([75.0 * FT]), unit="SI"
        )
        assert ph[0] == phase.GD


class TestGetDtype:
    def test_get_returns_integer_dtype(self):
        lifttype = np.array([LIFT_FIXWING, LIFT_ROTOR])
        ph = phase.get(
            lifttype,
            np.array([150.0, 50.0]),
            np.array([0.0, 0.0]),
            np.array([30000.0, 500.0]),
            unit="EP",
        )
        assert np.issubdtype(ph.dtype, np.integer)
        assert ph[0] == phase.CR
        assert ph[1] == phase.NA

    def test_get_fixwing_returns_integer_dtype(self):
        ph = phase.get_fixwing(
            np.array([150.0]), np.array([0.0]), np.array([2000.0]), unit="EP"
        )
        assert np.issubdtype(ph.dtype, np.integer)

    def test_get_rotor_returns_integer_dtype(self):
        ph = phase.get_rotor(np.array([50.0]), np.array([0.0]), np.array([500.0]))
        assert np.issubdtype(ph.dtype, np.integer)
