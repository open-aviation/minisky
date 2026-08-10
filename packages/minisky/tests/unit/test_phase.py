"""Unit tests for SI-only OpenAP flight-phase identification."""

import numpy as np
from minisky import quantities as q
from minisky.traffic.performance import phase
from minisky.traffic.performance.coeff import LiftType
from minisky.traffic.performance.phase import FlightPhase


def fixwing_phase(alt_ft: float, roc_fpm: float) -> FlightPhase:
    alt = np.array([q.ft_to_m(alt_ft)])
    roc = np.array([q.fpm_to_mps(roc_fpm)])
    result = phase.get_fixwing(roc, alt)
    return FlightPhase(int(result[0]))


class TestFixwingBoundaries:
    def test_ground_boundary(self) -> None:
        assert fixwing_phase(75.0, 500.0) == FlightPhase.GROUND
        assert fixwing_phase(75.0, -500.0) == FlightPhase.GROUND

    def test_terminal_boundary(self) -> None:
        assert fixwing_phase(76.0, 500.0) == FlightPhase.INITIAL_CLIMB
        assert fixwing_phase(76.0, -500.0) == FlightPhase.APPROACH
        assert fixwing_phase(1000.0, 500.0) == FlightPhase.INITIAL_CLIMB
        assert fixwing_phase(1000.0, -500.0) == FlightPhase.APPROACH

    def test_enroute_boundary(self) -> None:
        assert fixwing_phase(1001.0, 500.0) == FlightPhase.CLIMB
        assert fixwing_phase(1001.0, -500.0) == FlightPhase.DESCENT
        assert fixwing_phase(30000.0, 0.0) == FlightPhase.CRUISE


class TestGetDtype:
    def test_get_preserves_unknown_rotor_phase(self) -> None:
        lifttype = np.array([LiftType.FIXED_WING.value, LiftType.ROTORCRAFT.value])
        roc = np.array([0.0, 0.0])
        alt = np.array([q.ft_to_m(30000.0), q.ft_to_m(500.0)])
        result = phase.get(lifttype, roc, alt)
        assert np.issubdtype(result.dtype, np.integer)
        assert result[0] == FlightPhase.CRUISE
        assert result[1] == FlightPhase.UNKNOWN

    def test_get_fixwing_returns_integer_dtype(self) -> None:
        result = phase.get_fixwing(np.array([0.0]), np.array([q.ft_to_m(2000.0)]))
        assert np.issubdtype(result.dtype, np.integer)
