"""OpenAP-based aircraft performance model.

This module provides [`OpenAP`][.OpenAP], the aircraft performance implementation
used by the MiniSky traffic object ([`runtime.traffic.perf`][.OpenAP]). It combines the
coefficient database (`coeff`), flight-phase logic (`phase`), and the
empirical thrust/fuel-flow models (`thrust`) into per-aircraft vectorised
computations of drag, thrust, fuel flow, and kinematic envelope limits. All
internal quantities are in SI units.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import numpy as np

import minisky._internal.performance.coefficients as coeff
import minisky._internal.performance.phase as ph
import minisky.aero as aero  # noqa: PLR0402
from minisky import quantities as q
from minisky._internal.command import AcId, command
from minisky._internal.performance import thrust
from minisky._internal.performance.phase import FlightPhase
from minisky._internal.result import Ok, Result
from minisky._internal.traffic_arrays import TrafficArrays
from minisky.types import AircraftIndex

if TYPE_CHECKING:
    from minisky._internal.traffic import Traffic


class OpenAP(TrafficArrays):
    """
    Open-source Aircraft Performance (OpenAP) Model

    Holds per-aircraft performance state in numpy arrays. On aircraft
    creation, type-specific coefficients (mass, wing area, engines, drag
    polar, envelope limits) are looked up in the OpenAP database; unknown
    fixed-wing types fall back to the B744. Every update step the flight
    phase is inferred from the aircraft state, after which drag (parabolic
    drag polar), maximum and net thrust, fuel flow (quadratic ICAO model),
    and phase-dependent speed limits are recomputed. Both fixed-wing aircraft
    and simple rotorcraft (envelope-only) are supported.
    """

    def __init__(self, traffic: Traffic) -> None:
        super().__init__()
        self.traffic = traffic

        self.ac_warning = False
        self.eng_warning = False

        self.coeff = coeff.Coefficient()

        with self.settrafarrays():
            self.actype = np.array([], dtype=str)
            self.Sref: q.AreaM2[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.engtype = np.array([])

            self.mass: q.MassKg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.phase = np.array([], dtype=int)
            """Current OpenAP flight-phase identifiers."""
            self.cd0: q.ZeroLiftDragCoefficient[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Zero-lift drag coefficient selected for the current flight phase."""
            self.k: q.InducedDragFactor[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Induced-drag factor selected for the current flight phase."""
            self.bank: q.BankAngleDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.thrust: q.ForceN[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.drag: q.ForceN[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.fuelflow: q.MassFlowKgPerS[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

            self.hmax: q.PressureAltitudeM[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            # TODO(abraham): fixed-wing rows store CAS while rotor rows store TAS; a
            # per-lift-type envelope record would make the speed kind unambiguous.
            self.vmin: q.AirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Minimum operating speed; fixed-wing rows store CAS and rotorcraft rows store TAS."""
            self.vmax: q.AirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Maximum operating speed; fixed-wing rows store CAS and rotorcraft rows store TAS."""
            self.vsmin: q.VerticalRateMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vsmax: q.VerticalRateMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.axmax: q.AccelerationMps2[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

            self.lifttype = np.array([], dtype=int)
            """Per-aircraft `LiftType` values."""
            self.engnum = np.array([], dtype=int)
            self.engthrmax: q.ForceN[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.engbpr: q.BypassRatio[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Engine bypass ratio."""
            self.max_thrust: q.ForceN[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.ff_coeff_a: q.MassFlowKgPerS[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.ff_coeff_b: q.MassFlowKgPerS[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.ff_coeff_c: q.MassFlowKgPerS[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.engpower: q.PowerW[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.cd0_clean: q.ZeroLiftDragCoefficient[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.k_clean: q.InducedDragFactor[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.cd0_to: q.ZeroLiftDragCoefficient[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.k_to: q.InducedDragFactor[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.cd0_ld: q.ZeroLiftDragCoefficient[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.k_ld: q.InducedDragFactor[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.delta_cd_gear: q.DragCoefficient[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Landing-gear increment applied to the drag coefficient."""

            self.vminic: q.CalibratedAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vminer: q.CalibratedAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vminap: q.CalibratedAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vmaxic: q.CalibratedAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vmaxer: q.CalibratedAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vmaxap: q.CalibratedAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

            self.vminto: q.CalibratedAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.hcross: q.PressureAltitudeM[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.mmo: q.MachNumber[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

    def new_implementation(self, implementation: type[TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic object."""
        return implementation(self.traffic)

    def create(self, n: int = 1) -> None:
        """Initialise performance parameters for newly created aircraft.

        Called by the traffic object when aircraft are created. Looks up the
        type of the last created aircraft in the OpenAP coefficient database
        and fills the last `n` array elements with its mass, engine, drag
        polar, and flight-envelope coefficients. Rotorcraft types get the
        (simpler) rotor envelope; unknown fixed-wing types default to B744.

        Args:
            n: Number of appended aircraft. The current implementation assumes the whole batch shares the final aircraft's typecode.
        """
        # TODO(abraham): create(n) assumes the whole appended batch has the typecode
        # of the final aircraft. make batch creation carry per-row type information.
        super().create(n)

        actype = self.traffic.typecode[-1].upper()

        if actype in self.coeff.acs_rotor:
            # NOTE(abraham): OpenAP in core currently knows about the legacy
            # rotor coefficient schema even though multicopter performance is
            # plugin-specific
            self.lifttype[-n:] = coeff.LiftType.ROTORCRAFT
            aircraft = self.coeff.acs_rotor[actype]
            self.mass[-n:] = 0.5 * (aircraft.oew + aircraft.mtow)
            self.engnum[-n:] = aircraft.n_engines
            self.engpower[-n:] = aircraft.engines[0].power

        else:
            if actype not in self.coeff.acs_fixwing:
                actype = "B744"

            aircraft = self.coeff.acs_fixwing[actype]
            # TODO(abraham): parse variant-qualified aircraft type codes (for
            # example A320-232) and select `variant_engines[variant]`; also
            # expose selection among alternative engines that are not tied to a
            # variant.
            engine = aircraft.engines[aircraft.default_engine]
            coeff_a, coeff_b, coeff_c = thrust.compute_eng_ff_coeff(
                engine.ff_idl, engine.ff_app, engine.ff_co, engine.ff_to
            )

            self.lifttype[-n:] = coeff.LiftType.FIXED_WING

            self.Sref[-n:] = aircraft.wing_area
            self.mass[-n:] = 0.5 * (aircraft.oew + aircraft.mtow)

            self.engnum[-n:] = aircraft.engine_count

            self.ff_coeff_a[-n:] = coeff_a
            self.ff_coeff_b[-n:] = coeff_b
            self.ff_coeff_c[-n:] = coeff_c

            self.engthrmax[-n:] = engine.max_thrust
            self.engbpr[-n:] = engine.bpr

        if actype in self.coeff.acs_rotor:
            envelope = self.coeff.acs_rotor[actype].envelope
            self.vmin[-n:] = envelope.v_min
            self.vmax[-n:] = envelope.v_max
            self.vsmin[-n:] = envelope.vs_min
            self.vsmax[-n:] = envelope.vs_max
            self.hmax[-n:] = envelope.h_max

            self.cd0_clean[-n:] = np.nan
            self.k_clean[-n:] = np.nan
            self.cd0_to[-n:] = np.nan
            self.k_to[-n:] = np.nan
            self.cd0_ld[-n:] = np.nan
            self.k_ld[-n:] = np.nan
            self.delta_cd_gear[-n:] = np.nan

        else:
            limits = self.coeff.limits_fixwing[actype]
            self.vminic[-n:] = limits.vminic
            self.vminer[-n:] = limits.vminer
            self.vminap[-n:] = limits.vminap
            self.vmaxic[-n:] = limits.vmaxic
            self.vmaxer[-n:] = limits.vmaxer
            self.vmaxap[-n:] = limits.vmaxap

            self.vsmin[-n:] = limits.vsmin
            self.vsmax[-n:] = limits.vsmax
            self.hmax[-n:] = limits.hmax
            self.axmax[-n:] = limits.axmax
            self.vminto[-n:] = limits.vminto
            self.hcross[-n:] = limits.crosscl
            self.mmo[-n:] = limits.mmo

            polar = self.coeff.dragpolar_fixwing[actype]
            self.cd0_clean[-n:] = polar.cd0_clean
            self.k_clean[-n:] = polar.k_clean
            self.cd0_to[-n:] = polar.cd0_to
            self.k_to[-n:] = polar.k_to
            self.cd0_ld[-n:] = polar.cd0_ld
            self.k_ld[-n:] = polar.k_ld
            self.delta_cd_gear[-n:] = polar.delta_cd_gear

        self.actype[-n:] = [actype] * n

        mask = np.zeros_like(self.actype, dtype=bool)
        mask[-n:] = True
        self.vmin[-n:], self.vmax[-n:] = self._construct_v_limits(mask)

    def update(self) -> None:
        """Periodic update function for performance calculations.

        Re-derives the flight phase from the current speed, vertical rate,
        and altitude, then updates for all (fixed-wing) aircraft:

        - phase-dependent drag polar coefficients (cd0, k) and speed limits;
        - drag from the parabolic drag polar with lift equal to weight;
        - maximum thrust from the empirical bypass-ratio model;
        - net thrust as drag plus mass times current acceleration;
        - fuel flow from the quadratic ICAO fuel-flow fit;
        - maximum acceleration and phase-dependent maximum bank angle.
        """
        self.phase = ph.get(self.lifttype, self.traffic.vs, self.traffic.alt)

        self.vmin, self.vmax = self._construct_v_limits()

        idx_fixwing = np.where(self.lifttype == coeff.LiftType.FIXED_WING)[0]

        # ----- compute drag -----
        self.cd0[self.phase == FlightPhase.GROUND] = (
            self.cd0_to[self.phase == FlightPhase.GROUND]
            + self.delta_cd_gear[self.phase == FlightPhase.GROUND]
        )
        self.cd0[self.phase == FlightPhase.INITIAL_CLIMB] = self.cd0_to[
            self.phase == FlightPhase.INITIAL_CLIMB
        ]
        self.cd0[self.phase == FlightPhase.APPROACH] = self.cd0_ld[
            self.phase == FlightPhase.APPROACH
        ]
        self.cd0[self.phase == FlightPhase.CLIMB] = self.cd0_clean[self.phase == FlightPhase.CLIMB]
        self.cd0[self.phase == FlightPhase.CRUISE] = self.cd0_clean[
            self.phase == FlightPhase.CRUISE
        ]
        self.cd0[self.phase == FlightPhase.DESCENT] = self.cd0_clean[
            self.phase == FlightPhase.DESCENT
        ]
        self.cd0[self.phase == FlightPhase.UNKNOWN] = self.cd0_clean[
            self.phase == FlightPhase.UNKNOWN
        ]

        self.k[self.phase == FlightPhase.GROUND] = self.k_to[self.phase == FlightPhase.GROUND]
        self.k[self.phase == FlightPhase.INITIAL_CLIMB] = self.k_to[
            self.phase == FlightPhase.INITIAL_CLIMB
        ]
        self.k[self.phase == FlightPhase.APPROACH] = self.k_ld[self.phase == FlightPhase.APPROACH]
        self.k[self.phase == FlightPhase.CLIMB] = self.k_clean[self.phase == FlightPhase.CLIMB]
        self.k[self.phase == FlightPhase.CRUISE] = self.k_clean[self.phase == FlightPhase.CRUISE]
        self.k[self.phase == FlightPhase.DESCENT] = self.k_clean[self.phase == FlightPhase.DESCENT]
        self.k[self.phase == FlightPhase.UNKNOWN] = self.k_clean[self.phase == FlightPhase.UNKNOWN]

        rho = aero.vdensity(self.traffic.alt[idx_fixwing])
        vtas = self.traffic.tas[idx_fixwing]
        rhovs = 0.5 * rho * vtas**2 * self.Sref[idx_fixwing]
        cl = self.mass[idx_fixwing] * aero.g0 / rhovs
        self.drag[idx_fixwing] = rhovs * (self.cd0[idx_fixwing] + self.k[idx_fixwing] * cl**2)

        # ----- compute maximum thrust -----
        max_thrustratio_fixwing = thrust.compute_max_thr_ratio(
            self.phase[idx_fixwing],
            self.engbpr[idx_fixwing],
            self.traffic.tas[idx_fixwing],
            self.traffic.alt[idx_fixwing],
            self.traffic.vs[idx_fixwing],
            self.engnum[idx_fixwing] * self.engthrmax[idx_fixwing],
        )
        self.max_thrust[idx_fixwing] = (
            max_thrustratio_fixwing * self.engnum[idx_fixwing] * self.engthrmax[idx_fixwing]
        )

        # ----- compute net thrust -----
        self.thrust[idx_fixwing] = (
            self.drag[idx_fixwing]
            + self.mass[idx_fixwing] * self.traffic.kinematics.ax[idx_fixwing]
        )

        # ----- compute fuel flow -----
        thrustratio_fixwing = self.thrust[idx_fixwing] / (
            self.engnum[idx_fixwing] * self.engthrmax[idx_fixwing]
        )
        self.fuelflow[idx_fixwing] = self.engnum[idx_fixwing] * (
            self.ff_coeff_a[idx_fixwing] * thrustratio_fixwing**2
            + self.ff_coeff_b[idx_fixwing] * thrustratio_fixwing
            + self.ff_coeff_c[idx_fixwing]
        )

        # ----- update max acceleration ----
        self.axmax = self.calc_axmax()

        self.bank = np.where((self.phase == FlightPhase.GROUND), 15, self.bank)
        self.bank = np.where(
            (self.phase == FlightPhase.INITIAL_CLIMB)
            | (self.phase == FlightPhase.CRUISE)
            | (self.phase == FlightPhase.APPROACH),
            35,
            self.bank,
        )

    class PerformanceLimits(NamedTuple):
        tas: q.TrueAirspeedMps[np.ndarray]
        vertical_speed: q.VerticalRateMps[np.ndarray]
        altitude: q.PressureAltitudeM[np.ndarray]

    def limits(
        self,
        intent_v_tas: q.TrueAirspeedMps[np.ndarray],
        intent_vs: q.VerticalRateMps[np.ndarray],
        intent_h: q.PressureAltitudeM[np.ndarray],
        ax: q.AccelerationMps2[np.ndarray],
    ) -> PerformanceLimits:
        """Apply the aircraft performance envelope to commanded state.

        Clips the intended state to the aircraft flight envelope: altitude to
        the ceiling, speed to the CAS limits of the current flight phase and
        the maximum Mach number, and vertical speed to the climb/descent rate
        limits (reduced when simultaneously accelerating). Aircraft on the
        ground below their takeoff speed get zero vertical speed. Rotorcraft
        speed limits are applied directly on TAS.
        """
        allow_h = np.where(intent_h > self.hmax, self.hmax, intent_h)

        # TODO(abraham): #33 should evaluate the CAS envelope at the aircraft's
        # current altitude; using intended altitude can create false speed limiting
        # during large altitude changes.
        intent_v_cas = aero.vtas2cas(intent_v_tas, allow_h)
        allow_v_cas = np.where((intent_v_cas < self.vmin), self.vmin, intent_v_cas)
        allow_v_cas = np.where(intent_v_cas > self.vmax, self.vmax, allow_v_cas)
        allow_v_tas = aero.vcas2tas(allow_v_cas, allow_h)
        allow_v_tas = np.where(
            aero.vtas2mach(allow_v_tas, allow_h) > self.mmo,
            aero.vmach2tas(self.mmo, allow_h),
            allow_v_tas,
        )  # maximum cannot exceed MMO

        vs_max_with_acc = (1 - ax / self.axmax) * self.vsmax
        vs_min_with_acc = (1 - ax / self.axmax) * self.vsmin
        allow_vs = np.where(
            (intent_vs > 0) & (intent_vs > self.vsmax), vs_max_with_acc, intent_vs
        )  # for climb with vs larger than vsmax
        allow_vs = np.where(
            (intent_vs < 0) & (intent_vs < self.vsmin), vs_min_with_acc, allow_vs
        )  # for descent with vs smaller than vsmin (negative)
        allow_vs = np.where(
            (self.phase == FlightPhase.GROUND) & (self.traffic.tas < self.vminto), 0, allow_vs
        )  # takeoff aircraft

        ir = np.where(self.lifttype == coeff.LiftType.ROTORCRAFT)[0]
        allow_v_tas[ir] = np.where(
            (intent_v_tas[ir] < self.vmin[ir]), self.vmin[ir], intent_v_tas[ir]
        )
        allow_v_tas[ir] = np.where(
            (intent_v_tas[ir] > self.vmax[ir]), self.vmax[ir], allow_v_tas[ir]
        )
        allow_vs[ir] = np.where((intent_vs[ir] < self.vsmin[ir]), self.vsmin[ir], intent_vs[ir])
        allow_vs[ir] = np.where((intent_vs[ir] > self.vsmax[ir]), self.vsmax[ir], allow_vs[ir])

        return self.PerformanceLimits(allow_v_tas, allow_vs, allow_h)

    class CurrentPerformanceLimits(NamedTuple):
        minimum_tas: q.TrueAirspeedMps
        maximum_tas: q.TrueAirspeedMps
        minimum_vertical_speed: q.VerticalRateMps
        maximum_vertical_speed: q.VerticalRateMps

    def currentlimits(
        self, idx: AircraftIndex | np.ndarray | None = None
    ) -> CurrentPerformanceLimits:
        """Get the current kinematic performance envelope.

        Converts the phase-dependent CAS limits to TAS at the current
        altitude; the maximum is additionally capped by the maximum
        operating Mach number.

        Args:
            idx: Aircraft index/indices, or `None` to return limits for the full fleet.
        """
        vtasmin = aero.vcas2tas(self.vmin, self.traffic.alt)

        vtasmax = np.minimum(
            aero.vcas2tas(self.vmax, self.traffic.alt),
            aero.vmach2tas(self.mmo, self.traffic.alt),
        )

        if idx is not None:
            return self.CurrentPerformanceLimits(
                vtasmin[idx], vtasmax[idx], self.vsmin[idx], self.vsmax[idx]
            )
        return self.CurrentPerformanceLimits(vtasmin, vtasmax, self.vsmin, self.vsmax)

    class SpeedLimits(NamedTuple):
        minimum: q.AirspeedMps[np.ndarray]
        """Minimum speed; fixed-wing rows are CAS and rotor rows are TAS."""
        maximum: q.AirspeedMps[np.ndarray]
        """Maximum speed; fixed-wing rows are CAS and rotor rows are TAS."""

    def _construct_v_limits(self, mask: bool | np.ndarray = True) -> SpeedLimits:
        """Compute speed limits from aircraft model and flight phase.

        For fixed-wing aircraft the applicable minimum and maximum calibrated
        airspeed of the current flight phase is selected (initial climb,
        en-route, approach, or ground). Rotorcraft keep their static limits.
        The default mask selects every aircraft.

        Args:
            mask: Boolean mask selecting aircraft; the scalar default `True` selects the full fleet.
        """
        # TODO(abraham): `bool | ndarray` is an awkward mask contract; use `None`
        # for all aircraft and a typed boolean mask for subsets.
        n = len(self.actype)
        vmin = np.zeros(n)
        vmax = np.zeros(n)

        ifw = np.where(np.logical_and(self.lifttype == coeff.LiftType.FIXED_WING, mask))[0]
        vminfw = np.zeros(len(ifw))
        vmaxfw = np.zeros(len(ifw))

        vminfw = np.where(self.phase[ifw] == FlightPhase.UNKNOWN, 0, vminfw)
        vminfw = np.where(self.phase[ifw] == FlightPhase.INITIAL_CLIMB, self.vminic[ifw], vminfw)
        fixedwing_phase = self.phase[ifw]
        enroute = (
            (fixedwing_phase == FlightPhase.CLIMB)
            | (fixedwing_phase == FlightPhase.CRUISE)
            | (fixedwing_phase == FlightPhase.DESCENT)
        )
        vminfw = np.where(enroute, self.vminer[ifw], vminfw)
        vminfw = np.where(self.phase[ifw] == FlightPhase.APPROACH, self.vminap[ifw], vminfw)
        vminfw = np.where(self.phase[ifw] == FlightPhase.GROUND, 0, vminfw)

        vmaxfw = np.where(self.phase[ifw] == FlightPhase.UNKNOWN, self.vmaxer[ifw], vmaxfw)
        vmaxfw = np.where(self.phase[ifw] == FlightPhase.INITIAL_CLIMB, self.vmaxic[ifw], vmaxfw)
        vmaxfw = np.where(enroute, self.vmaxer[ifw], vmaxfw)
        vmaxfw = np.where(self.phase[ifw] == FlightPhase.APPROACH, self.vmaxap[ifw], vmaxfw)
        vmaxfw = np.where(self.phase[ifw] == FlightPhase.GROUND, self.vmaxic[ifw], vmaxfw)

        ir = np.where(np.logical_and(self.lifttype == coeff.LiftType.ROTORCRAFT, mask))[0]
        vminr = self.vmin[ir]
        vmaxr = self.vmax[ir]

        vmin[ifw] = vminfw
        vmax[ifw] = vmaxfw
        vmin[ir] = vminr
        vmax[ir] = vmaxr

        if isinstance(mask, bool):
            return self.SpeedLimits(vmin, vmax)
        return self.SpeedLimits(vmin[mask], vmax[mask])

    def calc_axmax(self) -> q.AccelerationMps2[np.ndarray]:
        """Compute the maximum longitudinal acceleration per aircraft.

        In flight the maximum acceleration follows from the excess thrust:
        (max_thrust - drag) / mass. Fixed constants are used for fixed-wing
        aircraft on the ground (2 m/s^2) and rotorcraft (3.5 m/s^2), with a
        global lower bound of 0.5 m/s^2.
        """
        axmax_fixwing_ground = 2
        axmax_rotor = 3.5

        axmax = (self.max_thrust - self.drag) / self.mass

        axmax[self.phase == FlightPhase.GROUND] = axmax_fixwing_ground

        axmax[self.lifttype == coeff.LiftType.ROTORCRAFT] = axmax_rotor

        axmax[axmax < 0.5] = 0.5

        return axmax

    @command(name="PERFSTATS", aliases=("PERFINFO", "PERFDATA"))
    def show_performance(self, acidx: AcId) -> Result[str, str]:
        """Report the current performance state of an aircraft.

        Implements the PERFSTATS stack command output: flight phase, thrust,
        drag, fuel flow, speed and vertical-speed envelopes, and ceiling in
        aviation units (kN, kg/s, kts, fpm, ft).
        """
        return Ok(
            f"Flight phase: {ph.readable_phase(FlightPhase(int(self.phase[acidx])))}\n"
            f"Thrust: {q.n_to_kn(self.thrust[acidx]):.0f} kN\n"
            f"Drag: {q.n_to_kn(self.drag[acidx]):.0f} kN\n"
            f"Fuel flow: {self.fuelflow[acidx]:.2f} kg/s\n"
            f"Speed envelope: [{q.mps_to_kt(self.vmin[acidx]):.0f}, {q.mps_to_kt(self.vmax[acidx]):.0f}] kts\n"
            f"Vertical speed envelope: [{q.mps_to_fpm(self.vsmin[acidx]):.0f}, {q.mps_to_fpm(self.vsmax[acidx]):.0f}] fpm\n"
            f"Ceiling: {q.m_to_ft(self.hmax[acidx]):.0f} ft"
        )
