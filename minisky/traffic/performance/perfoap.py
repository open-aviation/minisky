"""OpenAP-based aircraft performance model.

This module provides :class:`OpenAP`, the aircraft performance implementation
used by the MiniSky traffic object (``minisky.traf.perf``). It combines the
coefficient database (``coeff``), flight-phase logic (``phase``), and the
empirical thrust/fuel-flow models (``thrust``) into per-aircraft vectorised
computations of drag, thrust, fuel flow, and kinematic envelope limits. All
internal quantities are in SI units.
"""

import numpy as np
from openap import Drag, FuelFlow

import minisky
from minisky.tools import aero
from minisky.tools.aero import fpm, ft, kts
from minisky.core.trafficarrays import TrafficArrays

from . import coeff, thrust
from . import phase as ph


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

    Methods:
        create(): initialize new aircraft with performance parameters
        update(): update performance parameters

    Attributes:
        actype (ndarray): ICAO aircraft type code per aircraft.
        lifttype (ndarray): Lift type, fixed-wing (1) or rotor (2) [-].
        Sref (ndarray): Wing reference surface area [m^2].
        mass (ndarray): Effective mass, mean of OEW and MTOW [kg].
        phase (ndarray): Current flight phase identifier (see ``phase``) [-].
        cd0 (ndarray): Zero-lift drag coefficient for current phase [-].
        k (ndarray): Induced drag factor for current phase [-].
        bank (ndarray): Maximum bank angle for current phase [deg].
        thrust (ndarray): Net thrust (drag + mass * acceleration) [N].
        drag (ndarray): Total aerodynamic drag [N].
        fuelflow (ndarray): Fuel flow of all engines [kg/s].
        max_thrust (ndarray): Maximum available thrust at current state [N].
        hmax (ndarray): Flight ceiling [m].
        vmin (ndarray): Minimum operating calibrated airspeed [m/s].
        vmax (ndarray): Maximum operating calibrated airspeed [m/s].
        vsmin (ndarray): Maximum descent rate (negative) [m/s].
        vsmax (ndarray): Maximum climb rate [m/s].
        axmax (ndarray): Maximum longitudinal acceleration [m/s^2].
        mmo (ndarray): Maximum operating Mach number [-].
        engnum (ndarray): Number of engines [-].
        engthrmax (ndarray): Maximum static thrust per engine [N].
        engbpr (ndarray): Engine bypass ratio [-].
        ff_coeff_a/b/c (ndarray): Quadratic ICAO fuel-flow fit coefficients.
    """

    def __init__(self):
        super().__init__()

        self.ac_warning = False  # aircraft mdl to default warning
        self.eng_warning = False  # aircraft engine to default warning

        self.coeff = coeff.Coefficient()

        with self.settrafarrays():
            
            # --- fixed parameters ---
            self.actype = np.array([], dtype=str)  # aircraft type
            self.Sref = np.array([])  # wing reference surface area [m^2]
            self.engtype = np.array([])  # integer, aircraft.ENG_TF...

            # --- dynamic parameters ---
            self.mass = np.array([])  # effective mass [kg]
            self.phase = np.array([])
            self.cd0 = np.array([])
            self.k = np.array([])
            self.bank = np.array([])
            self.thrust = np.array([])  # thrust
            self.drag = np.array([])  # drag
            self.fuelflow = np.array([])  # fuel flow

            # Envelope limits per aircraft
            self.hmax = np.array([])  # Flight ceiling [m]
            self.vmin = np.array([])  # Minimum operating speed [m/s]
            self.vmax = np.array([])  # Maximum operating speed [m/s]
            self.vsmin = np.array([])  # Maximum descent speed [m/s]
            self.vsmax = np.array([])  # Maximum climb speed [m/s]
            self.axmax = np.array([])  # Max/min acceleration [m/s2]
            
            self.lifttype = np.array([])  # lift type, fixwing [1] or rotor [2]
            self.engnum = np.array([], dtype=int)  # number of engines
            self.engthrmax = np.array([])  # static engine thrust
            self.engbpr = np.array([])  # engine bypass ratio
            self.max_thrust = np.array([])  # thrust ratio at current alt spd
            self.ff_coeff_a = np.array([])  # icao fuel flows coefficient a
            self.ff_coeff_b = np.array([])  # icao fuel flows coefficient b
            self.ff_coeff_c = np.array([])  # icao fuel flows coefficient c
            self.engpower = np.array([])  # engine power, rotor ac
            self.cd0_clean = np.array([])  # Cd0, clean configuration
            self.k_clean = np.array([])  # k, clean configuration
            self.cd0_to = np.array([])  # Cd0, takeoff configuration
            self.k_to = np.array([])  # k, takeoff configuration
            self.cd0_ld = np.array([])  # Cd0, landing configuration
            self.k_ld = np.array([])  # k, landing configuration
            self.delta_cd_gear = np.array([])  # landing gear

            self.vminic = np.array([])
            self.vminer = np.array([])
            self.vminap = np.array([])
            self.vmaxic = np.array([])
            self.vmaxer = np.array([])
            self.vmaxap = np.array([])

            self.vminto = np.array([])
            self.hcross = np.array([])
            self.mmo = np.array([])

    def create(self, n=1):
        """Initialise performance parameters for newly created aircraft.

        Called by the traffic object when aircraft are created. Looks up the
        type of the last created aircraft in the OpenAP coefficient database
        and fills the last ``n`` array elements with its mass, engine, drag
        polar, and flight-envelope coefficients. Rotorcraft types get the
        (simpler) rotor envelope; unknown fixed-wing types default to B744.

        Args:
            n (int): Number of newly created aircraft (all assumed to be of
                the same type as the last created aircraft).
        """
        # cautious! considering multiple created aircraft with same type
        super().create(n)

        actype = minisky.traf.typecode[-1].upper()

        # initialize aircraft / engine performance parameters
        # check fixwing or rotor, default to fixwing
        if actype in self.coeff.actypes_rotor:
            self.lifttype[-n:] = coeff.LIFT_ROTOR
            self.mass[-n:] = 0.5 * (
                self.coeff.acs_rotor[actype]["oew"]
                + self.coeff.acs_rotor[actype]["mtow"]
            )
            self.engnum[-n:] = int(self.coeff.acs_rotor[actype]["n_engines"])
            self.engpower[-n:] = self.coeff.acs_rotor[actype]["engines"][0][1]

        else:
            
            # convert to known aircraft type
            if actype.lower() not in self.coeff.actypes_fixwing:
                actype = "B744"

            # populate fuel flow model
            es = self.coeff.acs_fixwing[actype]["engines"]
            e = es[list(es.keys())[0]]
            coeff_a, coeff_b, coeff_c = thrust.compute_eng_ff_coeff(
                e["ff_idl"], e["ff_app"], e["ff_co"], e["ff_to"]
            )

            self.lifttype[-n:] = coeff.LIFT_FIXWING

            self.Sref[-n:] = self.coeff.acs_fixwing[actype]["wing"]['area']
            self.mass[-n:] = 0.5 * (
                self.coeff.acs_fixwing[actype]["oew"]
                + self.coeff.acs_fixwing[actype]["mtow"]
            )

            self.engnum[-n:] = int(self.coeff.acs_fixwing[actype]["engine"]["number"])

            self.ff_coeff_a[-n:] = coeff_a
            self.ff_coeff_b[-n:] = coeff_b
            self.ff_coeff_c[-n:] = coeff_c

            all_ac_engs = list(self.coeff.acs_fixwing[actype]["engines"].keys())
            self.engthrmax[-n:] = self.coeff.acs_fixwing[actype]["engines"][
                all_ac_engs[0]
            ]["max_thrust"]
            self.engbpr[-n:] = self.coeff.acs_fixwing[actype]["engines"][
                all_ac_engs[0]
            ]["bpr"]

        # init type specific coefficients for flight envelops
        if actype in self.coeff.limits_rotor.keys():  # rotorcraft
            self.vmin[-n:] = self.coeff.limits_rotor[actype]["vmin"]
            self.vmax[-n:] = self.coeff.limits_rotor[actype]["vmax"]
            self.vsmin[-n:] = self.coeff.limits_rotor[actype]["vsmin"]
            self.vsmax[-n:] = self.coeff.limits_rotor[actype]["vsmax"]
            self.hmax[-n:] = self.coeff.limits_rotor[actype]["hmax"]

            self.vsmin[-n:] = self.coeff.limits_rotor[actype]["vsmin"]
            self.vsmax[-n:] = self.coeff.limits_rotor[actype]["vsmax"]
            self.hmax[-n:] = self.coeff.limits_rotor[actype]["hmax"]

            self.cd0_clean[-n:] = np.nan
            self.k_clean[-n:] = np.nan
            self.cd0_to[-n:] = np.nan
            self.k_to[-n:] = np.nan
            self.cd0_ld[-n:] = np.nan
            self.k_ld[-n:] = np.nan
            self.delta_cd_gear[-n:] = np.nan

        else:

            if actype not in self.coeff.limits_fixwing.keys():
                actype = "B744"

            self.vminic[-n:] = self.coeff.limits_fixwing[actype]["vminic"]
            self.vminer[-n:] = self.coeff.limits_fixwing[actype]["vminer"]
            self.vminap[-n:] = self.coeff.limits_fixwing[actype]["vminap"]
            self.vmaxic[-n:] = self.coeff.limits_fixwing[actype]["vmaxic"]
            self.vmaxer[-n:] = self.coeff.limits_fixwing[actype]["vmaxer"]
            self.vmaxap[-n:] = self.coeff.limits_fixwing[actype]["vmaxap"]

            self.vsmin[-n:] = self.coeff.limits_fixwing[actype]["vsmin"]
            self.vsmax[-n:] = self.coeff.limits_fixwing[actype]["vsmax"]
            self.hmax[-n:] = self.coeff.limits_fixwing[actype]["hmax"]
            self.axmax[-n:] = self.coeff.limits_fixwing[actype]["axmax"]
            self.vminto[-n:] = self.coeff.limits_fixwing[actype]["vminto"]
            self.hcross[-n:] = self.coeff.limits_fixwing[actype]["crosscl"]
            self.mmo[-n:] = self.coeff.limits_fixwing[actype]["mmo"]

            self.cd0_clean[-n:] = self.coeff.dragpolar_fixwing[actype]["cd0_clean"]
            self.k_clean[-n:] = self.coeff.dragpolar_fixwing[actype]["k_clean"]
            self.cd0_to[-n:] = self.coeff.dragpolar_fixwing[actype]["cd0_to"]
            self.k_to[-n:] = self.coeff.dragpolar_fixwing[actype]["k_to"]
            self.cd0_ld[-n:] = self.coeff.dragpolar_fixwing[actype]["cd0_ld"]
            self.k_ld[-n:] = self.coeff.dragpolar_fixwing[actype]["k_ld"]
            self.delta_cd_gear[-n:] = self.coeff.dragpolar_fixwing[actype][
                "delta_cd_gear"
            ]

        # append update actypes, after removing unknown types
        self.actype[-n:] = [actype] * n

        # Update envelope speed limits
        mask = np.zeros_like(self.actype, dtype=bool)
        mask[-n:] = True
        self.vmin[-n:], self.vmax[-n:] = self._construct_v_limits(mask)

    def update(self, dt=1):
        """Periodic update function for performance calculations.

        Re-derives the flight phase from the current speed, vertical rate,
        and altitude, then updates for all (fixed-wing) aircraft:

        - phase-dependent drag polar coefficients (cd0, k) and speed limits;
        - drag from the parabolic drag polar with lift equal to weight;
        - maximum thrust from the empirical bypass-ratio model;
        - net thrust as drag plus mass times current acceleration;
        - fuel flow from the quadratic ICAO fuel-flow fit;
        - maximum acceleration and phase-dependent maximum bank angle.

        Args:
            dt (float): Update timestep [s] (currently unused).
        """
        # update phase, infer from spd, roc, alt
        lenph1 = len(self.phase)
        self.phase = ph.get(
            self.lifttype,
            minisky.traf.tas,
            minisky.traf.vs,
            minisky.traf.alt,
            unit="SI",
        )

        # update speed limits, based on phase change
        self.vmin, self.vmax = self._construct_v_limits()

        idx_fixwing = np.where(self.lifttype == coeff.LIFT_FIXWING)[0]

        # ----- compute drag -----
        # update drage coefficient based on flight phase
        self.cd0[self.phase == ph.GD] = (
            self.cd0_to[self.phase == ph.GD] + self.delta_cd_gear[self.phase == ph.GD]
        )
        self.cd0[self.phase == ph.IC] = self.cd0_to[self.phase == ph.IC]
        self.cd0[self.phase == ph.AP] = self.cd0_ld[self.phase == ph.AP]
        self.cd0[self.phase == ph.CL] = self.cd0_clean[self.phase == ph.CL]
        self.cd0[self.phase == ph.CR] = self.cd0_clean[self.phase == ph.CR]
        self.cd0[self.phase == ph.DE] = self.cd0_clean[self.phase == ph.DE]
        self.cd0[self.phase == ph.NA] = self.cd0_clean[self.phase == ph.NA]

        self.k[self.phase == ph.GD] = self.k_to[self.phase == ph.GD]
        self.k[self.phase == ph.IC] = self.k_to[self.phase == ph.IC]
        self.k[self.phase == ph.AP] = self.k_ld[self.phase == ph.AP]
        self.k[self.phase == ph.CL] = self.k_clean[self.phase == ph.CL]
        self.k[self.phase == ph.CR] = self.k_clean[self.phase == ph.CR]
        self.k[self.phase == ph.DE] = self.k_clean[self.phase == ph.DE]
        self.k[self.phase == ph.NA] = self.k_clean[self.phase == ph.NA]

        rho = aero.vdensity(minisky.traf.alt[idx_fixwing])
        vtas = minisky.traf.tas[idx_fixwing]
        rhovs = 0.5 * rho * vtas**2 * self.Sref[idx_fixwing]
        cl = self.mass[idx_fixwing] * aero.g0 / rhovs
        self.drag[idx_fixwing] = rhovs * (
            self.cd0[idx_fixwing] + self.k[idx_fixwing] * cl**2
        )

        # ----- compute maximum thrust -----
        max_thrustratio_fixwing = thrust.compute_max_thr_ratio(
            self.phase[idx_fixwing],
            self.engbpr[idx_fixwing],
            minisky.traf.tas[idx_fixwing],
            minisky.traf.alt[idx_fixwing],
            minisky.traf.vs[idx_fixwing],
            self.engnum[idx_fixwing] * self.engthrmax[idx_fixwing],
        )
        self.max_thrust[idx_fixwing] = (
            max_thrustratio_fixwing
            * self.engnum[idx_fixwing]
            * self.engthrmax[idx_fixwing]
        )

        # ----- compute net thrust -----
        self.thrust[idx_fixwing] = (
            self.drag[idx_fixwing]
            + self.mass[idx_fixwing] * minisky.traf.ax[idx_fixwing]
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

        # TODO: implement thrust computation for rotor aircraft
        # idx_rotor = np.where(self.lifttype==coeff.LIFT_ROTOR)[0]
        # self.thrust[idx_rotor] = 0

        # update bank angle, due to phase change
        self.bank = np.where((self.phase == ph.GD), 15, self.bank)
        self.bank = np.where(
            (self.phase == ph.IC) | (self.phase == ph.CR) | (self.phase == ph.AP),
            35,
            self.bank,
        )

    def limits(self, intent_v_tas, intent_vs, intent_h, ax):
        """apply limits on indent speed, vertical speed, and altitude (called in pilot module)

        Clips the intended state to the aircraft flight envelope: altitude to
        the ceiling, speed to the CAS limits of the current flight phase and
        the maximum Mach number, and vertical speed to the climb/descent rate
        limits (reduced when simultaneously accelerating). Aircraft on the
        ground below their takeoff speed get zero vertical speed. Rotorcraft
        speed limits are applied directly on TAS.

        Args:
            intent_v_tas (float or 1D-array): intent true airspeed [m/s]
            intent_vs (float or 1D-array): intent vertical speed [m/s]
            intent_h (float or 1D-array): intent altitude [m]
            ax (float or 1D-array): acceleration [m/s^2]

        Returns:
            floats or 1D-arrays: Allowed TAS [m/s], Allowed vertical
                rate [m/s], Allowed altitude [m]
        """
        allow_h = np.where(intent_h > self.hmax, self.hmax, intent_h)

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
            (self.phase == ph.GD) & (minisky.traf.tas < self.vminto), 0, allow_vs
        )  # takeoff aircraft

        # corect rotercraft speed limits
        ir = np.where(self.lifttype == coeff.LIFT_ROTOR)[0]
        allow_v_tas[ir] = np.where(
            (intent_v_tas[ir] < self.vmin[ir]), self.vmin[ir], intent_v_tas[ir]
        )
        allow_v_tas[ir] = np.where(
            (intent_v_tas[ir] > self.vmax[ir]), self.vmax[ir], allow_v_tas[ir]
        )
        allow_vs[ir] = np.where(
            (intent_vs[ir] < self.vsmin[ir]), self.vsmin[ir], intent_vs[ir]
        )
        allow_vs[ir] = np.where(
            (intent_vs[ir] > self.vsmax[ir]), self.vsmax[ir], allow_vs[ir]
        )

        return allow_v_tas, allow_vs, allow_h

    def currentlimits(self, id=None):
        """Get current kinematic performance envelop.

        Converts the phase-dependent CAS limits to TAS at the current
        altitude; the maximum is additionally capped by the maximum
        operating Mach number.

        Args:
            id (int or 1D-array): Aircraft ID(s). Defualt to None (all aircraft).

        Returns:
            floats or 1D-arrays: Min TAS [m/s], Max TAS [m/s],
                Min VS [m/s], Max VS [m/s]
        """
        vtasmin = aero.vcas2tas(self.vmin, minisky.traf.alt)

        vtasmax = np.minimum(
            aero.vcas2tas(self.vmax, minisky.traf.alt),
            aero.vmach2tas(self.mmo, minisky.traf.alt),
        )

        if id is not None:
            return vtasmin[id], vtasmax[id], self.vsmin[id], self.vsmax[id]
        else:
            return vtasmin, vtasmax, self.vsmin, self.vsmax

    def _construct_v_limits(self, mask=True):
        """Compute speed limist base on aircraft model and flight phases

        For fixed-wing aircraft the applicable minimum and maximum calibrated
        airspeed of the current flight phase is selected (initial climb,
        en-route, approach, or ground). Rotorcraft keep their static limits.

        Args:
            mask: Indices (boolean) for aircraft to construct speed limits for.
                  When no indices are passed, all aircraft are updated.

        Returns:
            2D-array: vmin, vmax (CAS limits per aircraft [m/s])
        """
        n = len(self.actype)
        vmin = np.zeros(n)
        vmax = np.zeros(n)

        ifw = np.where(np.logical_and(self.lifttype == coeff.LIFT_FIXWING, mask))[0]
        vminfw = np.zeros(len(ifw))
        vmaxfw = np.zeros(len(ifw))

        # fixwing
        # obtain flight envelope for speed, roc, and alt, based on flight phase

        # --- minimum speed ---
        vminfw = np.where(self.phase[ifw] == ph.NA, 0, vminfw)
        vminfw = np.where(self.phase[ifw] == ph.IC, self.vminic[ifw], vminfw)
        vminfw = np.where(
            (self.phase[ifw] >= ph.CL) | (self.phase[ifw] <= ph.DE),
            self.vminer[ifw],
            vminfw,
        )
        vminfw = np.where(self.phase[ifw] == ph.AP, self.vminap[ifw], vminfw)
        vminfw = np.where(self.phase[ifw] == ph.GD, 0, vminfw)

        # --- maximum speed ---
        vmaxfw = np.where(self.phase[ifw] == ph.NA, self.vmaxer[ifw], vmaxfw)
        vmaxfw = np.where(self.phase[ifw] == ph.IC, self.vmaxic[ifw], vmaxfw)
        vmaxfw = np.where(
            (self.phase[ifw] >= ph.CL) | (self.phase[ifw] <= ph.DE),
            self.vmaxer[ifw],
            vmaxfw,
        )
        vmaxfw = np.where(self.phase[ifw] == ph.AP, self.vmaxap[ifw], vmaxfw)
        vmaxfw = np.where(self.phase[ifw] == ph.GD, self.vmaxic[ifw], vmaxfw)

        # rotor
        ir = np.where(np.logical_and(self.lifttype == coeff.LIFT_ROTOR, mask))[0]
        vminr = self.vmin[ir]
        vmaxr = self.vmax[ir]

        vmin[ifw] = vminfw
        vmax[ifw] = vmaxfw
        vmin[ir] = vminr
        vmax[ir] = vmaxr

        if isinstance(mask, bool):
            return vmin, vmax
        return vmin[mask], vmax[mask]

    def calc_axmax(self):
        """Compute the maximum longitudinal acceleration per aircraft.

        In flight the maximum acceleration follows from the excess thrust:
        (max_thrust - drag) / mass. Fixed constants are used for fixed-wing
        aircraft on the ground (2 m/s^2) and rotorcraft (3.5 m/s^2), with a
        global lower bound of 0.5 m/s^2.

        Returns:
            ndarray: Maximum acceleration per aircraft [m/s^2].
        """
        # accelerations depending on phase and wing type
        axmax_fixwing_ground = 2
        axmax_rotor = 3.5

        axmax = np.zeros(minisky.traf.ntraf)

        # fix-wing, in flight
        axmax = (self.max_thrust - self.drag) / self.mass

        # fix-wing, on ground
        axmax[self.phase == ph.GD] = axmax_fixwing_ground

        # drones
        axmax[self.lifttype == coeff.LIFT_ROTOR] = axmax_rotor

        # global minumum acceleration
        axmax[axmax < 0.5] = 0.5

        return axmax

    def show_performance(self, acid):
        """Report the current performance state of one aircraft.

        Implements the PERFSTATS stack command output: flight phase, thrust,
        drag, fuel flow, speed and vertical-speed envelopes, and ceiling in
        aviation units (kN, kg/s, kts, fpm, ft).

        Args:
            acid (int): Aircraft index.

        Returns:
            tuple: (True, message (str)) for the command stack.
        """
        return (
            True,
            f"Flight phase: {ph.readable_phase(self.phase[acid])}\n"
            f"Thrust: {self.thrust[acid] / 1000:.0f} kN\n"
            f"Drag: {self.drag[acid] / 1000:.0f} kN\n"
            f"Fuel flow: {self.fuelflow[acid]:.2f} kg/s\n"
            f"Speed envelope: [{self.vmin[acid] / kts:.0f}, {self.vmax[acid] / kts:.0f}] kts\n"
            f"Vertical speed envelope: [{self.vsmin[acid] / fpm:.0f}, {self.vsmax[acid] / fpm:.0f}] fpm\n"
            f"Ceiling: {self.hmax[acid] / ft:.0f} ft",
        )
