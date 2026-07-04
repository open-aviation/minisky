"""Empirical turbofan thrust and fuel-flow models for the OpenAP performance model.

Provides vectorised estimates of the maximum available thrust of turbofan
engines as a fraction of their maximum static thrust, based on engine bypass
ratio, airspeed, altitude, and vertical rate. Separate models are used for
the takeoff regime and for three in-flight altitude segments (below 10000 ft,
10000-35000 ft, above 35000 ft). Also provides a quadratic fit of the ICAO
engine emission databank fuel-flow points as a function of thrust ratio.
"""

from typing import Any

import numpy as np

from minisky.tools import aero
from minisky.traffic.performance import phase as ph


def compute_max_thr_ratio(
    phase: np.ndarray,
    bpr: np.ndarray,
    v: np.ndarray,
    h: np.ndarray,
    vs: np.ndarray,
    thr0: np.ndarray,
) -> np.ndarray:
    """Computer the dynamic thrust based on engine bypass-ratio, static maximum
    thrust, aircraft true airspeed, and aircraft altitude

    Selects the takeoff thrust model (:func:`tr_takeoff`) for aircraft on the
    ground and the in-flight model (:func:`inflight`) otherwise. The result
    is the ratio of the currently available maximum thrust to the maximum
    static thrust ``thr0``.

    Args:
        phase (int or 1D-array): phase of flight, option: phase.[NA, GD, IC,
            CL, CR, DE, AP]
        bpr (int or 1D-array): engine bypass ratio [-]
        v (int or 1D-array): aircraft true airspeed [m/s]
        h (int or 1D-array): aircraft altitude [m]
        vs (int or 1D-array): aircraft vertical rate [m/s]
        thr0 (int or 1D-array): total maximum static thrust of all engines [N]

    Returns:
        int or 1D-array: maximum thrust ratio (fraction of thr0) [-]
    """

    n = len(phase)

    # ---- thrust ratio at takeoff ----
    ratio_takeoff = tr_takeoff(bpr, v, h)

    # ---- thrust ratio in flight ----
    ratio_inflight = inflight(v, h, vs, thr0)

    # thrust ratio array
    #   LD and GN assume ZERO thrust
    tr = np.ones(n) * ratio_inflight
    tr = np.where(phase == ph.GD, ratio_takeoff, tr)

    return tr


def tr_takeoff(bpr: np.ndarray, v: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Compute thrust ration at take-off.

    Empirical polynomial model of the thrust lapse of a turbofan during the
    takeoff regime, as a function of Mach number and ambient pressure ratio,
    parameterised by the engine bypass ratio.

    Args:
        bpr (int or 1D-array): engine bypass ratio [-]
        v (int or 1D-array): aircraft true airspeed [m/s]
        h (int or 1D-array): aircraft altitude [m]

    Returns:
        int or 1D-array: takeoff thrust ratio (fraction of static thrust) [-]
    """
    G0 = 0.0606 * bpr + 0.6337
    Mach = aero.vtas2mach(v, h)
    P0 = aero.p0
    P = aero.vpressure(h)
    PP = P / P0

    A = -0.4327 * PP**2 + 1.3855 * PP + 0.0472
    Z = 0.9106 * PP**3 - 1.7736 * PP**2 + 1.8697 * PP
    X = 0.1377 * PP**3 - 0.4374 * PP**2 + 1.3003 * PP

    ratio = (
        A
        - 0.377 * (1 + bpr) / np.sqrt((1 + 0.82 * bpr) * G0) * Z * Mach
        + (0.23 + 0.19 * np.sqrt(bpr)) * X * Mach**2
    )

    return ratio


def inflight(v: np.ndarray, h: np.ndarray, vs: np.ndarray, thr0: np.ndarray) -> np.ndarray:
    """Compute thrust ration for inflight.

    Empirical model of the in-flight maximum thrust of a turbofan. The
    thrust at a reference top-of-climb condition (Mach 0.8 at 35000 ft) is
    estimated from the static thrust, then scaled with pressure-ratio-based
    lapse laws for three altitude segments (above 35000 ft, 10000-35000 ft,
    and below 10000 ft), with corrections for calibrated airspeed and rate
    of climb. The result is converted back to a fraction of the maximum
    static thrust.

    Args:
        v (int or 1D-array): aircraft true airspeed [m/s]
        h (int or 1D-array): aircraft altitude [m]
        vs (int or 1D-array): aircraft vertical rate [m/s]
        thr0 (int or 1D-array): total maximum static thrust of all engines [N]

    Returns:
        int or 1D-array: in-flight thrust ratio (fraction of thr0) [-]
    """

    def dfunc(mratio):
        d = -0.4204 * mratio + 1.0824
        return d

    def nfunc(roc):
        n = 2.667e-05 * roc + 0.8633
        return n

    def mfunc(vratio, roc):
        m = -1.2043e-1 * vratio - 8.8889e-9 * roc**2 + 2.4444e-5 * roc + 4.7379e-1
        return m

    roc = np.abs(np.asarray(vs / aero.fpm))
    v = np.where(v < 10, 10, v)

    mach = aero.vtas2mach(v, h)
    vcas = aero.vtas2cas(v, h)

    p = aero.vpressure(h)
    p10 = aero.vpressure(10000 * aero.ft)
    p35 = aero.vpressure(35000 * aero.ft)

    # approximate thrust at top of climb (REF 2)
    F35 = (200 + 0.2 * thr0 / 4.448) * 4.448
    mach_ref = 0.8
    vcas_ref = aero.vmach2cas(mach_ref, 35000 * aero.ft)

    # segment 3: alt > 35000:
    d = dfunc(mach / mach_ref)
    b = (mach / mach_ref) ** (-0.11)
    ratio_seg3 = d * np.log(p / p35) + b

    # segment 2: 10000 < alt <= 35000:
    a = (vcas / vcas_ref) ** (-0.1)
    n = nfunc(roc)
    ratio_seg2 = a * (p / p35) ** (-0.355 * (vcas / vcas_ref) + n)

    # segment 1: alt <= 10000:
    F10 = F35 * a * (p10 / p35) ** (-0.355 * (vcas / vcas_ref) + n)
    m = mfunc(vcas / vcas_ref, roc)
    ratio_seg1 = m * (p / p35) + (F10 / F35 - m * (p10 / p35))

    ratio = np.where(
        h > 35000 * aero.ft,
        ratio_seg3,
        np.where(h > 10000 * aero.ft, ratio_seg2, ratio_seg1),
    )

    # convert to maximum static thrust ratio
    ratio_F0 = ratio * F35 / thr0

    return ratio_F0


def compute_eng_ff_coeff(
    ffidl: float, ffapp: float, ffco: float, ffto: float
) -> tuple[Any, Any, Any]:
    """Compute fuel flow based on engine icao fuel flow model

    Fits a quadratic polynomial through the four fuel-flow measurement
    points of the ICAO engine emission databank (at 7%, 30%, 85%, and 100%
    thrust) plus the origin. The resulting coefficients give fuel flow per
    engine as a function of thrust ratio x: ff = a*x^2 + b*x + c.

    Args:
        ffidl (float or 1D-array): fuel flow at idle thrust (7%) [kg/s]
        ffapp (float or 1D-array): fuel flow at approach thrust (30%) [kg/s]
        ffco (float or 1D-array): fuel flow at climb-out thrust (85%) [kg/s]
        ffto (float or 1D-array): fuel flow at takeoff thrust (100%) [kg/s]

    Returns:
        list of coeff: [a, b, c], fuel flow calc: ax^2 + bx + c
    """

    # standard fuel flow at test thrust ratios
    y = [0, ffidl, ffapp, ffco, ffto]
    x = [0, 0.07, 0.3, 0.85, 1.0]

    a, b, c = np.polyfit(x, y, 2)

    return a, b, c
