"""Aircraft performance package based on the OpenAP model.

This package implements MiniSky's aircraft performance modelling using the
open-source OpenAP aircraft performance library:

- ``perfoap``: the :class:`OpenAP` performance model that computes drag,
  thrust, fuel flow, and kinematic envelope limits per aircraft.
- ``coeff``: loads aircraft, engine, flight envelope, and drag polar
  coefficients from the OpenAP database.
- ``phase``: flight-phase identification (ground, initial climb, climb,
  cruise, descent, approach) from speed, vertical rate, and altitude.
- ``thrust``: empirical turbofan thrust and ICAO fuel-flow models.

The active performance model instance lives on the traffic object as
``minisky.traf.perf``.
"""

import minisky.traffic.performance.coeff
import minisky.traffic.performance.phase
