"""Physical and dimensionless quantities specific to multicopter performance."""

from __future__ import annotations

from typing import Annotated, TypeAlias, TypeVar

import isqx
from isqx import aerospace
from minisky import quantities as q

_T = TypeVar("_T")

FlatPlateDragAreaM2: TypeAlias = q.AreaM2[_T]
r"""Equivalent flat-plate parasite-drag area, $C_D S$, where $C_D$ is the
[drag coefficient][isqx.DRAG_COEFFICIENT] and $S$ is the reference area.
"""

ThrustToWeightRatio: TypeAlias = Annotated[_T, aerospace.THRUST_LOADING]
r"""Dimensionless maximum thrust divided by aircraft weight, $T_{\max}/(mg)$"""

CruiseSpeedFraction: TypeAlias = Annotated[_T, isqx.Dimensionless("cruise_speed_fraction")]
r"""Cruise speed as a fraction of the airframe's maximum speed.

$$
V_{\mathrm{cruise}} = f\,V_{\max}.
$$

where $0 < f \le 1$.

Used when deriving battery energy from the configured maximum range for
an aircraft whose usable battery energy is not supplied directly.
"""
