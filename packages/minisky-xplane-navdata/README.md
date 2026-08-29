# minisky-xplane-navdata

Provides navigation data used by [minisky](https://github.com/open-aviation/minisky), including:

- waypoints/navaids (X-Plane data cycle 2013.10, build 20131334, `NavXP810` / `FixXP700`): `nav.dat` + `fix.dat`
- airway legs (X-Plane data cycle 2013.10, build 20131334, `FixXP700`): `awy.dat`
- runway thresholds (X-Plane data cycle 2013.10, build 20131335): `apt.zip` -> `apt.dat`

Copyright: © 2013 Robin A. Peel (`robin@x-plane.com`).

SPDX-License-Identifier: GPL-2.0-or-later

BlueSky originally added its `nav.dat`, `fix.dat`, and `awy.dat` navigation database in commit [`996de35`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/996de352cb9f657ec2837581a6bcd2090ce70fcf).

Converted into Parquet/JSON in commit [`fe74938`](https://github.com/open-aviation/minisky/commit/fe7493872b23d100be0b1f313427f2571258af48) on 2025-03-05.

Also note that `airports.dat`/`icao-countries.dat`/`fir/*.txt` were transformed into `airport.parquet`/`country.parquet`/`fir.json`, but the provenance is unknown.

See also: [minisky PR #19](https://github.com/open-aviation/minisky/pull/19#issuecomment-5084422073)
