# Navigation data

minisky core **does not include any aviation navigation data**. To add X-Plane data alongside the core, install:

```sh
# with uv
uv add minisky minisky-xplane-navdata
# with pip
pip install minisky minisky-xplane-navdata
```

??? Details

    --8<-- "packages/minisky-xplane-navdata/README.md:3"

Once installed, construct the navdata provider explicitly and pass it to the runtime:

```python
from minisky import MagneticDeclinationGrid, MiniSky, MiniSkyConfig
from minisky_xplane_navdata import load

navdata = load()
magnetic_declination = MagneticDeclinationGrid.load_default()
# you can also supply your own with MagneticDeclinationgrid.from_csv()

runtime = MiniSky(MiniSkyConfig(), navdata=navdata, magnetic_declination=magnetic_declination)
```

See: [`WaypointData`][minisky.WaypointData], [`AirportData`][minisky.AirportData], [`AirwayData`][minisky.AirwayData], [`FirData`][minisky.FirData], and [`CountryData`][minisky.CountryData].

Also see: [`MagneticDeclinationGrid.load_default()`][minisky.MagneticDeclinationGrid.load_default].
