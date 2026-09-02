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

Once installed, the default runtime uses it automatically:

```python
from minisky import MiniSky

runtime = MiniSky()
# tries to load minisky-xplane-navdata internally
```

If you are running multiple runtimes, reuse the navigation and magnetic declination data to avoid repeated file I/O:

```python
from minisky import MagneticDeclinationGrid, MiniSky
from minisky_xplane_navdata import load

navdata = load()
magnetic_declination = MagneticDeclinationGrid.load_default()

runtime_a = MiniSky(navdata=navdata, magnetic_declination=magnetic_declination)
runtime_b = MiniSky(navdata=navdata, magnetic_declination=magnetic_declination)
```

See: [`WaypointData`][minisky.WaypointData], [`AirportData`][minisky.AirportData], [`AirwayData`][minisky.AirwayData], [`FirData`][minisky.FirData], and [`CountryData`][minisky.CountryData].

Also see: [`MagneticDeclinationGrid.load_default()`][minisky.MagneticDeclinationGrid.load_default].
