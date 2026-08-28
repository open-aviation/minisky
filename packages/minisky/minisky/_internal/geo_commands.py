"""Stack commands backed by geographic utilities."""

from minisky._internal.command import LatLonDeg, command
from minisky._internal.result import Ok, Result
from minisky.geo import MagneticDeclination


class GeoCommands:
    """Stack-facing geographic utility commands."""

    def __init__(self, magnetic_declination: MagneticDeclination) -> None:
        self.magnetic_declination = magnetic_declination

    @command(name="MAGVAR", aliases=("MAGDEC", "MAGDECL", "VAR"))
    def show_magnetic_variation(self, position: LatLonDeg) -> Result[str, str]:
        """Show magnetic variation at an aviation position."""
        variation = self.magnetic_declination(position.lat, position.lon)
        return Ok(f"Magnetic variation at {position.lat},{position.lon} = {variation} deg")
