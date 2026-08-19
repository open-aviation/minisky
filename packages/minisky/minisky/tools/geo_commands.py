"""Stack commands backed by geographic utilities."""

from minisky.result import Ok, Result
from minisky.stack_command import LatLonDeg, command
from minisky.tools.geo import magdec


class GeoCommands:
    """Stack-facing geographic utility commands."""

    @command(name="MAGVAR", aliases=("MAGDEC", "MAGDECL", "VAR"))
    def show_magnetic_variation(self, position: LatLonDeg) -> Result[str, str]:
        """Show magnetic variation at an aviation position."""
        variation = magdec(position.lat, position.lon)
        return Ok(f"Magnetic variation at {position.lat},{position.lon} = {variation} deg")
