from minisky.core import settings

# Register settings defaults
if settings.prefer_compiled:
    try:
        from . import cgeo as geo

        # print("Using compiled geo functions")
    except ImportError:
        from . import geo

        # print("Using Python-based geo functions")
else:
    from . import geo

    print("Using Python-based geo functions")


def init():
    # print("Reading magnetic variation data")
    geo.load_magnetic_declination()
