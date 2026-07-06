"""MiniSky REST API server (development entry point).

The application now lives in :mod:`minisky.server` so it is importable from the
installed package (and reachable via the ``minisky-server`` console script).
This root-level module simply re-exports it so ``fastapi dev minisky-api.py``
and ``fastapi run minisky-api.py`` keep working from a source checkout.

Run with::

    fastapi dev minisky-api.py    # development, auto-reload
    fastapi run minisky-api.py    # production
    minisky-server                # installed console script

Interactive OpenAPI docs are served at ``/docs``.
"""

from minisky.server import app

__all__ = ["app"]
