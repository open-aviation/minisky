"""Per-aircraft state for MiniSky plugins."""

from __future__ import annotations

from typing import TYPE_CHECKING

from minisky.core.trafficarrays import TrafficArrays

if TYPE_CHECKING:
    from minisky.traffic import Traffic


class Entity(TrafficArrays):
    """Base class for plugin-owned per-aircraft arrays.

    Create a fresh entity in every plugin build and declare arrays or lists
    inside `with self.settrafarrays():`. MiniSky sizes and attaches the entity
    when the plugin loads, then retires it during shutdown.

    `traffic` is available only while the plugin is active. Do not use it in
    `__init__`.
    """

    def __init__(self) -> None:
        super().__init__()
        self._traffic: Traffic | None = None
        self._prepared_traffic: Traffic | None = None
        self._retired = False

    @property
    def traffic(self) -> Traffic:
        """Return the owning traffic object while the plugin is active."""
        if self._traffic is None:
            raise RuntimeError("plugin entity is detached")
        return self._traffic

    @property
    def ownerless(self) -> bool:
        return (
            not self._retired
            and self._parent is None
            and self._traffic is None
            and self._prepared_traffic is None
        )

    def _prepare(self, traffic: Traffic) -> None:
        """Size arrays for existing traffic without exposing live traffic."""
        if not self.ownerless:
            raise RuntimeError("plugin entity must be fresh and detached")
        try:
            if traffic.ntraf:
                # NOTE(abraham): custom create() may initialize private arrays,
                # but traffic remains unavailable until publication.
                self.create(traffic.ntraf)
        except BaseException:
            TrafficArrays.reset(self)
            raise
        self._prepared_traffic = traffic

    def _publish(self) -> None:
        traffic = self._prepared_traffic
        if traffic is None or self._parent is not None:
            raise RuntimeError("plugin entity is not prepared")
        self.reparent(traffic)
        self._traffic = traffic
        self._prepared_traffic = None

    def _abort(self) -> None:
        """Undo preparation after a failed load."""
        self.detach()
        self._traffic = None
        self._prepared_traffic = None
        TrafficArrays.reset(self)

    def _retire(self) -> None:
        """Detach permanently when the owning plugin stops."""
        self.detach()
        self._traffic = None
        self._prepared_traffic = None
        self._retired = True
