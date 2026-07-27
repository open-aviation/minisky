"""Entity base class for MiniSky plugin-owned per-aircraft data.

`Entity` extends [`TrafficArrays`][minisky.core.trafficarrays.TrafficArrays]
so plugin data can participate in the owning runtime's aircraft-array tree.
An entity is attached explicitly to one [`Traffic`][minisky.traffic.Traffic]
object; it is not a process-wide singleton and does not use a proxy.

Usage:

```python
class MyPlugin(Entity):
    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
        with self.settrafarrays():
            self.mydata = np.array([])
```

Arrays and lists registered inside `settrafarrays()` grow, shrink, and reset
with the aircraft in that runtime. Separate runtimes can therefore load
independent instances of the same plugin class.

For replaceable core traffic components, inherit from `TrafficArrays`
directly and select implementations through the runtime's
`ReplaceableManager`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from minisky.core.trafficarrays import TrafficArrays

if TYPE_CHECKING:
    from minisky.traffic import Traffic


class Entity(TrafficArrays):
    """Base class for plugin-owned per-aircraft arrays.

    Combines the automatic create, delete, and reset behavior of
    [`TrafficArrays`][minisky.core.trafficarrays.TrafficArrays] with an
    explicit reference to the traffic tree that owns the plugin entity.

    Usage:

    ```python
    class MyPlugin(Entity):
        def __init__(self, traffic: Traffic) -> None:
            super().__init__(traffic)
            with self.settrafarrays():
                self.mydata = np.array([])

        def create(self, n: int = 1) -> None:
            super().create(n)
            self.mydata[-n:] = default_values
    ```

    Args:
        traffic: Traffic object whose tree owns this entity.

    Attributes:
        traffic: The owning runtime's traffic object.
    """

    def __init__(self, traffic: Traffic) -> None:
        """Attach this entity to `traffic` and initialize its array bookkeeping."""
        self.traffic = traffic
        super().__init__(traffic)
