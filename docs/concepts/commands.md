# Commands

minisky inherits much of BlueSky's **stack command** language (a text-based [domain-specific language (DSL)](https://en.wikipedia.org/wiki/Domain-specific_language)), but makes some tweaks, most notably the requirement to [specify explicit units](./types.md).

This DSL is used by scenario files (`.scn`) and the [interactive console](../user-guide/server.md#interactive-console), for example:

=== "Interactive console"

    ```text title="uv run minisky console"
    > CRE KL001 B738 52 4 90 FL100 250KT[CAS]
    > KL001 ALT FL200
    > POS KL001
    ```

=== "Scenario file (.scn)"

    To run commands at a specified *simulation time*, specify a timestamp for each command:

    ```text title="example.scn"
    00:00:00.00> CRE KL001 B738 52 4 90 FL100 250KT[CAS]
    00:00:00.00> KL001 ADDWPT HELEN FL100 250KT[CAS]
    00:00:10.00> KL001 ALT FL200
    00:30:00.00> QUIT
    ```

    Run with:

    ```bash
    uv run minisky run --scenario example.scn --speed 10
    ```

    Here, `--speed 10` changes how quickly simulation time is paced against wall-clock time (see [basics](./basics.md) for more information).

=== "Python"

    ```python
    runtime.commands.stack("KL001 ALT FL200")
    runtime.simulation.step()
    ```

See the [complete stack command reference](../reference/commands.md) for all supported commands. If you are a plugin developer, see the [guide on adding commands](../developer-guide/commands.md).

!!! warning

    We strongly recommend using the typed [`MiniSky` Python API][minisky.MiniSky] instead of the DSL, so mistakes can be caught by static type checkers (e.g. pyright) instead of at runtime. The Python API is also usually much easier to work with.
