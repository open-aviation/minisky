# MiniSky

MiniSky is a hackable air traffic control simulator, a fork of
[BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) stripped down to its essentials.

It is designed to be a minimal tool for coders: no integrated graphical interface and no
complex network architecture. Uncommon commands and features are progressively removed to
reach a bare-minimum simulator that is easy to read, embed, and extend.

## Three ways to use it

1. Run a scenario file to completion, no interaction needed.

    ```bash
    uv run minisky run --scenario scenarios/kl204.scn
    ```

    → [Running scenarios](guides/running-scenarios.md)

2. Start a FastAPI server and drive the simulation over HTTP.

    ```bash
    uv run minisky server
    httpx "http://localhost:8000/stack/MCRE 3"
    ```

3. Import `minisky` and step the simulation from your own code.

    ```python
    from minisky import MiniSky
    from minisky import quantities as q
    from minisky.values import CasMps, StdPressureAltM

    with MiniSky() as runtime:
        runtime.traffic.cre(
            "KL315", lat=52.0, lon=4.0, hdg=45, alt=StdPressureAltM(q.ft_to_m(5000.0)), airspeed=CasMps(q.kt_to_mps(250.0))
        )
        runtime.simulation.step()
    ```
