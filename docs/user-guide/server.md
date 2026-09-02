# Server

A [`MiniSky`][minisky.MiniSky] runtime can be controlled by standard tools like `curl` through an **experimental** REST API:

```bash
# make sure to install the optional CLI dependencies
uv add 'minisky[cli]'
uv run minisky server
```

The [OpenAPI reference](https://fastapi.tiangolo.com/reference/openapi/docs/) can then be accessed at the `/docs` HTTP endpoint.

You can also optionally customise the server through a configuration TOML file (see the [configuration guide](./configuration.md)).

!!! note

    The REST API currently exposes a single runtime. Support for managing multiple runtimes is planned.

## Interactive console

Once you have a server running, attach an interactive console with:

```bash
uv run minisky console
```

To send [commands](../concepts/commands.md):

```text
> MCRE 3
> POS EHAM
> KL001 ALT FL200
```

To send an HTTP request or a minisky operation, prefix it with `/`:

```text
> /all
> /conflicts
> /load packages/minisky/scenarios/kl204.scn
> /clear
> /exit
```

`/load` reads a [scenario file](../concepts/commands.md) on the client machine and uploads it to the server. `/clear` and `/exit` are handled by the console itself. Otherwise, all slash-prefixed paths are sent as HTTP `GET` requests.

## Streaming

!!! warning

    This is experimental and can change at any time without notice.

To get WebSocket simulation snapshots on `/stream`, use:

```bash
uv run minisky stream
uv run minisky stream --raw
```

See [`Snapshot`][minisky.Snapshot], [`SimInfo`][minisky.SimInfo] and [`AcData`][minisky.AcData] API reference for more information.
