# `minisky.stack`

The text-command interpreter. Every command — from scenario files, the console, or the
REST API — is queued with [`stack()`][minisky.stack.stack] and executed during
[`process()`][minisky.stack.process] on the next simulation step. See the
[stack command reference](../reference/commands.md) for the available commands.

::: minisky.stack
    options:
      members:
        - stack
        - process
        - ic
        - ic_StringIO
        - reset
        - Command

## Argument parsing

Parsers for the aviation-aware argument types (`alt`, `spd`, `hdg`, `latlon`, `wpt`, ...)
used in command signatures.

::: minisky.stack.argparser
