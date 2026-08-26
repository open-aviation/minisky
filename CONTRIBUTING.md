# Contributing

## Setup

Prerequisites: uv, pnpm.

```sh
# build the frontend + install the python workspace (includes all packages/*)
just sync
```

## Development

When you modify a command (changing signatures, updating docstrings), remember to re-generate the command schema so the documentation and `HELP` command picks it up:

```sh
just gen-command-schema
```

Otherwise, CI will fail when stale.

Code quality:

```sh
# format the repo
just fmt
# static checks: ruff, pyright, generated command-schema, frontend.
just check
```

Testing:

```sh
# unit + integration
just test
# python only
just test-unit
# rest api (opt-in)
just test-api
```

Zensical/mkdocstrings documentation:

```sh
# preview
just docs-serve
# build site
just docs-build
```
