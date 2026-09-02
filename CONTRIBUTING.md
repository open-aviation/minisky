# Contributing

## Setup

Prerequisites: uv, pnpm.

```sh
# build the frontend + install the python workspace (includes all packages/*)
just sync
```

## Development

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

## Releasing

Minisky follows semantic versioning. The core is currently in alpha (v0.0.x). Plugin versions need not follow core versioning in lockstep.

1. Update the package version in the `pyproject.toml`.
2. Add a section in `CHANGELOG.md` to use as the GitHub release notes (optional, but highly recommended):

   ```md
   ## `<package>` v<version>

   Release notes go here.
   ```

3. On the `main` branch, push a `<package>/<version>` tag, for example:

   ```sh
   git tag minisky/0.0.1
   git push origin minisky/0.0.1
   ```

4. GitHub Actions should then parse the changelog, publish to PyPI and create the GitHub release.
