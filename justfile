sync:
    pnpm install
    pnpm build
    uv sync --all-packages

fmt:
    uv run ruff check packages scripts docs --fix
    uv run ruff format packages scripts docs
    pnpm lint:fix

check:
    uv run ruff check packages scripts docs
    uv run ruff format packages scripts docs --check
    uv run pyright
    pnpm check

# Run unit and integration tests, excluding API tests.
test:
    uv run pytest

# Run fast unit tests.
test-unit:
    uv run pytest packages/*/tests/unit

# Run opt-in REST API tests.
test-api:
    uv run pytest -m api packages/minisky/tests/test_api.py

docs-serve:
    uv run --group docs zensical serve

docs-build:
    uv run --group docs zensical build

_docs-illustration-output name theme format:
    printf '#import "/docs/assets/illustrations/render.typ": render\n#import "/docs/assets/illustrations/{{name}}.typ": main, colours-{{theme}}\n#render(main, colours-{{theme}}, "{{theme}}")\n' | typst compile --root . --format {{format}} - docs/assets/illustrations/{{name}}-{{theme}}.{{format}}

_docs-illustration name:
    just _docs-illustration-output {{name}} light svg
    just _docs-illustration-output {{name}} dark svg
    just _docs-illustration-output {{name}} light png
    just _docs-illustration-output {{name}} dark png

docs-illustrations:
    just _docs-illustration traffic-arrays
    just _docs-illustration simulation-timing
