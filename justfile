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

_docs-illustration name:
    typst compile --input theme=light docs/assets/illustrations/{{name}}.typ docs/assets/illustrations/{{name}}-light.svg
    typst compile --input theme=dark docs/assets/illustrations/{{name}}.typ docs/assets/illustrations/{{name}}-dark.svg
    typst compile --input theme=light --format png docs/assets/illustrations/{{name}}.typ docs/assets/illustrations/{{name}}-light.png
    typst compile --input theme=dark --format png docs/assets/illustrations/{{name}}.typ docs/assets/illustrations/{{name}}-dark.png

docs-illustrations:
    just _docs-illustration traffic-arrays
    just _docs-illustration simulation-timing
