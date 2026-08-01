sync:
    pnpm install
    pnpm build
    uv sync --all-packages

fmt:
    uv run ruff check packages tests --fix
    uv run ruff format packages tests
    pnpm lint:fix

check:
    uv run ruff check packages tests
    uv run ruff format packages tests --check
    uv run pyright
    pnpm check

# Run unit and integration tests, excluding API tests.
test:
    uv run pytest

# Run fast unit tests.
test-unit:
    uv run pytest tests/unit

# Run opt-in REST API tests.
test-api:
    uv run pytest -m api tests/test_api.py

docs-serve:
    uv run --group docs zensical serve

docs-build:
    uv run --group docs zensical build
