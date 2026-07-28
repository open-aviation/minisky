plugin_dir := "example_plugins/tangram/tangram_minisky"

sync:
    pnpm install
    pnpm build
    uv sync --all-packages

fmt:
    uv run ruff check minisky example_plugins tests --fix
    uv run ruff format example_plugins/tangram.py {{plugin_dir}}
    pnpm lint:fix

check:
    uv run ruff check minisky example_plugins tests
    uv run ruff format example_plugins/tangram.py {{plugin_dir}} --check
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

docs-generate:
    uv run minisky commands docs

docs-serve: docs-generate
    uv run --group docs zensical serve

docs-build: docs-generate
    uv run --group docs zensical build
