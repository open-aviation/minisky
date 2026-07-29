minisky_dir := "packages/minisky"
plugin_dir := "packages/tangram-minisky"

sync:
    pnpm install
    pnpm build
    uv sync --all-packages

fmt:
    uv run ruff check {{minisky_dir}}/minisky {{minisky_dir}}/example_plugins {{plugin_dir}} tests --fix
    uv run ruff format {{minisky_dir}}/example_plugins/tangram.py {{plugin_dir}}
    pnpm lint:fix

check:
    uv run ruff check {{minisky_dir}}/minisky {{minisky_dir}}/example_plugins {{plugin_dir}} tests
    uv run ruff format {{minisky_dir}}/example_plugins/tangram.py {{plugin_dir}} --check
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
