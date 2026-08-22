sync:
    pnpm install
    pnpm build
    uv sync --all-packages

fmt:
    uv run ruff check packages scripts --fix
    uv run ruff format packages scripts
    pnpm lint:fix

check:
    uv run ruff check packages scripts
    uv run ruff format packages scripts --check
    uv run pyright
    uv run scripts/command_schema.py check minisky minisky_example minisky_example_customautopilot minisky_multicopter minisky_tangram
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

gen-command-schema:
    uv run scripts/command_schema.py export minisky minisky_example minisky_example_customautopilot minisky_multicopter minisky_tangram
