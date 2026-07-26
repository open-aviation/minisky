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
