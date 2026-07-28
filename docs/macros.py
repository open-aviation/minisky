from minisky import DEFAULT_SETTINGS_FILE, MiniSky, MiniSkySettings


def command_docs() -> str:
    settings = MiniSkySettings.from_file(DEFAULT_SETTINGS_FILE)
    with MiniSky(settings) as runtime:
        primary = {}
        synonyms: dict[str, list[str]] = {}
        for name, command in sorted(runtime.commands.cmddict.items()):
            if command.name == name:
                primary[name] = command
            else:
                synonyms.setdefault(command.name, []).append(name)

        rows = [
            "| Command | Usage | Description | Synonyms |",
            "| --- | --- | --- | --- |",
        ]
        for name, command in sorted(primary.items()):
            usage = (command.brief or "").replace("|", "\\|").replace("\n", " ")
            help_text = (command.help or "").replace("|", "\\|")
            help_text = help_text.strip().splitlines()[0] if help_text.strip() else ""
            aliases = ", ".join(f"`{alias}`" for alias in sorted(synonyms.get(name, [])))
            rows.append(f"| `{name}` | `{usage}` | {help_text} | {aliases} |")

    return "\n".join(rows)


def define_env(env) -> None:
    env.macro(command_docs)
