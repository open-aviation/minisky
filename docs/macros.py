from minisky import MiniSky, MiniSkyConfig
from minisky.command import format_command_form


def _summary(text: str) -> str:
    return text.strip().split("\n\n", maxsplit=1)[0].replace("\n", " ")


def command_docs() -> str:
    with MiniSky(MiniSkyConfig()) as runtime:
        commands = sorted(set(runtime.commands.cmddict.values()), key=lambda command: command.name)
        lines: list[str] = []
        for command in commands:
            aliases = ", ".join(f"`{alias}`" for alias in sorted(command.aliases))
            suffix = f" (aliases: {aliases})" if aliases else ""
            lines.append(f"`{command.name}`{suffix}")
            lines.append("")
            for form in command.forms:
                # NOTE: in the future we will use jinja and have much richer displays
                syntax = format_command_form(command.name, form.parameters)
                description = _summary(form.help)
                detail = f": {description}" if description else ""
                lines.append(f"  - `{syntax}`{detail}")
            lines.append("")
    return "\n".join(lines)


def define_env(env) -> None:
    env.macro(command_docs)
