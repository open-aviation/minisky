"""The stack parses all text-based commands in the simulation."""

from clearsky.stack.argparser import ArgumentError, refdata
from clearsky.stack.cmdparser import (
    append_commands,
    command,
    commandgroup,
    get_commands,
    remove_commands,
)
from clearsky.stack.stackbase import (
    get_scendata,
    get_scenname,
    routetosender,
    sender,
    set_scendata,
    stack,
)


def init():
    import clearsky.stack.simstack as simstack
    from clearsky.stack.importer import Importer

    simstack.init()
