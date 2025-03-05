"""BlueSky: The open-source ATM simulator."""

from minisky import stack, tools
from minisky.core import varexplorer
from minisky.core.settings import data
from minisky.simulation import ConsoleIO, Runner, Simulation
from minisky.tools.navdata import Navdatabase
from minisky.traffic import Traffic

# Constants
BS_OK = 0
BS_ARGERR = 1
BS_FUNERR = 2
BS_CMDERR = 4

# simulation states
INIT, HOLD, OP, END = (0, 1, 2, 3)

# Main singleton objects in BlueSky
runner = None
traf = None
navdb = None
sim = None
scr = None
navdb = None


def init(scenario=None):
    """Initialize minisky modules.

    Arguments:
    - scenario: Start with a running scenario [filename]
    """
    global traf, sim, scr, runner
    global navdb

    # Initialise tools
    tools.init()

    navdb = Navdatabase()

    # Initialize singletons
    traf = Traffic()
    sim = Simulation()
    scr = ConsoleIO()
    runner = Runner()

    # Initialize remaining modules
    varexplorer.init()

    if scenario:
        stack.stack(f"IC {scenario}")
    else:
        # without scenario, sim shall be up
        runner.prevent_shutdown()

    stack.init()
