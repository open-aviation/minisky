"""BlueSky simulation control object."""

import datetime
import time
from random import seed

import numpy as np

# Local imports
import minisky
from minisky.tools import areafilter

# Minimum sleep interval
MINSLEEP = 1e-3


class Simulation:
    """The simulation object."""

    def __init__(self):
        self.state = minisky.INIT
        self.prevstate = None

        # Simulation time [seconds]
        self.simt = 0

        # Simulation timestep [seconds]
        self.simdt = 1

        # System time [seconds]
        self.syst = 0

        # Simulated UTC clock time, can be set by setutc()
        self.utc = datetime.datetime.today()

        # Flag indicating running at fixed rate or fast time

        # Flag indicating whether timestep can be varied to ensure realtime op
        self.rtmode = False

        # Keep track of known clients
        self.clients = set()

    def step(self):
        """Perform one simulation timestep.

        Call this function instead of update if you don't want to run with a fixed
        real-time rate.
        """
        if self.state == minisky.INIT:
            # Simulation starts as soon as there is traffic, or pending commands
            if minisky.traf.ntraf > 0 or len(minisky.stack.get_scendata()[0]) > 0:
                self.op()

        # Always update stack
        minisky.stack.process()

        if self.state == minisky.OP:
            self.simt += self.simdt

            # Update UTC time
            self.utc += datetime.timedelta(seconds=self.simdt)

            minisky.traf.update()

    def stop(self):
        """Stack stop/quit command."""
        self.state = minisky.END
        minisky.runner.stop()

    def op(self):
        """Set simulation state to OPERATE."""
        self.syst = time.time() + self.simdt
        self.state = minisky.OP
        minisky.scr.echo("Simulation running in real-time mode")

    def hold(self):
        """Set simulation state to HOLD."""
        self.syst = time.time() + self.simdt
        self.state = minisky.HOLD
        minisky.scr.echo("Simulation paused")

    def reset(self):
        """Reset all simulation objects."""
        self.state = minisky.INIT
        self.syst = 0
        self.simt = 0
        self.simdt = 1
        self.utc = datetime.datetime.utcnow().replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        minisky.navdb.reset()
        minisky.traf.reset()
        minisky.stack.reset()
        areafilter.reset()
        minisky.scr.reset()
        minisky.scr.echo("Simulation reset")

    def realtime(self, flag=None):
        if flag is not None:
            self.rtmode = flag

        return True, "Realtime mode is o" + ("n" if self.rtmode else "ff")

    def event(self, eventname, eventdata, sender_rte):
        """Handle events coming from the network."""
        # Keep track of event processing
        event_processed = False

        if eventname == b"STACK":
            # We received a single stack command. Add it to the existing stack
            minisky.stack.stack(eventdata, sender_id=sender_rte)
            event_processed = True

        elif eventname == b"BATCH":
            # We are in a batch simulation, and received an entire scenario. Assign it to the stack.
            self.reset()
            minisky.stack.set_scendata(eventdata["scentime"], eventdata["scencmd"])
            self.op()
            event_processed = True

        return event_processed

    def setutc(self, *args):
        """Set simulated clock time offset."""
        if not args:
            pass  # avoid error message, just give time

        elif len(args) == 1:
            if args[0].upper() == "RUN":
                self.utc = datetime.datetime.utcnow().replace(
                    hour=0, minute=0, second=0, microsecond=0
                )

            elif args[0].upper() == "REAL":
                self.utc = datetime.datetime.today().replace(microsecond=0)

            elif args[0].upper() == "UTC":
                self.utc = datetime.datetime.utcnow().replace(microsecond=0)

            else:
                try:
                    self.utc = datetime.datetime.strptime(
                        args[0], "%H:%M:%S.%f" if "." in args[0] else "%H:%M:%S"
                    )
                except ValueError:
                    return False, "Input time invalid"

        elif len(args) == 3:
            day, month, year = args
            try:
                self.utc = datetime.datetime(year, month, day)
            except ValueError:
                return False, "Input date invalid."
        elif len(args) == 4:
            day, month, year, timestring = args
            try:
                self.utc = datetime.datetime.strptime(
                    f"{year},{month},{day},{timestring}",
                    (
                        "%Y,%m,%d,%H:%M:%S.%f"
                        if "." in timestring
                        else "%Y,%m,%d,%H:%M:%S"
                    ),
                )
            except ValueError:
                return False, "Input date invalid."
        else:
            return False, "Syntax error"

        return True, "Simulation UTC " + str(self.utc)

    @staticmethod
    def setseed(value):
        """Set random seed for this simulation."""
        seed(value)
        np.random.seed(value)
        minisky.scr.echo("random seed set")
