"""BlueSky simulation control object."""

import datetime
import time
from random import seed

import numpy as np

# Local imports
import minisky
import minisky.core as core
from minisky.core import simtime
from minisky.tools import areafilter

# Minimum sleep interval
MINSLEEP = 1e-3


class Simulation:
    """The simulation object."""

    def __init__(self):
        self.state = minisky.INIT
        self.prevstate = None

        # System time [seconds]
        self.syst = -1.0

        # Benchmark time and timespan [seconds]
        self.bencht = 0.0
        self.benchdt = -1.0

        # Simulation time [seconds]
        self.simt = 0.0

        # Simulation timestep [seconds]
        self.simdt = minisky.core.settings.simdt

        # Simulation timestep multiplier: run sim at n x speed
        self.dtmult = 1.0

        # Simulated UTC clock time
        self.utc = datetime.datetime.utcnow().replace(
            hour=0, minute=0, second=0, microsecond=0
        )

        # Flag indicating running at fixed rate or fast time
        self.ffmode = False
        self.ffstop = None

        # Flag indicating whether timestep can be varied to ensure realtime op
        self.rtmode = False

        # Keep track of known clients
        self.clients = set()

    def step(self, dt_increment=0):
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
            simtime.preupdate()

            # Determine interval towards next timestep
            self.simt, self.simdt = simtime.step(dt_increment)

            # Update UTC time
            self.utc += datetime.timedelta(seconds=self.simdt)

            # Update traffic and other update functions for the next timestep
            minisky.traf.update()
            simtime.update()

    def update(self):
        """Perform a simulation update.
        This involves performing a simulation step, and when running in real-time mode
        (or a multiple thereof), sleeping an appropriate time."""
        if self.state == minisky.INIT:
            if self.syst < 0.0:
                self.syst = time.time()

            if self.benchdt > 0.0:
                self.fastforward(self.benchdt)
                self.bencht = time.time()

        # When running at a fixed rate, or when in hold/init,
        # increment system time with sysdt and calculate remainder to sleep.
        remainder = self.syst - time.time()
        if (not self.ffmode or self.state != minisky.OP) and remainder > MINSLEEP:
            time.sleep(remainder)

        # Perform one simulation timestep
        if remainder < 0.0 and self.rtmode:
            # Allow a variable timestep when we are running realtime
            self.step(-remainder)
        else:
            # Don't accumulate delay when we aren't running realtime
            if remainder < 0:
                self.syst -= remainder
            self.step()

        # Always update syst
        self.syst += self.simdt / self.dtmult

        # Stop fast-time/benchmark if enabled and set interval has passed
        if self.ffstop is not None and self.simt >= self.ffstop:
            if self.benchdt > 0.0:
                minisky.scr.echo(
                    "Benchmark complete: %d samples in %.3f seconds."
                    % (minisky.scr.samplecount, time.time() - self.bencht)
                )
                self.benchdt = -1.0
                self.hold()
            else:
                self.op()

        # Inform main of our state change
        if self.state != self.prevstate:
            self.prevstate = self.state

    def stop(self):
        """Stack stop/quit command."""
        self.state = minisky.END
        minisky.runner.stop()

    def op(self):
        """Set simulation state to OPERATE."""
        self.syst = time.time() + self.simdt
        self.ffmode = False
        self.ffstop = None
        self.state = minisky.OP
        self.set_dtmult(1.0)
        minisky.scr.echo("Simulation running in real-time mode")

    def hold(self):
        """Set simulation state to HOLD."""
        self.syst = time.time() + self.simdt / self.dtmult
        self.state = minisky.HOLD
        self.ffmode = False
        self.ffstop = None
        minisky.scr.echo("Simulation paused")

    def reset(self):
        """Reset all simulation objects."""
        self.state = minisky.INIT
        self.syst = -1.0
        self.simt = 0.0
        self.simdt = minisky.core.settings.simdt
        simtime.reset()
        self.utc = datetime.datetime.utcnow().replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        self.ffmode = False
        self.set_dtmult(1.0)
        simtime.reset()
        # core.reset()
        minisky.navdb.reset()
        minisky.traf.reset()
        minisky.stack.reset()
        areafilter.reset()
        minisky.scr.reset()
        minisky.scr.echo("Simulation reset")

    def set_dtmult(self, mult):
        """Set simulation speed multiplier."""
        self.dtmult = mult

    def realtime(self, flag=None):
        if flag is not None:
            self.rtmode = flag

        return True, "Realtime mode is o" + ("n" if self.rtmode else "ff")

    def fastforward(self, nsec=None):
        """Run in fast-time (for nsec seconds if specified)."""
        self.state = minisky.OP
        self.ffmode = True
        self.ffstop = (self.simt + nsec) if nsec else None
        minisky.scr.echo("Entering fast-time mode")

    def benchmark(self, fname="IC", dt=300.0):
        """Run a simulation benchmark.
        Use scenario given by fname.
        Run for <dt> seconds."""
        minisky.stack.ic(fname)
        self.bencht = 0.0  # Start time will be set at next sim cycle
        self.benchdt = dt

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
