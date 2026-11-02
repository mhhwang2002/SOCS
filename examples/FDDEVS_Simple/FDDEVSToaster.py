"""Toaster example ported from FDDEVS to the unified FDDEVS class."""

from socs.devs.FDDEVS import FDDEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import SimEngineThread


class Toaster(FDDEVS):
    """Atomic toaster model with idle and toasting phases."""

    def __init__(self, name, tt, parent=None):
        super().__init__(name, parent)
        self.tt = tt
        self.phase = "I"
        self.addInputPorts("?push")
        self.addOutputPorts("!pop")

    def delta_0(self):
        """Reset the toaster to its initial idle phase."""
        self.phase = "I"

    def tau(self):
        """Return the time until the next internal event."""
        if self.phase == "T":
            return self.tt
        return float("inf")

    def delta_x(self, x):
        """Handle a push event and reschedule if toasting starts."""
        if x.port == self.IP["?push"] and self.phase == "I":
            self.phase = "T"
            return True
        return False

    def delta_y(self):
        """Emit the pop event and return to the idle phase."""
        if self.phase == "T":
            self.phase = "I"
            return PortValue(self.OP["!pop"])
        return None

    def get_state(self):
        """Return the current toaster phase."""
        return self.phase

    def set_state(self, state):
        """Set the current toaster phase."""
        self.phase = state


def make_input_event(model):
    """Create an input event for the toaster's push port."""
    return model, PortValue(model.IP["?push"])


def runSimulation(model):
    """Run the toaster model with the interactive simulation engine."""
    engine = SimEngineThread(model, make_input_event)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.runInteractiveMenu()


if __name__ == "__main__":
    toaster = Toaster("T1", 22.5)
    choice = True
    while choice:
        command = input("[s]im, [x]it> ").strip().lower()
        if command in ("s", "sim"):
            runSimulation(toaster)
        elif command in ("x", "exit"):
            choice = False
        else:
            print(f"{command!r} is an invalid input.")
    print("good bye~!")
