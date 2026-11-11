"""Toaster example using the DEVS atomic model base."""

from math import inf, isfinite

from socs.devs.DEVS import DEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import SimEngineThread


class Toaster(DEVS):
    """Atomic toaster model with idle and toasting phases."""

    def __init__(self, name, tt=10, parent=None):
        if not isfinite(tt) or tt <= 0:
            raise ValueError("Toasting time must be finite and positive.")
        super().__init__(name, parent)
        self.tt = tt
        self.phase = "Idle"
        self.addInputPorts("?push")
        self.addOutputPorts("!pop")

    def delta_0(self):
        """Reset the toaster to its initial idle phase."""
        self.phase = "Idle"

    def tau(self):
        """Return the time until the next internal event."""
        if self.phase == "Toasting":
            return self.tt
        return inf

    def delta_x(self, x):
        """Handle a push event and reschedule if toasting starts."""
        if isinstance(x, PortValue) and x.port == "?push" and self.phase == "Idle":
            self.phase = "Toasting"
            return True
        return False

    def delta_y(self):
        """Emit the pop event and return to the idle phase."""
        if self.phase == "Toasting":
            self.phase = "Idle"
            return PortValue("!pop")
        return None

    def get_state(self):
        """Return the current toaster phase."""
        return self.phase

    def set_state(self, state):
        """Set the current toaster phase."""
        self.phase = {"I": "Idle", "T": "Toasting"}.get(state, state)


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
    if __package__:
        from .DEVSphToaster import main
    else:
        from DEVSphToaster import main

    main(Toaster, __doc__)
