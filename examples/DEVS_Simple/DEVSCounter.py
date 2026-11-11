"""Counter example using the DEVS atomic model base."""

from socs.devs.DEVS import DEVS
from socs.devs.DEVSInterfaces import PortValue


class Counter(DEVS):
    """Atomic model that accumulates integers and emits each new sum."""

    def __init__(self, name, count_no, parent=None):
        super().__init__(name, parent)
        self.count_no = count_no
        self.sum = 0
        self.count = 1
        self.phase = "C"
        self.addOutputPorts("!c")

    def delta_0(self):
        """Initialize the counter state."""
        self.phase, self.count, self.sum = "C", 1, 0

    def tau(self):
        """Return the time until the next internal event."""
        if self.phase == "C":
            return 5
        return float("inf")

    def delta_y(self):
        """Emit the current sum, then update the counter state."""
        output = PortValue(self.OP["!c"], self.sum)
        if self.phase == "C":
            self.sum += self.count
            self.count += 1
            if self.count > self.count_no:
                self.phase = "D"
        return output

    def get_state(self):
        """Return the complete model state."""
        return self.phase, self.count, self.sum, self.count_no

    def set_state(self, state):
        """Restore the complete model state."""
        self.phase, self.count, self.sum, self.count_no = state

    def display_state(self):
        """Return the phase label, completed count and sum for the example UI."""
        phase = "Count" if self.phase == "C" else "Done"
        return phase, self.count - 1, self.sum


def runSimulation(model):
    """Run the model with the interactive simulation engine."""
    from socs.devs.SimEngine import SimEngineThread

    engine = SimEngineThread(model)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.runInteractiveMenu()


if __name__ == "__main__":
    if __package__:
        from .DEVSphCounter import main
    else:
        from DEVSphCounter import main

    main(Counter, __doc__)
