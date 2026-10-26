#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""
Simulatable classes for Classic Discrete Event System Specification (DEVS).
Originally written in 2011 and modernized for Python 3.
"""

from .DEVSInterfaces import XY
from .DEVSTime import Manager, Clock
from .DEVSBase import TES


class Simulatable(TES, XY, Manager):
    """Simulatable using Classic Discrete Event System Specification (DEVS)."""

    def __init__(self, name, parent=None, tl=0, tn=float("inf")):
        super().__init__(name, parent)
        XY.__init__(self)
        Manager.__init__(self, tl, tn)

        self.t_last_update = tl
        self.StatePercentage = {}

    def delta_0(self):
        """Create the initial state."""
        raise NotImplementedError("Simulatable.delta_0() must be overridden.")

    def ta(self):
        """Return the remaining time advance."""
        return self.timeAdvance()

    def delta_x(self, x):
        """Apply an external transition."""
        raise NotImplementedError("Simulatable.delta_x() must be overridden.")

    def delta_y(self):
        """Produce output and perform the internal transition."""
        raise NotImplementedError("Simulatable.delta_y() must be overridden.")

    def Delta_0(self, t):
        """Reset the model state and clock at time t."""
        Clock.tNow = t
        self.t_last_update = t
        self.StatePercentage = {}

        self.delta_0()
        self.set_timeLast(t)
        self.set_timeNext(t + self.ta())

    def Delta_x(self, x, t):
        """Inject input x at time t and update the model state."""
        if t < self.timeLast() or t > self.timeNext():
            raise ValueError(
                "Delta_x scheduling error: "
                f"tL={self.timeLast()}, t={t}, tN={self.timeNext()}"
            )

        self.Delta_null(t)
        self.delta_x(x)
        self.set_timeLast(t)
        self.set_timeNext(t + self.ta())

    def Delta_y(self, t):
        """Produce output and update the model state at time t."""
        if t != self.timeNext():
            raise ValueError(
                f"Delta_y scheduling error: t={t}, tN={self.timeNext()}"
            )

        self.Delta_null(t)
        y = self.delta_y()
        self.set_timeLast(t)
        self.set_timeNext(t + self.ta())
        return y

    def Delta_null(self, t):
        """Update time-dependent statistics without a state transition."""
        self.updateStatistics(t)

    def updateStatistics(self, t):
        """Accumulate time spent in each statistics state."""
        state = self.get_statistics_state()
        if state not in self.StatePercentage:
            self.StatePercentage[state] = 0.0

        t_last_update = max(self.t_last_update, self.timeLast())
        self.StatePercentage[state] += t - t_last_update
        self.t_last_update = t

    def get_statistics_state(self):
        """Return the current state used for statistics."""
        return self.state_to_s()

    def get_statistics_str(self):
        """Return normalized state-time statistics as a string."""
        total_t = sum(self.StatePercentage.values())
        if total_t <= 0:
            return ""

        return "".join(
            f"P({state})={duration / total_t:.3f},"
            for state, duration in self.StatePercentage.items()
        )

    def __str__(self):
        """Return a string representation of the model and its interfaces."""
        result = TES.__str__(self) + "\n"
        result += self.to_IP_s() + "\n"
        result += self.to_OP_s() + "\n"
        result += "clock=" + str(self.clock) + "\n"
        return result

    def state_to_s(self):
        """Return a string representation of the model state."""
        return ""

    def qstate_to_s(self):
        """Return a string representation of the total state."""
        state = self.state_to_s()
        if state:
            return f"({self.name}:{state},{self.clock})"
        return f"({self.name}:{self.clock})"


#class DEVS(Simulatable):
#    """Classic DEVS atomic model base class."""
#
#    def __init__(self, name, parent=None, tl=0, tn=float("inf")):
#        super().__init__(name, parent, tl, tn)
#
#    def delta_x(self, x):
#        """Dispatch external input to the external transition function."""
#        self.delta_ext(x)
#
#    def delta_y(self):
#        """Produce output and dispatch the internal transition."""
#        y = self._lambda()
#        self.delta_int()
#        return y


if __name__ == "__main__":
    # A concrete subclass is required to run a meaningful simulation.
    print("Simulatable module syntax check passed.")

