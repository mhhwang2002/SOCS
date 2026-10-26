#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""Finite and deterministic DEVS base class.

This class replaces the former FSDEVS -> FSTDEVS -> FRTDEVS -> FDDEVS
inheritance chain with a single deterministic layer over Simulatable.
"""

import math
from numbers import Real

from .Simulatable import Simulatable


class FDDEVS(Simulatable):
    """Finite-state deterministic DEVS atomic model.

    Subclasses implement delta_0(), tau(), delta_x(), delta_y(),
    set_state(), and get_state().

    delta_x(x) must update the model state and return True when the next
    internal event must be rescheduled, or False to preserve its schedule.
    delta_y() returns the output event, or None for an unobservable event.
    """

    def __init__(self, name, parent=None, tl=0, tn=float("inf")):
        super().__init__(name, parent, tl, tn)

    def ta(self):
        """Return the deterministic time advance."""
        value = self.tau()
        if not isinstance(value, Real):
            raise TypeError(
                f"tau() must return a real number; got {type(value).__name__}."
            )
        if math.isnan(value) or value < 0:
            raise ValueError("tau() must return a non-negative value.")
        return value

    def Delta_x(self, x, t):
        """Apply an external event, rescheduling only when requested."""
        if t < self.timeLast() or t > self.timeNext():
            raise ValueError(
                f"Delta_x scheduling error: "
                f"tL={self.timeLast()}, t={t}, tN={self.timeNext()}"
            )

        self.Delta_null(t)
        reschedule = self.delta_x(x)
        if reschedule:
            self.set_timeLast(t)
            self.set_timeNext(t + self.ta())

    def set_state(self, state):
        """Set the model's current state; implement in a concrete model."""
        raise NotImplementedError("Implement set_state(state) in the model.")

    def get_state(self):
        """Return the model's current state; implement in the concrete model."""
        raise NotImplementedError("Implement get_state() in the model.")

    def state_to_s(self):
        """Return a string representation of the current model state."""
        state = self.get_state()
        if isinstance(state, str):
            return state
        if hasattr(state, "__iter__"):
            return ",".join(str(value) for value in state)
        return str(state)
