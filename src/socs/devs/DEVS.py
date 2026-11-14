#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""Finite and deterministic DEVS base class."""

import math
from abc import ABC, abstractmethod
from numbers import Real

from .TESBase import PortValue, TES


class DEVS(TES, ABC):
    """Finite-state deterministic DEVS atomic model.

    Subclasses implement set_initial_state(), tau(), delta_x(), delta_y(),
    set_state(), and get_state().

    delta_x(x) must update the model state and return True when the next
    internal event must be rescheduled, or False to preserve its schedule.
    delta_y() returns the output event, or None for an unobservable event.
    """

    def __init__(self, name, parent=None, tl=0, tn=float("inf")):
        super().__init__(name, parent, tl, tn)

    @abstractmethod
    def set_initial_state(self) -> None:
        """Set the initial model state."""

    @abstractmethod
    def tau(self) -> float | int:
        """Return the time until the next internal event."""

    @abstractmethod
    def delta_x(self, event: PortValue) -> bool:
        """Process an external event and return whether to reschedule."""

    @abstractmethod
    def delta_y(self) -> PortValue | list[PortValue] | tuple[PortValue, ...] | set[PortValue] | None:
        """Process an internal event and return its output events, if any."""

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

    def set_init_total_state(self, t):
        """Reset the model state and clock at time t."""
        self._statistics_since = t
        self.StatePercentage = {}
        self.tNow = t
        self.tLast = t
        self.set_initial_state()
        self.tNext = t + self.ta()

    def delta_cont(self, elapsed):
        """Advance continuous state and optionally request rescheduling."""
        return False

    def Delta_x(self, x: PortValue | None, t) -> bool:
        """Apply an external event, rescheduling only when requested."""
        self.validate_event_time(t)

        if x is not None and not isinstance(x, PortValue):
            raise TypeError("Delta_x expects a PortValue or None.")
        previous_time = self.tLast
        self.tNow = t
        reschedule = False
        if t > previous_time and type(self).delta_cont is not DEVS.delta_cont:
            self._continuous_elapsed = t - previous_time
            try:
                reschedule = self.delta_x(None)
            finally:
                del self._continuous_elapsed
            if not isinstance(reschedule, bool):
                raise TypeError("delta_x() must return a bool.")

        if x is not None:
            self.update_statistics(t)
            input_reschedule = self.delta_x(x)
            if not isinstance(input_reschedule, bool):
                raise TypeError("delta_x() must return a bool.")
            reschedule = reschedule or input_reschedule
        if not isinstance(reschedule, bool):
            raise TypeError("delta_x() must return a bool.")
        self.tLast = t
        if reschedule:
            self.tNext = t + self.ta()
        return reschedule

    def Delta_y(self, t) -> PortValue | list[PortValue] | tuple[PortValue, ...] | set[PortValue] | None:
        """Process a scheduled internal event and update the clock."""
        self.validate_event_time(t, internal=True)
        self.tNow = t
        self.update_statistics(t)
        output = self.delta_y()
        if output is not None and not isinstance(output, PortValue):
            if not isinstance(output, (list, tuple, set)) or not all(
                isinstance(event, PortValue) for event in output
            ):
                raise TypeError("delta_y() must return PortValue events or None.")
        self.tLast = t
        self.tNext = t + self.ta()
        return output

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
