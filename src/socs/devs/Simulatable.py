#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""
Simulatable classes for Classic Discrete Event System Specification (DEVS).
Originally written in 2011 and modernized for Python 3.
"""
from copy import deepcopy
from .DEVSInterfaces import XY
from .DEVSTime import Manager, Clock
from .DEVSBase import TES
from .Snapshot import ModelSnapshot

from uuid import UUID
class Simulatable(TES, XY, Manager):
    """Simulatable using Classic Discrete Event System Specification (DEVS)."""

    def __init__(self, name, parent=None, tl=0, tn=float("inf")):
        super().__init__(name, parent)
        XY.__init__(self)
        Manager.__init__(self, tl, tn)

        self.t_last_update = tl
        self.StatePercentage = {}
        self._model_id = None

    @property
    def model_id(self) -> UUID | None:
        return self._model_id

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

        self.delta_x(x)
        self.set_timeLast(t)
        self.set_timeNext(t + self.ta())

    def Delta_y(self, t):
        """Produce output and update the model state at time t."""
        if t != self.timeNext():
            raise ValueError(
                f"Delta_y scheduling error: t={t}, tN={self.timeNext()}"
            )

        y = self.delta_y()
        self.set_timeLast(t)
        self.set_timeNext(t + self.ta())
        return y

    def Delta_null(self, t):
        """update continuous state transition and return True denoting unscheduled event; otherwise, return False"""
        self.updateStatistics(t)
        return False
    
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

# --------------------------------------------------
    # Snapshot
    # --------------------------------------------------

    def capture_state(self):
        """Capture this model's dynamic state and its descendants."""
        get_state = getattr(self, "get_state", None)
        set_state = getattr(self, "set_state", None)

        if callable(get_state) != callable(set_state):
            raise TypeError(
                f"{self.path}: get_state() and set_state() "
                "must be implemented together."
            )

        has_model_state = callable(get_state)
        model_state = deepcopy(get_state()) if has_model_state else None

        children = {}

        for child in self.children:
            if not isinstance(child, Simulatable):
                raise TypeError(
                    f"{self.path}: child {child.name!r} is not Simulatable."
                )

            if child.name in children:
                raise ValueError(
                    f"{self.path}: duplicate child name {child.name!r}."
                )

            children[child.name] = child.capture_state()

        return ModelSnapshot(
            model_id=id(self),
            model_path=self.path,
            common_state={
                "time_last": self.timeLast(),
                "time_next": self.timeNext(),
                "t_last_update": self.t_last_update,
                "state_percentage": deepcopy(self.StatePercentage),
            },
            model_state=model_state,
            has_model_state=has_model_state,
            children=children,
        )

    def restore_state(self, snapshot):
        """Restore this model and its descendants from a ModelSnapshot."""
        self._validate_snapshot(snapshot)

        if snapshot.has_model_state:
            self.set_state(deepcopy(snapshot.model_state))

        common_state = snapshot.common_state

        self.set_timeLast(common_state["time_last"])
        self.set_timeNext(common_state["time_next"])

        self.t_last_update = common_state["t_last_update"]
        self.StatePercentage = deepcopy(common_state["state_percentage"])

        for child in self.children:
            child.restore_state(snapshot.children[child.name])

        self._after_restore_state()

    def _validate_snapshot(self, snapshot):
        """Validate snapshot identity and hierarchy before restoration."""
        if not isinstance(snapshot, ModelSnapshot):
            raise TypeError("snapshot must be a ModelSnapshot instance.")

        if snapshot.model_id != id(self):
            raise ValueError(
                f"Snapshot model mismatch: expected {self.path!r}, "
                f"got {snapshot.model_path!r}."
            )

        current_child_names = [child.name for child in self.children]

        if len(current_child_names) != len(set(current_child_names)):
            raise ValueError(
                f"{self.path}: duplicate child names in model hierarchy."
            )

        if set(current_child_names) != set(snapshot.children):
            raise ValueError(
                f"{self.path}: snapshot hierarchy does not match "
                "the current model hierarchy."
            )

        get_state = getattr(self, "get_state", None)
        set_state = getattr(self, "set_state", None)

        supports_model_state = callable(get_state) and callable(set_state)

        if callable(get_state) != callable(set_state):
            raise TypeError(
                f"{self.path}: get_state() and set_state() "
                "must be implemented together."
            )

        if supports_model_state != snapshot.has_model_state:
            raise ValueError(
                f"{self.path}: snapshot model-state contract "
                "does not match the model."
            )

        if not isinstance(snapshot.common_state, dict):
            raise TypeError(
                f"{self.path}: snapshot common_state must be a dictionary."
            )

        required_keys = {
            "time_last",
            "time_next",
            "t_last_update",
            "state_percentage",
        }

        if not required_keys.issubset(snapshot.common_state):
            raise ValueError(
                f"{self.path}: snapshot common_state is incomplete."
            )

        for child in self.children:
            child._validate_snapshot(snapshot.children[child.name])

    def _after_restore_state(self):
        """Rebuild derived runtime state after restoration."""

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

