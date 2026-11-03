"""Fixed-priority controller for the passive LogBridge example."""

from socs.devs.DEVSInterfaces import PortValue

from .LoadPassive import LoadPassive
from .LogBridge import LogBridge


class FixedPriorityController:
    """Authorize waiting Loads in a fixed priority order."""

    def __init__(self, priority=("L11", "L21")):
        if not priority:
            raise ValueError("priority must contain at least one Load name.")
        if len(set(priority)) != len(priority):
            raise ValueError("priority must not contain duplicate Load names.")
        self.priority = tuple(priority)
        self.dispatch_history = []

    def dispatch(self, engine, model):
        """Dispatch one GO event when the Bridge and Loads are ready.

        Returns the selected Load, or None when no dispatch is possible.
        """
        loads = {
            component.name: component
            for component in model.children
            if isinstance(component, LoadPassive)
        }
        bridges = [
            component
            for component in model.children
            if isinstance(component, LogBridge)
        ]

        if len(bridges) != 1:
            raise ValueError("The model must contain exactly one LogBridge.")
        bridge = bridges[0]

        if bridge.Phase != "I":
            return None
        if engine.XBuffer:
            return None

        # A Load in S has already received GO and is about to emit its walker.
        # Do not authorize another Load before that zero-time transition runs.
        if any(load.phase == "S" for load in loads.values()):
            return None

        for load_name in self.priority:
            load = loads.get(load_name)
            if load is not None and load.phase == "W":
                engine.inject(load, PortValue(load.IP["?go"]))
                self.dispatch_history.append((engine.tNow, load.name))
                return load

        return None
