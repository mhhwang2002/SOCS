"""FIFO controller for the passive LogBridge example."""

from socs.devs.DEVSInterfaces import PortValue

from .LoadPassive import LoadPassive
from .LogBridge import LogBridge


class FIFOController:
    """Authorize the waiting Load with the earliest waiting-entry time."""

    def __init__(self):
        self.dispatch_history = []

    def dispatch(self, engine, model):
        """Dispatch one GO event when the Bridge and Loads are ready.

        The Load with the smallest timeLast() has waited the longest.
        Returns the selected Load, or None when no dispatch is possible.
        """
        loads = [
            component
            for component in model.children
            if isinstance(component, LoadPassive)
        ]
        bridges = [
            component
            for component in model.children
            if isinstance(component, LogBridge)
        ]

        if len(bridges) != 1:
            raise ValueError("The model must contain exactly one LogBridge.")
        if bridges[0].Phase != "I":
            return None
        if engine.XBuffer:
            return None
        if any(load.phase == "S" for load in loads):
            return None

        waiting_loads = [load for load in loads if load.phase == "W"]
        if not waiting_loads:
            return None

        # min() is stable, so equal timestamps retain model child order.
        selected = min(waiting_loads, key=lambda load: load.timeLast())
        engine.inject(selected, PortValue(selected.IP["?go"]))
        self.dispatch_history.append((engine.tNow, selected.name))
        return selected
