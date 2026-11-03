from socs.decision.simulation_executor import SimulationDecisionExecutor
from socs.devs.DEVSInterfaces import PortValue

from .LoadPassive import LoadPassive
from .LogBridge import LogBridge
from .decisions import EnterBridgeDecision


class LogBridgeDecisionExecutor(
    SimulationDecisionExecutor[EnterBridgeDecision]
):
    """Apply a bridge-entry decision and simulate the resulting branch."""

    def apply_decision(self, model, decision: EnterBridgeDecision) -> None:
        """Queue the GO input for the selected waiting Load."""

        if not isinstance(decision, EnterBridgeDecision):
            raise TypeError("decision must be an EnterBridgeDecision instance.")

        target = next(
            (
                component
                for component in model.children
                if isinstance(component, LoadPassive)
                and component.name == decision.load_name
            ),
            None,
        )
        if target is None:
            raise ValueError(
                f"Load {decision.load_name!r} does not exist in the model."
            )

        bridge = next(
            (
                component
                for component in model.children
                if isinstance(component, LogBridge)
            ),
            None,
        )
        if bridge is None:
            raise ValueError("The model must contain one LogBridge.")

        if bridge.Phase != "I":
            raise ValueError("The LogBridge is not idle.")

        if target.phase != "W":
            raise ValueError(
                f"Load {target.name!r} is not waiting; current phase is {target.phase!r}."
            )

        if self.engine.XBuffer:
            raise ValueError("Cannot apply a decision while XBuffer is not empty.")

        self.engine.inject(target, PortValue(target.IP["?go"]))
