"""Executors for generic DEVS external-input decisions."""

from .control import apply_control
from .decision import ControlDecision
from .executor import DecisionExecutor
from .simulation_executor import SimulationDecisionExecutor
from ..devs.TESEngin import TESEnginBase


class ControlRuntimeExecutor(DecisionExecutor[TESEnginBase, ControlDecision]):
    """Apply a control decision to a live simulation engine."""

    def execute(
        self,
        state: TESEnginBase,
        decision: ControlDecision,
    ) -> TESEnginBase:
        if not isinstance(state, TESEnginBase):
            raise TypeError("state must be a TESEnginBase instance.")
        apply_control(state, decision)
        return state


class ControlSimulationDecisionExecutor(
    SimulationDecisionExecutor[ControlDecision]
):
    """Apply a control decision to a restored branch before simulation."""

    def apply_decision(self, model, decision: ControlDecision) -> None:
        apply_control(self.engine, decision)
