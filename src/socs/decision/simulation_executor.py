"""DEVS simulation-backed decision execution."""

from abc import abstractmethod
import math
from typing import TypeVar

from .decision import Decision
from .executor import DecisionExecutor
from ..devs.TESEngin import ExecutionResult, TESEnginBase
from ..devs.Snapshot import SystemSnapshot


DecisionT = TypeVar("DecisionT", bound=Decision)


class SimulationDecisionExecutor(
    DecisionExecutor[SystemSnapshot, DecisionT],
):
    """Execute a decision by restoring and advancing a DEVS simulation."""

    def __init__(self, engine: TESEnginBase, end_time: float) -> None:
        self.engine = engine
        self.end_time = float(end_time)

        if not math.isfinite(self.end_time) or self.end_time < 0:
            raise ValueError("end_time must be a finite, non-negative number.")

    def execute(
        self,
        state: SystemSnapshot,
        decision: DecisionT,
    ) -> SystemSnapshot:
        """Execute one decision from a snapshot and return the resulting snapshot."""
        if not isinstance(state, SystemSnapshot):
            raise TypeError("state must be a SystemSnapshot instance.")
        if state.t_now > self.end_time:
            raise ValueError(
                "The snapshot time cannot be later than the execution time horizon."
            )

        restored = False
        try:
            self.engine.restore_snapshot(state)
            restored = True
            self.engine.set_simulation_time_bound(self.end_time)
            self.apply_decision(self.engine.model, decision)
            self.run_branch()
            return self.engine.capture_snapshot()
        except Exception:
            # Keep the executor reusable after a failed branch evaluation.
            if restored:
                self.engine.restore_snapshot(state)
            raise

    @abstractmethod
    def apply_decision(self, model, decision: DecisionT) -> None:
        """Interpret and apply decision data to the restored model."""
        raise NotImplementedError

    def run_branch(self) -> None:
        """Advance the engine until the configured absolute time horizon."""
        while True:
            result = self.engine.execute_one_step()
            if result is ExecutionResult.TimeBoundReached:
                return
