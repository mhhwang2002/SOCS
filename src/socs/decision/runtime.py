"""Invoke decision generation after each DEVS execution."""

from dataclasses import dataclass
from typing import Generic, TypeVar

from ..devs.SimEngine import (
    ExecutionResult,
    SimEngineBase,
)
from ..devs.Snapshot import SystemSnapshot
from .decision import Decision
from .generator import DecisionGenerator

DecisionT = TypeVar("DecisionT", bound=Decision)


@dataclass(frozen=True)
class DecisionRuntimeStep(Generic[DecisionT]):
    """Result of one engine step and its optional decision generation."""

    execution_result: ExecutionResult
    snapshot: SystemSnapshot | None = None
    decisions: tuple[DecisionT, ...] = ()


class DecisionRuntimeController(Generic[DecisionT]):
    """Run one engine step and generate decisions after every DEVS event.

    Internal DEVS transitions and external-input transitions both trigger
    decision generation. Continuous, animation, and end-of-run steps do not.
    This controller does not select or execute generated decisions.
    """

    def __init__(
        self,
        engine: SimEngineBase,
        generator: DecisionGenerator[SystemSnapshot],
    ) -> None:
        if not isinstance(engine, SimEngineBase):
            raise TypeError("engine must be a SimEngineBase instance.")
        if not isinstance(generator, DecisionGenerator):
            raise TypeError("generator must be a DecisionGenerator instance.")

        self.engine = engine
        self.generator = generator
        self.devs_execution_count = 0

    def execute_one_step(self) -> DecisionRuntimeStep[DecisionT]:
        """Execute one engine step and generate decisions after DEVS events."""
        result = self.engine.executeOneStep()

        if result not in (
            ExecutionResult.ScheduledEventOccurred,
            ExecutionResult.ExternalEventOccurred,
        ):
            return DecisionRuntimeStep(execution_result=result)

        snapshot = self.engine.capture_snapshot()
        decisions = tuple(self.generator.generate(snapshot))

        for decision in decisions:
            if not isinstance(decision, Decision):
                raise TypeError(
                    "DecisionGenerator must yield Decision instances."
                )

        self.devs_execution_count += 1
        return DecisionRuntimeStep(
            execution_result=result,
            snapshot=snapshot,
            decisions=decisions,
        )
