"""Coordinate DEVS execution and live decision handling."""

from dataclasses import dataclass
from typing import Generic, TypeVar

from ..devs.SimEngine import ExecutionResult, SimEngineBase
from ..devs.Snapshot import SystemSnapshot
from .decision import Decision
from .executor import DecisionExecutor
from .generator import DecisionGenerator
from .lifecycle import DecisionRecord, DecisionStatus
from .selector import DecisionSelector

DecisionT = TypeVar("DecisionT", bound=Decision)


@dataclass(frozen=True)
class DecisionRuntimeStep(Generic[DecisionT]):
    """Result of one engine step and its optional decision handling."""

    execution_result: ExecutionResult
    snapshot: SystemSnapshot | None = None
    decisions: tuple[DecisionT, ...] = ()
    records: tuple[DecisionRecord, ...] = ()
    selected_record: DecisionRecord | None = None
    decision_error: Exception | None = None


class DecisionRuntimeController(Generic[DecisionT]):
    """Run a DEVS step, generate decisions, select one, and apply it.

    Internal DEVS transitions and external-input transitions both trigger
    decision generation. Continuous, animation, and end-of-run steps do not.

    The selector and executor are optional as a pair. When supplied, generated
    decisions are wrapped in runtime records, one is selected, and the live
    executor applies it. This is distinct from branch evaluation using a
    SystemSnapshot-backed SimulationDecisionExecutor.
    """

    def __init__(
        self,
        engine: SimEngineBase,
        generator: DecisionGenerator[SystemSnapshot],
        selector: DecisionSelector | None = None,
        executor: DecisionExecutor[SimEngineBase, DecisionT] | None = None,
    ) -> None:
        if not isinstance(engine, SimEngineBase):
            raise TypeError("engine must be a SimEngineBase instance.")
        if not isinstance(generator, DecisionGenerator):
            raise TypeError("generator must be a DecisionGenerator instance.")
        if (selector is None) != (executor is None):
            raise ValueError("selector and executor must be provided together.")
        if selector is not None and not isinstance(selector, DecisionSelector):
            raise TypeError("selector must be a DecisionSelector instance.")
        if executor is not None and not isinstance(executor, DecisionExecutor):
            raise TypeError("executor must be a DecisionExecutor instance.")

        self.engine = engine
        self.generator = generator
        self.selector = selector
        self.executor = executor
        self.devs_execution_count = 0
        self._next_decision_id = 1

    def execute_one_step(self) -> DecisionRuntimeStep[DecisionT]:
        """Execute one engine step and process decisions after DEVS events."""
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

        if self.selector is None:
            return DecisionRuntimeStep(
                execution_result=result,
                snapshot=snapshot,
                decisions=decisions,
            )

        records = tuple(
            DecisionRecord(
                decision_id=self._next_decision_id + index,
                node_id=self.devs_execution_count,
                decision=decision,
            )
            for index, decision in enumerate(decisions)
        )
        self._next_decision_id += len(records)

        selected = self.selector.select(records)
        if selected is None:
            return DecisionRuntimeStep(
                execution_result=result,
                snapshot=snapshot,
                decisions=decisions,
                records=records,
            )

        if not isinstance(selected, DecisionRecord):
            raise TypeError(
                "DecisionSelector must return a DecisionRecord or None."
            )
        if not any(selected is record for record in records):
            raise ValueError(
                "DecisionSelector must return one of the supplied records."
            )

        selected.status = DecisionStatus.EXECUTING
        try:
            self.executor.execute(self.engine, selected.decision)
        except Exception as exc:
            selected.status = DecisionStatus.FAILED
            selected.error = str(exc)
            return DecisionRuntimeStep(
                execution_result=result,
                snapshot=snapshot,
                decisions=decisions,
                records=records,
                selected_record=selected,
                decision_error=exc,
            )

        selected.status = DecisionStatus.EXECUTED
        return DecisionRuntimeStep(
            execution_result=result,
            snapshot=snapshot,
            decisions=decisions,
            records=records,
            selected_record=selected,
        )
