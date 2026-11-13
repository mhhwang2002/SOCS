"""Exhaustive decision-tree optimization for the LogBridge example."""

from collections.abc import Callable
import math

from socs.decision.expansion import DecisionExpansionController
from socs.decision.lifecycle import DecisionLifeCycleManager
from socs.decision.search import ExhaustiveDecisionTreeSearch, ExhaustiveSearchResult
from socs.devs.TESEngin import ExecutionResult, TESEnginBase
from socs.devs.Snapshot import SystemSnapshot
from socs.tree.decision_tree import DecisionTree

from .decision_generator import LogBridgeDecisionGenerator
from .decision_executor import LogBridgeDecisionExecutor
from .LoadPassive import LoadPassive
from .LogBridge import LogBridge


class LogBridgePathExecutor(LogBridgeDecisionExecutor):
    """Advance each branch to its next decision point or terminal condition."""

    def __init__(
        self,
        engine: TESEnginBase,
        end_time: float,
        generator: LogBridgeDecisionGenerator,
        completion_evaluator: Callable[[SystemSnapshot], float | None],
    ) -> None:
        super().__init__(engine=engine, end_time=end_time)
        self.generator = generator
        self.completion_evaluator = completion_evaluator

    def run_branch(self) -> None:
        """Stop at a new decision point, completion, deadlock, or time bound."""
        while True:
            snapshot = self.engine.capture_snapshot()
            if self.completion_evaluator(snapshot) is not None:
                return

            if not self.engine.XBuffer and not math.isfinite(
                self.engine.model.tNext
            ):
                return

            result = self.engine.executeOneStep()
            if result is ExecutionResult.TimeBoundReached:
                return

            if result in (
                ExecutionResult.ScheduledEventOccurred,
                ExecutionResult.ExternalEventOccurred,
            ):
                snapshot = self.engine.capture_snapshot()
                if self.completion_evaluator(snapshot) is not None:
                    return
                if next(self.generator.generate(snapshot), None) is not None:
                    return


def advance_to_first_decision_point(
    engine: TESEnginBase,
    generator: LogBridgeDecisionGenerator,
    end_time: float,
) -> SystemSnapshot:
    """Run the initial model until the first state with Decision candidates."""
    engine.set_simulation_time_bound(end_time)

    while True:
        if not engine.XBuffer and not math.isfinite(engine.model.tNext):
            raise RuntimeError(
                "The initial model deadlocked before reaching a decision point."
            )

        result = engine.executeOneStep()
        if result is ExecutionResult.TimeBoundReached:
            raise RuntimeError(
                "The simulation time bound was reached before the first decision point."
            )

        if result in (
            ExecutionResult.ScheduledEventOccurred,
            ExecutionResult.ExternalEventOccurred,
        ):
            snapshot = engine.capture_snapshot()
            if next(generator.generate(snapshot), None) is not None:
                return snapshot


class LogBridgeMakespanEvaluator:
    """Return the completion time for a fully completed LogBridge schedule."""

    def __init__(
        self,
        source_load_names=("L11", "L21"),
        destination_load_names=("L12", "L22"),
        bridge_name="LB",
    ) -> None:
        self.source_load_names = tuple(source_load_names)
        self.destination_load_names = tuple(destination_load_names)
        self.bridge_name = bridge_name

    def __call__(self, snapshot: SystemSnapshot) -> float | None:
        """Return makespan for completed schedules, or None if infeasible."""
        children = snapshot.root_model.children
        required = (
            set(self.source_load_names)
            | set(self.destination_load_names)
            | {self.bridge_name}
        )
        if not required.issubset(children):
            raise ValueError("Snapshot does not contain the configured LogBridge models.")

        states = {}
        for name in required:
            child = children[name]
            if not child.has_model_state or not child.model_state:
                raise ValueError(f"Model {name!r} has no valid saved state.")
            states[name] = child.model_state[0]

        sources_idle = all(
            states[name] == "I" for name in self.source_load_names
        )
        destinations_complete = all(
            states[name] == "W" for name in self.destination_load_names
        )
        bridge_idle = states[self.bridge_name] == "I"

        if sources_idle and destinations_complete and bridge_idle:
            return snapshot.t_now
        return None


def run_logbridge_exhaustive_search(
    model,
    end_time: float = 200.0,
    engine: TESEnginBase | None = None,
) -> ExhaustiveSearchResult:
    """Generate all finite Decision paths and minimize total completion time.

    An existing engine may be supplied so its model instances can also be
    reused later to replay and validate the resulting DecisionTree snapshots.
    """
    if engine is None:
        from socs.devs.TESEngin import TESEnginThread

        engine = TESEnginThread(model)
    elif engine.model is not model:
        raise ValueError("The supplied engine must own the supplied model instance.")
    engine.bVerbose = False

    generator = LogBridgeDecisionGenerator()
    evaluator = LogBridgeMakespanEvaluator()

    root_snapshot = advance_to_first_decision_point(
        engine=engine,
        generator=generator,
        end_time=end_time,
    )

    tree = DecisionTree()
    root_id = tree.add_root(root_snapshot)

    lifecycle_manager = DecisionLifeCycleManager()
    executor = LogBridgePathExecutor(
        engine=engine,
        end_time=end_time,
        generator=generator,
        completion_evaluator=evaluator,
    )
    expansion_controller = DecisionExpansionController(
        tree=tree,
        lifecycle_manager=lifecycle_manager,
        executor=executor,
    )
    expansion_controller.register_node(root_id, generator)

    search = ExhaustiveDecisionTreeSearch(
        expansion_controller=expansion_controller,
        evaluator=evaluator,
    )
    return search.run(root_id)
