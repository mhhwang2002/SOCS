"""Coordinate one-step expansion of a decision tree."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Hashable

from .decision import Decision
from .generator import DecisionGenerator
from .lifecycle import DecisionLifeCycleManager, DecisionStatus
from .executor import DecisionExecutor
from ..devs.Snapshot import SystemSnapshot
from ..tree.decision_tree import DecisionTree


class ExpansionStatus(Enum):
    """Outcome of one decision expansion attempt."""

    EXECUTED = auto()
    NO_DECISION = auto()
    FAILED = auto()


@dataclass(frozen=True)
class ExpansionResult:
    """Result returned by a single-node expansion attempt."""

    parent_node_id: Hashable
    status: ExpansionStatus
    decision_id: int | None = None
    child_node_id: int | None = None
    error: Exception | None = None


class DecisionExpansionController:
    """Connect a decision manager, evaluator, and decision tree.

    A node must first be registered with a generator. When an expansion
    succeeds, the resulting child node is registered automatically with the
    same generator so that it can be expanded later.
    """

    def __init__(
        self,
        tree: DecisionTree,
        lifecycle_manager: DecisionLifeCycleManager,
        executor: DecisionExecutor[SystemSnapshot, Decision],
    ) -> None:
        if not isinstance(tree, DecisionTree):
            raise TypeError("tree must be a DecisionTree instance.")
        if not isinstance(lifecycle_manager, DecisionLifeCycleManager):
            raise TypeError("lifecycle_manager must be a DecisionLifeCycleManager instance.")
        if not isinstance(executor, DecisionExecutor):
            raise TypeError("executor must be a DecisionExecutor instance.")

        self.tree = tree
        self.lifecycle_manager = lifecycle_manager
        self.executor = executor
        self._node_generators: dict[Hashable, DecisionGenerator] = {}

    def register_node(
        self,
        node_id: Hashable,
        generator: DecisionGenerator,
    ) -> None:
        """Register a tree node for expansion using the supplied generator."""
        if node_id in self._node_generators:
            raise ValueError(f"Node {node_id!r} is already registered.")

        snapshot = self.tree.get_snapshot(node_id)
        self.lifecycle_manager.register_node(node_id, snapshot, generator)
        self._node_generators[node_id] = generator

    def expand(self, node_id: Hashable) -> ExpansionResult:
        """Execute one pending decision from a node and add its child node.

        If no decision is pending, at most one decision is requested from the
        node's lazy generator. Generator exceptions are allowed to propagate;
        evaluation or tree-insertion failures are recorded as FAILED results.
        """
        try:
            generator = self._node_generators[node_id]
        except KeyError:
            raise KeyError(f"Node {node_id!r} is not registered for expansion.") from None

        pending = self.lifecycle_manager.get_pending_decisions(node_id)
        if not pending:
            self.lifecycle_manager.generate_decisions(node_id, max_count=1)

        record = self.lifecycle_manager.select_decision(node_id)
        if record is None:
            return ExpansionResult(
                parent_node_id=node_id,
                status=ExpansionStatus.NO_DECISION,
            )

        child_node_id = None
        try:
            parent_snapshot = self.tree.get_snapshot(node_id)
            result_snapshot = self.executor.execute(
                parent_snapshot,
                record.decision,
            )
            child_node_id = self.tree.add_child(
                node_id,
                record.decision,
                result_snapshot,
            )
            try:
                self.lifecycle_manager.register_node(
                    child_node_id,
                    result_snapshot,
                    generator,
                )
            except Exception:
                self.tree.remove_vertex(child_node_id)
                child_node_id = None
                raise

            self._node_generators[child_node_id] = generator
            self.lifecycle_manager.mark_executed(
                record.decision_id,
                child_node_id,
            )
            return ExpansionResult(
                parent_node_id=node_id,
                status=ExpansionStatus.EXECUTED,
                decision_id=record.decision_id,
                child_node_id=child_node_id,
            )
        except Exception as exc:
            if record.status is DecisionStatus.EXECUTING:
                self.lifecycle_manager.mark_failed(record.decision_id, exc)
            return ExpansionResult(
                parent_node_id=node_id,
                status=ExpansionStatus.FAILED,
                decision_id=record.decision_id,
                error=exc,
            )
