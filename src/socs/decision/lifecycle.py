"""Manage generated decisions and their execution states."""

from collections.abc import Iterator
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Hashable

from .decision import Decision
from .generator import DecisionGenerator


class DecisionStatus(Enum):
    """Execution status of a generated decision."""

    PENDING = auto()
    EXECUTING = auto()
    EXECUTED = auto()
    FAILED = auto()


@dataclass
class DecisionRecord:
    """Manager-owned metadata for a decision."""

    decision_id: int
    node_id: Hashable
    decision: Decision
    status: DecisionStatus = DecisionStatus.PENDING
    child_node_id: Hashable | None = None
    error: str | None = None


@dataclass
class _NodeDecisionPool:
    """Decision generation state associated with one tree node."""

    iterator: Iterator[Decision]
    exhausted: bool = False


class DecisionLifeCycleManager:
    """Generate, store, select, and track decisions independently of the tree."""

    def __init__(self) -> None:
        self._node_pools: dict[Hashable, _NodeDecisionPool] = {}
        self._records: dict[int, DecisionRecord] = {}
        self._node_decision_ids: dict[Hashable, list[int]] = {}
        self._next_decision_id = 1

    def register_node(
        self,
        node_id: Hashable,
        state: Any,
        generator: DecisionGenerator,
    ) -> None:
        """Register a node and create its lazy decision iterator."""
        if node_id in self._node_pools:
            raise ValueError(f"Node {node_id!r} is already registered.")
        if not isinstance(generator, DecisionGenerator):
            raise TypeError("generator must be a DecisionGenerator instance.")

        iterator = iter(generator.generate(state))
        self._node_pools[node_id] = _NodeDecisionPool(iterator=iterator)
        self._node_decision_ids[node_id] = []

    def generate_decisions(
        self,
        node_id: Hashable,
        max_count: int | None = None,
    ) -> list[DecisionRecord]:
        """Generate and store up to max_count decisions for a registered node.

        If max_count is None, generation continues until the iterator is exhausted.
        """
        pool = self._get_pool(node_id)
        if max_count is not None:
            if isinstance(max_count, bool) or not isinstance(max_count, int):
                raise TypeError("max_count must be an integer or None.")
            if max_count < 0:
                raise ValueError("max_count cannot be negative.")

        generated: list[DecisionRecord] = []
        while not pool.exhausted and (
            max_count is None or len(generated) < max_count
        ):
            try:
                decision = next(pool.iterator)
            except StopIteration:
                pool.exhausted = True
                break

            if not isinstance(decision, Decision):
                raise TypeError(
                    "DecisionGenerator must yield Decision instances."
                )

            record = DecisionRecord(
                decision_id=self._next_decision_id,
                node_id=node_id,
                decision=decision,
            )
            self._next_decision_id += 1
            self._records[record.decision_id] = record
            self._node_decision_ids[node_id].append(record.decision_id)
            generated.append(record)

        return generated

    def get_pending_decisions(
        self,
        node_id: Hashable,
    ) -> list[DecisionRecord]:
        """Return pending decisions for a node in generation order."""
        self._get_pool(node_id)
        return [
            self._records[decision_id]
            for decision_id in self._node_decision_ids[node_id]
            if self._records[decision_id].status is DecisionStatus.PENDING
        ]

    def mark_executing(self, decision_id: int) -> DecisionRecord:
        """Mark a pending decision as executing and return its record."""
        record = self.get_record(decision_id)
        if record.status is not DecisionStatus.PENDING:
            raise ValueError(
                f"Decision {record.decision_id} is not pending."
            )
        record.status = DecisionStatus.EXECUTING
        return record

    def select_decision(self, node_id: Hashable) -> DecisionRecord | None:
        """Select the first pending decision using the legacy FIFO behavior.

        New code should use a DecisionSelector and mark_executing().
        """
        pending = self.get_pending_decisions(node_id)
        if not pending:
            return None
        return self.mark_executing(pending[0].decision_id)

    def get_record(self, decision_id: int) -> DecisionRecord:
        """Return the manager record for a decision ID."""
        try:
            return self._records[decision_id]
        except KeyError:
            raise KeyError(
                f"Decision ID {decision_id!r} is not registered."
            ) from None

    def mark_executed(
        self,
        decision_id: int,
        child_node_id: Hashable,
    ) -> None:
        """Mark an executing decision as completed and store its child node ID."""
        record = self.get_record(decision_id)
        self._require_executing(record)
        record.status = DecisionStatus.EXECUTED
        record.child_node_id = child_node_id
        record.error = None

    def mark_failed(self, decision_id: int, error: Exception | str) -> None:
        """Mark an executing decision as failed."""
        record = self.get_record(decision_id)
        self._require_executing(record)
        record.status = DecisionStatus.FAILED
        record.error = str(error)

    def is_exhausted(self, node_id: Hashable) -> bool:
        """Return whether the node's decision generator is exhausted."""
        return self._get_pool(node_id).exhausted

    def is_complete(self, node_id: Hashable) -> bool:
        """Return whether generation is exhausted and no decision is unresolved."""
        pool = self._get_pool(node_id)
        if not pool.exhausted:
            return False
        return all(
            self._records[decision_id].status
            in (DecisionStatus.EXECUTED, DecisionStatus.FAILED)
            for decision_id in self._node_decision_ids[node_id]
        )

    def _get_pool(self, node_id: Hashable) -> _NodeDecisionPool:
        try:
            return self._node_pools[node_id]
        except KeyError:
            raise KeyError(f"Node {node_id!r} is not registered.") from None

    @staticmethod
    def _require_executing(record: DecisionRecord) -> None:
        if record.status is not DecisionStatus.EXECUTING:
            raise ValueError(
                f"Decision {record.decision_id} is not executing."
            )
