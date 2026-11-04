"""Replay and validate paths stored in a DecisionTree."""

from dataclasses import dataclass
from typing import Any

from socs.devs.Snapshot import SystemSnapshot
from socs.tree import DecisionTree

from .executor import DecisionExecutor


@dataclass(frozen=True)
class PathValidationResult:
    """Outcome of replaying a DecisionTree path."""

    valid: bool
    node_id: int
    validated_edges: int
    failed_edge_id: int | None = None
    expected_snapshot: SystemSnapshot | None = None
    actual_snapshot: SystemSnapshot | None = None
    error: str | None = None


class DecisionPathValidator:
    """Replay a stored decision path and compare every resulting snapshot."""

    def __init__(self, tree: DecisionTree, executor: DecisionExecutor):
        if not isinstance(tree, DecisionTree):
            raise TypeError("tree must be a DecisionTree instance.")
        if not isinstance(executor, DecisionExecutor):
            raise TypeError("executor must be a DecisionExecutor instance.")
        self.tree = tree
        self.executor = executor

    def validate(self, node_id: int) -> PathValidationResult:
        """Replay the root-to-node path and validate each stored snapshot."""
        if node_id not in self.tree.V:
            raise KeyError(f"Vertex {node_id!r} is not registered.")
        if node_id == self.tree.root_id:
            return PathValidationResult(True, node_id, 0)

        chain = []
        current_id = node_id
        while current_id != self.tree.root_id:
            incoming = self.tree.in_edges(current_id)
            if len(incoming) != 1:
                raise RuntimeError("Decision tree contains an invalid parent relation.")
            edge = incoming[0]
            chain.append((edge, edge.src, current_id))
            current_id = edge.src
        chain.reverse()

        validated = 0
        for edge, parent_id, child_id in chain:
            expected = self.tree.get_snapshot(child_id)
            try:
                actual = self.executor.execute(
                    self.tree.get_snapshot(parent_id),
                    self.tree.get_decision(edge),
                )
            except Exception as exc:
                return PathValidationResult(
                    False, node_id, validated, self.tree.get_edge_id(edge),
                    expected_snapshot=expected, error=f"{type(exc).__name__}: {exc}",
                )

            if actual != expected:
                return PathValidationResult(
                    False, node_id, validated, self.tree.get_edge_id(edge),
                    expected_snapshot=expected, actual_snapshot=actual,
                    error="Replayed snapshot does not match the stored snapshot.",
                )
            validated += 1

        return PathValidationResult(True, node_id, validated)
