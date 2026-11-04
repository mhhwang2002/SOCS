"""Replay and validate paths stored in a DecisionTree."""

from dataclasses import dataclass
import math
from typing import Any, Callable

from socs.devs.Snapshot import SystemSnapshot
from socs.tree import DecisionTree

from .executor import DecisionExecutor


@dataclass(frozen=True)
class PathValidationResult:
    """Outcome and detailed checks from replaying a DecisionTree path."""

    valid: bool
    node_id: int
    validated_edges: int
    failed_edge_id: int | None = None
    expected_snapshot: SystemSnapshot | None = None
    actual_snapshot: SystemSnapshot | None = None
    error: str | None = None
    pre_decision_checks: int = 0
    post_decision_checks: int = 0
    final_state_matches: bool | None = None
    expected_terminal_status: Any = None
    actual_terminal_status: Any = None
    termination_matches: bool | None = None
    expected_makespan: float | None = None
    actual_makespan: float | None = None
    makespan_matches: bool | None = None


class DecisionPathValidator:
    """Replay a stored path and compare its intermediate and terminal results."""

    def __init__(
        self,
        tree: DecisionTree,
        executor: DecisionExecutor,
        terminal_classifier: Callable[[SystemSnapshot], Any] | None = None,
        cost_evaluator: Callable[[SystemSnapshot], float | None] | None = None,
    ):
        if not isinstance(tree, DecisionTree):
            raise TypeError("tree must be a DecisionTree instance.")
        if not isinstance(executor, DecisionExecutor):
            raise TypeError("executor must be a DecisionExecutor instance.")
        if terminal_classifier is not None and not callable(terminal_classifier):
            raise TypeError("terminal_classifier must be callable or None.")
        if cost_evaluator is not None and not callable(cost_evaluator):
            raise TypeError("cost_evaluator must be callable or None.")

        self.tree = tree
        self.executor = executor
        self.terminal_classifier = terminal_classifier
        self.cost_evaluator = cost_evaluator

    @staticmethod
    def _costs_match(expected: float | None, actual: float | None) -> bool:
        if expected is None or actual is None:
            return expected is None and actual is None
        return math.isclose(expected, actual, rel_tol=1e-9, abs_tol=1e-9)

    def validate(self, node_id: int) -> PathValidationResult:
        """Replay root-to-node and validate continuity, states, and terminal metrics."""
        if node_id not in self.tree.V:
            raise KeyError(f"Vertex {node_id!r} is not registered.")

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

        replay_snapshot = self.tree.get_snapshot(self.tree.root_id)
        pre_checks = 0
        post_checks = 0
        validated = 0

        for edge, parent_id, child_id in chain:
            expected_before = self.tree.get_snapshot(parent_id)
            if replay_snapshot != expected_before:
                return PathValidationResult(
                    False, node_id, validated, self.tree.get_edge_id(edge),
                    expected_snapshot=expected_before,
                    actual_snapshot=replay_snapshot,
                    error="Snapshot immediately before Decision does not match its parent Node.",
                    pre_decision_checks=pre_checks,
                    post_decision_checks=post_checks,
                    final_state_matches=False,
                )
            pre_checks += 1

            try:
                actual_after = self.executor.execute(
                    replay_snapshot,
                    self.tree.get_decision(edge),
                )
            except Exception as exc:
                return PathValidationResult(
                    False, node_id, validated, self.tree.get_edge_id(edge),
                    expected_snapshot=self.tree.get_snapshot(child_id),
                    error=f"{type(exc).__name__}: {exc}",
                    pre_decision_checks=pre_checks,
                    post_decision_checks=post_checks,
                )

            expected_after = self.tree.get_snapshot(child_id)
            if actual_after != expected_after:
                return PathValidationResult(
                    False, node_id, validated, self.tree.get_edge_id(edge),
                    expected_snapshot=expected_after,
                    actual_snapshot=actual_after,
                    error="Snapshot immediately after Decision does not match its child Node.",
                    pre_decision_checks=pre_checks,
                    post_decision_checks=post_checks,
                    final_state_matches=False,
                )

            post_checks += 1
            validated += 1
            replay_snapshot = actual_after

        expected_final = self.tree.get_snapshot(node_id)
        final_matches = replay_snapshot == expected_final
        expected_status = actual_status = None
        termination_matches = None
        expected_cost = actual_cost = None
        makespan_matches = None

        if self.terminal_classifier is not None:
            expected_status = self.terminal_classifier(expected_final)
            actual_status = self.terminal_classifier(replay_snapshot)
            termination_matches = expected_status == actual_status

        if self.cost_evaluator is not None:
            expected_cost = self.cost_evaluator(expected_final)
            actual_cost = self.cost_evaluator(replay_snapshot)
            makespan_matches = self._costs_match(expected_cost, actual_cost)

        valid = (
            final_matches
            and termination_matches is not False
            and makespan_matches is not False
        )
        return PathValidationResult(
            valid=valid,
            node_id=node_id,
            validated_edges=validated,
            expected_snapshot=expected_final,
            actual_snapshot=replay_snapshot,
            error=None if valid else "Final state, termination status, or makespan mismatch.",
            pre_decision_checks=pre_checks,
            post_decision_checks=post_checks,
            final_state_matches=final_matches,
            expected_terminal_status=expected_status,
            actual_terminal_status=actual_status,
            termination_matches=termination_matches,
            expected_makespan=expected_cost,
            actual_makespan=actual_cost,
            makespan_matches=makespan_matches,
        )
