"""Replay and validate explicit paths stored in a DecisionGraph."""

from collections.abc import Iterable
from dataclasses import dataclass
import math
from typing import Any, Callable

from socs.devs.Snapshot import SystemSnapshot
from socs.graph.decision_graph import DecisionGraph, Edge


@dataclass(frozen=True)
class PathValidationResult:
    """Outcome and detailed checks from replaying a DecisionGraph path."""

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
    expected_cost: float | None = None
    actual_cost: float | None = None
    cost_matches: bool | None = None

    @property
    def expected_makespan(self) -> float | None:
        """Backward-compatible alias for expected_cost."""
        return self.expected_cost

    @property
    def actual_makespan(self) -> float | None:
        """Backward-compatible alias for actual_cost."""
        return self.actual_cost

    @property
    def makespan_matches(self) -> bool | None:
        """Backward-compatible alias for cost_matches."""
        return self.cost_matches


class DecisionPathValidator:
    """Replay an explicit edge-ID path and compare its states and metrics."""

    def __init__(
        self,
        graph: DecisionGraph,
        executor,
        terminal_classifier: Callable[[SystemSnapshot], Any] | None = None,
        cost_evaluator: Callable[[SystemSnapshot], float | None] | None = None,
    ):
        if not isinstance(graph, DecisionGraph):
            raise TypeError("graph must be a DecisionGraph instance.")
        if not callable(getattr(executor, "execute", None)):
            raise TypeError("executor must provide an execute(state, decision) method.")
        if terminal_classifier is not None and not callable(terminal_classifier):
            raise TypeError("terminal_classifier must be callable or None.")
        if cost_evaluator is not None and not callable(cost_evaluator):
            raise TypeError("cost_evaluator must be callable or None.")

        self.graph = graph
        self.executor = executor
        self.terminal_classifier = terminal_classifier
        self.cost_evaluator = cost_evaluator

    @staticmethod
    def _costs_match(expected: float | None, actual: float | None) -> bool:
        if expected is None or actual is None:
            return expected is None and actual is None
        return math.isclose(expected, actual, rel_tol=1e-9, abs_tol=1e-9)

    def _edge_index(self) -> dict[int, Edge]:
        """Build an edge-ID lookup and reject ambiguous IDs."""
        index: dict[int, Edge] = {}
        for edge in self.graph.edges():
            edge_id = self.graph.get_edge_id(edge)
            if edge_id in index:
                raise ValueError(f"Duplicate edge ID {edge_id!r} in DecisionGraph.")
            index[edge_id] = edge
        return index

    def validate(self, edge_ids: Iterable[int]) -> PathValidationResult:
        """Replay an ordered edge-ID path starting at the graph root.

        An empty sequence represents the zero-edge path at the root.
        """
        if isinstance(edge_ids, (str, bytes)):
            raise TypeError("edge_ids must be an iterable of integer edge IDs.")

        try:
            requested_ids = tuple(edge_ids)
        except TypeError:
            raise TypeError(
                "edge_ids must be an iterable of integer edge IDs."
            ) from None

        if any(
            not isinstance(edge_id, int) or isinstance(edge_id, bool)
            for edge_id in requested_ids
        ):
            raise TypeError("Every edge ID must be an integer.")

        if self.graph.root_id not in self.graph.V:
            raise RuntimeError("DecisionGraph does not have a registered root.")

        edge_index = self._edge_index()
        current_id = self.graph.root_id
        selected_edges: list[tuple[int, Edge]] = []

        for edge_id in requested_ids:
            edge = edge_index.get(edge_id)
            if edge is None:
                raise KeyError(f"Edge ID {edge_id!r} is not registered.")
            if edge.src != current_id:
                raise ValueError(
                    f"Edge ID {edge_id!r} starts at node {edge.src!r}, "
                    f"but the current path node is {current_id!r}."
                )
            selected_edges.append((edge_id, edge))
            current_id = edge.dst

        node_id = current_id
        replay_snapshot = self.graph.get_snapshot(self.graph.root_id)
        pre_checks = 0
        post_checks = 0
        validated = 0

        for edge_id, edge in selected_edges:
            expected_before = self.graph.get_snapshot(edge.src)
            if replay_snapshot != expected_before:
                return PathValidationResult(
                    False, node_id, validated, edge_id,
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
                    self.graph.get_decision(edge),
                )
            except Exception as exc:
                return PathValidationResult(
                    False, node_id, validated, edge_id,
                    expected_snapshot=self.graph.get_snapshot(edge.dst),
                    error=f"{type(exc).__name__}: {exc}",
                    pre_decision_checks=pre_checks,
                    post_decision_checks=post_checks,
                )

            expected_after = self.graph.get_snapshot(edge.dst)
            if actual_after != expected_after:
                return PathValidationResult(
                    False, node_id, validated, edge_id,
                    expected_snapshot=expected_after,
                    actual_snapshot=actual_after,
                    error="Snapshot immediately after Decision does not match its destination Node.",
                    pre_decision_checks=pre_checks,
                    post_decision_checks=post_checks,
                    final_state_matches=False,
                )

            post_checks += 1
            validated += 1
            replay_snapshot = actual_after

        expected_final = self.graph.get_snapshot(node_id)
        final_matches = replay_snapshot == expected_final
        expected_status = actual_status = None
        termination_matches = None
        expected_cost = actual_cost = None
        cost_matches = None

        if self.terminal_classifier is not None:
            expected_status = self.terminal_classifier(expected_final)
            actual_status = self.terminal_classifier(replay_snapshot)
            termination_matches = expected_status == actual_status

        if self.cost_evaluator is not None:
            expected_cost = self.cost_evaluator(expected_final)
            actual_cost = self.cost_evaluator(replay_snapshot)
            cost_matches = self._costs_match(expected_cost, actual_cost)

        valid = (
            final_matches
            and termination_matches is not False
            and cost_matches is not False
        )

        return PathValidationResult(
            valid=valid,
            node_id=node_id,
            validated_edges=validated,
            expected_snapshot=expected_final,
            actual_snapshot=replay_snapshot,
            error=None if valid else "Final state, termination status, or cost mismatch.",
            pre_decision_checks=pre_checks,
            post_decision_checks=post_checks,
            final_state_matches=final_matches,
            expected_terminal_status=expected_status,
            actual_terminal_status=actual_status,
            termination_matches=termination_matches,
            expected_cost=expected_cost,
            actual_cost=actual_cost,
            cost_matches=cost_matches,
        )
