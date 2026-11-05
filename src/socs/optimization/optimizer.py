"""Simulation-based optimization over a DecisionGraph."""

from collections import deque
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass
from enum import Enum
import math

from socs.decision.decision import Decision
from socs.devs.Snapshot import SystemSnapshot
from socs.graph.decision_graph import DecisionGraph


@dataclass(frozen=True)
class TerminalBranch:
    """Represent one terminal branch occurrence at a graph node."""

    branch_id: int
    node_id: int
    kind: "TerminalBranchKind"
    decision: Decision | None = None


class TerminalBranchKind(Enum):
    """Describe why an optimizer branch terminates at its node."""

    EMPTY_GENERATOR = "empty_generator"
    NO_OP_DECISION = "no_op_decision"


@dataclass(frozen=True)
class TerminalOutcome:
    """Represent one terminal execution path and its objective cost."""

    edge_ids: tuple[int, ...]
    terminal_branch_id: int
    cost: float = math.inf


@dataclass(frozen=True)
class OptimizationResult:
    """Store terminal outcomes and the best outcomes of an optimization run."""

    graph: DecisionGraph
    root_node_id: int
    terminal_branches: tuple[TerminalBranch, ...]
    terminal_outcomes: tuple[TerminalOutcome, ...]
    best_cost: float | None
    optimal_outcomes: tuple[TerminalOutcome, ...]

    @property
    def node_count(self) -> int:
        return len(self.graph.V)

    @property
    def outcome_count(self) -> int:
        return len(self.terminal_outcomes)

    def optimal_paths(self) -> list[list[Decision]]:
        """Return the decisions belonging to each optimal outcome."""
        edges_by_id = {
            self.graph.get_edge_id(edge): edge
            for edge in self.graph.edges()
        }
        branches = {branch.branch_id: branch for branch in self.terminal_branches}
        paths = []
        for outcome in self.optimal_outcomes:
            path = [
                self.graph.get_decision(edges_by_id[edge_id])
                for edge_id in outcome.edge_ids
            ]
            branch = branches[outcome.terminal_branch_id]
            if branch.kind is TerminalBranchKind.NO_OP_DECISION:
                path.append(branch.decision)
            paths.append(path)
        return paths

    def get_terminal_branch(self, branch_id: int) -> TerminalBranch:
        """Return a terminal branch by its stable occurrence ID."""
        for branch in self.terminal_branches:
            if branch.branch_id == branch_id:
                return branch
        raise KeyError(f"Terminal branch ID {branch_id!r} is not registered.")

    def terminal_node_id(self, outcome: TerminalOutcome) -> int:
        """Return the node at which an outcome terminates."""
        if not outcome.edge_ids:
            return self.root_node_id

        edges_by_id = {
            self.graph.get_edge_id(edge): edge
            for edge in self.graph.edges()
        }
        return edges_by_id[outcome.edge_ids[-1]].dst


class Optimizer:
    """Explore generated Decision branches and evaluate terminal outcomes."""

    def __init__(
        self,
        graph: DecisionGraph,
        generator: Callable[[SystemSnapshot], Iterable[Decision]],
        executor,
        objective: Callable[[SystemSnapshot], float],
    ) -> None:
        if not isinstance(graph, DecisionGraph):
            raise TypeError("graph must be a DecisionGraph instance.")
        if not callable(generator):
            raise TypeError("generator must be callable.")
        if not callable(getattr(executor, "execute", None)):
            raise TypeError("executor must provide an execute(state, decision) method.")
        if not callable(objective):
            raise TypeError("objective must be callable.")

        self.graph = graph
        self.generator = generator
        self.executor = executor
        self.objective = objective
        self._terminal_branches: list[TerminalBranch] = []
        self._next_terminal_branch_id = 1
        self._built = False

    def run(self, root_node_id: int) -> OptimizationResult:
        """Build the solution graph and evaluate its terminal outcomes."""
        if root_node_id not in self.graph.V:
            raise KeyError(f"Root node {root_node_id!r} is not in the DecisionGraph.")
        if root_node_id != self.graph.root_id:
            raise ValueError("root_node_id must be the DecisionGraph root.")
        if self._built:
            raise RuntimeError("This Optimizer has already built its solution graph.")

        self.build_solution_graph(root_node_id)
        self._built = True
        return self.get_optimization_result(root_node_id)

    def build_solution_graph(self, root_node_id: int) -> None:
        """Expand each unique graph node at most once using breadth-first search."""
        queue = deque([root_node_id])
        scheduled = {root_node_id}

        while queue:
            node_id = queue.popleft()
            snapshot = self.graph.get_snapshot(node_id)
            decisions = list(self.generator(snapshot))

            if not decisions:
                self._record_terminal_branch(
                    node_id, TerminalBranchKind.EMPTY_GENERATOR
                )
                continue

            for decision in decisions:
                if not isinstance(decision, Decision):
                    raise TypeError("generator must yield Decision instances.")

                child_snapshot = self.executor.execute(snapshot, decision)
                edge = self.graph.add_transition(node_id, decision, child_snapshot)

                if edge is None:
                    self._record_terminal_branch(
                        node_id, TerminalBranchKind.NO_OP_DECISION, decision
                    )
                    continue

                child_node_id = edge.dst
                if child_node_id not in scheduled:
                    scheduled.add(child_node_id)
                    queue.append(child_node_id)

    def _record_terminal_branch(
        self,
        node_id: int,
        kind: TerminalBranchKind,
        decision: Decision | None = None,
    ) -> None:
        self._terminal_branches.append(
            TerminalBranch(
                branch_id=self._next_terminal_branch_id,
                node_id=node_id,
                kind=kind,
                decision=decision,
            )
        )
        self._next_terminal_branch_id += 1

    def _iter_paths_to(self, target_node_id: int) -> Iterator[tuple[int, ...]]:
        """Yield simple root-to-node paths as edge ID sequences."""
        def visit(
            node_id: int,
            edge_ids: tuple[int, ...],
            visited: frozenset[int],
        ) -> Iterator[tuple[int, ...]]:
            if node_id == target_node_id:
                yield edge_ids
                return

            for edge in self.graph.out_edges(node_id):
                if edge.dst in visited:
                    continue
                yield from visit(
                    edge.dst,
                    edge_ids + (self.graph.get_edge_id(edge),),
                    visited | {edge.dst},
                )

        yield from visit(self.graph.root_id, (), frozenset({self.graph.root_id}))

    def get_optimization_result(self, root_node_id: int) -> OptimizationResult:
        """Evaluate each terminal execution path and select the best outcomes."""
        outcomes = []
        for branch in self._terminal_branches:
            snapshot = self.graph.get_snapshot(branch.node_id)
            for edge_ids in self._iter_paths_to(branch.node_id):
                cost = self.objective(snapshot)
                if not isinstance(cost, (int, float)):
                    raise TypeError("objective must return a numeric value.")
                if math.isnan(cost) or cost == -math.inf:
                    raise ValueError(
                        "objective must return a valid cost or positive infinity."
                    )
                outcomes.append(
                    TerminalOutcome(
                        edge_ids=edge_ids,
                        terminal_branch_id=branch.branch_id,
                        cost=float(cost),
                    )
                )

        feasible = [outcome for outcome in outcomes if outcome.cost != math.inf]
        if feasible:
            best_cost = min(outcome.cost for outcome in feasible)
            optimal_outcomes = tuple(
                outcome
                for outcome in feasible
                if math.isclose(
                    outcome.cost,
                    best_cost,
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                )
            )
        else:
            best_cost = None
            optimal_outcomes = ()

        return OptimizationResult(
            graph=self.graph,
            root_node_id=root_node_id,
            terminal_branches=tuple(self._terminal_branches),
            terminal_outcomes=tuple(outcomes),
            best_cost=best_cost,
            optimal_outcomes=optimal_outcomes,
        )
