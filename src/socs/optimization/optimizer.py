"""Simulation-based optimization."""

from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass
import math

from socs.decision.decision import Decision
from socs.devs.Snapshot import SystemSnapshot
from socs.tree.decision_tree import DecisionTree


@dataclass(frozen=True)
class OptimizationResult:
    """Summary of an optimization run."""

    tree: DecisionTree
    root_node_id: int
    leaf_node_ids: tuple[int, ...]
    costs: dict[int, float]
    best_node_id: int | None
    best_cost: float | None
    optimal_node_ids: tuple[int, ...]

    @property
    def node_count(self) -> int:
        return len(self.tree.V)

    @property
    def leaf_count(self) -> int:
        return len(self.leaf_node_ids)

    def best_path(self) -> list[Decision]:
        if self.best_node_id is None:
            return []
        return self.tree.get_path(self.best_node_id)

    def optimal_paths(self) -> list[list[Decision]]:
        return [self.tree.get_path(node_id) for node_id in self.optimal_node_ids]


class Optimizer:
    """Explore generated Decision branches and evaluate terminal states."""

    def __init__(
        self,
        tree: DecisionTree,
        generator: Callable[[SystemSnapshot], Iterable[Decision]],
        executor,
        objective: Callable[[SystemSnapshot], float],
    ) -> None:
        if not isinstance(tree, DecisionTree):
            raise TypeError("tree must be a DecisionTree instance.")
        if not callable(generator):
            raise TypeError("generator must be callable.")
        if not callable(getattr(executor, "execute", None)):
            raise TypeError("executor must provide an execute(state, decision) method.")
        if not callable(objective):
            raise TypeError("objective must be callable.")

        self.tree = tree
        self.generator = generator
        self.executor = executor
        self.objective = objective

    def run(self, root_node_id: int) -> OptimizationResult:
        """Build the solution tree and evaluate its terminal states."""

        if root_node_id not in self.tree.V:
            raise KeyError(f"Root node {root_node_id!r} is not in the DecisionTree.")
        if root_node_id != self.tree.root_id:
            raise ValueError("root_node_id must be the DecisionTree root.")

        self.build_solution_tree(root_node_id)

        return self.get_optimization_result(root_node_id)

    def build_solution_tree(self, root_node_id: int) -> None:
        """Expand all generated Decision branches using breadth-first search."""

        queue = deque([root_node_id])

        while queue:
            node_id = queue.popleft()
            parent_snapshot = self.tree.get_snapshot(node_id)
            decisions = self.generator(parent_snapshot)

            for decision in decisions:
                if not isinstance(decision, Decision):
                    raise TypeError("generator must yield Decision instances.")

                child_snapshot = self.executor.execute(parent_snapshot, decision)
                child_node_id = self.tree.add_child(
                    node_id,
                    decision,
                    child_snapshot,
                )

                if child_snapshot != parent_snapshot:
                    queue.append(child_node_id)

    def get_optimization_result(
        self,
        root_node_id: int,
    ) -> OptimizationResult:
        """Evaluate terminal snapshots and select the optimal paths."""

        leaf_node_ids = self.tree.get_leaves()

        costs = {}
        for node_id in leaf_node_ids:
            cost = self.objective(self.tree.get_snapshot(node_id))

            if not isinstance(cost, (int, float)):
                raise TypeError("objective must return a numeric value.")

            if math.isnan(cost) or cost == -math.inf:
                raise ValueError("objective must return a valid cost or positive infinity.")

            costs[node_id] = cost

        feasible = [
            (cost, node_id)
            for node_id, cost in costs.items()
            if cost != math.inf
        ]

        if feasible:
            best_cost = min(cost for cost, _ in feasible)

            optimal_node_ids = tuple(
                node_id
                for cost, node_id in feasible
                if math.isclose(
                    cost,
                    best_cost,
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                )
            )

            best_node_id = optimal_node_ids[0]
        else:
            best_cost = None
            best_node_id = None
            optimal_node_ids = ()

        return OptimizationResult(
            tree=self.tree,
            root_node_id=root_node_id,
            leaf_node_ids=tuple(leaf_node_ids),
            costs=costs,
            best_node_id=best_node_id,
            best_cost=best_cost,
            optimal_node_ids=optimal_node_ids,
        )