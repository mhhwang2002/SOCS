"""Exhaustive simulation-based optimization."""

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
    costs: dict[int, float | None]
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


class ExhaustiveOptimizer:
    """Explore every generated Decision branch using breadth-first traversal."""

    def __init__(
        self,
        tree: DecisionTree,
        generator: Callable[[SystemSnapshot], Iterable[Decision]],
        executor,
        objective: Callable[[SystemSnapshot], float | None],
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
        """Expand all generated alternatives and evaluate terminal snapshots."""
        if root_node_id not in self.tree.V:
            raise KeyError(f"Root node {root_node_id!r} is not in the DecisionTree.")
        if root_node_id != self.tree.root_id:
            raise ValueError("root_node_id must be the DecisionTree root.")

        queue = deque([root_node_id])
        leaves: list[int] = []

        while queue:
            node_id = queue.popleft()
            parent_snapshot = self.tree.get_snapshot(node_id)
            decisions = list(self.generator(parent_snapshot))

            for decision in decisions:
                if not isinstance(decision, Decision):
                    raise TypeError("generator must yield Decision instances.")

                child_snapshot = self.executor.execute(parent_snapshot, decision)
                child_node_id = self.tree.add_child(
                    node_id,
                    decision,
                    child_snapshot,
                )

                if child_snapshot == parent_snapshot:
                    leaves.append(child_node_id)
                else:
                    queue.append(child_node_id)

            if not decisions:
                leaves.append(node_id)

        costs = {
            node_id: self.objective(self.tree.get_snapshot(node_id))
            for node_id in leaves
        }
        feasible = [
            (cost, node_id)
            for node_id, cost in costs.items()
            if cost is not None
        ]

        if feasible:
            best_cost = min(cost for cost, _ in feasible)
            optimal_node_ids = tuple(
                node_id
                for cost, node_id in feasible
                if math.isclose(cost, best_cost, rel_tol=1e-9, abs_tol=1e-9)
            )
            best_node_id = optimal_node_ids[0]
        else:
            best_cost = None
            best_node_id = None
            optimal_node_ids = ()

        return OptimizationResult(
            tree=self.tree,
            root_node_id=root_node_id,
            leaf_node_ids=tuple(leaves),
            costs=costs,
            best_node_id=best_node_id,
            best_cost=best_cost,
            optimal_node_ids=optimal_node_ids,
        )