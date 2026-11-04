"""Exhaustive breadth-first search over a DecisionTree."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Callable, Hashable

from .expansion import DecisionExpansionController, ExpansionStatus
from ..devs.Snapshot import SystemSnapshot
from ..tree.decision_tree import DecisionTree


@dataclass(frozen=True)
class ExhaustiveSearchResult:
    """Summary of an exhaustive decision-tree search."""

    tree: DecisionTree
    root_node_id: Hashable
    leaf_node_ids: tuple[Hashable, ...]
    costs: dict[Hashable, float | None]
    best_node_id: Hashable | None
    best_cost: float | None
    optimal_node_ids: tuple[Hashable, ...]

    @property
    def node_count(self) -> int:
        """Return the number of Snapshot nodes in the generated tree."""
        return len(self.tree.V)

    @property
    def leaf_count(self) -> int:
        """Return the number of terminal paths evaluated."""
        return len(self.leaf_node_ids)

    def best_path(self):
        """Return the Decision sequence for the first optimal path."""
        if self.best_node_id is None:
            return []
        return self.tree.get_path(self.best_node_id)

    def optimal_paths(self):
        """Return every Decision sequence with the minimum evaluated cost."""
        return [
            self.tree.get_path(node_id)
            for node_id in self.optimal_node_ids
        ]


class ExhaustiveDecisionTreeSearch:
    """Expand every generated Decision using breadth-first traversal."""

    def __init__(
        self,
        expansion_controller: DecisionExpansionController,
        evaluator: Callable[[SystemSnapshot], float | None],
    ) -> None:
        if not isinstance(expansion_controller, DecisionExpansionController):
            raise TypeError(
                "expansion_controller must be a DecisionExpansionController."
            )
        if not callable(evaluator):
            raise TypeError("evaluator must be callable.")

        self.expansion_controller = expansion_controller
        self.evaluator = evaluator

    def run(self, root_node_id: Hashable) -> ExhaustiveSearchResult:
        """Expand the complete tree and evaluate every terminal leaf."""
        tree = self.expansion_controller.tree
        queue = deque([root_node_id])
        leaves: list[Hashable] = []

        while queue:
            node_id = queue.popleft()
            results = self.expansion_controller.expand_all(node_id)

            if len(results) == 1 and results[0].status is ExpansionStatus.NO_DECISION:
                leaves.append(node_id)
                continue

            for result in results:
                if result.status is ExpansionStatus.FAILED:
                    raise RuntimeError(
                        f"Decision expansion failed at node {node_id!r}: "
                        f"{result.error}"
                    )
                if result.status is ExpansionStatus.EXECUTED:
                    child_id = result.child_node_id
                    parent_snapshot = tree.get_snapshot(node_id)
                    child_snapshot = tree.get_snapshot(child_id)

                    # A no-op decision that cannot advance simulated time is
                    # a terminal branch (for example, Wait at a deadlock).
                    if child_snapshot == parent_snapshot:
                        leaves.append(child_id)
                    else:
                        queue.append(child_id)

        costs = {
            node_id: self.evaluator(tree.get_snapshot(node_id))
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

        return ExhaustiveSearchResult(
            tree=tree,
            root_node_id=root_node_id,
            leaf_node_ids=tuple(leaves),
            costs=costs,
            best_node_id=best_node_id,
            best_cost=best_cost,
            optimal_node_ids=optimal_node_ids,
        )
