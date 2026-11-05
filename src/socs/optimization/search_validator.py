"""Validation for DecisionTree optimization results."""

from dataclasses import dataclass
import math
from typing import Hashable

from socs.tree.validators import DecisionTreeValidator

from .optimizer import OptimizationResult


@dataclass(frozen=True)
class SearchResultIssue:
    """Describe one inconsistency found in an optimization search result."""

    code: str
    message: str
    node_id: Hashable | None = None


@dataclass(frozen=True)
class SearchResultValidationResult:
    """Summary of consistency checks for an optimization search result."""

    valid: bool
    checks: tuple[tuple[str, bool], ...]
    issues: tuple[SearchResultIssue, ...]
    tree_node_count: int
    tree_leaf_count: int
    reported_leaf_count: int
    feasible_leaf_count: int
    best_cost: float | None

    @property
    def errors(self) -> tuple[SearchResultIssue, ...]:
        """Return all validation issues."""
        return self.issues

    def format(self) -> str:
        """Format a readable search-result validation report."""
        lines = [
            "Search Result Validation",
            "",
            f"Tree nodes: {self.tree_node_count}",
            f"Tree leaves: {self.tree_leaf_count}",
            f"Reported leaves: {self.reported_leaf_count}",
            f"Feasible leaves: {self.feasible_leaf_count}",
            f"Best cost: {self.best_cost}",
            "",
            "Checks:",
        ]
        lines.extend(
            f"  {name}: {'PASS' if passed else 'FAIL'}"
            for name, passed in self.checks
        )
        if self.issues:
            lines.extend(["", "Issues:"])
            for issue in self.issues:
                location = (
                    f" (Node {issue.node_id})" if issue.node_id is not None else ""
                )
                lines.append(f"  ERROR [{issue.code}]{location}: {issue.message}")
        lines.extend(["", f"Result: {'VALID' if self.valid else 'INVALID'}"])
        return "\n".join(lines)


class SearchResultValidator:
    """Validate the consistency of an optimization result.

    The validator is read-only. It checks that the result's terminal-node
    inventory, cost mapping, and optimum metadata agree with the DecisionTree.
    It does not rerun the simulation or the search algorithm.
    """

    def __init__(self, result: OptimizationResult):
        if not isinstance(result, OptimizationResult):
            raise TypeError("result must be an OptimizationResult instance.")
        self.result = result

    def validate(self) -> SearchResultValidationResult:
        """Run all consistency checks and return a detailed report."""
        result = self.result
        tree = result.tree
        issues: list[SearchResultIssue] = []
        checks: dict[str, bool] = {}

        def fail(code: str, message: str, node_id=None) -> None:
            issues.append(SearchResultIssue(code, message, node_id))

        tree_validation = DecisionTreeValidator(tree).validate()
        checks["DecisionTree structure"] = tree_validation.valid
        if not tree_validation.valid:
            fail(
                "INVALID_TREE",
                "The DecisionTree failed structural validation; inspect its validator report.",
            )

        node_ids = set(tree.V)
        actual_leaves = {
            node_id for node_id in node_ids if not tree.get_children(node_id)
        }
        reported_leaves = list(result.leaf_node_ids)
        reported_leaf_set = set(reported_leaves)

        checks["Root node"] = (
            result.root_node_id == tree.root_id
            and result.root_node_id in node_ids
        )
        if not checks["Root node"]:
            fail(
                "ROOT_MISMATCH",
                f"Result root {result.root_node_id!r} does not match tree root {tree.root_id!r}.",
            )

        checks["Terminal-node uniqueness"] = (
            len(reported_leaves) == len(reported_leaf_set)
        )
        if not checks["Terminal-node uniqueness"]:
            fail("DUPLICATE_LEAF", "The result contains duplicate terminal node IDs.")

        unknown_leaves = reported_leaf_set - node_ids
        checks["Terminal-node references"] = not unknown_leaves
        for node_id in sorted(unknown_leaves, key=repr):
            fail(
                "UNKNOWN_LEAF",
                "A reported terminal node does not exist in the tree.",
                node_id,
            )

        nonterminal_reports = reported_leaf_set & {
            node_id for node_id in node_ids if tree.get_children(node_id)
        }
        checks["Terminal-node status"] = not nonterminal_reports
        for node_id in sorted(nonterminal_reports, key=repr):
            fail(
                "NONTERMINAL_REPORTED_AS_LEAF",
                "A reported terminal node has children.",
                node_id,
            )

        missing_leaves = actual_leaves - reported_leaf_set
        extra_leaves = reported_leaf_set - actual_leaves
        checks["Complete leaf coverage"] = not missing_leaves and not extra_leaves
        for node_id in sorted(missing_leaves, key=repr):
            fail(
                "MISSING_TERMINAL",
                "A structural tree leaf is missing from the search result.",
                node_id,
            )
        for node_id in sorted(extra_leaves, key=repr):
            if node_id in node_ids:
                fail(
                    "EXTRA_TERMINAL",
                    "A reported terminal node is not a structural leaf.",
                    node_id,
                )

        cost_keys = set(result.costs)
        checks["Cost coverage"] = cost_keys == reported_leaf_set
        for node_id in sorted(reported_leaf_set - cost_keys, key=repr):
            fail(
                "MISSING_COST",
                "No objective cost was recorded for this terminal node.",
                node_id,
            )
        for node_id in sorted(cost_keys - reported_leaf_set, key=repr):
            fail(
                "EXTRA_COST",
                "A cost is recorded for a node not listed as terminal.",
                node_id,
            )

        costs_valid = True
        for node_id, cost in result.costs.items():
            if (
                isinstance(cost, bool)
                or not isinstance(cost, (int, float))
                or math.isnan(cost)
                or cost == -math.inf
            ):
                costs_valid = False
                fail(
                    "INVALID_COST",
                    f"Cost must be a finite number or positive infinity; received {cost!r}.",
                    node_id,
                )
        checks["Cost values"] = costs_valid

        feasible = [
            (float(result.costs[node_id]), node_id)
            for node_id in result.leaf_node_ids
            if node_id in result.costs
            and not isinstance(result.costs[node_id], bool)
            and isinstance(result.costs[node_id], (int, float))
            and math.isfinite(result.costs[node_id])
        ]

        if feasible:
            expected_best_cost = min(cost for cost, _ in feasible)
            expected_optimal_ids = tuple(
                node_id
                for cost, node_id in feasible
                if math.isclose(
                    cost,
                    expected_best_cost,
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                )
            )
        else:
            expected_best_cost = None
            expected_optimal_ids = ()

        best_cost_matches = (
            result.best_cost is None and expected_best_cost is None
        ) or (
            result.best_cost is not None
            and expected_best_cost is not None
            and isinstance(result.best_cost, (int, float))
            and not isinstance(result.best_cost, bool)
            and math.isfinite(result.best_cost)
            and math.isclose(
                result.best_cost,
                expected_best_cost,
                rel_tol=1e-9,
                abs_tol=1e-9,
            )
        )
        checks["Best objective value"] = best_cost_matches
        if not best_cost_matches:
            fail(
                "BEST_COST_MISMATCH",
                f"Reported best cost {result.best_cost!r}; expected {expected_best_cost!r}.",
            )

        checks["Optimal-node set and order"] = (
            tuple(result.optimal_node_ids) == expected_optimal_ids
        )
        if not checks["Optimal-node set and order"]:
            fail(
                "OPTIMAL_NODES_MISMATCH",
                f"Reported optimal nodes {tuple(result.optimal_node_ids)!r}; expected {expected_optimal_ids!r}.",
            )

        checks["Reported counts"] = (
            result.node_count == len(node_ids)
            and result.leaf_count == len(reported_leaves)
        )
        if not checks["Reported counts"]:
            fail("COUNT_MISMATCH", "Search result count properties are inconsistent.")

        return SearchResultValidationResult(
            valid=all(checks.values()) and not issues,
            checks=tuple(checks.items()),
            issues=tuple(issues),
            tree_node_count=len(node_ids),
            tree_leaf_count=len(actual_leaves),
            reported_leaf_count=len(reported_leaves),
            feasible_leaf_count=len(feasible),
            best_cost=result.best_cost,
        )
