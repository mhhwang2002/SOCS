"""Validation for simulation-based optimization results."""

from dataclasses import dataclass
import math

from socs.decision.decision import Decision
from socs.graph.decision_graph import DecisionGraph, Edge
from socs.graph.validators import DecisionGraphValidator

from .optimizer import (
    OptimizationResult,
    TerminalBranch,
    TerminalBranchKind,
    TerminalOutcome,
)


@dataclass(frozen=True)
class SearchResultIssue:
    """Describe one inconsistency found in an optimization result."""

    code: str
    message: str
    outcome_index: int | None = None
    edge_ids: tuple[int, ...] | None = None
    node_id: int | None = None


@dataclass(frozen=True)
class SearchResultValidationResult:
    """Summary of consistency checks for an optimization result."""

    valid: bool
    checks: tuple[tuple[str, bool], ...]
    issues: tuple[SearchResultIssue, ...]
    graph_node_count: int
    graph_edge_count: int
    terminal_outcome_count: int
    feasible_outcome_count: int
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
            f"Graph nodes: {self.graph_node_count}",
            f"Graph edges: {self.graph_edge_count}",
            f"Terminal outcomes: {self.terminal_outcome_count}",
            f"Feasible outcomes: {self.feasible_outcome_count}",
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
                location = ""
                if issue.outcome_index is not None:
                    location += f" (Outcome {issue.outcome_index})"
                if issue.edge_ids is not None:
                    location += f" (Edges {issue.edge_ids})"
                if issue.node_id is not None:
                    location += f" (Node {issue.node_id})"
                lines.append(f"  ERROR [{issue.code}]{location}: {issue.message}")
        lines.extend(["", f"Result: {'VALID' if self.valid else 'INVALID'}"])
        return "\\n".join(lines)


class SearchResultValidator:
    """Validate graph paths, terminal outcomes, costs, and optimum metadata.

    This validator is read-only. It checks internal consistency and graph-path
    coverage; it does not rerun the generator, executor, objective, or search.
    """

    def __init__(self, result: OptimizationResult):
        if not isinstance(result, OptimizationResult):
            raise TypeError("result must be an OptimizationResult instance.")
        self.result = result

    @staticmethod
    def _is_valid_cost(cost) -> bool:
        return (
            not isinstance(cost, bool)
            and isinstance(cost, (int, float))
            and not math.isnan(cost)
            and cost != -math.inf
        )

    @staticmethod
    def _outcome_key(outcome: TerminalOutcome):
        return outcome.edge_ids, outcome.terminal_branch_id

    def _edge_index(self, graph: DecisionGraph) -> dict[int, Edge]:
        index = {}
        for edges in graph.E.values():
            for edge in edges:
                edge_id = graph.get_edge_id(edge)
                if edge_id in index:
                    raise ValueError(f"Duplicate edge ID {edge_id!r}.")
                index[edge_id] = edge
        return index

    @staticmethod
    def _simple_paths_to(graph: DecisionGraph, target_id: int):
        """Yield simple root-to-target paths as edge-ID tuples."""
        def visit(node_id, edge_ids, visited):
            if node_id == target_id:
                yield edge_ids
                return
            for edge in graph.out_edges(node_id):
                if edge.dst not in visited:
                    yield from visit(
                        edge.dst,
                        edge_ids + (graph.get_edge_id(edge),),
                        visited | {edge.dst},
                    )

        yield from visit(graph.root_id, (), frozenset({graph.root_id}))

    def validate(self) -> SearchResultValidationResult:
        """Run all consistency checks and return a detailed report."""
        result = self.result
        graph = result.graph
        issues: list[SearchResultIssue] = []
        checks: dict[str, bool] = {}

        def fail(code, message, outcome_index=None, edge_ids=None, node_id=None):
            issues.append(
                SearchResultIssue(code, message, outcome_index, edge_ids, node_id)
            )

        graph_valid = isinstance(graph, DecisionGraph)
        if graph_valid:
            graph_validation = DecisionGraphValidator(graph).validate()
            graph_valid = graph_validation.valid
            if not graph_valid:
                fail(
                    "INVALID_GRAPH",
                    "The DecisionGraph failed structural validation; inspect its validator report.",
                )
        checks["DecisionGraph structure"] = graph_valid

        if not isinstance(graph, DecisionGraph):
            return SearchResultValidationResult(
                valid=False,
                checks=tuple(checks.items()),
                issues=tuple(issues),
                graph_node_count=0,
                graph_edge_count=0,
                terminal_outcome_count=len(result.terminal_outcomes),
                feasible_outcome_count=0,
                best_cost=result.best_cost,
            )

        node_ids = set(graph.V)
        edge_index = {}
        try:
            edge_index = self._edge_index(graph)
            edge_index_valid = True
        except Exception as exc:
            edge_index_valid = False
            fail("EDGE_INDEX_INVALID", f"Could not index graph edges: {exc}")
        checks["Edge ID index"] = edge_index_valid

        root_matches = (
            result.root_node_id == graph.root_id
            and result.root_node_id in node_ids
        )
        checks["Root node"] = root_matches
        if not root_matches:
            fail(
                "ROOT_MISMATCH",
                f"Result root {result.root_node_id!r} does not match graph root {graph.root_id!r}.",
            )

        branches = tuple(result.terminal_branches)
        branch_index = {}
        branches_valid = True
        for index, branch in enumerate(branches):
            if not isinstance(branch, TerminalBranch):
                branches_valid = False
                fail("INVALID_TERMINAL_BRANCH", "Entry is not a TerminalBranch.", index)
                continue
            if (
                not isinstance(branch.branch_id, int)
                or isinstance(branch.branch_id, bool)
                or branch.branch_id <= 0
                or branch.branch_id in branch_index
            ):
                branches_valid = False
                fail(
                    "INVALID_TERMINAL_BRANCH_ID",
                    f"Terminal branch ID {branch.branch_id!r} is invalid or duplicated.",
                    index,
                    node_id=branch.node_id,
                )
                continue
            branch_index[branch.branch_id] = branch
            if branch.node_id not in node_ids:
                branches_valid = False
                fail(
                    "UNKNOWN_TERMINAL_BRANCH_NODE",
                    f"Terminal branch node {branch.node_id!r} is not registered.",
                    index,
                    node_id=branch.node_id,
                )
                continue
            if branch.kind is TerminalBranchKind.EMPTY_GENERATOR:
                if branch.decision is not None or graph.out_edges(branch.node_id):
                    branches_valid = False
                    fail(
                        "INVALID_EMPTY_GENERATOR_BRANCH",
                        "An empty-generator branch must have no decision and its node must have no outgoing edges.",
                        index,
                        node_id=branch.node_id,
                    )
            elif branch.kind is TerminalBranchKind.NO_OP_DECISION:
                if not isinstance(branch.decision, Decision):
                    branches_valid = False
                    fail(
                        "INVALID_NO_OP_BRANCH",
                        "A no-op branch must store a Decision instance.",
                        index,
                        node_id=branch.node_id,
                    )
            else:
                branches_valid = False
                fail(
                    "INVALID_TERMINAL_BRANCH_KIND",
                    f"Unknown terminal branch kind {branch.kind!r}.",
                    index,
                    node_id=branch.node_id,
                )
        checks["Terminal branch registry"] = branches_valid

        outcomes = tuple(result.terminal_outcomes)
        identities = []
        valid_paths = True
        valid_terminal_metadata = True
        valid_costs = True
        represented_identities = []

        for index, outcome in enumerate(outcomes):
            if not isinstance(outcome, TerminalOutcome):
                valid_paths = False
                valid_terminal_metadata = False
                valid_costs = False
                fail("INVALID_OUTCOME", "Entry is not a TerminalOutcome.", index)
                continue

            edge_ids = outcome.edge_ids
            path_valid = isinstance(edge_ids, tuple)
            current_node = graph.root_id
            visited = {current_node}

            if not path_valid:
                fail(
                    "INVALID_EDGE_SEQUENCE",
                    "Outcome edge_ids must be a tuple.",
                    index,
                    edge_ids if isinstance(edge_ids, tuple) else None,
                )
            else:
                for edge_id in edge_ids:
                    if (
                        not isinstance(edge_id, int)
                        or isinstance(edge_id, bool)
                        or edge_id not in edge_index
                    ):
                        path_valid = False
                        fail(
                            "UNKNOWN_EDGE",
                            f"Edge ID {edge_id!r} is not a valid graph edge ID.",
                            index,
                            edge_ids,
                        )
                        break

                    edge = edge_index[edge_id]
                    if edge.src != current_node:
                        path_valid = False
                        fail(
                            "DISCONNECTED_PATH",
                            f"Edge {edge_id} starts at node {edge.src}, expected node {current_node}.",
                            index,
                            edge_ids,
                        )
                        break
                    if edge.dst in visited:
                        path_valid = False
                        fail(
                            "NON_SIMPLE_PATH",
                            f"Path revisits node {edge.dst}.",
                            index,
                            edge_ids,
                        )
                        break
                    current_node = edge.dst
                    visited.add(current_node)

            if not path_valid:
                valid_paths = False

            branch_id = outcome.terminal_branch_id
            branch_id_valid = (
                isinstance(branch_id, int) and not isinstance(branch_id, bool)
            )
            branch = branch_index.get(branch_id) if branch_id_valid else None
            if branch is None:
                valid_terminal_metadata = False
                fail(
                    "UNKNOWN_TERMINAL_BRANCH",
                    f"Terminal branch ID {branch_id!r} is invalid or not registered.",
                    index,
                    edge_ids if isinstance(edge_ids, tuple) else None,
                )
            elif path_valid and current_node != branch.node_id:
                valid_terminal_metadata = False
                fail(
                    "TERMINAL_BRANCH_NODE_MISMATCH",
                    f"Outcome ends at node {current_node!r}, but its terminal branch belongs to node {branch.node_id!r}.",
                    index,
                    edge_ids,
                    current_node,
                )

            if not self._is_valid_cost(outcome.cost):
                valid_costs = False
                fail(
                    "INVALID_COST",
                    f"Cost must be a number other than NaN or negative infinity; received {outcome.cost!r}.",
                    index,
                    edge_ids if isinstance(edge_ids, tuple) else None,
                )

            if isinstance(edge_ids, tuple) and branch_id_valid:
                identities.append(self._outcome_key(outcome))
                represented_identities.append(self._outcome_key(outcome))

        checks["Outcome paths"] = valid_paths
        checks["Terminal branch metadata"] = valid_terminal_metadata
        checks["Outcome cost values"] = valid_costs

        identities_unique = True
        for index, identity in enumerate(identities):
            if any(identity == previous for previous in identities[:index]):
                identities_unique = False
                break
        checks["Outcome identity uniqueness"] = identities_unique
        if not identities_unique:
            fail(
                "DUPLICATE_OUTCOME",
                "The result contains duplicate (edge_ids, terminal_branch_id) outcomes.",
            )

        # Every terminal branch must be represented by each simple path to its node.
        expected_identities = set()
        for branch in branch_index.values():
            if branch.node_id in node_ids:
                expected_identities.update(
                    (edge_ids, branch.branch_id)
                    for edge_ids in self._simple_paths_to(graph, branch.node_id)
                )
        actual_identity_set = set(represented_identities)
        missing_identities = expected_identities - actual_identity_set
        extra_identities = actual_identity_set - expected_identities
        checks["Terminal branch path coverage"] = (
            not missing_identities and not extra_identities
        )
        for edge_ids, branch_id in sorted(missing_identities):
            fail(
                "MISSING_TERMINAL_OUTCOME",
                f"Terminal branch {branch_id} has no outcome for a simple root-to-node path.",
                edge_ids=edge_ids,
                node_id=branch_index[branch_id].node_id,
            )
        for edge_ids, branch_id in sorted(extra_identities):
            fail(
                "UNEXPECTED_TERMINAL_OUTCOME",
                f"Outcome references terminal branch {branch_id} without a matching branch path.",
                edge_ids=edge_ids,
                node_id=branch_index.get(branch_id).node_id
                if branch_id in branch_index
                else None,
            )

        feasible = [
            outcome
            for outcome in outcomes
            if isinstance(outcome, TerminalOutcome)
            and self._is_valid_cost(outcome.cost)
            and math.isfinite(outcome.cost)
        ]
        if feasible:
            expected_best_cost = min(float(item.cost) for item in feasible)
            expected_optimal = tuple(
                item
                for item in feasible
                if math.isclose(
                    item.cost,
                    expected_best_cost,
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                )
            )
        else:
            expected_best_cost = None
            expected_optimal = ()

        best_matches = (
            result.best_cost is None and expected_best_cost is None
        ) or (
            self._is_valid_cost(result.best_cost)
            and result.best_cost is not None
            and expected_best_cost is not None
            and math.isfinite(result.best_cost)
            and math.isclose(
                result.best_cost,
                expected_best_cost,
                rel_tol=1e-9,
                abs_tol=1e-9,
            )
        )
        checks["Best objective value"] = best_matches
        if not best_matches:
            fail(
                "BEST_COST_MISMATCH",
                f"Reported best cost {result.best_cost!r}; expected {expected_best_cost!r}.",
            )

        optimal_matches = tuple(result.optimal_outcomes) == expected_optimal
        checks["Optimal outcomes"] = optimal_matches
        if not optimal_matches:
            fail(
                "OPTIMAL_OUTCOMES_MISMATCH",
                "Reported optimal outcomes do not match the minimum-cost outcomes.",
            )

        return SearchResultValidationResult(
            valid=all(checks.values()) and not issues,
            checks=tuple(checks.items()),
            issues=tuple(issues),
            graph_node_count=len(node_ids),
            graph_edge_count=len(edge_index),
            terminal_outcome_count=len(outcomes),
            feasible_outcome_count=len(feasible),
            best_cost=result.best_cost,
        )
