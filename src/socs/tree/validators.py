"""Structural validation for SOCS decision trees."""

from collections import deque
from dataclasses import dataclass
from typing import Any

from socs.decision.decision import Decision
from socs.devs.Snapshot import SystemSnapshot
from socs.graph.agraph import Edge

from .decision_tree import DecisionTree


@dataclass(frozen=True)
class TreeValidationIssue:
    """Describe one structural issue found in a decision tree."""

    code: str
    message: str
    severity: str = "ERROR"
    node_id: int | None = None
    edge_id: int | None = None


@dataclass(frozen=True)
class DecisionTreeValidationResult:
    """Summary of a structural DecisionTree validation."""

    valid: bool
    root_id: int
    node_count: int
    edge_count: int
    checks: tuple[tuple[str, bool], ...]
    issues: tuple[TreeValidationIssue, ...]

    @property
    def errors(self) -> tuple[TreeValidationIssue, ...]:
        """Return validation errors."""
        return tuple(issue for issue in self.issues if issue.severity == "ERROR")

    @property
    def warnings(self) -> tuple[TreeValidationIssue, ...]:
        """Return validation warnings."""
        return tuple(issue for issue in self.issues if issue.severity == "WARNING")

    def format(self) -> str:
        """Format the validation result as a readable report."""
        lines = [
            "DecisionTree Validation",
            "",
            f"Root: {self.root_id if self.root_id else '<not set>'}",
            f"Nodes: {self.node_count}",
            f"Edges: {self.edge_count}",
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
                location = []
                if issue.node_id is not None:
                    location.append(f"Node {issue.node_id}")
                if issue.edge_id is not None:
                    location.append(f"Edge {issue.edge_id}")
                suffix = f" ({', '.join(location)})" if location else ""
                lines.append(
                    f"  {issue.severity} [{issue.code}]{suffix}: {issue.message}"
                )
        lines.extend(["", f"Result: {'VALID' if self.valid else 'INVALID'}"])
        return "\n".join(lines)


class DecisionTreeValidator:
    """Validate DecisionTree structure without modifying the tree."""

    def __init__(self, tree: DecisionTree):
        if not isinstance(tree, DecisionTree):
            raise TypeError("tree must be a DecisionTree instance.")
        self.tree = tree

    def validate(self) -> DecisionTreeValidationResult:
        """Run all structural checks and return a detailed report."""
        tree = self.tree
        issues: list[TreeValidationIssue] = []
        checks: dict[str, bool] = {}

        def add_issue(code, message, node_id=None, edge_id=None):
            issues.append(
                TreeValidationIssue(
                    code=code,
                    message=message,
                    node_id=node_id,
                    edge_id=edge_id,
                )
            )

        vertices = list(tree.V.keys())
        edges = list(tree.edges())
        vertex_set = set(vertices)
        edge_count = len(edges)
        root_id = tree.root_id

        checks["Root"] = (
            root_id != tree.INVALID_VERTEX_ID and root_id in vertex_set
        )
        if root_id == tree.INVALID_VERTEX_ID:
            add_issue("ROOT_MISSING", "The tree has no registered root.")
        elif root_id not in vertex_set:
            add_issue(
                "ROOT_NOT_REGISTERED",
                "The configured root ID is not present in the vertex collection.",
                node_id=root_id,
            )

        checks["Node IDs"] = True
        for node_id in vertices:
            if isinstance(node_id, bool) or not isinstance(node_id, int) or node_id <= 0:
                checks["Node IDs"] = False
                add_issue(
                    "INVALID_NODE_ID",
                    "Node IDs must be positive integers; zero is reserved.",
                    node_id=node_id if isinstance(node_id, int) else None,
                )

        checks["Snapshots"] = True
        for node_id in vertices:
            try:
                snapshot = tree.get_snapshot(node_id)
            except Exception as exc:
                checks["Snapshots"] = False
                add_issue(
                    "SNAPSHOT_ACCESS",
                    f"Could not retrieve the node snapshot: {type(exc).__name__}: {exc}",
                    node_id=node_id if isinstance(node_id, int) else None,
                )
                continue
            if not isinstance(snapshot, SystemSnapshot):
                checks["Snapshots"] = False
                add_issue(
                    "INVALID_SNAPSHOT",
                    "Every node must contain a SystemSnapshot.",
                    node_id=node_id if isinstance(node_id, int) else None,
                )

        checks["Edge endpoints"] = True
        checks["Edge attributes"] = True
        checks["Edge IDs"] = True
        seen_edge_objects = set()
        seen_edge_ids = {}
        valid_edges = []

        for edge in edges:
            if not isinstance(edge, Edge):
                checks["Edge endpoints"] = False
                add_issue("INVALID_EDGE_OBJECT", "An edge entry is not an Edge instance.")
                continue

            object_id = id(edge)
            if object_id in seen_edge_objects:
                checks["Edge attributes"] = False
                add_issue(
                    "DUPLICATE_EDGE_REFERENCE",
                    "The same Edge object occurs more than once in the outgoing index.",
                )
                continue
            seen_edge_objects.add(object_id)

            endpoints_valid = edge.src in vertex_set and edge.dst in vertex_set
            if not endpoints_valid:
                checks["Edge endpoints"] = False
                add_issue(
                    "MISSING_ENDPOINT",
                    "An edge references a source or destination node that does not exist.",
                )
                continue

            valid_edges.append(edge)
            if edge.src == edge.dst:
                add_issue(
                    "SELF_LOOP",
                    "A decision tree cannot contain a self-loop.",
                    node_id=edge.src if isinstance(edge.src, int) else None,
                )

            try:
                decision = tree.get_decision(edge)
                if not isinstance(decision, Decision):
                    raise TypeError("Edge decision is not a Decision instance.")
                edge_id = tree.get_edge_id(edge)
            except Exception as exc:
                checks["Edge attributes"] = False
                add_issue(
                    "INVALID_EDGE_ATTRIBUTES",
                    f"Could not retrieve a valid Decision and edge ID: {type(exc).__name__}: {exc}",
                )
                continue

            if isinstance(edge_id, bool) or not isinstance(edge_id, int) or edge_id <= 0:
                checks["Edge IDs"] = False
                add_issue(
                    "INVALID_EDGE_ID",
                    "Edge IDs must be positive integers.",
                )
            elif edge_id in seen_edge_ids:
                checks["Edge IDs"] = False
                add_issue(
                    "DUPLICATE_EDGE_ID",
                    f"Edge ID {edge_id} is also used by another edge.",
                    edge_id=edge_id,
                )
            else:
                seen_edge_ids[edge_id] = edge

        checks["Adjacency indexes"] = True
        outgoing_objects = [
            id(edge)
            for edge_list in tree.E.values()
            for edge in edge_list
        ]
        incoming_objects = [
            id(edge)
            for edge_list in tree.BE.values()
            for edge in edge_list
        ]
        if sorted(outgoing_objects) != sorted(incoming_objects):
            checks["Adjacency indexes"] = False
            add_issue(
                "ADJACENCY_INDEX_MISMATCH",
                "Outgoing and incoming adjacency indexes do not contain the same edge objects.",
            )

        for source, edge_list in tree.E.items():
            if source not in vertex_set:
                checks["Adjacency indexes"] = False
                add_issue(
                    "UNKNOWN_OUTGOING_INDEX_NODE",
                    "Outgoing adjacency index contains an unknown source node.",
                    node_id=source if isinstance(source, int) else None,
                )
            for edge in edge_list:
                if edge.src != source:
                    checks["Adjacency indexes"] = False
                    add_issue(
                        "OUTGOING_INDEX_SOURCE_MISMATCH",
                        "An edge is stored under a source different from edge.src.",
                    )

        for destination, edge_list in tree.BE.items():
            if destination not in vertex_set:
                checks["Adjacency indexes"] = False
                add_issue(
                    "UNKNOWN_INCOMING_INDEX_NODE",
                    "Incoming adjacency index contains an unknown destination node.",
                    node_id=destination if isinstance(destination, int) else None,
                )
            for edge in edge_list:
                if edge.dst != destination:
                    checks["Adjacency indexes"] = False
                    add_issue(
                        "INCOMING_INDEX_DESTINATION_MISMATCH",
                        "An edge is stored under a destination different from edge.dst.",
                    )

        indegree = {node_id: 0 for node_id in vertices}
        adjacency = {node_id: [] for node_id in vertices}
        for edge in valid_edges:
            indegree[edge.dst] += 1
            adjacency[edge.src].append(edge.dst)

        roots = [node_id for node_id, degree in indegree.items() if degree == 0]
        checks["Single root"] = (
            root_id in vertex_set and roots == [root_id]
        )
        if root_id in vertex_set and roots != [root_id]:
            add_issue(
                "ROOT_COUNT",
                f"Expected only configured root {root_id}; found root candidates {roots}.",
            )

        checks["Parent-child relations"] = True
        for node_id, degree in indegree.items():
            expected_degree = 0 if node_id == root_id else 1
            if degree != expected_degree:
                checks["Parent-child relations"] = False
                add_issue(
                    "INVALID_PARENT_COUNT",
                    f"Expected {expected_degree} parent edge(s), found {degree}.",
                    node_id=node_id if isinstance(node_id, int) else None,
                )

        checks["Edge count"] = (
            len(valid_edges) == max(0, len(vertices) - 1)
            if vertices
            else False
        )
        if not checks["Edge count"]:
            add_issue(
                "INVALID_EDGE_COUNT",
                "A non-empty tree must contain exactly node_count - 1 edges.",
            )

        checks["Cycle detection"] = True
        color = {node_id: 0 for node_id in vertices}
        for start in vertices:
            if color[start] != 0:
                continue
            stack = [(start, iter(adjacency[start]))]
            color[start] = 1
            while stack:
                current, children = stack[-1]
                try:
                    child = next(children)
                except StopIteration:
                    color[current] = 2
                    stack.pop()
                    continue
                if color[child] == 1:
                    checks["Cycle detection"] = False
                    add_issue(
                        "CYCLE_DETECTED",
                        "A directed cycle was found.",
                        node_id=child if isinstance(child, int) else None,
                    )
                elif color[child] == 0:
                    color[child] = 1
                    stack.append((child, iter(adjacency[child])))

        checks["Reachability"] = False
        if root_id in vertex_set:
            reached = set()
            pending = deque([root_id])
            while pending:
                node_id = pending.popleft()
                if node_id in reached:
                    continue
                reached.add(node_id)
                pending.extend(adjacency[node_id])
            checks["Reachability"] = reached == vertex_set
            for node_id in vertices:
                if node_id not in reached:
                    add_issue(
                        "UNREACHABLE_NODE",
                        "Node cannot be reached from the configured root.",
                        node_id=node_id if isinstance(node_id, int) else None,
                    )

        valid = all(checks.values()) and not issues
        return DecisionTreeValidationResult(
            valid=valid,
            root_id=root_id,
            node_count=len(vertices),
            edge_count=edge_count,
            checks=tuple(checks.items()),
            issues=tuple(issues),
        )
