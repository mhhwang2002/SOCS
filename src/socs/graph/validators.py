"""Structural validators for attributed and decision graphs."""

from collections import deque
from dataclasses import dataclass
from typing import Any

from socs.decision.decision import Decision
from socs.devs.Snapshot import SystemSnapshot
from socs.graph.agraph import AGraph, Edge
from socs.graph.decision_graph import DecisionGraph


@dataclass(frozen=True)
class GraphValidationIssue:
    """Describe one structural inconsistency in a graph."""

    code: str
    message: str
    vertex_id: Any = None
    edge_id: Any = None


@dataclass(frozen=True)
class GraphValidationResult:
    """Summary of graph validation checks."""

    valid: bool
    checks: tuple[tuple[str, bool], ...]
    issues: tuple[GraphValidationIssue, ...]

    @property
    def errors(self) -> tuple[GraphValidationIssue, ...]:
        """Return all validation issues."""
        return self.issues

    def format(self) -> str:
        """Format a readable validation report."""
        lines = ["Graph Validation", ""]
        lines.extend(
            f"  {name}: {'PASS' if passed else 'FAIL'}"
            for name, passed in self.checks
        )
        if self.issues:
            lines.extend(["", "Issues:"])
            for issue in self.issues:
                location = []
                if issue.vertex_id is not None:
                    location.append(f"Vertex {issue.vertex_id!r}")
                if issue.edge_id is not None:
                    location.append(f"Edge {issue.edge_id!r}")
                suffix = f" ({', '.join(location)})" if location else ""
                lines.append(
                    f"  ERROR [{issue.code}]{suffix}: {issue.message}"
                )
        lines.extend(["", f"Result: {'VALID' if self.valid else 'INVALID'}"])
        return "\n".join(lines)


class AGraphValidator:
    """Validate the structural integrity of an AGraph."""

    def __init__(self, graph: AGraph):
        if not isinstance(graph, AGraph):
            raise TypeError("graph must be an AGraph instance.")
        self.graph = graph

    def validate(self) -> GraphValidationResult:
        """Check vertex, edge, and adjacency-index consistency."""
        graph = self.graph
        issues: list[GraphValidationIssue] = []
        checks: dict[str, bool] = {}

        def fail(code: str, message: str, vertex_id=None, edge_id=None):
            issues.append(
                GraphValidationIssue(code, message, vertex_id, edge_id)
            )

        vertices_valid = isinstance(graph.V, dict)
        if vertices_valid:
            for vertex_id in graph.V:
                if graph.V[vertex_id] is None:
                    vertices_valid = False
                    fail(
                        "INVALID_VERTEX_ATTRIBUTES",
                        "Vertex attributes cannot be None.",
                        vertex_id=vertex_id,
                    )
        else:
            fail("INVALID_VERTEX_INDEX", "V must be a dictionary.")
        checks["Vertex index"] = vertices_valid

        indexes_are_dicts = isinstance(graph.E, dict) and isinstance(graph.BE, dict)
        if not indexes_are_dicts:
            fail("INVALID_EDGE_INDEX", "E and BE must be dictionaries.")
            checks["Adjacency indexes"] = False
            checks["Edge registration"] = False
            checks["Edge endpoints"] = False
            return GraphValidationResult(
                valid=False,
                checks=tuple(checks.items()),
                issues=tuple(issues),
            )

        edge_lists_valid = True
        outgoing_entries: list[tuple[Any, Any]] = []
        incoming_entries: list[tuple[Any, Any]] = []

        for source, edge_list in graph.E.items():
            if source not in graph.V:
                edge_lists_valid = False
                fail(
                    "UNKNOWN_OUTGOING_VERTEX",
                    "E contains an unregistered source vertex.",
                    vertex_id=source,
                )
            if not isinstance(edge_list, list):
                edge_lists_valid = False
                fail(
                    "INVALID_OUTGOING_LIST",
                    "Each E entry must be a list of edges.",
                    vertex_id=source,
                )
                continue
            outgoing_entries.extend((source, edge) for edge in edge_list)

        for destination, edge_list in graph.BE.items():
            if destination not in graph.V:
                edge_lists_valid = False
                fail(
                    "UNKNOWN_INCOMING_VERTEX",
                    "BE contains an unregistered destination vertex.",
                    vertex_id=destination,
                )
            if not isinstance(edge_list, list):
                edge_lists_valid = False
                fail(
                    "INVALID_INCOMING_LIST",
                    "Each BE entry must be a list of edges.",
                    vertex_id=destination,
                )
                continue
            incoming_entries.extend((destination, edge) for edge in edge_list)

        checks["Adjacency indexes"] = edge_lists_valid

        outgoing_ids = [id(edge) for _, edge in outgoing_entries]
        incoming_ids = [id(edge) for _, edge in incoming_entries]
        outgoing_counts = {item: outgoing_ids.count(item) for item in set(outgoing_ids)}
        incoming_counts = {item: incoming_ids.count(item) for item in set(incoming_ids)}

        registration_valid = True
        for _, edge in outgoing_entries + incoming_entries:
            if not isinstance(edge, Edge):
                registration_valid = False
                fail("INVALID_EDGE_OBJECT", "An adjacency index contains a non-Edge object.")

        for edge_object_id in set(outgoing_ids) | set(incoming_ids):
            out_count = outgoing_counts.get(edge_object_id, 0)
            in_count = incoming_counts.get(edge_object_id, 0)
            if out_count != 1 or in_count != 1:
                registration_valid = False
                fail(
                    "EDGE_INDEX_MISMATCH",
                    "Each edge object must occur exactly once in both E and BE.",
                )
        checks["Edge registration"] = registration_valid

        endpoints_valid = True
        for source, edge in outgoing_entries:
            if not isinstance(edge, Edge):
                endpoints_valid = False
                continue
            if edge.src != source:
                endpoints_valid = False
                fail(
                    "SOURCE_INDEX_MISMATCH",
                    "Edge source does not match its E index key.",
                    vertex_id=source,
                )
            if edge.src not in graph.V or edge.dst not in graph.V:
                endpoints_valid = False
                fail(
                    "UNKNOWN_EDGE_ENDPOINT",
                    "Both edge endpoints must be registered vertices.",
                )

        for destination, edge in incoming_entries:
            if not isinstance(edge, Edge):
                endpoints_valid = False
                continue
            if edge.dst != destination:
                endpoints_valid = False
                fail(
                    "DESTINATION_INDEX_MISMATCH",
                    "Edge destination does not match its BE index key.",
                    vertex_id=destination,
                )
        checks["Edge endpoints"] = endpoints_valid

        return GraphValidationResult(
            valid=all(checks.values()) and not issues,
            checks=tuple(checks.items()),
            issues=tuple(issues),
        )


class DecisionGraphValidator:
    """Validate SOCS-specific invariants of a DecisionGraph."""

    def __init__(self, graph: DecisionGraph):
        if not isinstance(graph, DecisionGraph):
            raise TypeError("graph must be a DecisionGraph instance.")
        self.graph = graph

    def validate(self) -> GraphValidationResult:
        """Check graph structure, snapshots, transitions, and reachability."""
        graph = self.graph
        issues: list[GraphValidationIssue] = []
        checks: dict[str, bool] = {}

        def fail(code: str, message: str, vertex_id=None, edge_id=None):
            issues.append(
                GraphValidationIssue(code, message, vertex_id, edge_id)
            )

        base_result = AGraphValidator(graph).validate()
        checks["AGraph structure"] = base_result.valid
        issues.extend(base_result.issues)

        root_valid = (
            isinstance(graph.root_id, int)
            and not isinstance(graph.root_id, bool)
            and graph.root_id != graph.INVALID_VERTEX_ID
            and graph.root_id in graph.V
        )
        checks["Root"] = root_valid
        if not root_valid:
            fail(
                "INVALID_ROOT",
                "DecisionGraph must have exactly one registered root.",
                vertex_id=graph.root_id,
            )

        node_ids_valid = True
        for node_id in graph.V:
            if (
                not isinstance(node_id, int)
                or isinstance(node_id, bool)
                or node_id <= graph.INVALID_VERTEX_ID
            ):
                node_ids_valid = False
                fail(
                    "INVALID_NODE_ID",
                    "Node IDs must be positive integers.",
                    vertex_id=node_id,
                )
        checks["Node IDs"] = node_ids_valid

        snapshots_valid = True
        snapshots: list[tuple[Any, SystemSnapshot]] = []
        for node_id in graph.V:
            try:
                snapshot = graph.get_snapshot(node_id)
            except Exception as exc:
                snapshots_valid = False
                fail(
                    "INVALID_SNAPSHOT",
                    f"Could not retrieve node snapshot: {type(exc).__name__}: {exc}",
                    vertex_id=node_id,
                )
                continue
            if not isinstance(snapshot, SystemSnapshot):
                snapshots_valid = False
                fail(
                    "INVALID_SNAPSHOT",
                    "Every node must store a SystemSnapshot.",
                    vertex_id=node_id,
                )
                continue
            for previous_id, previous_snapshot in snapshots:
                try:
                    equal = snapshot == previous_snapshot
                except Exception as exc:
                    equal = False
                    snapshots_valid = False
                    fail(
                        "SNAPSHOT_COMPARISON_ERROR",
                        f"Snapshot comparison failed: {type(exc).__name__}: {exc}",
                        vertex_id=node_id,
                    )
                if equal:
                    snapshots_valid = False
                    fail(
                        "DUPLICATE_SNAPSHOT",
                        f"Snapshot duplicates the snapshot stored at node {previous_id!r}.",
                        vertex_id=node_id,
                    )
                    break
            snapshots.append((node_id, snapshot))
        checks["Snapshot validity and uniqueness"] = snapshots_valid

        edge_ids_valid = True
        seen_edge_ids: dict[Any, int] = {}
        decisions_valid = True
        transitions_valid = True

        for source, edge_list in graph.E.items():
            if not isinstance(edge_list, list):
                continue
            for edge in edge_list:
                if not isinstance(edge, Edge):
                    decisions_valid = False
                    transitions_valid = False
                    continue

                edge_id = edge.attrs.get("edge_id")
                if (
                    not isinstance(edge_id, int)
                    or isinstance(edge_id, bool)
                    or edge_id <= 0
                ):
                    edge_ids_valid = False
                    fail(
                        "INVALID_EDGE_ID",
                        "Edge IDs must be positive integers.",
                        edge_id=edge_id,
                    )
                elif edge_id in seen_edge_ids:
                    edge_ids_valid = False
                    fail(
                        "DUPLICATE_EDGE_ID",
                        f"Edge ID is already used by edge from node {seen_edge_ids[edge_id]!r}.",
                        vertex_id=edge.src,
                        edge_id=edge_id,
                    )
                else:
                    seen_edge_ids[edge_id] = edge.src

                decision = edge.attrs.get("decision")
                if not isinstance(decision, Decision):
                    decisions_valid = False
                    fail(
                        "INVALID_DECISION",
                        "Every decision edge must store a Decision instance.",
                        vertex_id=edge.src,
                        edge_id=edge_id,
                    )

                if edge.src in graph.V and edge.dst in graph.V:
                    try:
                        if graph.get_snapshot(edge.src) == graph.get_snapshot(edge.dst):
                            transitions_valid = False
                            fail(
                                "NO_OP_EDGE",
                                "A decision edge cannot connect equal snapshots.",
                                vertex_id=edge.src,
                                edge_id=edge_id,
                            )
                    except Exception as exc:
                        transitions_valid = False
                        fail(
                            "SNAPSHOT_COMPARISON_ERROR",
                            f"Could not compare transition snapshots: {type(exc).__name__}: {exc}",
                            vertex_id=edge.src,
                            edge_id=edge_id,
                        )

        checks["Edge IDs"] = edge_ids_valid
        checks["Decision metadata"] = decisions_valid
        checks["State-changing transitions"] = transitions_valid

        reachable_valid = False
        if root_valid and isinstance(graph.E, dict):
            reached = {graph.root_id}
            queue = deque([graph.root_id])
            while queue:
                current = queue.popleft()
                for edge in graph.out_edges(current):
                    if isinstance(edge, Edge) and edge.dst in graph.V:
                        if edge.dst not in reached:
                            reached.add(edge.dst)
                            queue.append(edge.dst)
            unreachable = set(graph.V) - reached
            reachable_valid = not unreachable
            for node_id in sorted(unreachable, key=repr):
                fail(
                    "UNREACHABLE_NODE",
                    "Node cannot be reached from the DecisionGraph root.",
                    vertex_id=node_id,
                )
        else:
            fail(
                "REACHABILITY_NOT_CHECKED",
                "Reachability cannot be checked without a valid root.",
            )
        checks["Root reachability"] = reachable_valid

        return GraphValidationResult(
            valid=all(checks.values()) and not issues,
            checks=tuple(checks.items()),
            issues=tuple(issues),
        )
