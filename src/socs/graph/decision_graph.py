
"""Decision graph built on top of the generic attributed graph."""

from collections.abc import Iterator

from socs.decision.decision import Decision
from socs.devs.Snapshot import SystemSnapshot
from socs.graph.agraph import AGraph, Edge


class DecisionGraph(AGraph):
    """Store simulation snapshots as vertices and decisions as edges."""

    INVALID_VERTEX_ID = 0

    def __init__(self):
        super().__init__()
        self._next_node_id = 1
        self._next_edge_id = 1
        self._root_id = self.INVALID_VERTEX_ID

    @property
    def root_id(self) -> int:
        """Return the root vertex ID, or INVALID_VERTEX_ID if unset."""
        return self._root_id

    def _allocate_node_id(self) -> int:
        node_id = self._next_node_id
        self._next_node_id += 1
        return node_id

    @staticmethod
    def _validate_snapshot(snapshot: SystemSnapshot) -> None:
        if not isinstance(snapshot, SystemSnapshot):
            raise TypeError("snapshot must be a SystemSnapshot instance.")

    @staticmethod
    def _validate_decision(decision: Decision) -> None:
        if not isinstance(decision, Decision):
            raise TypeError("decision must be a Decision instance.")

    def add_root(self, snapshot: SystemSnapshot) -> int:
        """Create the graph's root vertex and return its ID."""
        if self._root_id != self.INVALID_VERTEX_ID:
            raise ValueError("The decision graph already has a root.")

        self._validate_snapshot(snapshot)

        node_id = self._allocate_node_id()
        super().add_vertex(node_id, {"snapshot": snapshot})
        self._root_id = node_id
        return node_id

    def add_or_get_node(
        self,
        snapshot: SystemSnapshot,
    ) -> tuple[int, bool]:
        """Return an existing node for an equal snapshot, or create one."""
        if self._root_id == self.INVALID_VERTEX_ID:
            raise ValueError("A root must be added before other nodes.")

        self._validate_snapshot(snapshot)

        for node_id in self.V:
            if self.get_snapshot(node_id) == snapshot:
                return node_id, False

        node_id = self._allocate_node_id()
        super().add_vertex(node_id, {"snapshot": snapshot})
        return node_id, True

    def add_transition(
        self,
        parent_id: int,
        decision: Decision,
        snapshot: SystemSnapshot,
    ) -> Edge | None:
        """Add a state-changing transition, or return None if state is unchanged."""
        if self._root_id == self.INVALID_VERTEX_ID:
            raise ValueError("A root must be added before transitions.")
        if parent_id not in self.V:
            raise KeyError(f"Parent vertex {parent_id!r} is not registered.")

        self._validate_decision(decision)
        self._validate_snapshot(snapshot)

        parent_snapshot = self.get_snapshot(parent_id)

        if snapshot == parent_snapshot:
            return None

        destination_id, _ = self.add_or_get_node(snapshot)

        edge_id = self._next_edge_id
        self._next_edge_id += 1

        return super().add_edge(
            parent_id,
            destination_id,
            {
                "decision": decision,
                "edge_id": edge_id,
            },
        )

    def get_snapshot(self, node_id: int) -> SystemSnapshot:
        """Return the snapshot stored at a vertex."""
        try:
            return self.V[node_id].snapshot
        except KeyError:
            raise KeyError(
                f"Vertex {node_id!r} is not registered."
            ) from None

    def get_decision(self, edge: Edge) -> Decision:
        """Return the decision stored on an edge belonging to this graph."""
        if not isinstance(edge, Edge):
            raise TypeError("edge must be an Edge instance.")

        if edge.src not in self.V or not any(
            item is edge for item in self.out_edges(edge.src)
        ):
            raise ValueError("The edge does not belong to this decision graph.")

        return edge.attrs.decision

    def get_edge_id(self, edge: Edge) -> int:
        """Return the stable ID assigned to an edge."""
        self.get_decision(edge)
        return edge.attrs.edge_id

    def get_parents(self, node_id: int) -> list[int]:
        """Return unique parent IDs in insertion order."""
        if node_id not in self.V:
            raise KeyError(f"Vertex {node_id!r} is not registered.")

        return list(dict.fromkeys(
            edge.src for edge in self.in_edges(node_id)
        ))

    def get_children(self, node_id: int) -> list[int]:
        """Return destination IDs in edge insertion order."""
        if node_id not in self.V:
            raise KeyError(f"Vertex {node_id!r} is not registered.")

        return [edge.dst for edge in self.out_edges(node_id)]

    def _iter_edge_paths(
        self,
        node_id: int,
    ) -> Iterator[list[Edge]]:
        """Yield simple root-to-node paths without repeating vertices."""
        if node_id not in self.V:
            raise KeyError(f"Vertex {node_id!r} is not registered.")

        def visit(
            current_id: int,
            reverse_edges: list[Edge],
            visited: set[int],
        ) -> Iterator[list[Edge]]:
            if current_id == self._root_id:
                yield list(reversed(reverse_edges))
                return

            for edge in self.in_edges(current_id):
                if edge.src in visited:
                    continue

                yield from visit(
                    edge.src,
                    reverse_edges + [edge],
                    visited | {edge.src},
                )

        yield from visit(node_id, [], {node_id})

    def get_paths(self, node_id: int) -> Iterator[list[Decision]]:
        """Yield all simple root-to-node decision paths."""
        for edges in self._iter_edge_paths(node_id):
            yield [self.get_decision(edge) for edge in edges]

    def get_path(self, node_id: int) -> list[Decision]:
        """Return the unique root-to-node path, if one exists."""
        paths = iter(self.get_paths(node_id))

        try:
            path = next(paths)
        except StopIteration:
            raise RuntimeError(
                f"Vertex {node_id!r} is not reachable from the root."
            ) from None

        try:
            next(paths)
        except StopIteration:
            return path

        raise RuntimeError(
            "Multiple paths exist; use get_paths() instead."
        )

    def get_leaves(self) -> list[int]:
        """Return vertices with no outgoing edges."""
        return [
            node_id
            for node_id in self.V
            if not self.out_edges(node_id)
        ]

    def format_path(
        self,
        node_id: int,
        node_formatter=None,
        edge_formatter=None,
    ) -> str:
        """Format the unique root-to-node path."""
        paths = iter(self._iter_edge_paths(node_id))

        try:
            edges = next(paths)
        except StopIteration:
            raise RuntimeError(
                f"Vertex {node_id!r} is not reachable from the root."
            ) from None

        try:
            next(paths)
        except StopIteration:
            pass
        else:
            raise RuntimeError(
                "Multiple paths exist; a specific path is required."
            )

        node_formatter = node_formatter or (
            lambda current_id, snapshot: f"Node {current_id}"
        )
        edge_formatter = edge_formatter or (
            lambda current_id, decision: f"Edge {current_id}: {decision}"
        )

        node_ids = [self._root_id]
        node_ids.extend(edge.dst for edge in edges)

        lines = []

        for index, current_id in enumerate(node_ids):
            node_text = node_formatter(
                current_id,
                self.get_snapshot(current_id),
            )
            if not isinstance(node_text, str):
                raise TypeError("node_formatter must return a string.")

            lines.append(node_text)

            if index < len(edges):
                edge = edges[index]
                edge_text = edge_formatter(
                    self.get_edge_id(edge),
                    self.get_decision(edge),
                )
                if not isinstance(edge_text, str):
                    raise TypeError("edge_formatter must return a string.")

                lines.append(f"  -- {edge_text} -->")

        return "\n".join(lines)

    def print_path(
        self,
        node_id: int,
        node_formatter=None,
        edge_formatter=None,
    ) -> None:
        """Print the unique root-to-node path."""
        print(self.format_path(node_id, node_formatter, edge_formatter))

    def add_vertex(self, vertex, attrs=None):
        """Prevent direct vertex insertion from bypassing graph invariants."""
        raise NotImplementedError(
            "Use add_root() or add_or_get_node() to add vertices."
        )

    def add_edge(self, src, dst, attrs=None):
        """Prevent direct edge insertion from bypassing graph invariants."""
        raise NotImplementedError(
            "Use add_transition() to add decision edges."
        )

    def remove_edge(self, edge):
        """Prevent independent removal of decision transitions."""
        raise NotImplementedError(
            "Decision edges cannot be removed independently."
        )

    def remove_vertex(self, vertex):
        """Remove a non-root vertex without outgoing edges or reuse its ID."""
        if vertex not in self.V:
            raise KeyError(f"Vertex {vertex!r} is not registered.")
        if vertex == self._root_id:
            raise ValueError("The root vertex cannot be removed.")
        if self.out_edges(vertex):
            raise ValueError("Only vertices without outgoing edges can be removed.")

        for edge in self.in_edges(vertex):
            AGraph.remove_edge(self, edge)

        AGraph.remove_vertex(self, vertex)
