"""Decision tree built on top of the generic attributed graph."""

from socs.decision.decision import Decision
from socs.devs.Snapshot import SystemSnapshot
from socs.graph.agraph import AGraph, Edge


class DecisionTree(AGraph):
    """Store simulation snapshots as vertices and decisions as edges.

    Vertex IDs are positive integers allocated by the tree. ID zero is
    reserved as the invalid vertex ID. IDs are never reused.
    """

    INVALID_VERTEX_ID = 0

    def __init__(self):
        super().__init__()
        self._next_node_id = 1
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
        """Create the tree's root vertex and return its ID."""
        if self._root_id != self.INVALID_VERTEX_ID:
            raise ValueError("The decision tree already has a root.")

        self._validate_snapshot(snapshot)
        node_id = self._allocate_node_id()
        super().add_vertex(node_id, {"snapshot": snapshot})
        self._root_id = node_id
        return node_id

    def add_child(
        self,
        parent_id: int,
        decision: Decision,
        snapshot: SystemSnapshot,
    ) -> int:
        """Add a child vertex connected to its parent by a decision edge."""
        if self._root_id == self.INVALID_VERTEX_ID:
            raise ValueError("A root must be added before child vertices.")
        if parent_id not in self.V:
            raise KeyError(f"Parent vertex {parent_id!r} is not registered.")

        self._validate_decision(decision)
        self._validate_snapshot(snapshot)

        node_id = self._allocate_node_id()
        super().add_vertex(node_id, {"snapshot": snapshot})
        try:
            super().add_edge(parent_id, node_id, {"decision": decision})
        except Exception:
            super().remove_vertex(node_id)
            raise

        return node_id

    def get_snapshot(self, node_id: int) -> SystemSnapshot:
        """Return the snapshot stored at a vertex."""
        try:
            return self.V[node_id].snapshot
        except KeyError:
            raise KeyError(f"Vertex {node_id!r} is not registered.") from None

    def get_decision(self, edge: Edge) -> Decision:
        """Return the decision stored on an edge belonging to this tree."""
        if not isinstance(edge, Edge):
            raise TypeError("edge must be an Edge instance.")
        if edge.src not in self.V or not any(
            item is edge for item in self.out_edges(edge.src)
        ):
            raise ValueError("The edge does not belong to this decision tree.")
        return edge.attrs.decision

    def get_parent(self, node_id: int) -> int | None:
        """Return a vertex's parent ID, or None for the root."""
        if node_id not in self.V:
            raise KeyError(f"Vertex {node_id!r} is not registered.")
        incoming = self.in_edges(node_id)
        if not incoming:
            return None
        if len(incoming) != 1:
            raise RuntimeError("Decision tree contains a vertex with multiple parents.")
        return incoming[0].src

    def get_children(self, node_id: int) -> list[int]:
        """Return child vertex IDs in insertion order."""
        return [edge.dst for edge in self.out_edges(node_id)]

    def get_path(self, node_id: int) -> list[Decision]:
        """Return the decisions from the root to the specified vertex."""
        if node_id not in self.V:
            raise KeyError(f"Vertex {node_id!r} is not registered.")

        path = []
        current_id = node_id
        while current_id != self._root_id:
            incoming = self.in_edges(current_id)
            if len(incoming) != 1:
                raise RuntimeError("Decision tree contains an invalid parent relation.")
            edge = incoming[0]
            path.append(self.get_decision(edge))
            current_id = edge.src

        path.reverse()
        return path

    def add_vertex(self, vertex, attrs=None):
        """Prevent direct vertex insertion from bypassing tree invariants."""
        raise NotImplementedError("Use add_root() or add_child() to add vertices.")

    def add_edge(self, src, dst, attrs=None):
        """Prevent direct edge insertion from bypassing tree invariants."""
        raise NotImplementedError("Use add_child() to add decision edges.")

    def remove_edge(self, edge):
        """Prevent edge removal from leaving orphaned vertices."""
        raise NotImplementedError("Decision edges cannot be removed independently.")

    def remove_vertex(self, vertex):
        """Remove a non-root leaf vertex without reusing its ID."""
        if vertex not in self.V:
            raise KeyError(f"Vertex {vertex!r} is not registered.")
        if vertex == self._root_id:
            raise ValueError("The root vertex cannot be removed.")
        if self.out_edges(vertex):
            raise ValueError("Only leaf vertices can be removed.")
        for edge in self.in_edges(vertex):
            AGraph.remove_edge(self, edge)
        AGraph.remove_vertex(self, vertex)
