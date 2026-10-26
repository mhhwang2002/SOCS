#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

"""Attributed graph with support for parallel edges."""

from dataclasses import dataclass
from collections.abc import Mapping, Callable

from socs.graph.attributes import AttrDict


@dataclass(eq=False)
class Edge:
    """Represent a directed edge and its attributes."""

    src: object
    dst: object
    attrs: AttrDict


class AGraph:
    """Represent a directed multigraph with attributed vertices and edges."""

    def __init__(self):
        self.V = {}
        self.E = {}
        self.BE = {}

    def add_vertex(self, vertex, attrs=None):
        """Add a vertex with optional attributes."""
        if vertex in self.V:
            raise ValueError(f"Vertex {vertex!r} already exists.")

        attrs = {} if attrs is None else attrs
        if not isinstance(attrs, Mapping):
            raise TypeError("Vertex attributes must be a mapping.")

        self.V[vertex] = AttrDict(attrs)
        return self.V[vertex]

    def add_edge(self, src, dst, attrs=None):
        """Add a directed edge and return the created edge."""
        if src not in self.V:
            raise KeyError(f"Source vertex {src!r} is not registered.")
        if dst not in self.V:
            raise KeyError(f"Destination vertex {dst!r} is not registered.")

        attrs = {} if attrs is None else attrs
        if not isinstance(attrs, Mapping):
            raise TypeError("Edge attributes must be a mapping.")

        edge = Edge(src=src, dst=dst, attrs=AttrDict(attrs))
        self.E.setdefault(src, []).append(edge)
        self.BE.setdefault(dst, []).append(edge)
        return edge

    def vertex_attrs(self, vertex):
        """Return the attributes of a registered vertex."""
        return self.V[vertex]

    @staticmethod
    def _matches(edge, attrs=None, predicate=None):
        """Return whether an edge satisfies all supplied filters."""
        if attrs is not None:
            if not all(
                key in edge.attrs and edge.attrs[key] == value
                for key, value in attrs.items()
            ):
                return False

        if predicate is not None and not predicate(edge):
            return False

        return True

    def edges(self, src=None, dst=None, attrs=None, predicate=None):
        """Return edges matching endpoint and attribute filters."""
        if attrs is not None and not isinstance(attrs, Mapping):
            raise TypeError("Edge filters must be a mapping.")

        if predicate is not None and not callable(predicate):
            raise TypeError("Predicate must be callable.")

        if src is not None and src not in self.V:
            raise KeyError(f"Vertex {src!r} is not registered.")
        if dst is not None and dst not in self.V:
            raise KeyError(f"Vertex {dst!r} is not registered.")

        if src is not None:
            candidates = self.E.get(src, [])
        elif dst is not None:
            candidates = self.BE.get(dst, [])
        else:
            candidates = (
                edge
                for edge_list in self.E.values()
                for edge in edge_list
            )

        return [
            edge for edge in candidates
            if (dst is None or edge.dst == dst)
            and (src is None or edge.src == src)
            and self._matches(edge, attrs, predicate)
        ]

    def out_edges(self, vertex, attrs=None, predicate=None):
        """Return outgoing edges matching optional filters."""
        if vertex not in self.V:
            raise KeyError(f"Vertex {vertex!r} is not registered.")
        return self.edges(src=vertex, attrs=attrs, predicate=predicate)

    def in_edges(self, vertex, attrs=None, predicate=None):
        """Return incoming edges matching optional filters."""
        if vertex not in self.V:
            raise KeyError(f"Vertex {vertex!r} is not registered.")
        return self.edges(dst=vertex, attrs=attrs, predicate=predicate)

    def out_vertices(self, vertex):
        """Return the set of destination vertices."""
        return {edge.dst for edge in self.out_edges(vertex)}

    def in_vertices(self, vertex):
        """Return the set of source vertices."""
        return {edge.src for edge in self.in_edges(vertex)}

    def remove_edge(self, edge):
        """Remove one specific edge from both adjacency indexes."""
        if not isinstance(edge, Edge):
            raise TypeError("Expected an Edge instance.")

        outgoing = self.E.get(edge.src, [])
        incoming = self.BE.get(edge.dst, [])

        if not any(item is edge for item in outgoing):
            return False
        if not any(item is edge for item in incoming):
            raise RuntimeError("Graph adjacency indexes are inconsistent.")

        self.E[edge.src] = [item for item in outgoing if item is not edge]
        self.BE[edge.dst] = [item for item in incoming if item is not edge]

        if not self.E[edge.src]:
            del self.E[edge.src]
        if not self.BE[edge.dst]:
            del self.BE[edge.dst]

        return True

    def remove_vertex(self, vertex):
        """Remove a vertex and all incident edges."""
        if vertex not in self.V:
            raise KeyError(f"Vertex {vertex!r} is not registered.")

        incident = []
        seen_ids = set()

        for edge in self.E.get(vertex, []) + self.BE.get(vertex, []):
            if id(edge) not in seen_ids:
                seen_ids.add(id(edge))
                incident.append(edge)

        for edge in incident:
            self.remove_edge(edge)

        self.E.pop(vertex, None)
        self.BE.pop(vertex, None)
        del self.V[vertex]

    def num_vertices(self):
        """Return the number of vertices."""
        return len(self.V)

    def num_edges(self):
        """Return the number of edges."""
        return sum(len(edge_list) for edge_list in self.E.values())
