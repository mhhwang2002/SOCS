import pytest

from socs.graph.agraph import AGraph, Edge


def test_add_vertex_with_attributes():
    graph = AGraph()

    attrs = graph.add_vertex(1, {"value": 111})

    assert graph.num_vertices() == 1
    assert attrs.value == 111
    assert graph.V[1]["value"] == 111


def test_add_vertex_without_attributes():
    graph = AGraph()

    attrs = graph.add_vertex(1)

    assert attrs == {}


def test_duplicate_vertex_is_rejected():
    graph = AGraph()
    graph.add_vertex(1)

    with pytest.raises(ValueError):
        graph.add_vertex(1)


def test_add_edge_with_attributes():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)

    edge = graph.add_edge(
        1, 2, {"type": "precedence", "cost": 100}
    )

    assert isinstance(edge, Edge)
    assert edge.src == 1
    assert edge.dst == 2
    assert edge.attrs.type == "precedence"
    assert edge.attrs["cost"] == 100
    assert graph.num_edges() == 1


def test_parallel_edges_are_preserved():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)

    first = graph.add_edge(1, 2, {"type": "precedence", "time": 10})
    second = graph.add_edge(1, 2, {"type": "precedence", "cost": 100})

    assert graph.num_edges() == 2
    assert graph.out_edges(1) == [first, second]
    assert graph.in_edges(2) == [first, second]


def test_forward_and_backward_adjacency_share_edge():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)

    edge = graph.add_edge(1, 2, {"cost": 100})

    assert graph.E[1][0] is edge
    assert graph.BE[2][0] is edge

    edge.attrs.cost = 200

    assert graph.BE[2][0].attrs.cost == 200


def test_in_and_out_vertices():
    graph = AGraph()
    for vertex in (1, 2, 3):
        graph.add_vertex(vertex)

    graph.add_edge(1, 3)
    graph.add_edge(2, 3)

    assert graph.out_vertices(1) == {3}
    assert graph.in_vertices(3) == {1, 2}


def test_edges_require_registered_vertices():
    graph = AGraph()
    graph.add_vertex(1)

    with pytest.raises(KeyError):
        graph.add_edge(1, 2)

    with pytest.raises(KeyError):
        graph.add_edge(2, 1)


def test_unknown_vertex_query_is_rejected():
    graph = AGraph()

    with pytest.raises(KeyError):
        graph.out_edges(99)

    with pytest.raises(KeyError):
        graph.in_edges(99)


def test_attribute_inputs_are_copied():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)

    attrs = {"cost": 100}
    edge = graph.add_edge(1, 2, attrs)
    attrs["cost"] = 500

    assert edge.attrs.cost == 100


def test_remove_one_parallel_edge():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)

    first = graph.add_edge(1, 2, {"cost": 100})
    second = graph.add_edge(1, 2, {"cost": 200})

    assert graph.remove_edge(first) is True
    assert graph.num_edges() == 1
    assert graph.out_edges(1) == [second]
    assert graph.in_edges(2) == [second]


def test_remove_edge_twice_returns_false():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)
    edge = graph.add_edge(1, 2)

    assert graph.remove_edge(edge) is True
    assert graph.remove_edge(edge) is False


def test_remove_edge_rejects_non_edge():
    graph = AGraph()

    with pytest.raises(TypeError):
        graph.remove_edge("not an edge")


def test_remove_vertex_removes_incoming_and_outgoing_edges():
    graph = AGraph()
    for vertex in (1, 2, 3):
        graph.add_vertex(vertex)

    graph.add_edge(1, 2)
    graph.add_edge(2, 3)
    graph.add_edge(1, 3)

    graph.remove_vertex(2)

    assert 2 not in graph.V
    assert graph.num_vertices() == 2
    assert graph.num_edges() == 1
    assert graph.out_vertices(1) == {3}
    assert graph.in_vertices(3) == {1}


def test_remove_vertex_handles_self_loop():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_edge(1, 1)

    graph.remove_vertex(1)

    assert graph.num_vertices() == 0
    assert graph.num_edges() == 0
    assert graph.E == {}
    assert graph.BE == {}


def test_remove_unknown_vertex_raises_key_error():
    graph = AGraph()

    with pytest.raises(KeyError):
        graph.remove_vertex(99)


def test_edge_filter_by_attributes():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)

    first = graph.add_edge(1, 2, {"type": "precedence", "cost": 100})
    second = graph.add_edge(1, 2, {"type": "precedence", "cost": 20})
    graph.add_edge(1, 2, {"type": "other", "cost": 200})

    assert graph.edges(
        src=1, dst=2, attrs={"type": "precedence"}
    ) == [first, second]


def test_edge_filter_by_predicate():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)

    expensive = graph.add_edge(1, 2, {"cost": 100})
    graph.add_edge(1, 2, {"cost": 20})

    assert graph.edges(
        predicate=lambda edge: edge.attrs.cost > 50
    ) == [expensive]


def test_out_edges_support_filters():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)
    graph.add_vertex(3)

    first = graph.add_edge(1, 2, {"type": "precedence"})
    graph.add_edge(1, 3, {"type": "other"})

    assert graph.out_edges(1, attrs={"type": "precedence"}) == [first]


def test_in_edges_support_filters():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)
    graph.add_vertex(3)

    first = graph.add_edge(1, 3, {"cost": 100})
    graph.add_edge(2, 3, {"cost": 20})

    assert graph.in_edges(
        3, predicate=lambda edge: edge.attrs.cost >= 100
    ) == [first]

