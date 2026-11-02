import pytest

from socs.decision.decision import Decision
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.graph.agraph import AGraph
from socs.tree import DecisionTree


class DecisionStub(Decision):
    def __init__(self, name):
        self.name = name


def make_snapshot(value):
    return SystemSnapshot(
        t_now=float(value),
        root_model=ModelSnapshot(
            model_id=1,
            model_path="root",
            common_state={"value": value},
        ),
        t_next_continuous=float("inf"),
        t_next_animation=float("inf"),
    )


def test_decision_tree_is_an_agraph():
    assert issubclass(DecisionTree, AGraph)
    assert isinstance(DecisionTree(), AGraph)


def test_root_gets_vertex_id_one():
    tree = DecisionTree()
    snapshot = make_snapshot(0)

    root_id = tree.add_root(snapshot)

    assert root_id == 1
    assert tree.root_id == 1
    assert tree.get_snapshot(root_id) is snapshot
    assert tree.get_parent(root_id) is None


def test_child_ids_are_positive_and_increase_in_creation_order():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(0))

    first_id = tree.add_child(root_id, DecisionStub("A"), make_snapshot(1))
    second_id = tree.add_child(root_id, DecisionStub("B"), make_snapshot(2))

    assert (root_id, first_id, second_id) == (1, 2, 3)
    assert all(node_id >= 1 for node_id in tree.V)


def test_add_child_stores_decision_on_edge():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(0))
    decision = DecisionStub("A")
    child_id = tree.add_child(root_id, decision, make_snapshot(1))

    edge = tree.out_edges(root_id)[0]

    assert edge.dst == child_id
    assert tree.get_decision(edge) is decision


def test_parent_children_and_path_queries():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(0))
    decision_a = DecisionStub("A")
    decision_b = DecisionStub("B")
    child_id = tree.add_child(root_id, decision_a, make_snapshot(1))
    grandchild_id = tree.add_child(child_id, decision_b, make_snapshot(2))

    assert tree.get_children(root_id) == [child_id]
    assert tree.get_children(child_id) == [grandchild_id]
    assert tree.get_children(grandchild_id) == []
    assert tree.get_parent(child_id) == root_id
    assert tree.get_parent(grandchild_id) == child_id
    assert tree.get_path(root_id) == []
    assert tree.get_path(grandchild_id) == [decision_a, decision_b]


def test_only_one_root_is_allowed():
    tree = DecisionTree()
    tree.add_root(make_snapshot(0))

    with pytest.raises(ValueError):
        tree.add_root(make_snapshot(1))


def test_child_cannot_be_added_before_root():
    tree = DecisionTree()

    with pytest.raises(ValueError):
        tree.add_child(1, DecisionStub("A"), make_snapshot(1))


def test_invalid_snapshot_and_decision_types_are_rejected():
    tree = DecisionTree()

    with pytest.raises(TypeError):
        tree.add_root({"not": "a snapshot"})

    root_id = tree.add_root(make_snapshot(0))
    with pytest.raises(TypeError):
        tree.add_child(root_id, object(), make_snapshot(1))
    with pytest.raises(TypeError):
        tree.add_child(root_id, DecisionStub("A"), {})


def test_unknown_vertex_queries_raise_key_error():
    tree = DecisionTree()
    tree.add_root(make_snapshot(0))

    with pytest.raises(KeyError):
        tree.get_snapshot(99)
    with pytest.raises(KeyError):
        tree.get_parent(99)
    with pytest.raises(KeyError):
        tree.get_children(99)
    with pytest.raises(KeyError):
        tree.get_path(99)


def test_direct_graph_mutations_are_blocked():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(0))

    with pytest.raises(NotImplementedError):
        tree.add_vertex(99)
    with pytest.raises(NotImplementedError):
        tree.add_edge(root_id, root_id)


def test_only_leaf_non_root_vertices_can_be_removed_and_ids_are_not_reused():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(0))
    child_id = tree.add_child(root_id, DecisionStub("A"), make_snapshot(1))
    grandchild_id = tree.add_child(child_id, DecisionStub("B"), make_snapshot(2))

    with pytest.raises(ValueError):
        tree.remove_vertex(root_id)
    with pytest.raises(ValueError):
        tree.remove_vertex(child_id)

    tree.remove_vertex(grandchild_id)
    next_id = tree.add_child(child_id, DecisionStub("C"), make_snapshot(3))

    assert next_id == 4
    assert grandchild_id not in tree.V


def test_get_decision_rejects_an_edge_from_another_graph():
    tree = DecisionTree()
    tree.add_root(make_snapshot(0))

    other = AGraph()
    other.add_vertex(1)
    other.add_vertex(2)
    foreign_edge = other.add_edge(1, 2)

    with pytest.raises(ValueError):
        tree.get_decision(foreign_edge)
