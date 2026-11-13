import pytest

from socs.decision.decision import Decision
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.graph.agraph import AGraph
from socs.tree import DecisionTree


class DecisionStub(Decision):
    def __init__(self, name):
        self.name = name

    def __str__(self):
        return self.name


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


def test_format_path_shows_nodes_edges_and_decisions():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(0))
    first_decision = DecisionStub("A")
    second_decision = DecisionStub("B")
    child_id = tree.add_child(root_id, first_decision, make_snapshot(1))
    leaf_id = tree.add_child(child_id, second_decision, make_snapshot(2))

    text = tree.format_path(leaf_id)

    assert text.splitlines() == [
        "Node 1",
        "  -- Edge 1: A -->",
        "Node 2",
        "  -- Edge 2: B -->",
        "Node 3",
    ]
    assert tree.get_edge_id(tree.out_edges(root_id)[0]) == 1
    assert tree.get_edge_id(tree.out_edges(child_id)[0]) == 2


def test_format_path_supports_custom_formatters():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(0))
    decision = DecisionStub("go")
    child_id = tree.add_child(root_id, decision, make_snapshot(1))

    text = tree.format_path(
        child_id,
        node_formatter=lambda node_id, snapshot: f"N{node_id}@{snapshot.t_now}",
        edge_formatter=lambda edge_id, item: f"E{edge_id}:{item.name}",
    )

    assert text == "N1@0.0\n  -- E1:go -->\nN2@1.0"


def test_print_path_writes_formatted_path(capsys):
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(0))
    child_id = tree.add_child(root_id, DecisionStub("go"), make_snapshot(1))

    tree.print_path(child_id)

    assert "Node 1" in capsys.readouterr().out


def test_snapshot_formatter_includes_engine_and_recursive_model_state():
    from socs.tree import SnapshotFormatter

    root_snapshot = ModelSnapshot(
        model_id=1,
        model_path="Root",
        common_state={
            "time_last": 2.0,
            "time_next": 5.0,
            "statistics_since": 2.0,
            "state_percentage": 0.4,
        },
        model_state=("ACTIVE", 3),
        has_model_state=True,
        children={
            "Child": ModelSnapshot(
                model_id=2,
                model_path="Root.Child",
                common_state={
                    "time_last": 2.0,
                    "time_next": float("inf"),
                    "statistics_since": 2.0,
                    "state_percentage": 0.0,
                },
                model_state={"phase": "WAITING"},
                has_model_state=True,
            )
        },
    )
    snapshot = SystemSnapshot(
        t_now=2.0,
        root_model=root_snapshot,
        t_next_continuous=4.0,
        t_next_animation=float("inf"),
        x_buffer=[(2, "?go")],
    )
    tree = DecisionTree()
    root_id = tree.add_root(snapshot)

    text = tree.format_path(root_id, node_formatter=SnapshotFormatter())

    assert "Node 1" in text
    assert "Simulation time: 2" in text
    assert "Next continuous event: 4" in text
    assert "Next animation event: inf" in text
    assert "Simulation time bound: <not provided>" in text
    assert "Pending external inputs: 1" in text
    assert "XBuffer[1]: target=Root.Child, event=?go" in text
    assert "Model: Root" in text
    assert "time_last: 2" in text
    assert "time_next: 5" in text
    assert "statistics_since:" not in text
    assert "state_percentage:" not in text
    assert "model_state: ('ACTIVE', 3)" in text
    assert "Model: Root.Child" in text
    assert "time_next: inf" in text
    assert "model_state: {'phase': 'WAITING'}" in text


def test_snapshot_formatter_can_optionally_include_state_percentage():
    from socs.tree import SnapshotFormatter

    root_snapshot = ModelSnapshot(
        model_id=1,
        model_path="Root",
        common_state={"state_percentage": {"B,5": 20.0}},
        has_model_state=False,
    )
    snapshot = SystemSnapshot(
        t_now=2.0,
        root_model=root_snapshot,
        t_next_continuous=float("inf"),
        t_next_animation=float("inf"),
    )
    tree = DecisionTree()
    root_id = tree.add_root(snapshot)

    default_text = tree.format_path(root_id, node_formatter=SnapshotFormatter())
    optional_text = tree.format_path(
        root_id,
        node_formatter=SnapshotFormatter(show_state_percentage=True),
    )

    assert "state_percentage:" not in default_text
    assert "state_percentage: {'B,5': 20.0}" in optional_text


def test_snapshot_formatter_omits_uncaptured_model_state():
    from socs.tree import SnapshotFormatter

    root_snapshot = ModelSnapshot(
        model_id=1,
        model_path="CoupledRoot",
        common_state={"time_last": 0.0, "time_next": float("inf")},
        has_model_state=False,
        children={
            "Atomic": ModelSnapshot(
                model_id=2,
                model_path="CoupledRoot.Atomic",
                common_state={},
                model_state=("ACTIVE", 1),
                has_model_state=True,
            )
        },
    )
    snapshot = SystemSnapshot(
        t_now=0.0,
        root_model=root_snapshot,
        t_next_continuous=float("inf"),
        t_next_animation=float("inf"),
    )
    tree = DecisionTree()
    root_id = tree.add_root(snapshot)

    text = tree.format_path(root_id, node_formatter=SnapshotFormatter())

    assert "Model: CoupledRoot" in text
    assert "Model: CoupledRoot.Atomic" in text
    assert "model_state: <not captured>" not in text
    assert "model_state: ('ACTIVE', 1)" in text


def test_snapshot_formatter_can_be_used_by_print_path(capsys):
    from socs.tree import SnapshotFormatter

    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot(7))

    tree.print_path(root_id, node_formatter=SnapshotFormatter())

    output = capsys.readouterr().out
    assert "Node 1" in output
    assert "Simulation time: 7" in output
    assert "Model: root" in output


def test_snapshot_formatter_validates_indent():
    from socs.tree import SnapshotFormatter

    with pytest.raises(ValueError):
        SnapshotFormatter(indent=-1)
    with pytest.raises(ValueError):
        SnapshotFormatter(indent=True)
