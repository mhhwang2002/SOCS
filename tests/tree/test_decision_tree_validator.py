import pytest

from socs.decision.decision import ControlDecision
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.tree import DecisionTree, DecisionTreeValidator


def make_snapshot(value):
    return SystemSnapshot(
        t_now=float(value),
        root_model=ModelSnapshot(
            model_id=1,
            model_path="Root",
            common_state={"value": value},
        ),
        t_next_continuous=float("inf"),
        t_next_animation=float("inf"),
    )


def make_tree():
    tree = DecisionTree()
    root = tree.add_root(make_snapshot(0))
    first = tree.add_child(
        root, ControlDecision("Root.Load", "?go"), make_snapshot(1)
    )
    leaf = tree.add_child(
        first, ControlDecision("Root.Load", "?stop"), make_snapshot(2)
    )
    return tree, root, first, leaf


def test_valid_tree_passes_all_structural_checks():
    tree, root, first, leaf = make_tree()

    result = DecisionTreeValidator(tree).validate()

    assert result.valid
    assert result.root_id == root
    assert result.node_count == 3
    assert result.edge_count == 2
    assert all(passed for _, passed in result.checks)
    assert result.issues == ()


def test_validation_result_formats_readable_report():
    tree, *_ = make_tree()

    report = DecisionTreeValidator(tree).validate().format()

    assert "DecisionTree Validation" in report
    assert "Root: 1" in report
    assert "Nodes: 3" in report
    assert "Edges: 2" in report
    assert "Cycle detection: PASS" in report
    assert "Result: VALID" in report


def test_empty_tree_is_invalid():
    result = DecisionTreeValidator(DecisionTree()).validate()

    assert not result.valid
    assert "ROOT_MISSING" in {issue.code for issue in result.issues}


def test_non_decision_tree_is_rejected():
    with pytest.raises(TypeError):
        DecisionTreeValidator(object())


def test_missing_snapshot_is_detected():
    tree, root, *_ = make_tree()
    del tree.V[root]["snapshot"]

    result = DecisionTreeValidator(tree).validate()

    assert not result.valid
    assert "SNAPSHOT_ACCESS" in {issue.code for issue in result.issues}


def test_unreachable_node_and_second_root_are_detected():
    tree, root, *_ = make_tree()
    from socs.graph.agraph import AGraph

    orphan_id = tree.add_child(
        root, ControlDecision("Root.Orphan", "?go"), make_snapshot(9)
    )
    AGraph.remove_edge(tree, tree.in_edges(orphan_id)[0])

    result = DecisionTreeValidator(tree).validate()
    codes = {issue.code for issue in result.issues}

    assert not result.valid
    assert "ROOT_COUNT" in codes
    assert "UNREACHABLE_NODE" in codes


def test_multiple_parent_edges_are_detected():
    tree, root, first, leaf = make_tree()
    from socs.graph.agraph import AGraph

    AGraph.add_edge(
        tree,
        first,
        leaf,
        {"decision": ControlDecision("Root.Other", "?go"), "edge_id": 99},
    )

    result = DecisionTreeValidator(tree).validate()
    codes = {issue.code for issue in result.issues}

    assert not result.valid
    assert "INVALID_PARENT_COUNT" in codes


def test_adjacency_index_corruption_is_detected():
    tree, root, *_ = make_tree()
    tree.BE.clear()

    result = DecisionTreeValidator(tree).validate()

    assert not result.valid
    assert "ADJACENCY_INDEX_MISMATCH" in {
        issue.code for issue in result.issues
    }


def test_cycle_is_detected():
    tree, root, first, leaf = make_tree()
    from socs.graph.agraph import AGraph

    AGraph.add_edge(tree, leaf, first, {"decision": ControlDecision("Root.Load", "?go"), "edge_id": 99})

    result = DecisionTreeValidator(tree).validate()
    codes = {issue.code for issue in result.issues}

    assert not result.valid
    assert "CYCLE_DETECTED" in codes


def test_duplicate_edge_id_is_detected():
    tree, root, first, leaf = make_tree()
    second_edge = tree.out_edges(first)[0]
    first_edge = tree.out_edges(root)[0]
    second_edge.attrs.edge_id = first_edge.attrs.edge_id

    result = DecisionTreeValidator(tree).validate()

    assert not result.valid
    assert "DUPLICATE_EDGE_ID" in {issue.code for issue in result.issues}


def test_validator_does_not_modify_tree():
    tree, *_ = make_tree()
    before_vertices = list(tree.V)
    before_edges = [(edge.src, edge.dst, tree.get_edge_id(edge)) for edge in tree.edges()]
    before_snapshots = [tree.get_snapshot(node_id) for node_id in tree.V]

    DecisionTreeValidator(tree).validate()

    assert list(tree.V) == before_vertices
    assert [(edge.src, edge.dst, tree.get_edge_id(edge)) for edge in tree.edges()] == before_edges
    assert [tree.get_snapshot(node_id) for node_id in tree.V] == before_snapshots



def test_missing_edge_decision_is_detected():
    tree, root, *_ = make_tree()
    edge = tree.out_edges(root)[0]
    del edge.attrs["decision"]

    result = DecisionTreeValidator(tree).validate()

    assert not result.valid
    assert "INVALID_EDGE_ATTRIBUTES" in {issue.code for issue in result.issues}
