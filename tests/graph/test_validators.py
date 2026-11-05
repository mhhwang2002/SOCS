"""Tests for AGraph and DecisionGraph validators."""

from dataclasses import dataclass

import pytest

from socs.decision.decision import Decision
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.graph.agraph import AGraph
from socs.graph.decision_graph import DecisionGraph
from socs.graph.validators import AGraphValidator, DecisionGraphValidator


@dataclass(frozen=True)
class AddDecision(Decision):
    amount: int


def make_snapshot(value: int) -> SystemSnapshot:
    return SystemSnapshot(
        t_now=float(value),
        root_model=ModelSnapshot(
            model_id=1,
            model_path="root",
            common_state={"value": value},
            model_state=value,
            has_model_state=True,
        ),
        t_next_continuous=float("inf"),
        t_next_animation=float("inf"),
    )


def make_valid_decision_graph() -> DecisionGraph:
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0))
    graph.add_transition(root, AddDecision(1), make_snapshot(1))
    return graph


def test_agraph_accepts_parallel_edges_and_self_loops():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)
    graph.add_edge(1, 2)
    graph.add_edge(1, 2)
    graph.add_edge(2, 2)

    result = AGraphValidator(graph).validate()

    assert result.valid


def test_agraph_detects_missing_reverse_index_entry():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)
    edge = graph.add_edge(1, 2)
    graph.BE[2].remove(edge)

    result = AGraphValidator(graph).validate()

    assert not result.valid
    assert any(issue.code == "EDGE_INDEX_MISMATCH" for issue in result.issues)


def test_agraph_detects_wrong_adjacency_key():
    graph = AGraph()
    graph.add_vertex(1)
    graph.add_vertex(2)
    edge = graph.add_edge(1, 2)
    graph.E[1].remove(edge)
    graph.E[2] = [edge]

    result = AGraphValidator(graph).validate()

    assert not result.valid
    assert any(issue.code == "SOURCE_INDEX_MISMATCH" for issue in result.issues)


def test_decision_graph_accepts_valid_graph():
    result = DecisionGraphValidator(make_valid_decision_graph()).validate()

    assert result.valid, result.format()


def test_decision_graph_detects_missing_root():
    graph = DecisionGraph()

    result = DecisionGraphValidator(graph).validate()

    assert not result.valid
    assert any(issue.code == "INVALID_ROOT" for issue in result.issues)


def test_decision_graph_detects_duplicate_snapshots():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0))
    graph.add_or_get_node(make_snapshot(1))
    # Bypass the public API to simulate a corrupted graph.
    AGraph.add_vertex(graph, 99, {"snapshot": make_snapshot(0)})

    result = DecisionGraphValidator(graph).validate()

    assert not result.valid
    assert any(issue.code == "DUPLICATE_SNAPSHOT" for issue in result.issues)


def test_decision_graph_detects_unreachable_node():
    graph = DecisionGraph()
    graph.add_root(make_snapshot(0))
    AGraph.add_vertex(graph, 99, {"snapshot": make_snapshot(99)})

    result = DecisionGraphValidator(graph).validate()

    assert not result.valid
    assert any(issue.code == "UNREACHABLE_NODE" for issue in result.issues)


def test_decision_graph_detects_no_op_edge():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0))
    AGraph.add_edge(
        graph,
        root,
        root,
        {"decision": AddDecision(0), "edge_id": 1},
    )

    result = DecisionGraphValidator(graph).validate()

    assert not result.valid
    assert any(issue.code == "NO_OP_EDGE" for issue in result.issues)


def test_decision_graph_detects_duplicate_edge_ids():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0))
    graph.add_transition(root, AddDecision(1), make_snapshot(1))
    graph.add_transition(root, AddDecision(2), make_snapshot(2))
    edges = graph.out_edges(root)
    edges[1].attrs.edge_id = edges[0].attrs.edge_id

    result = DecisionGraphValidator(graph).validate()

    assert not result.valid
    assert any(issue.code == "DUPLICATE_EDGE_ID" for issue in result.issues)


def test_decision_graph_allows_cycles():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0))
    child_edge = graph.add_transition(root, AddDecision(1), make_snapshot(1))
    child_id = child_edge.dst
    graph.add_transition(child_id, AddDecision(-1), make_snapshot(0))

    result = DecisionGraphValidator(graph).validate()

    assert result.valid, result.format()
