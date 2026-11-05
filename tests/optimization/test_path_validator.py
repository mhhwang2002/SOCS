"""Tests for DecisionPathValidator with DecisionGraph paths."""

from dataclasses import dataclass

import pytest

from socs.decision.decision import Decision
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.graph.decision_graph import DecisionGraph
from socs.optimization.path_validator import DecisionPathValidator


@dataclass(frozen=True)
class Add(Decision):
    amount: int


class AddExecutor:
    def execute(self, state, decision):
        return SystemSnapshot(
            state.t_now + decision.amount,
            ModelSnapshot(
                1,
                "root",
                {"value": state.root_model.common_state["value"] + decision.amount},
            ),
            state.t_next_continuous,
            state.t_next_animation,
            list(state.x_buffer),
        )


def make_snapshot(value, time):
    return SystemSnapshot(
        time,
        ModelSnapshot(1, "root", {"value": value}),
        float("inf"),
        float("inf"),
        [],
    )


def add_transition(graph, source_id, decision, snapshot):
    edge = graph.add_transition(source_id, decision, snapshot)
    assert edge is not None
    return graph.get_edge_id(edge), edge.dst


def test_validator_replays_valid_path():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0, 0))
    first_edge_id, child = add_transition(graph, root, Add(2), make_snapshot(2, 2))
    second_edge_id, _ = add_transition(graph, child, Add(3), make_snapshot(5, 5))

    result = DecisionPathValidator(graph, AddExecutor()).validate(
        [first_edge_id, second_edge_id]
    )

    assert result.valid
    assert result.validated_edges == 2
    assert result.failed_edge_id is None
    assert result.node_id == 3


def test_validator_reports_first_mismatch():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0, 0))
    edge_id, _ = add_transition(graph, root, Add(2), make_snapshot(99, 2))

    result = DecisionPathValidator(graph, AddExecutor()).validate([edge_id])

    assert not result.valid
    assert result.validated_edges == 0
    assert result.failed_edge_id == 1
    assert result.actual_snapshot is not None


def test_validator_rejects_unknown_edge_id():
    graph = DecisionGraph()
    graph.add_root(make_snapshot(0, 0))

    with pytest.raises(KeyError):
        DecisionPathValidator(graph, AddExecutor()).validate([99])


def test_validator_rejects_disconnected_edge_sequence():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0, 0))
    _, first_node = add_transition(graph, root, Add(2), make_snapshot(2, 2))
    second_edge_id, _ = add_transition(
        graph, first_node, Add(3), make_snapshot(5, 5)
    )

    with pytest.raises(ValueError, match="starts at node"):
        DecisionPathValidator(graph, AddExecutor()).validate([second_edge_id])


def test_validator_checks_intermediate_snapshots_and_terminal_metrics():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0, 0))
    first_edge_id, child = add_transition(graph, root, Add(2), make_snapshot(2, 2))
    second_edge_id, _ = add_transition(graph, child, Add(3), make_snapshot(5, 5))

    validator = DecisionPathValidator(
        graph,
        AddExecutor(),
        terminal_classifier=lambda snapshot: (
            "COMPLETED" if snapshot.t_now == 5 else "INCOMPLETE"
        ),
        cost_evaluator=lambda snapshot: (
            snapshot.t_now if snapshot.t_now == 5 else None
        ),
    )
    result = validator.validate([first_edge_id, second_edge_id])

    assert result.valid
    assert result.pre_decision_checks == 2
    assert result.post_decision_checks == 2
    assert result.final_state_matches is True
    assert result.expected_terminal_status == "COMPLETED"
    assert result.actual_terminal_status == "COMPLETED"
    assert result.termination_matches is True
    assert result.expected_makespan == 5
    assert result.actual_makespan == 5
    assert result.makespan_matches is True


def test_validator_checks_root_as_a_valid_zero_edge_path():
    graph = DecisionGraph()
    graph.add_root(make_snapshot(0, 0))

    result = DecisionPathValidator(
        graph,
        AddExecutor(),
        terminal_classifier=lambda snapshot: "READY",
        cost_evaluator=lambda snapshot: None,
    ).validate([])

    assert result.valid
    assert result.validated_edges == 0
    assert result.pre_decision_checks == 0
    assert result.post_decision_checks == 0
    assert result.final_state_matches is True
    assert result.termination_matches is True
    assert result.makespan_matches is True


def test_validator_can_replay_two_distinct_paths_to_the_same_node():
    graph = DecisionGraph()
    root = graph.add_root(make_snapshot(0, 0))
    left_edge_id, left_node = add_transition(
        graph, root, Add(1), make_snapshot(1, 1)
    )
    right_edge_id, right_node = add_transition(
        graph, root, Add(2), make_snapshot(2, 2)
    )
    left_to_merge_id, merged_node = add_transition(
        graph, left_node, Add(2), make_snapshot(3, 3)
    )
    right_to_merge_id, reused_node = add_transition(
        graph, right_node, Add(1), make_snapshot(3, 3)
    )

    assert merged_node == reused_node

    validator = DecisionPathValidator(graph, AddExecutor())
    left_result = validator.validate([left_edge_id, left_to_merge_id])
    right_result = validator.validate([right_edge_id, right_to_merge_id])

    assert left_result.valid
    assert right_result.valid
    assert left_result.node_id == right_result.node_id == merged_node


def test_validator_rejects_non_integer_edge_ids():
    graph = DecisionGraph()
    graph.add_root(make_snapshot(0, 0))

    with pytest.raises(TypeError, match="integer"):
        DecisionPathValidator(graph, AddExecutor()).validate(["1"])
