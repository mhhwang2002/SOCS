from dataclasses import dataclass

import pytest

from socs.decision.decision import Decision
from socs.decision.executor import DecisionExecutor
from socs.decision.path_validator import DecisionPathValidator
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.tree import DecisionTree


@dataclass(frozen=True)
class Add(Decision):
    amount: int


class AddExecutor(DecisionExecutor):
    def execute(self, state, decision):
        return SystemSnapshot(
            state.t_now + decision.amount,
            ModelSnapshot(1, "root", {"value": state.root_model.common_state["value"] + decision.amount}),
            state.t_next_continuous,
            state.t_next_animation,
            list(state.x_buffer),
        )


def make_snapshot(value, time):
    return SystemSnapshot(time, ModelSnapshot(1, "root", {"value": value}), float("inf"), float("inf"), [])


def test_validator_replays_valid_path():
    tree = DecisionTree()
    root = tree.add_root(make_snapshot(0, 0))
    child = tree.add_child(root, Add(2), make_snapshot(2, 2))
    leaf = tree.add_child(child, Add(3), make_snapshot(5, 5))

    result = DecisionPathValidator(tree, AddExecutor()).validate(leaf)

    assert result.valid
    assert result.validated_edges == 2
    assert result.failed_edge_id is None


def test_validator_reports_first_mismatch():
    tree = DecisionTree()
    root = tree.add_root(make_snapshot(0, 0))
    leaf = tree.add_child(root, Add(2), make_snapshot(99, 2))

    result = DecisionPathValidator(tree, AddExecutor()).validate(leaf)

    assert not result.valid
    assert result.validated_edges == 0
    assert result.failed_edge_id == 1
    assert result.actual_snapshot is not None


def test_validator_rejects_unknown_node():
    tree = DecisionTree()
    tree.add_root(make_snapshot(0, 0))
    validator = DecisionPathValidator(tree, AddExecutor())
    with pytest.raises(KeyError):
        validator.validate(99)


def test_validator_checks_pre_and_post_decision_snapshots_and_terminal_metrics():
    tree = DecisionTree()
    root = tree.add_root(make_snapshot(0, 0))
    child = tree.add_child(root, Add(2), make_snapshot(2, 2))
    leaf = tree.add_child(child, Add(3), make_snapshot(5, 5))

    validator = DecisionPathValidator(
        tree,
        AddExecutor(),
        terminal_classifier=lambda snapshot: "COMPLETED" if snapshot.t_now == 5 else "INCOMPLETE",
        cost_evaluator=lambda snapshot: snapshot.t_now if snapshot.t_now == 5 else None,
    )

    result = validator.validate(leaf)

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
    tree = DecisionTree()
    root = tree.add_root(make_snapshot(0, 0))

    result = DecisionPathValidator(
        tree,
        AddExecutor(),
        terminal_classifier=lambda snapshot: "READY",
        cost_evaluator=lambda snapshot: None,
    ).validate(root)

    assert result.valid
    assert result.validated_edges == 0
    assert result.pre_decision_checks == 0
    assert result.post_decision_checks == 0
    assert result.final_state_matches is True
    assert result.termination_matches is True
    assert result.makespan_matches is True
