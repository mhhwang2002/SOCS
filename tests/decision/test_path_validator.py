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
