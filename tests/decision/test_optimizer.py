from copy import deepcopy

import pytest

from socs.decision.decision import Decision
from socs.decision.executor import DecisionExecutor
from socs.decision.generator import DecisionGenerator
from socs.decision.optimizer import ExhaustiveOptimizer, OptimizationResult
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.tree import DecisionTree


class AddDecision(Decision):
    def __init__(self, amount):
        self.amount = amount


class AddDecisionGenerator(DecisionGenerator):
    def __init__(self, decisions):
        self.decisions = decisions

    def generate(self, state):
        if state.root_model.model_state == 0:
            yield from self.decisions


class CounterExecutor(DecisionExecutor):
    def execute(self, state, decision):
        result = deepcopy(state)
        result.root_model.model_state += decision.amount
        return result


class NoOpExecutor(DecisionExecutor):
    def execute(self, state, decision):
        return state


class NoOpDecision(Decision):
    pass


def make_snapshot(value=0):
    model = ModelSnapshot(
        model_id=1,
        model_path="counter",
        common_state={},
        model_state=value,
        has_model_state=True,
    )
    return SystemSnapshot(
        t_now=0.0,
        root_model=model,
        t_next_continuous=float("inf"),
        t_next_animation=float("inf"),
    )


def test_exhaustive_optimizer_expands_every_decision_and_selects_best():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())
    decisions = [AddDecision(10), AddDecision(20)]
    generator = AddDecisionGenerator(decisions)
    executor = CounterExecutor()

    optimizer = ExhaustiveOptimizer(
        tree=tree,
        generator=generator,
        executor=executor,
        objective=lambda snapshot: snapshot.root_model.model_state,
    )
    result = optimizer.run(root_id)

    assert isinstance(result, OptimizationResult)
    assert result.node_count == 3
    assert result.leaf_count == 2
    assert result.best_cost == 10
    assert result.tree.get_path(result.best_node_id) == [decisions[0]]
    assert result.optimal_paths() == [[decisions[0]]]


def test_exhaustive_optimizer_treats_empty_decision_set_as_leaf():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())

    result = ExhaustiveOptimizer(
        tree=tree,
        generator=AddDecisionGenerator([]),
        executor=CounterExecutor(),
        objective=lambda snapshot: snapshot.root_model.model_state,
    ).run(root_id)

    assert result.leaf_node_ids == (root_id,)
    assert result.costs == {root_id: 0}
    assert result.best_node_id == root_id


def test_exhaustive_optimizer_stops_an_unchanged_branch():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())

    result = ExhaustiveOptimizer(
        tree=tree,
        generator=AddDecisionGenerator([NoOpDecision()]),
        executor=NoOpExecutor(),
        objective=lambda snapshot: snapshot.t_now,
    ).run(root_id)

    assert result.node_count == 2
    assert result.leaf_count == 1
    assert result.leaf_node_ids == (2,)


def test_exhaustive_optimizer_requires_tree_root():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())
    child_id = tree.add_child(root_id, NoOpDecision(), make_snapshot())

    optimizer = ExhaustiveOptimizer(
        tree=tree,
        generator=AddDecisionGenerator([]),
        executor=NoOpExecutor(),
        objective=lambda snapshot: 0,
    )

    with pytest.raises(ValueError, match="DecisionTree root"):
        optimizer.run(child_id)
