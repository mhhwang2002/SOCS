from copy import deepcopy

import pytest

from socs.decision.decision import Decision
from socs.optimization.optimizer import Optimizer, OptimizationResult
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.tree import DecisionTree


class AddDecision(Decision):
    def __init__(self, amount):
        self.amount = amount


def make_generator(decisions):
    def generate(state):
        if state.root_model.model_state == 0:
            yield from decisions
    return generate


class CounterExecutor:
    def execute(self, state, decision):
        result = deepcopy(state)
        result.root_model.model_state += decision.amount
        return result


class NoOpExecutor:
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


def test_optimizer_expands_every_decision_and_selects_best():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())
    decisions = [AddDecision(10), AddDecision(20)]
    generator = make_generator(decisions)
    executor = CounterExecutor()

    optimizer = Optimizer(
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


def test_optimizer_treats_empty_decision_set_as_leaf():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())

    result = Optimizer(
        tree=tree,
        generator=make_generator([]),
        executor=CounterExecutor(),
        objective=lambda snapshot: snapshot.root_model.model_state,
    ).run(root_id)

    assert result.leaf_node_ids == (root_id,)
    assert result.costs == {root_id: 0}
    assert result.best_node_id == root_id


def test_optimizer_stops_an_unchanged_branch():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())

    result = Optimizer(
        tree=tree,
        generator=make_generator([NoOpDecision()]),
        executor=NoOpExecutor(),
        objective=lambda snapshot: snapshot.t_now,
    ).run(root_id)

    assert result.node_count == 2
    assert result.leaf_count == 1
    assert result.leaf_node_ids == (2,)


def test_optimizer_requires_tree_root():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())
    child_id = tree.add_child(root_id, NoOpDecision(), make_snapshot())

    optimizer = Optimizer(
        tree=tree,
        generator=make_generator([]),
        executor=NoOpExecutor(),
        objective=lambda snapshot: 0,
    )

    with pytest.raises(ValueError, match="DecisionTree root"):
        optimizer.run(child_id)

def test_optimizer_excludes_infeasible_leaves():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())
    decisions = [AddDecision(10), AddDecision(20)]

    def objective(snapshot):
        value = snapshot.root_model.model_state
        return float("inf") if value == 10 else value

    result = Optimizer(
        tree=tree,
        generator=make_generator(decisions),
        executor=CounterExecutor(),
        objective=objective,
    ).run(root_id)

    assert result.leaf_count == 2
    assert result.costs == {
        2: float("inf"),
        3: 20,
    }
    assert result.best_cost == 20
    assert result.best_node_id == 3
    assert result.optimal_node_ids == (3,)


def test_optimizer_returns_no_solution_when_all_leaves_are_infeasible():
    tree = DecisionTree()
    root_id = tree.add_root(make_snapshot())

    result = Optimizer(
        tree=tree,
        generator=make_generator([AddDecision(10), AddDecision(20)]),
        executor=CounterExecutor(),
        objective=lambda snapshot: float("inf"),
    ).run(root_id)

    assert result.leaf_count == 2
    assert all(cost == float("inf") for cost in result.costs.values())
    assert result.best_cost is None
    assert result.best_node_id is None
    assert result.optimal_node_ids == ()
    assert result.best_path() == []
    assert result.optimal_paths() == []