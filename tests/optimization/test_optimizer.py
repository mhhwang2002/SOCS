from copy import deepcopy

import pytest

from socs.decision.decision import Decision
from socs.optimization.optimizer import (
    Optimizer,
    OptimizationResult,
    TerminalBranchKind,
    TerminalOutcome,
)
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.graph.decision_graph import DecisionGraph


class AddDecision(Decision):
    def __init__(self, amount):
        self.amount = amount


class NoOpDecision(Decision):
    pass


class CounterExecutor:
    def execute(self, state, decision):
        result = deepcopy(state)
        result.root_model.model_state += decision.amount
        return result


class NoOpExecutor:
    def execute(self, state, decision):
        return state


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


def test_optimizer_expands_decisions_and_selects_best_outcome():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    decisions = [AddDecision(10), AddDecision(20)]

    def generator(snapshot):
        if snapshot.root_model.model_state == 0:
            yield from decisions

    result = Optimizer(
        graph,
        generator,
        CounterExecutor(),
        lambda snapshot: snapshot.root_model.model_state,
    ).run(root_id)

    assert isinstance(result, OptimizationResult)
    assert result.node_count == 3
    assert result.outcome_count == 2
    assert result.best_cost == 10
    assert len(result.optimal_outcomes) == 1
    assert result.optimal_paths() == [[decisions[0]]]
    assert result.terminal_outcomes[0].edge_ids == (1,)


def test_empty_generator_creates_terminal_outcome():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())

    result = Optimizer(
        graph,
        lambda snapshot: (),
        CounterExecutor(),
        lambda snapshot: snapshot.root_model.model_state,
    ).run(root_id)

    assert result.outcome_count == 1
    branch = result.terminal_branches[0]
    assert branch.kind is TerminalBranchKind.EMPTY_GENERATOR
    assert result.terminal_outcomes == (
        TerminalOutcome(
            edge_ids=(), terminal_branch_id=branch.branch_id, cost=0.0
        ),
    )
    assert result.optimal_paths() == [[]]
    assert result.terminal_node_id(result.terminal_outcomes[0]) == root_id


def test_noop_decision_creates_outcome_without_graph_edge_or_node():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    decision = NoOpDecision()

    result = Optimizer(
        graph,
        lambda snapshot: [decision],
        NoOpExecutor(),
        lambda snapshot: 7,
    ).run(root_id)

    assert result.node_count == 1
    assert result.graph.E == {}
    assert result.outcome_count == 1
    outcome = result.terminal_outcomes[0]
    assert outcome.edge_ids == ()
    branch = result.get_terminal_branch(outcome.terminal_branch_id)
    assert branch.kind is TerminalBranchKind.NO_OP_DECISION
    assert branch.decision is decision
    assert outcome.cost == 7
    assert result.optimal_paths() == [[decision]]


def test_merged_node_preserves_distinct_path_outcomes():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    plus_one = AddDecision(1)
    plus_two = AddDecision(2)

    def generator(snapshot):
        value = snapshot.root_model.model_state
        if value == 0:
            yield plus_one
            yield plus_two
        elif value == 1:
            yield plus_two
        elif value == 2:
            yield plus_one

    result = Optimizer(
        graph,
        generator,
        CounterExecutor(),
        lambda snapshot: snapshot.root_model.model_state,
    ).run(root_id)

    assert result.node_count == 4
    assert result.outcome_count == 2
    assert {outcome.edge_ids for outcome in result.terminal_outcomes} == {
        (1, 3),
        (2, 4),
    }
    assert all(outcome.cost == 3 for outcome in result.terminal_outcomes)
    assert len(result.optimal_outcomes) == 2


def test_multiple_noop_decisions_create_separate_outcomes():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    decisions = [NoOpDecision(), NoOpDecision()]

    result = Optimizer(
        graph,
        lambda snapshot: decisions,
        NoOpExecutor(),
        lambda snapshot: 1,
    ).run(root_id)

    assert result.outcome_count == 2
    assert [
        result.get_terminal_branch(item.terminal_branch_id).decision
        for item in result.terminal_outcomes
    ] == decisions
    assert len({item.terminal_branch_id for item in result.terminal_outcomes}) == 2


def test_infeasible_outcomes_are_excluded_from_optimum():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    decisions = [AddDecision(10), AddDecision(20)]

    def generator(snapshot):
        if snapshot.root_model.model_state == 0:
            yield from decisions

    def objective(snapshot):
        value = snapshot.root_model.model_state
        return math.inf if value == 10 else value

    import math

    result = Optimizer(graph, generator, CounterExecutor(), objective).run(root_id)

    assert sorted(outcome.cost for outcome in result.terminal_outcomes) == [20.0, math.inf]
    assert result.best_cost == 20
    assert len(result.optimal_outcomes) == 1
    assert result.optimal_paths() == [[decisions[1]]]


def test_all_infeasible_outcomes_have_no_optimum():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())

    def generator(snapshot):
        if snapshot.root_model.model_state == 0:
            yield AddDecision(1)
            yield AddDecision(2)

    result = Optimizer(
        graph,
        generator,
        CounterExecutor(),
        lambda snapshot: float("inf"),
    ).run(root_id)

    assert result.outcome_count == 2
    assert result.best_cost is None
    assert result.optimal_outcomes == ()
    assert result.optimal_paths() == []


def test_optimizer_requires_graph_root():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    other_id, _ = graph.add_or_get_node(make_snapshot(1))
    optimizer = Optimizer(graph, lambda snapshot: (), NoOpExecutor(), lambda snapshot: 0)

    with pytest.raises(ValueError, match="DecisionGraph root"):
        optimizer.run(other_id)


def test_objective_rejects_nan_and_negative_infinity():
    for invalid_cost in (float("nan"), -float("inf")):
        graph = DecisionGraph()
        root_id = graph.add_root(make_snapshot())
        optimizer = Optimizer(
            graph,
            lambda snapshot: (),
            NoOpExecutor(),
            lambda snapshot, cost=invalid_cost: cost,
        )
        with pytest.raises(ValueError, match="valid cost"):
            optimizer.run(root_id)
