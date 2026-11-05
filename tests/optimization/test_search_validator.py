"""Tests for optimization-result validation."""

from dataclasses import replace
import math

from socs.decision.decision import Decision
from socs.optimization.optimizer import Optimizer, TerminalBranchKind
from socs.optimization.search_validator import SearchResultValidator
from socs.devs.Snapshot import ModelSnapshot, SystemSnapshot
from socs.graph.decision_graph import DecisionGraph


class AddDecision(Decision):
    def __init__(self, amount):
        self.amount = amount


class CounterExecutor:
    def execute(self, state, decision):
        from copy import deepcopy

        result = deepcopy(state)
        result.root_model.model_state += decision.amount
        return result


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
        t_next_continuous=math.inf,
        t_next_animation=math.inf,
    )


def make_result():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    decisions = [AddDecision(10), AddDecision(20)]

    def generator(snapshot):
        if snapshot.root_model.model_state == 0:
            yield from decisions

    return Optimizer(
        graph,
        generator,
        CounterExecutor(),
        lambda snapshot: snapshot.root_model.model_state,
    ).run(root_id)


def test_search_result_validator_accepts_valid_result():
    result = make_result()

    validation = SearchResultValidator(result).validate()

    assert validation.valid
    assert validation.graph_node_count == 3
    assert validation.graph_edge_count == 2
    assert validation.terminal_outcome_count == 2
    assert validation.feasible_outcome_count == 2
    assert validation.best_cost == 10
    assert "Result: VALID" in validation.format()


def test_search_result_validator_detects_wrong_best_cost():
    result = make_result()
    corrupted = replace(result, best_cost=result.best_cost + 1.0)

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(issue.code == "BEST_COST_MISMATCH" for issue in validation.issues)


def test_search_result_validator_detects_missing_terminal_path():
    result = make_result()
    corrupted = replace(
        result,
        terminal_outcomes=result.terminal_outcomes[:-1],
        optimal_outcomes=(),
    )

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(
        issue.code == "MISSING_TERMINAL_OUTCOME" for issue in validation.issues
    )


def test_search_result_validator_detects_wrong_optimal_outcomes():
    result = make_result()
    corrupted = replace(result, optimal_outcomes=(result.terminal_outcomes[1],))

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(
        issue.code == "OPTIMAL_OUTCOMES_MISMATCH" for issue in validation.issues
    )


def test_search_result_validator_detects_unknown_edge_id():
    result = make_result()
    bad_outcome = replace(result.terminal_outcomes[0], edge_ids=(999,))
    corrupted = replace(
        result,
        terminal_outcomes=(bad_outcome, result.terminal_outcomes[1]),
    )

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(issue.code == "UNKNOWN_EDGE" for issue in validation.issues)


def test_search_result_validator_detects_duplicate_outcomes():
    result = make_result()
    corrupted = replace(
        result,
        terminal_outcomes=(
            result.terminal_outcomes[0],
            result.terminal_outcomes[0],
        ),
    )

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(issue.code == "DUPLICATE_OUTCOME" for issue in validation.issues)


def test_search_result_validator_detects_invalid_cost():
    result = make_result()
    bad_outcome = replace(result.terminal_outcomes[0], cost=float("nan"))
    corrupted = replace(
        result,
        terminal_outcomes=(bad_outcome, result.terminal_outcomes[1]),
    )

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(issue.code == "INVALID_COST" for issue in validation.issues)


def test_search_result_validator_accepts_multiple_paths_to_merged_node():
    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    plus_one = AddDecision(1)
    plus_two = AddDecision(2)

    def generator(snapshot):
        value = snapshot.root_model.model_state
        if value == 0:
            yield plus_one
            yield plus_two
        elif value in (1, 2):
            yield AddDecision(3 - value)

    result = Optimizer(
        graph,
        generator,
        CounterExecutor(),
        lambda snapshot: snapshot.root_model.model_state,
    ).run(root_id)

    validation = SearchResultValidator(result).validate()

    assert validation.valid
    assert {outcome.edge_ids for outcome in result.terminal_outcomes} == {
        (1, 3),
        (2, 4),
    }


def test_search_result_validator_accepts_root_noop_terminal_outcome():
    class NoOpDecision(Decision):
        pass

    class NoOpExecutor:
        def execute(self, state, decision):
            return state

    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    decision = NoOpDecision()
    result = Optimizer(
        graph,
        lambda snapshot: [decision],
        NoOpExecutor(),
        lambda snapshot: 5,
    ).run(root_id)

    validation = SearchResultValidator(result).validate()

    assert validation.valid
    assert validation.terminal_outcome_count == 1
    branch = result.get_terminal_branch(
        result.terminal_outcomes[0].terminal_branch_id
    )
    assert branch.kind is TerminalBranchKind.NO_OP_DECISION
    assert branch.decision is decision


def test_search_result_validator_detects_missing_noop_outcome():
    class NoOpDecision(Decision):
        pass

    class MixedExecutor:
        def execute(self, state, decision):
            if isinstance(decision, NoOpDecision):
                return state
            return CounterExecutor().execute(state, decision)

    graph = DecisionGraph()
    root_id = graph.add_root(make_snapshot())
    noop = NoOpDecision()
    change = AddDecision(1)
    result = Optimizer(
        graph,
        lambda snapshot: [change, noop] if snapshot.root_model.model_state == 0 else (),
        MixedExecutor(),
        lambda snapshot: snapshot.root_model.model_state,
    ).run(root_id)
    noop_branch = next(
        branch
        for branch in result.terminal_branches
        if branch.kind is TerminalBranchKind.NO_OP_DECISION
    )
    corrupted = replace(
        result,
        terminal_outcomes=tuple(
            outcome
            for outcome in result.terminal_outcomes
            if outcome.terminal_branch_id != noop_branch.branch_id
        ),
        optimal_outcomes=(),
    )

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(
        issue.code == "MISSING_TERMINAL_OUTCOME"
        and issue.node_id == root_id
        for issue in validation.issues
    )


def test_search_result_validator_detects_invalid_terminal_branch_reference():
    result = make_result()
    bad_outcome = replace(
        result.terminal_outcomes[0], terminal_branch_id=999
    )
    corrupted = replace(
        result,
        terminal_outcomes=(bad_outcome, result.terminal_outcomes[1]),
        optimal_outcomes=(),
    )

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(
        issue.code == "UNKNOWN_TERMINAL_BRANCH" for issue in validation.issues
    )
