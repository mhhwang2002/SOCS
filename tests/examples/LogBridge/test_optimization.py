import math

from examples.LogBridge.LoadPassive_LogBridge import makeModel
from examples.LogBridge.optimization import run_logbridge_optimization
from socs.decision.decision import ControlDecision


def _outcome_path(result, outcome):
    edges_by_id = {
        result.graph.get_edge_id(edge): edge
        for edge in result.graph.edges()
    }
    decisions = [
        result.graph.get_decision(edges_by_id[edge_id])
        for edge_id in outcome.edge_ids
    ]
    terminal_branch = result.get_terminal_branch(outcome.terminal_branch_id)
    if terminal_branch.decision is not None:
        decisions.append(terminal_branch.decision)
    return decisions


def test_logbridge_optimization_finds_minimum_makespan():
    result = run_logbridge_optimization(makeModel())

    assert result.optimal_outcomes
    assert result.best_cost is not None
    assert result.best_cost == 84.44444444444444
    assert result.optimal_paths()[0] == [
        ControlDecision("Load_LogBridge.L11", "?go"),
        ControlDecision("Load_LogBridge.L21", "?go"),
    ]
    assert len(result.optimal_outcomes) == 2
    assert result.node_count == 14
    assert result.outcome_count == 10
    assert result.best_cost == min(
        outcome.cost
        for outcome in result.terminal_outcomes
        if math.isfinite(outcome.cost)
    )

    optimal_paths = {
        tuple(
            (
                "NoControl" if decision.target_model is None else "ControlDecision",
                decision.target_model.rsplit(".", 1)[-1]
                if decision.target_model
                else None,
            )
            for decision in path
        )
        for path in result.optimal_paths()
    }
    assert optimal_paths == {
        (("ControlDecision", "L11"), ("ControlDecision", "L21")),
        (
            ("NoControl", None),
            ("ControlDecision", "L21"),
            ("ControlDecision", "L11"),
        ),
    }


def test_logbridge_optimization_enumerates_wait_alternatives():
    result = run_logbridge_optimization(makeModel())
    paths = [
        _outcome_path(result, outcome)
        for outcome in result.terminal_outcomes
    ]

    assert any(ControlDecision(None) in path for path in paths)
    assert len(paths) == 10


def test_incomplete_paths_are_not_considered_feasible():
    result = run_logbridge_optimization(makeModel())

    assert sum(
        outcome.cost == math.inf for outcome in result.terminal_outcomes
    ) == 4
    assert sum(
        math.isfinite(outcome.cost) for outcome in result.terminal_outcomes
    ) == 6
    assert result.optimal_outcomes


def test_optimization_report_can_include_snapshot_details(capsys):
    from examples.LogBridge.LoadPassive_LogBridge import runOptimization

    runOptimization(end_time=200.0, include_snapshot_details=True)

    output = capsys.readouterr().out
    assert "Optimal path 1:" in output
    assert "Simulation time:" in output
    assert "Simulation time bound: 200" in output
    assert "Model: Load_LogBridge" in output
    assert "time_last:" in output
    assert "model_state:" in output


def test_optimization_report_can_hide_snapshot_details(capsys):
    from examples.LogBridge.LoadPassive_LogBridge import runOptimization

    runOptimization(end_time=200.0, include_snapshot_details=False)

    output = capsys.readouterr().out
    assert "Optimal path 1:" in output
    assert "-- Edge" in output
    assert "Simulation time:" not in output


def test_completion_condition_is_separate_from_objective_value():
    from examples.LogBridge.optimization import LogBridgeMakespanEvaluator

    evaluator = LogBridgeMakespanEvaluator()
    result = run_logbridge_optimization(makeModel())

    completed_outcome = result.optimal_outcomes[0]
    completed = result.graph.get_snapshot(
        result.terminal_node_id(completed_outcome)
    )
    incomplete_outcome = next(
        outcome
        for outcome in result.terminal_outcomes
        if outcome.cost == math.inf
    )
    incomplete = result.graph.get_snapshot(
        result.terminal_node_id(incomplete_outcome)
    )

    assert evaluator.is_complete(completed)
    assert evaluator(completed) == completed.t_now
    assert not evaluator.is_complete(incomplete)
    assert evaluator(incomplete) == math.inf
