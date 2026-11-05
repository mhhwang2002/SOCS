from examples.LogBridge.LoadPassive_LogBridge import makeModel
from socs.decision.decision import ControlDecision
from examples.LogBridge.optimization import run_logbridge_optimization
import math

def test_logbridge_optimization_finds_minimum_makespan():
    result = run_logbridge_optimization(makeModel())

    assert result.optimal_node_ids
    assert result.best_cost is not None
    assert result.best_cost == 84.44444444444444
    assert result.optimal_paths()[0] == [
        ControlDecision("Load_LogBridge.L11", "?go"),
        ControlDecision("Load_LogBridge.L21", "?go"),
    ]
    assert len(result.optimal_node_ids) == 2
    assert result.node_count == 18
    assert result.leaf_count == 10
    assert result.best_cost == min(
        cost for cost in result.costs.values() if cost != math.inf
    )
    optimal_paths = {
        tuple(
            ("NoControl" if decision.target_model is None else "ControlDecision", decision.target_model.rsplit(".", 1)[-1] if decision.target_model else None)
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
        result.tree.get_path(node_id)
        for node_id in result.leaf_node_ids
    ]

    assert any(ControlDecision(None) in path for path in paths)
    assert len(paths) == 10


def test_incomplete_paths_are_not_considered_feasible():
    result = run_logbridge_optimization(makeModel())

    assert sum(cost == math.inf for cost in result.costs.values()) == 4
    assert sum(math.isfinite(cost) for cost in result.costs.values()) == 6
    assert result.optimal_node_ids


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

    completed = result.tree.get_snapshot(result.optimal_node_ids[0])
    incomplete = result.tree.get_snapshot(
        next(node_id for node_id, cost in result.costs.items() if cost == math.inf)
    )

    assert evaluator.is_complete(completed)
    assert evaluator(completed) == completed.t_now
    assert not evaluator.is_complete(incomplete)
    assert evaluator(incomplete) == math.inf
