"""Tests for exhaustive search-result validation."""

from dataclasses import replace

from examples.LogBridge.LoadPassive_LogBridge import makeModel
from examples.LogBridge.exhaustive_search import run_logbridge_exhaustive_search
from socs.optimization.search_validator import SearchResultValidator


def make_result():
    return run_logbridge_exhaustive_search(makeModel())


def test_search_result_validator_accepts_valid_logbridge_result():
    result = make_result()

    validation = SearchResultValidator(result).validate()

    assert validation.valid
    assert validation.tree_node_count == 18
    assert validation.tree_leaf_count == 10
    assert validation.reported_leaf_count == 10
    assert validation.feasible_leaf_count == 6
    assert validation.best_cost == 84.44444444444444
    assert "Result: VALID" in validation.format()


def test_search_result_validator_detects_missing_leaf_cost():
    result = make_result()
    costs = dict(result.costs)
    costs.pop(result.leaf_node_ids[0])
    corrupted = replace(result, costs=costs)

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(issue.code == "MISSING_COST" for issue in validation.issues)
    assert not dict(validation.checks)["Cost coverage"]


def test_search_result_validator_detects_wrong_best_cost():
    result = make_result()
    corrupted = replace(result, best_cost=result.best_cost + 1.0)

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(issue.code == "BEST_COST_MISMATCH" for issue in validation.issues)


def test_search_result_validator_detects_incomplete_leaf_inventory():
    result = make_result()
    corrupted = replace(result, leaf_node_ids=result.leaf_node_ids[:-1])

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(issue.code == "MISSING_TERMINAL" for issue in validation.issues)


def test_search_result_validator_detects_wrong_optimal_nodes():
    result = make_result()
    corrupted = replace(result, optimal_node_ids=(result.leaf_node_ids[0],))

    validation = SearchResultValidator(corrupted).validate()

    assert not validation.valid
    assert any(issue.code == "OPTIMAL_NODES_MISMATCH" for issue in validation.issues)
