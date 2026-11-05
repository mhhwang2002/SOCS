"""Run exhaustive LogBridge search and fully validate every terminal path."""

import math

from socs.optimization.path_validator import DecisionPathValidator
from socs.optimization.search_validator import SearchResultValidator
from socs.devs.SimEngine import SimEngineThread
from socs.tree import DecisionTreeValidator

from .LoadPassive_LogBridge import makeModel
from .exhaustive_search import (
    LogBridgeMakespanEvaluator,
    LogBridgePathExecutor,
    run_logbridge_exhaustive_search,
)


def main() -> int:
    """Build the DecisionTree and validate every terminal path."""
    model = makeModel()
    engine = SimEngineThread(model)
    engine.bVerbose = False

    result = run_logbridge_exhaustive_search(model, engine=engine)
    evaluator = LogBridgeMakespanEvaluator()
    executor = LogBridgePathExecutor(
        engine=engine,
        end_time=200.0,
        completion_condition=evaluator.is_complete,
    )

    def classify_terminal(snapshot):
        """Classify a terminal snapshot using the LogBridge search rules."""
        if evaluator(snapshot) is not None:
            return "COMPLETED"
        if snapshot.t_now >= 200.0:
            return "TIME_LIMIT"
        next_devs_time = snapshot.root_model.common_state["time_next"]
        if not snapshot.x_buffer and not math.isfinite(next_devs_time):
            return "DEADLOCK"
        return "INCOMPLETE"

    validator = DecisionPathValidator(
        result.tree,
        executor,
        terminal_classifier=classify_terminal,
        cost_evaluator=evaluator,
    )

    print("Decision-tree nodes:", result.node_count)
    print("Terminal paths:", result.leaf_count)
    print("Optimal makespan:", result.best_cost)

    search_validation = SearchResultValidator(result).validate()
    print("\n" + search_validation.format())
    if not search_validation.valid:
        return 1

    tree_validation = DecisionTreeValidator(result.tree).validate()
    print()
    print(tree_validation.format())
    if not tree_validation.valid:
        return 1

    print("\nPath validation:")

    all_valid = True
    for node_id in result.leaf_node_ids:
        validation = validator.validate(node_id)
        status = "PASS" if validation.valid else "FAIL"
        print(
            f"  Leaf {node_id}: {status} "
            f"({validation.validated_edges} decisions; "
            f"pre={validation.pre_decision_checks}, "
            f"post={validation.post_decision_checks})"
        )
        print(
            f"    Final state: {'MATCH' if validation.final_state_matches else 'MISMATCH'}; "
            f"termination: {validation.expected_terminal_status} / "
            f"{validation.actual_terminal_status}; "
            f"makespan: {validation.expected_makespan} / "
            f"{validation.actual_makespan}"
        )
        if not validation.valid:
            all_valid = False
            print("    Failed edge:", validation.failed_edge_id)
            print("    Error:", validation.error)

    print(
        "\nResult:",
        "ALL PATHS VALIDATED" if all_valid else "PATH VALIDATION FAILED",
    )
    return 0 if all_valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
