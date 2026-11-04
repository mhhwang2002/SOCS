"""Run exhaustive LogBridge search and replay every terminal Decision path."""

from socs.decision.path_validator import DecisionPathValidator
from socs.devs.SimEngine import SimEngineThread

from .LoadPassive_LogBridge import makeModel
from .decision_generator import LogBridgeDecisionGenerator
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
    executor = LogBridgePathExecutor(
        engine=engine,
        end_time=200.0,
        generator=LogBridgeDecisionGenerator(),
        completion_evaluator=LogBridgeMakespanEvaluator(),
    )
    validator = DecisionPathValidator(result.tree, executor)

    print("Decision-tree nodes:", result.node_count)
    print("Terminal paths:", result.leaf_count)
    print("Optimal makespan:", result.best_cost)
    print("\nPath validation:")

    all_valid = True
    for node_id in result.leaf_node_ids:
        validation = validator.validate(node_id)
        status = "PASS" if validation.valid else "FAIL"
        print(
            f"  Leaf {node_id}: {status} "
            f"({validation.validated_edges} edges)"
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
