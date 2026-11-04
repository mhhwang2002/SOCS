from .decision import Decision, ControlDecision, NoControl
from .generator import DecisionGenerator
from .selector import DecisionSelector, FIFOSelector, FixedPrioritySelector
from .executor import DecisionExecutor
from .control_executor import (
    ControlRuntimeExecutor,
    ControlSimulationDecisionExecutor,
)
from .lifecycle import (
    DecisionLifeCycleManager,
    DecisionRecord,
    DecisionStatus,
)

# Backward-compatible class aliases.
DecisionManager = DecisionLifeCycleManager
DecisionEvaluator = DecisionExecutor


__all__ = [
    "Decision",
    "ControlDecision",
    "NoControl",
    "DecisionGenerator",
    "DecisionSelector",
    "FIFOSelector",
    "FixedPrioritySelector",
    "DecisionExecutor",
    "ControlRuntimeExecutor",
    "ControlSimulationDecisionExecutor",
    "DecisionLifeCycleManager",
    "DecisionRecord",
    "DecisionStatus",
    "DecisionManager",
    "DecisionEvaluator",
    "DecisionExpansionController",
    "ExpansionResult",
    "ExpansionStatus",
    "DecisionRuntimeController",
    "DecisionRuntimeStep",
    "ExhaustiveDecisionTreeSearch",
    "ExhaustiveSearchResult",
]


def __getattr__(name):
    if name in {
        "DecisionExpansionController",
        "ExpansionResult",
        "ExpansionStatus",
    }:
        from .expansion import (
            DecisionExpansionController,
            ExpansionResult,
            ExpansionStatus,
        )

        return {
            "DecisionExpansionController": DecisionExpansionController,
            "ExpansionResult": ExpansionResult,
            "ExpansionStatus": ExpansionStatus,
        }[name]

    if name in {"ExhaustiveDecisionTreeSearch", "ExhaustiveSearchResult"}:
        from .search import ExhaustiveDecisionTreeSearch, ExhaustiveSearchResult

        return {
            "ExhaustiveDecisionTreeSearch": ExhaustiveDecisionTreeSearch,
            "ExhaustiveSearchResult": ExhaustiveSearchResult,
        }[name]

    if name in {"DecisionRuntimeController", "DecisionRuntimeStep"}:
        from .runtime import DecisionRuntimeController, DecisionRuntimeStep

        return {
            "DecisionRuntimeController": DecisionRuntimeController,
            "DecisionRuntimeStep": DecisionRuntimeStep,
        }[name]

    raise AttributeError(
        f"module {__name__!r} has no attribute {name!r}"
    )
from .path_validator import DecisionPathValidator, PathValidationResult
__all__ += ["DecisionPathValidator", "PathValidationResult"]
