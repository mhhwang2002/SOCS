from .decision import Decision
from .generator import DecisionGenerator
from .selector import DecisionSelector, FIFOSelector, FixedPrioritySelector
from .executor import DecisionExecutor
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
    "DecisionGenerator",
    "DecisionSelector",
    "FIFOSelector",
    "FixedPrioritySelector",
    "DecisionExecutor",
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

    if name in {"DecisionRuntimeController", "DecisionRuntimeStep"}:
        from .runtime import DecisionRuntimeController, DecisionRuntimeStep

        return {
            "DecisionRuntimeController": DecisionRuntimeController,
            "DecisionRuntimeStep": DecisionRuntimeStep,
        }[name]

    raise AttributeError(
        f"module {__name__!r} has no attribute {name!r}"
    )