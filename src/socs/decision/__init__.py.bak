from .decision import Decision
from .generator import DecisionGenerator
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
    "DecisionExecutor",
    "DecisionLifeCycleManager",
    "DecisionRecord",
    "DecisionStatus",
    "DecisionManager",
    "DecisionEvaluator",
    "DecisionExpansionController",
    "ExpansionResult",
    "ExpansionStatus",
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

    raise AttributeError(
        f"module {__name__!r} has no attribute {name!r}"
    )