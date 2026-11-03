from .decision import Decision
from .generator import DecisionGenerator
from .evaluator import DecisionEvaluator
from .manager import (
    DecisionManager,
    DecisionRecord,
    DecisionStatus,
)

__all__ = [
    "Decision",
    "DecisionGenerator",
    "DecisionEvaluator",
    "DecisionManager",
    "DecisionRecord",
    "DecisionStatus",
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