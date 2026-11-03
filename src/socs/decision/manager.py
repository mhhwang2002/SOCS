"""Backward-compatible imports for the decision lifecycle manager."""

from .lifecycle import (
    DecisionLifeCycleManager,
    DecisionRecord,
    DecisionStatus,
)

DecisionManager = DecisionLifeCycleManager

__all__ = [
    "DecisionLifeCycleManager",
    "DecisionManager",
    "DecisionRecord",
    "DecisionStatus",
]
