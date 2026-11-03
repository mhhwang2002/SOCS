"""Backward-compatible imports for decision execution."""

from .executor import DecisionExecutor

DecisionEvaluator = DecisionExecutor

__all__ = ["DecisionExecutor", "DecisionEvaluator"]
