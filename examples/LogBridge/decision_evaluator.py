"""Backward-compatible imports for the LogBridge decision executor."""

from .decision_executor import LogBridgeDecisionExecutor

LogBridgeDecisionEvaluator = LogBridgeDecisionExecutor

__all__ = ["LogBridgeDecisionExecutor", "LogBridgeDecisionEvaluator"]
