"""Backward-compatible imports for simulation-backed decision execution."""

from .simulation_executor import SimulationDecisionExecutor

SimulationDecisionEvaluator = SimulationDecisionExecutor

__all__ = ["SimulationDecisionExecutor", "SimulationDecisionEvaluator"]
