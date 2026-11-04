"""Backward-compatible LogBridge branch executor alias."""

from socs.decision.control_executor import ControlSimulationDecisionExecutor


class LogBridgeDecisionExecutor(ControlSimulationDecisionExecutor):
    """Compatibility name for the generic control branch executor."""

    pass
