"""Backward-compatible LogBridge runtime executor alias."""

from socs.decision.control_executor import ControlRuntimeExecutor


class LogBridgeRuntimeExecutor(ControlRuntimeExecutor):
    """Compatibility name for the generic DEVS control runtime executor."""

    pass
