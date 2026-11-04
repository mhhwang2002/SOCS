from socs.decision.decision import ControlDecision, NoControl


class EnterBridgeDecision(ControlDecision):
    """Backward-compatible factory for a LogBridge entry control."""

    def __init__(self, load_name: str):
        super().__init__(f"Load_LogBridge.{load_name}", "?go")
        object.__setattr__(self, "load_name", load_name)


class WaitDecision(NoControl):
    """Backward-compatible name for an explicit no-control decision."""

    pass
