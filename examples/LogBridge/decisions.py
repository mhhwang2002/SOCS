from dataclasses import dataclass

from socs.decision.decision import Decision


@dataclass(frozen=True)
class EnterBridgeDecision(Decision):
    """Authorize one waiting Load to enter the LogBridge."""

    load_name: str
