"""Decision types used by simulation-based optimization."""

from dataclasses import dataclass
from typing import Any


class Decision:
    """Base class for optimization decisions."""

    pass


@dataclass(frozen=True)
class ControlDecision(Decision):
    """Represent an external input to a DEVS model.

    ``target_model`` is the model's hierarchical path. ``xevent`` may be an
    existing PortValue, an input-port name, or a ``(port_name, value)`` pair.
    A ``None`` target represents an explicit no-control alternative.
    """

    target_model: str | None
    xevent: Any = None

    def __post_init__(self) -> None:
        if self.target_model is None:
            if self.xevent is not None:
                raise ValueError("xevent must be None when target_model is None.")
            return
        if not isinstance(self.target_model, str) or not self.target_model.strip():
            raise ValueError("target_model must be a non-empty model path or None.")
        if self.xevent is None:
            raise ValueError("xevent must be provided when target_model is set.")

    def __str__(self) -> str:
        if self.target_model is None:
            return "NoControl"
        return f"Control({self.target_model}, {self.xevent})"
