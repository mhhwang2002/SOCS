"""In-memory snapshot data structures for SOCS models."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ModelSnapshot:
    """Store the captured state of one model and its descendants."""

    model_id: int
    model_path: str
    common_state: dict[str, Any]
    model_state: Any = None
    has_model_state: bool = False
    children: dict[str, "ModelSnapshot"] = field(default_factory=dict)


@dataclass
class SystemSnapshot:
    """Store the engine and root-model state for an in-memory checkpoint."""

    t_now: float
    root_model: ModelSnapshot
    t_next_continuous: float
    t_next_animation: float
    x_buffer: list[tuple[int | None, Any]] = field(default_factory=list)
