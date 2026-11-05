"""In-memory snapshot data structures for SOCS models."""

from dataclasses import dataclass, field
from typing import Any


@dataclass(eq=False)
class ModelSnapshot:
    """Store the captured state of one model and its descendants."""

    model_id: int
    model_path: str
    common_state: dict[str, Any]
    model_state: Any = None
    has_model_state: bool = False
    children: dict[str, "ModelSnapshot"] = field(default_factory=dict)

    def __eq__(self, other):
        if not isinstance(other, ModelSnapshot):
            return NotImplemented

        if self.model_id != other.model_id:
            return False

        if self.model_path != other.model_path:
            return False

        if self.has_model_state != other.has_model_state:
            return False

        if self.common_state != other.common_state:
            return False

        if self.model_state != other.model_state:
            return False

        return self.children == other.children


@dataclass(eq=False)
class SystemSnapshot:
    """Store the engine and root-model state for an in-memory checkpoint."""

    t_now: float
    root_model: ModelSnapshot
    t_next_continuous: float
    t_next_animation: float
    x_buffer: list[tuple[int | None, Any]] = field(default_factory=list)

    def __eq__(self, other):
        if not isinstance(other, SystemSnapshot):
            return NotImplemented

        if self.t_now != other.t_now:
            return False

        if self.t_next_continuous != other.t_next_continuous:
            return False

        if self.t_next_animation != other.t_next_animation:
            return False

        if self.root_model != other.root_model:
            return False

        return self.x_buffer == other.x_buffer
