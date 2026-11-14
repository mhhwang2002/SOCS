"""Thread-safe visual state exchanged between SOCS and a renderer."""

from dataclasses import dataclass
from threading import Lock
from typing import Iterable


@dataclass(frozen=True)
class VisualEntityState:
    """The renderable state of one entity at a simulation instant."""

    entity_id: str
    position: tuple[float, float, float]
    heading: float = 0.0
    color: tuple[float, float, float, float] | None = None
    visible: bool = True

    def __post_init__(self):
        if not self.entity_id:
            raise ValueError("entity_id must not be empty.")
        if len(self.position) != 3:
            raise ValueError("position must contain exactly three coordinates.")
        if self.color is not None and len(self.color) != 4:
            raise ValueError("color must contain red, green, blue, and alpha.")


class VisualStateBuffer:
    """Keep the latest complete visual state for the renderer to consume."""

    def __init__(self):
        self._lock = Lock()
        self._simulation_time = 0.0
        self._entities: dict[str, VisualEntityState] = {}
        self._revision = 0

    def publish(
        self,
        simulation_time: float,
        entities: Iterable[VisualEntityState],
    ) -> None:
        """Replace the published state with a complete entity snapshot."""
        entity_map = {}
        for entity in entities:
            if not isinstance(entity, VisualEntityState):
                raise TypeError("entities must contain VisualEntityState values.")
            if entity.entity_id in entity_map:
                raise ValueError(f"Duplicate visual entity id: {entity.entity_id!r}.")
            entity_map[entity.entity_id] = entity

        with self._lock:
            self._simulation_time = float(simulation_time)
            self._entities = entity_map
            self._revision += 1

    def read(self):
        """Return a consistent copy of the latest state and its revision."""
        with self._lock:
            return self._revision, self._simulation_time, dict(self._entities)
