"""Connect SimEngineThread animation callbacks to a visual state buffer."""

from collections.abc import Callable, Iterable

from socs.devs.Simulatable import Simulatable
from socs.devs.SimEngine import SimEngineThread

from .state import VisualEntityState, VisualStateBuffer


class VisualSimEngineThread(SimEngineThread):
    """Publish model visualization state from the simulation thread.

    The provider must return a complete state for all currently visualized
    entities. It runs on the simulation thread and must not call renderer APIs.
    """

    def __init__(
        self,
        sim: Simulatable,
        state_provider: Callable[[Simulatable], Iterable[VisualEntityState]],
        state_buffer: VisualStateBuffer | None = None,
        inject_CBF=None,
        TimeIntervalContinuous=float("inf"),
        TimeIntervalAnimation=1.0,
        TimeSimulationEnd=float("inf"),
    ):
        if not callable(state_provider):
            raise TypeError("state_provider must be callable.")
        self.state_provider = state_provider
        self.state_buffer = state_buffer or VisualStateBuffer()
        super().__init__(
            sim,
            inject_CBF=inject_CBF,
            TimeIntervalContinuous=TimeIntervalContinuous,
            TimeIntervalAnimation=TimeIntervalAnimation,
            TimeSimulationEnd=TimeSimulationEnd,
        )

    def doAnimation(self):
        """Publish current entity state without touching renderer objects."""
        self._publish_visual_state()

    def executeOneStep(self):
        """Publish state after every event, including state-changing events."""
        result = super().executeOneStep()
        self._publish_visual_state()
        return result

    def _publish_visual_state(self):
        self.state_buffer.publish(self.tNow, self.state_provider(self.model))
