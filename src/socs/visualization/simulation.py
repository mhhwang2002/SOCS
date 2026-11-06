"""Connect SimEngineThread animation callbacks to a visual state buffer."""

from collections.abc import Callable, Iterable
import math
import time

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
        real_time_factor=None,
    ):
        if not callable(state_provider):
            raise TypeError("state_provider must be callable.")
        if real_time_factor is not None:
            if isinstance(real_time_factor, bool) or not isinstance(
                real_time_factor, (int, float)
            ):
                raise TypeError(
                    "real_time_factor must be a positive finite number or None."
                )
            if not math.isfinite(real_time_factor) or real_time_factor <= 0:
                raise ValueError(
                    "real_time_factor must be a positive finite number or None."
                )
        self.state_provider = state_provider
        self.state_buffer = state_buffer or VisualStateBuffer()
        self.real_time_factor = real_time_factor
        self._last_animation_time = None
        super().__init__(
            sim,
            inject_CBF=inject_CBF,
            TimeIntervalContinuous=TimeIntervalContinuous,
            TimeIntervalAnimation=TimeIntervalAnimation,
            TimeSimulationEnd=TimeSimulationEnd,
        )

    def doAnimation(self):
        """Publish state and optionally pace animation ticks to wall time."""
        self._publish_visual_state()
        if self.real_time_factor is None:
            return

        if self._last_animation_time is not None:
            elapsed_simulation_time = self.tNow - self._last_animation_time
            if elapsed_simulation_time > 0:
                time.sleep(elapsed_simulation_time / self.real_time_factor)
        self._last_animation_time = self.tNow

    def executeOneStep(self):
        """Publish state after every event, including state-changing events."""
        result = super().executeOneStep()
        self._publish_visual_state()
        return result

    def _publish_visual_state(self):
        self.state_buffer.publish(self.tNow, self.state_provider(self.model))
