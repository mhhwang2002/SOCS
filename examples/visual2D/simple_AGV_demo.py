"""Run a small live SOCS AGV simulation in a PySide6 2D window."""

import sys
from math import atan2, degrees

from PySide6.QtWidgets import QApplication

from socs.devs.FDDEVS import FDDEVS
from socs.visual2D.viewer import AGVView
from socs.visualization import VisualEntityState, VisualSimEngineThread


class AGV(FDDEVS):
    """A simple AGV that follows a rectangular route indefinitely."""

    max_speed = 5.0
    acceleration = 5.0
    deceleration = 5.0

    route = (
        (0.0, 0.0, 0.0),
        (14.0, 0.0, 0.0),
        (14.0, 4.0, 0.0),
        (0.0, 4.0, 0.0),
    )

    def __init__(self, name="AGV1"):
        super().__init__(name)
        self.route_index = 0
        self.position = self.route[0]
        self.phase = "accel"
        self._configure_motion()

    def delta_0(self):
        self.route_index = 0
        self.position = self.route[0]
        self.phase = "accel"
        self._configure_motion()

    def tau(self):
        if self.phase == "accel":
            return self.accel_time
        if self.phase == "max_speed":
            return self.cruise_time
        return self.decel_time

    def delta_cont(self, t):
        """Update position using the current acceleration phase."""
        elapsed = self.timeElapsed(t)
        if self.phase == "accel":
            distance = 0.5 * self.acceleration * min(elapsed, self.accel_time) ** 2
        elif self.phase == "max_speed":
            distance = self.accel_distance + self.peak_speed * min(
                elapsed, self.cruise_time
            )
        else:
            decel_elapsed = min(elapsed, self.decel_time)
            distance = (
                self.accel_distance
                + self.cruise_distance
                + self.peak_speed * decel_elapsed
                - 0.5 * self.deceleration * decel_elapsed**2
            )
        self.position = tuple(
            self.segment_start[axis] + self.direction[axis] * distance
            for axis in range(3)
        )
        return False

    def delta_y(self):
        if self.phase == "accel":
            self.phase = "max_speed" if self.cruise_time > 0.0 else "decel"
        elif self.phase == "max_speed":
            self.phase = "decel"
        else:
            self.route_index = (self.route_index + 1) % len(self.route)
            self.position = self.route[self.route_index]
            self.phase = "accel"
            self._configure_motion()
        return None

    def get_state(self):
        return self.route_index, self.position, self.phase

    def set_state(self, state):
        self.route_index, self.position, self.phase = state
        self._configure_motion()

    def _configure_motion(self):
        """Calculate triangular or trapezoidal motion for the active leg."""
        self.segment_start = self.route[self.route_index]
        endpoint = self.route[(self.route_index + 1) % len(self.route)]
        delta = tuple(endpoint[i] - self.segment_start[i] for i in range(3))
        self.distance = sum(component**2 for component in delta) ** 0.5
        self.direction = tuple(component / self.distance for component in delta)

        a = self.acceleration
        d = self.deceleration
        peak_speed = min(
            self.max_speed,
            (2.0 * self.distance * a * d / (a + d)) ** 0.5,
        )
        self.peak_speed = peak_speed
        self.accel_time = peak_speed / a
        self.decel_time = peak_speed / d
        self.accel_distance = 0.5 * a * self.accel_time**2
        self.decel_distance = 0.5 * d * self.decel_time**2
        self.cruise_distance = max(
            0.0, self.distance - self.accel_distance - self.decel_distance
        )
        self.cruise_time = self.cruise_distance / peak_speed

    @property
    def heading(self):
        return degrees(atan2(self.direction[1], self.direction[0]))


def get_visual_state(model):
    """Adapt the SOCS AGV state to the renderer's data format."""
    return [
        VisualEntityState(
            entity_id=model.name,
            position=model.position,
            heading=model.heading,
            color=(0.15, 0.55, 0.95, 1.0),
        )
    ]


def main():
    """Start the simulation engine and the PySide6 viewer."""
    app = QApplication(sys.argv)
    model = AGV()
    engine = VisualSimEngineThread(
        model,
        get_visual_state,
        TimeIntervalAnimation=1.0 / 20.0,
        real_time_factor=1.0,
    )
    engine.StepByStep = True
    engine.bVerbose = False

    def run_automatically():
        engine.StepByStep = False
        engine.make_run()
        print("Automatic run: press I for interactive mode.")

    def run_interactively():
        engine.StepByStep = True
        engine.make_run()
        print("Interactive run: press SPACE for the next phase; A for automatic mode.")

    view = AGVView(
        state_buffer=engine.state_buffer,
        route=AGV.route,
        entity_id=model.name,
        on_advance=engine.make_run,
        on_auto=run_automatically,
        on_interactive=run_interactively,
    )
    view.resize(1000, 700)
    view.setFocus()
    view.show()

    print("PySide6 demo started. A: automatic, I: interactive, SPACE: next phase, F: fit layout.")
    engine.start()
    try:
        return app.exec()
    finally:
        engine.terminate()


if __name__ == "__main__":
    raise SystemExit(main())
