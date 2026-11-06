"""Run a small live SOCS AGV simulation in a PySide6 2D window."""

import sys
from math import atan2, degrees

from PySide6.QtWidgets import QApplication

from socs.devs.FDDEVS import FDDEVS
from socs.visual2D.viewer import AGVView
from socs.visualization import VisualEntityState, VisualSimEngineThread


class AGV(FDDEVS):
    """A simple AGV that follows a rectangular route indefinitely."""

    route = (
        (0.0, 0.0, 0.0),
        (6.0, 0.0, 0.0),
        (6.0, 4.0, 0.0),
        (0.0, 4.0, 0.0),
    )

    def __init__(self, name="AGV1"):
        super().__init__(name)
        self.route_index = 0

    def delta_0(self):
        self.route_index = 0

    def tau(self):
        return 1.0

    def delta_y(self):
        self.route_index = (self.route_index + 1) % len(self.route)
        return None

    def get_state(self):
        return self.route_index

    def set_state(self, state):
        self.route_index = state

    @property
    def position(self):
        return self.route[self.route_index]

    @property
    def heading(self):
        start = self.position
        end = self.route[(self.route_index + 1) % len(self.route)]
        return degrees(atan2(end[1] - start[1], end[0] - start[0]))


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
    engine = VisualSimEngineThread(model, get_visual_state)
    engine.StepByStep = True
    engine.bVerbose = False

    view = AGVView(
        state_buffer=engine.state_buffer,
        route=AGV.route,
        entity_id=model.name,
        on_advance=engine.make_run,
    )
    view.resize(1000, 700)
    view.setFocus()
    view.show()

    print("PySide6 demo started. Press SPACE for the next destination; F fits the layout.")
    engine.start()
    try:
        return app.exec()
    finally:
        engine.terminate()


if __name__ == "__main__":
    raise SystemExit(main())
