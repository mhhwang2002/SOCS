"""Run a small live SOCS simulation in a Panda3D window."""

from math import atan2, degrees

from socs.devs.FDDEVS import FDDEVS
from socs.visual3D import VisualEntityState, VisualSimEngineThread
from socs.visual3D.panda_viewer import Panda3DViewer


class AGV(FDDEVS):
    """A simple AGV that follows a rectangular route indefinitely."""

    route = (
        (0.0, 0.0, 0.25),
        (6.0, 0.0, 0.25),
        (6.0, 4.0, 0.25),
        (0.0, 4.0, 0.25),
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
    """Create the model, start the engine, and open the live viewer."""
    model = AGV()
    engine = VisualSimEngineThread(model, get_visual_state)
    engine.StepByStep = True
    engine.bVerbose = False

    viewer = Panda3DViewer(engine.state_buffer, title="SOCS Panda3D AGV Demo")
    viewer.set_camera((3.0, -10.0, 13.0), target=(3.0, 2.0, 0.0))
    viewer.add_polyline(
        tuple((x, y, 0.05) for x, y, _z in AGV.route) + (AGV.route[0],),
        color=(0.8, 0.8, 0.8, 1.0),
        thickness=4.0,
    )
    agv_node = viewer.load_entity(model.name, "models/box")
    agv_node.setScale(0.65, 0.9, 0.3)

    viewer.bind_key("space", engine.make_run)
    viewer.bind_key("escape", viewer.close)

    print("Panda3D demo started. Press SPACE to advance one SOCS event; ESC to close.")
    engine.start()
    try:
        viewer.run()
    finally:
        engine.terminate()


if __name__ == "__main__":
    main()
