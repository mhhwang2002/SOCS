"""Run the AS/RS DEVS example with a live Panda3D visualization."""

from socs.visual3D import VisualEntityState, VisualTESEnginThread
from socs.visual3D.panda_viewer import Panda3DViewer

from .asrs_devs import (
    AISLE_Y,
    ASRSController,
    CELL_WIDTH,
    Crane,
    HOME_X,
    LOAD_CENTER_OFFSET,
    LOAD_HEIGHT,
    LEVEL_PITCH,
    RACK_COLUMNS,
    RACK_DEPTH,
    RACK_LEVELS,
    UNLOAD_Y,
    iter_cells,
    make_asrs_model,
)


class ASRSTESEnginThread(VisualTESEnginThread):
    """Pause the live demo after the controller retrieves every occupied cell."""

    def __init__(self, model, *args, **kwargs):
        self.crane = next(child for child in model.children if isinstance(child, Crane))
        self.controller = next(
            child for child in model.children if isinstance(child, ASRSController)
        )
        super().__init__(model, *args, **kwargs)

    def execute_one_step(self):
        result = super().execute_one_step()
        if self.controller.phase == "completed" and not self.SkipLoop:
            self.pause()
            print("Controller completed all occupied-cell retrievals; simulation paused.")
        return result


def visual_state(model):
    """Return a complete visualization snapshot for the rack and crane."""
    crane = next(child for child in model.children if isinstance(child, Crane))
    states = []
    for cell in iter_cells(model):
        states.append(
            VisualEntityState(
                cell.name,
                (
                    (cell.bay - 0.5) * CELL_WIDTH,
                    RACK_DEPTH / 2.0,
                    (cell.level - 1) * LEVEL_PITCH + LOAD_CENTER_OFFSET,
                ),
                color=(0.76, 0.82, 0.84, 1.0),
                visible=cell.occupied,
            )
        )

    states.extend(
        (
            VisualEntityState(
                "CraneChassis", (crane.x, AISLE_Y, 0.18), color=(0.95, 0.38, 0.08, 1.0)
            ),
            VisualEntityState(
                "CraneMast", (crane.x, AISLE_Y, 2.55), color=(0.95, 0.38, 0.08, 1.0)
            ),
            VisualEntityState(
                "CraneHoist", (crane.x, AISLE_Y, crane.z), color=(0.16, 0.20, 0.22, 1.0)
            ),
            VisualEntityState(
                "ForkLeft",
                (crane.x - 0.18, crane.y - 0.5, crane.z + crane.fork_lift - 0.08),
                color=(0.88, 0.71, 0.22, 1.0),
            ),
            VisualEntityState(
                "ForkRight",
                (crane.x + 0.18, crane.y - 0.5, crane.z + crane.fork_lift - 0.08),
                color=(0.88, 0.71, 0.22, 1.0),
            ),
            VisualEntityState(
                "ForkCarriage",
                (crane.x, AISLE_Y + 0.05, crane.z + crane.fork_lift - 0.08),
                color=(0.88, 0.71, 0.22, 1.0),
            ),
            VisualEntityState(
                "ForkLoad",
                (
                    crane.x,
                    crane.y,
                    crane.z + crane.fork_lift - 0.08 + LOAD_HEIGHT / 2.0,
                ),
                color=(0.70, 0.78, 0.80, 1.0),
                visible=crane.has_load,
            ),
            VisualEntityState(
                "CraneGuideLeft",
                (crane.x - 0.16, AISLE_Y, 2.55),
                color=(0.16, 0.20, 0.22, 1.0),
            ),
            VisualEntityState(
                "CraneGuideRight",
                (crane.x + 0.16, AISLE_Y, 2.55),
                color=(0.16, 0.20, 0.22, 1.0),
            ),
            VisualEntityState(
                "CraneWheelFL",
                (crane.x - 0.27, AISLE_Y - 0.31, 0.12),
                color=(0.12, 0.14, 0.16, 1.0),
            ),
            VisualEntityState(
                "CraneWheelFR",
                (crane.x - 0.27, AISLE_Y + 0.31, 0.12),
                color=(0.12, 0.14, 0.16, 1.0),
            ),
            VisualEntityState(
                "CraneWheelRL",
                (crane.x + 0.27, AISLE_Y - 0.31, 0.12),
                color=(0.12, 0.14, 0.16, 1.0),
            ),
            VisualEntityState(
                "CraneWheelRR",
                (crane.x + 0.27, AISLE_Y + 0.31, 0.12),
                color=(0.12, 0.14, 0.16, 1.0),
            ),
        )
    )
    return states


def add_rack_geometry(viewer):
    """Build a shaded warehouse floor, rack frames, shelves, and rails."""
    length = RACK_COLUMNS * CELL_WIDTH
    height = RACK_LEVELS * LEVEL_PITCH
    rack_color = (0.06, 0.22, 0.30, 1.0)
    beam_color = (0.05, 0.38, 0.50, 1.0)
    deck_color = (0.34, 0.40, 0.43, 1.0)
    rail_color = (0.10, 0.12, 0.14, 1.0)

    viewer.add_box(
        "Warehouse floor",
        (length / 2.0, -0.15, -0.06),
        (length + 3.0, 6.0, 0.12),
        (0.34, 0.39, 0.43, 1.0),
    )

    for bay in range(RACK_COLUMNS + 1):
        x = bay * CELL_WIDTH
        for y in (0.0, RACK_DEPTH):
            viewer.add_box(
                f"Rack upright B{bay:02d} Y{y:g}",
                (x, y, height / 2.0),
                (0.07, 0.07, height),
                rack_color,
            )
    for level in range(1, RACK_LEVELS + 1):
        z = level * LEVEL_PITCH - 0.025
        for y in (0.0, RACK_DEPTH):
            viewer.add_box(
                f"Rack beam L{level} Y{y:g}",
                (length / 2.0, y, z),
                (length, 0.05, 0.05),
                beam_color,
            )
        for bay in range(RACK_COLUMNS):
            x = (bay + 0.5) * CELL_WIDTH
            viewer.add_box(
                f"Rack cross support L{level} B{bay + 1:02d}",
                (x, RACK_DEPTH / 2.0, z),
                (0.05, RACK_DEPTH, 0.05),
                beam_color,
            )
    for level in range(RACK_LEVELS):
        for bay in range(RACK_COLUMNS):
            viewer.add_box(
                f"Shelf deck L{level + 1} B{bay + 1:02d}",
                ((bay + 0.5) * CELL_WIDTH, RACK_DEPTH / 2.0, level * LEVEL_PITCH + 0.035),
                (CELL_WIDTH - 0.09, RACK_DEPTH - 0.12, 0.025),
                deck_color,
            )

    for y in (AISLE_Y - 0.32, AISLE_Y + 0.32):
        viewer.add_box(
            f"Crane rail Y{y:g}",
            (length / 2.0, y, 0.08),
            (length + 2.0, 0.12, 0.10),
            rail_color,
        )

    viewer.add_box(
        "Unload station deck",
        (HOME_X, UNLOAD_Y, 0.355),
        (0.95, 0.90, 0.12),
        (0.16, 0.28, 0.32, 1.0),
    )
    for dx in (-0.36, 0.36):
        for dy in (-0.32, 0.32):
            viewer.add_box(
                f"Unload station leg {dx:+.2f} {dy:+.2f}",
                (HOME_X + dx, UNLOAD_Y + dy, 0.1475),
                (0.08, 0.08, 0.295),
                rail_color,
            )


def main():
    """Build the coupled model and launch the live 3D viewer."""
    model = make_asrs_model()
    engine = ASRSTESEnginThread(
        model,
        visual_state,
        TimeIntervalAnimation=0.05,
        TimeSimulationEnd=10000.0,
        real_time_factor=1.0,
    )
    engine.StepByStep = True
    engine.bVerbose = False

    viewer = Panda3DViewer(engine.state_buffer, title="SOCS AS/RS DEVS Demo")
    add_rack_geometry(viewer)
    for cell in iter_cells(model):
        viewer.create_box_entity(cell.name, (0.78, 0.76, LOAD_HEIGHT))

    viewer.create_box_entity("CraneChassis", (0.82, 0.68, 0.24))
    viewer.create_box_entity("CraneMast", (0.16, 0.18, 5.1))
    viewer.create_box_entity("CraneHoist", (0.62, 0.50, 0.24))
    moving_boxes = {
        "CraneGuideLeft": (0.045, 0.045, 4.85),
        "CraneGuideRight": (0.045, 0.045, 4.85),
        "CraneWheelFL": (0.20, 0.12, 0.16),
        "CraneWheelFR": (0.20, 0.12, 0.16),
        "CraneWheelRL": (0.20, 0.12, 0.16),
        "CraneWheelRR": (0.20, 0.12, 0.16),
        "ForkLeft": (0.07, 1.0, 0.045),
        "ForkRight": (0.07, 1.0, 0.045),
        "ForkCarriage": (0.52, 0.42, 0.08),
    }
    for entity_id, size in moving_boxes.items():
        viewer.create_box_entity(entity_id, size)
    viewer.create_box_entity("ForkLoad", (0.78, 0.76, LOAD_HEIGHT))

    reset_camera(viewer)
    viewer.bind_key("space", engine.make_run)
    viewer.bind_key("a", lambda: _run_automatically(engine))
    viewer.bind_key("i", lambda: _run_interactively(engine))
    viewer.bind_key("r", lambda: reset_camera(viewer))
    viewer.bind_key("escape", viewer.close)

    print("AS/RS demo: SPACE starts/continues, A runs automatically, I pauses at each DEVS phase, R resets view, ESC closes.")
    engine.start()
    try:
        viewer.run()
    finally:
        engine.terminate()


def reset_camera(viewer):
    """Restore a view that frames the full rack and crane aisle."""
    viewer.set_camera((8.0, -13.0, 10.5), target=(7.5, 0.0, 2.0))


def _run_automatically(engine):
    engine.StepByStep = False
    engine.make_run()


def _run_interactively(engine):
    engine.StepByStep = True
    engine.make_run()


if __name__ == "__main__":
    main()
    ASRSController,
