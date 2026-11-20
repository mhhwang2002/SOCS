"""03_04 paths executed by the single RMFSController and three physical robots."""

from importlib import import_module

from ..controlled import create_controlled_model

source = import_module("examples.RMFS.Tests.03_04_RobotPath_SweepInterference")
PATHS = source.PATHS
INITIAL_HEADINGS = source.INITIAL_HEADINGS


def create_model(**options):
    options.setdefault("initial_headings", INITIAL_HEADINGS)
    return create_controlled_model(PATHS, name="03_07_ControlledSweeps", **options)


if __name__ == "__main__":
    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name="03_07_ControlledSweeps", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=True, initial_headings=INITIAL_HEADINGS)
