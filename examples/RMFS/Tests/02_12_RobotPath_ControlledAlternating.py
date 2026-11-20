"""02_10 routes executed by RMFSController / TrafficManager / RoadMap.

Manual by default; A toggles automatic mode. Existing 02_10 is unchanged.
"""

from importlib import import_module

from ..controlled import create_controlled_model

source = import_module("examples.RMFS.Tests.02_10_RobotPath_AlternatingProgress")
PATHS = source.PATHS


def create_model(**options):
    return create_controlled_model(PATHS, name="02_12_ControlledAlternating", **options)


if __name__ == "__main__":
    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name="02_12_ControlledAlternating", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=True)
