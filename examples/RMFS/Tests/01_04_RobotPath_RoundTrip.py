"""Run an empty robot two cells east, turn 180 degrees, and return."""

from importlib import import_module

from ..GridPath import POINTS_PER_AXIS

ROW = POINTS_PER_AXIS // 2
PATH = (
    tuple((index, ROW) for index in range(3))
    + tuple((index, ROW) for index in range(1, -1, -1))
)


if __name__ == "__main__":
    straight_test = import_module("examples.RMFS.Tests.01_01_RobotPath_Straight")
    straight_test.main(PATH, name="01_04_RobotPath_RoundTrip", no_rack=True)
