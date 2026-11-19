"""Follow the left and bottom grid boundaries from top-left to bottom-right."""

from importlib import import_module

from ..GridPath import POINTS_PER_AXIS

LAST = POINTS_PER_AXIS - 1
PATH = (
    tuple((0, index) for index in range(LAST, -1, -1))
    + tuple((index, 0) for index in range(1, POINTS_PER_AXIS))
)


if __name__ == "__main__":
    straight_test = import_module("examples.RMFS.Tests.01_01_RobotPath_Straight")
    straight_test.main(PATH, name="01_03_RobotPath_LeftTop_RightBottom_Straight_L")
