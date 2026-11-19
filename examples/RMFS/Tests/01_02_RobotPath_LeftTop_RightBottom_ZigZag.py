"""Move R1 from the top-left to bottom-right, alternating east and south."""

from importlib import import_module

from ..GridPath import POINTS_PER_AXIS

LAST = POINTS_PER_AXIS - 1
PATH = ((0, LAST),) + tuple(
    point
    for index in range(1, POINTS_PER_AXIS)
    for point in ((index, LAST-index+1), (index, LAST-index))
)


if __name__ == "__main__":
    straight_test = import_module("examples.RMFS.Tests.01_01_RobotPath_Straight")
    straight_test.main(PATH, name="01_02_RobotPath_LeftTop_RightBottom_ZigZag")
