"""Follow an L path, then turn north and west to return along the other sides."""

from importlib import import_module

from ..GridPath import POINTS_PER_AXIS

LAST = POINTS_PER_AXIS - 1
PATH = (
    tuple((0, index) for index in range(LAST, -1, -1))
    + tuple((index, 0) for index in range(1, POINTS_PER_AXIS))
    + tuple((LAST, index) for index in range(1, POINTS_PER_AXIS))
    + tuple((index, LAST) for index in range(LAST-1, -1, -1))
)


if __name__ == "__main__":
    straight_test = import_module("examples.RMFS.Tests.01_01_RobotPath_Straight")
    straight_test.main(PATH, name="01_06_RobotPath_L_rotate_L", no_rack=True)
