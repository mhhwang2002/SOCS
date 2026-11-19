"""Follow an L path, turn 180 degrees, and retrace it to the home point."""

from importlib import import_module

from ..GridPath import POINTS_PER_AXIS

LAST = POINTS_PER_AXIS - 1
OUTBOUND = (
    tuple((0, index) for index in range(LAST, -1, -1))
    + tuple((index, 0) for index in range(1, POINTS_PER_AXIS))
)
PATH = OUTBOUND + tuple(reversed(OUTBOUND[:-1]))


if __name__ == "__main__":
    straight_test = import_module("examples.RMFS.Tests.01_01_RobotPath_Straight")
    straight_test.main(PATH, name="01_05_RobotPath_L_backward_L", no_rack=True)
