"""Opposite-direction corridor with off-corridor starts and goals.

No traffic control yet: a future manager can hold either robot at its start
until the other clears the shared corridor. Neither parked robot blocks
the other route; no reversing or alternate path is required.
"""

from importlib import import_module

from ..GridPath import POINTS_PER_AXIS
from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

LAST = POINTS_PER_AXIS - 1
CENTER = POINTS_PER_AXIS // 2
PATHS = {'R1': ((0, 3), (0, 2), (1, 2), (2, 2), (3, 2), (3, 3)), 'R2': ((3, 1), (3, 2), (2, 2), (1, 2), (0, 2), (0, 1))}


def create_model(*, lift_up=False, collision_check=True, stop_on_collision=False,
                 initial_headings=None):
    model = RMFS("02_07_RobotPath_OppositeDirection_EntryWait", collision_check=collision_check)
    controller = TestController(robot_paths=PATHS)
    robots = tuple(
        MRobot(name, path[0], lift_up=lift_up, stop_on_collision=stop_on_collision,
               initial_heading_deg=(initial_headings or {}).get(name))
        for name, path in PATHS.items()
    )
    sensors = tuple(PSensor(f"S{index+1}", stop_on_collision=stop_on_collision)
                    for index in range(len(robots)))
    model.add_components(controller, *robots, *sensors)
    for robot, sensor in zip(robots, sensors):
        model.add_couples(
            ((controller, f"!{robot.name}"), (robot, "?command")),
            ((robot, "!motion_plan"), (sensor, "?motion_plan")),
            ((sensor, "!sense"), (controller, f"?{robot.name}")),
            ((sensor, "!Collision"), (controller, f"?collision_{robot.name}")),
            ((robot, "!arrived"), (controller, f"?arrived_{robot.name}")),
            ((robot, "!rotation_started"),
             (controller, f"?rotation_started_{robot.name}")),
            ((robot, "!rotation_finished"),
             (controller, f"?rotation_finished_{robot.name}")),
        )
    return model, robots, sensors


if __name__ == "__main__":
    viewer = import_module("examples.RMFS.Tests.01_01_RobotPath_Straight")
    viewer.main(
        name="02_07_RobotPath_OppositeDirection_EntryWait", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=False,
    )

