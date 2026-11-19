"""Two eastbound robots and one westbound robot share a corridor.

Starts and goals are distinct. Opposing encounters occur before arrival.
There is no traffic control; collision checking is on and stopping is off.
"""

from importlib import import_module

from ..GridPath import POINTS_PER_AXIS
from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

LAST = POINTS_PER_AXIS - 1
CENTER = POINTS_PER_AXIS // 2
PATHS = {'R1': ((0, 1), (1, 1), (2, 1), (3, 1), (3, 2)), 'R2': ((0, 0), (0, 1), (1, 1), (2, 1), (3, 1), (3, 0)), 'R3': ((3, 1), (2, 1), (1, 1), (0, 1), (0, 2))}


def create_model(*, lift_up=False, collision_check=True, stop_on_collision=False,
                 initial_headings=None):
    model = RMFS("03_03_RobotPath_MixedDirection_Collision", collision_check=collision_check)
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
        name="03_03_RobotPath_MixedDirection_Collision", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=False,
    )

