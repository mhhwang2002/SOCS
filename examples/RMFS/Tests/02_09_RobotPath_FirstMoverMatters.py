"""Only R2 can clear the interference while the other waits at its start.

R2 starts on R1's path. R1 cannot pass stationary R2. R2 can move north
to (1, 2), leaving disjoint residual sweeps, while R1 waits at (0, 1).
Both receive commands at time zero here: no waiting control is implemented.
"""

from importlib import import_module

from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

PATHS = {'R1': ((0, 1), (1, 1), (2, 1), (3, 1)), 'R2': ((1, 1), (1, 2), (1, 3))}


def create_model(*, lift_up=False, collision_check=True, stop_on_collision=False,
                 initial_headings=None):
    model = RMFS("02_09_RobotPath_FirstMoverMatters", collision_check=collision_check)
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
        name="02_09_RobotPath_FirstMoverMatters", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=False,
    )

