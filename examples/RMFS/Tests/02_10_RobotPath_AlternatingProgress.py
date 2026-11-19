"""Counterexample: neither stationary-other first-mover test clears the problem.

A=(0,1), C=(2,1), B=(2,3), D=(0,3), E=(3,3), F=(0,0).
R1 follows A-C-B-E; R2 follows B-D-A-F. Each start blocks the other's
later path. A feasible serial schedule is R1 A-C, R2 B-D-A-F, then
R1 C-B-E. This is a geometric witness, not an implemented controller.
Both robots start at time zero in this uncontrolled example; a collision
is not inevitable merely because the path sweeps intersect.
"""

from importlib import import_module

from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

PATHS = {'R1': ((0, 1), (1, 1), (2, 1), (2, 2), (2, 3), (3, 3)), 'R2': ((2, 3), (1, 3), (0, 3), (0, 2), (0, 1), (0, 0))}


def create_model(*, lift_up=False, collision_check=True, stop_on_collision=False,
                 initial_headings=None):
    model = RMFS("02_10_RobotPath_AlternatingProgress", collision_check=collision_check)
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
        name="02_10_RobotPath_AlternatingProgress", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=False,
    )

