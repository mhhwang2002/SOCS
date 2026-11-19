"""A locally safe corridor entry can destroy completion feasibility.

Fixed paths, forward progress and waiting only; no passing or reversing.
R1 and R2 start outside their shared horizontal corridor. A feasible
schedule sends R1 completely through, then R2. R3 has a disjoint path.
After R1 reaches (0,2), admitting R2 to (3,2) is locally collision-free
but traps the pair on opposite ends of the same one-lane path. The
controller here sends paths without traffic control; it does not execute
this illustrative staged admission policy or detect deadlock itself.
"""

from importlib import import_module

from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

PATHS = {
    "R1": ((0, 3), (0, 2), (1, 2), (2, 2), (3, 2), (3, 3)),
    "R2": ((3, 1), (3, 2), (2, 2), (1, 2), (0, 2), (0, 1)),
    "R3": ((1, 0), (2, 0)),
}
INITIAL_HEADINGS = {"R1": 270, "R2": 90, "R3": 0}


def create_model(*, lift_up=False, collision_check=True, stop_on_collision=False,
                 initial_headings=None):
    model = RMFS("03_05_RobotPath_LocalSafe_Deadlock", collision_check=collision_check)
    initial_headings = {**INITIAL_HEADINGS, **(initial_headings or {})}
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
        name="03_05_RobotPath_LocalSafe_Deadlock", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=False,
        initial_headings=INITIAL_HEADINGS,
    )

