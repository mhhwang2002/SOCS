"""Three mutually involved objects share a central control point (1,1).

With fixed forward paths and stops only at control points, every first
move enters the center and leaves no admissible progress for the group.
Any two objects alone can finish by an appropriate priority order; the
third is essential to the cyclic obstruction. No traffic manager is
implemented here: the viewer initially shows all paths and RGB sweeps.
The staged deadlock states in the document are prescribed illustrations,
not behavior enforced by this example's uncontrolled TestController.
"""

from importlib import import_module

from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

PATHS = {
    "R1": ((0, 1), (1, 1), (2, 1), (3, 1)),
    "R2": ((2, 1), (1, 1), (1, 2), (1, 3)),
    "R3": ((1, 2), (1, 1), (0, 1), (0, 0)),
}
INITIAL_HEADINGS = {"R1": 0, "R2": 180, "R3": 270}


def create_model(*, lift_up=False, collision_check=True, stop_on_collision=False,
                 initial_headings=None):
    model = RMFS("03_06_RobotPath_ThreeWay_CyclicWait", collision_check=collision_check)
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
        name="03_06_RobotPath_ThreeWay_CyclicWait", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=False,
        initial_headings=INITIAL_HEADINGS,
    )

