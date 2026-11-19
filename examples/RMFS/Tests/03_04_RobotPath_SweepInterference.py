"""Three-robot initial layout for swept-area interference reasoning.

R1 is red, R2 green, and R3 blue (paths and translucent sweeps).
Full-sweep neighbors: N1={2,3}, N2={1}, N3={1}.
The initial bodies are disjoint. No traffic manager is implemented.
One geometric serial completion witness is R3 to (2,0), R1 to (2,1),
R2 to (0,3), then R1 and R2 to their distinct goals. This witness is
explanatory only; it is not the controller's automatic command order.
Launch without --non-interactive to inspect the paused initial state.
"""

from importlib import import_module

from ..MRobot import MRobot
from ..PSensor import PSensor
from ..TestController import TestController
from ..model import RMFS

PATHS = {
    "R1": ((0, 1), (1, 1), (2, 1), (2, 2), (2, 3), (3, 3)),
    "R2": ((2, 3), (1, 3), (0, 3), (0, 2), (0, 1), (0, 0)),
    "R3": ((3, 0), (3, 1), (2, 1), (2, 0)),
}
# Degrees: east=0, north=90, west=180, south=270.
INITIAL_HEADINGS = {"R1": 0, "R2": 180, "R3": 90}


def create_model(*, lift_up=False, collision_check=True, stop_on_collision=False,
                 initial_headings=None):
    model = RMFS("03_04_RobotPath_SweepInterference", collision_check=collision_check)
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
        name="03_04_RobotPath_SweepInterference", no_rack=True,
        model_factory=create_model, robot_paths=PATHS,
        collision_check=True, stop_on_collision=False,
        initial_headings=INITIAL_HEADINGS,
    )

