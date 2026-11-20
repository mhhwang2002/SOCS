"""Build one controller and one physical MRobot/PSensor pair per object."""

from .GridPath import CONTROL_POINTS
from .MRobot import MRobot
from .PSensor import PSensor
from .RMFSController import RMFSController
from .model import RMFS
from .path_geometry import Path, RectangularFootprint, TuplePath
from .wait_duplication import WaitDuplicationSolver


def create_controlled_model(robot_paths, *, name="RMFS_WaitDuplication", lift_up=False,
                            rack_size=(1.5, 1.5), collision_check=True,
                            stop_on_collision=True, initial_headings=None,
                            grid=None, allow_no_plan=False, plan_builder=None):
    points_map = CONTROL_POINTS if grid is None else grid["control_points"]
    paths = {name: Path(name, tuple(points_map[p] for p in points))
             for name, points in robot_paths.items()}
    shape = RectangularFootprint(*rack_size)
    solver = WaitDuplicationSolver(paths, {name: shape for name in paths})
    # A demonstration may supply a constructive witness. It is validated with
    # the same rules; the general search implementation is not changed.
    result = solver.solve() if plan_builder is None else plan_builder(solver)
    if result.status != "feasible" and not allow_no_plan:
        raise ValueError("No wait-duplication tuple path exists for these fixed routes.")
    if result.status == "feasible":
        solver.validate(result)
    model = RMFS(name, collision_check=collision_check)
    model.planning_error = (result.reason or "Exhaustive wait-duplication search found no plan."
                            if result.status != "feasible" else None)
    controller = RMFSController(paths, result.tuple_path or TuplePath(tuple(paths), ()),
                                planning_error=model.planning_error)
    model.controller = controller
    model.plan_result = result
    robots = tuple(MRobot(name, route[0], lift_up=lift_up, rack_size=rack_size,
                          stop_on_collision=stop_on_collision,
                          initial_heading_deg=(initial_headings or {}).get(name),
                          command_coordinates="metres", control_points=points_map)
                   for name, route in robot_paths.items())
    sensors = tuple(PSensor(f"S{i+1}", stop_on_collision=stop_on_collision)
                    for i in range(len(robots)))
    model.add_components(controller, *robots, *sensors)
    for robot, sensor in zip(robots, sensors):
        model.add_couples(
            ((controller, f"!{robot.name}"), (robot, "?command")),
            ((robot, "!motion_plan"), (sensor, "?motion_plan")),
            ((sensor, "!sense"), (controller, f"?{robot.name}")),
            ((sensor, "!Collision"), (controller, f"?collision_{robot.name}")),
            ((robot, "!arrived"), (controller, f"?arrived_{robot.name}")),
            ((robot, "!rotation_started"), (controller, f"?rotation_started_{robot.name}")),
            ((robot, "!rotation_finished"), (controller, f"?rotation_finished_{robot.name}")),
        )
    return model, robots, sensors
