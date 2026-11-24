"""Create and couple the RMFS controller and physical robot models."""

from argparse import ArgumentParser
from math import isfinite
from functools import partial
from pathlib import Path as FilePath

from .model import RMFS
from .MRobot import MRobot
from .PSensor import PSensor
from .RMFSController import RMFSController
from .OrderGenerator import OrderGenerator
from .path_geometry import Path, TuplePath, RectangularFootprint
from .motion_graph import compile_motion_graph, load_motion_graph, save_motion_graph


def create_model(*, robot_count, order_count=100, seed=0, layout=None,
                 robot_starts=None, rack_positions=None, motion_graph=None,
                 collision_check=False, initial_headings=None, full_cycle=False,
                 workstations=None, lift_duration=1.4, lower_duration=1.4,
                 service_duration=3.0, departure_policy='destination-clear',
                 rolling_passage=None, crossing_stops=None):
    """Build order dispatch with persistent physical robots.

    robot_starts maps robot names to layout node IDs; rack_positions maps rack
    IDs to metre coordinates. full_cycle=False preserves the first-leg experiment.
    """
    if not isinstance(robot_count, int) or isinstance(robot_count, bool) or robot_count < 1:
        raise ValueError('robot_count must be at least 1')

    segmented = rolling_passage is not None or crossing_stops is not None
    if segmented and (not full_cycle or departure_policy != 'destination-clear'):
        raise ValueError('Crossing policies require full-cycle orders with destination-clear route planning.')
    if segmented and (rolling_passage not in ('allow', 'forbid') or crossing_stops not in ('allow', 'forbid')):
        raise ValueError('Supply both rolling-passage and crossing-stops policies.')
    rolling = rolling_passage == 'allow'
    controller_options = {}
    if segmented:
        from .WarehouseTrafficManager import WarehouseTrafficManager
        controller_options.update(safe_prefix=True, motion_extension=rolling,
                                  traffic_manager_type=partial(
                                      WarehouseTrafficManager, stopping_allowed=crossing_stops == 'allow'))

    default_layout = layout is None
    if default_layout:
        from .WarehouseLayout import load_layout
        layout = load_layout(home_capacity=robot_count)
    if robot_starts is None:
        bottom = min(p[1] for p in layout["control_points"].values())
        homes = sorted((key for key, p in layout["control_points"].items()
                        if p[1] == bottom and len(layout["adjacency"][key]) == 1), key=repr)
        homes = list(layout.get('warehouse_homes', homes))
        if len(homes) < robot_count:
            raise ValueError("Supply robot_starts: not enough outer bottom home vertices.")
        robot_starts = {f"R{i+1:02}": p for i, p in enumerate(homes[:robot_count])}
    if len(robot_starts) != robot_count:
        raise ValueError("robot_starts must contain exactly robot_count robots.")
    rack_positions = layout.get("warehouse_racks") if rack_positions is None else rack_positions
    if not rack_positions:
        raise ValueError("Supply rack_positions for this layout.")
    if motion_graph is None:
        if default_layout:
            cache_name = 'warehouse30_motion_graph.json' if robot_count > 20 else 'warehouse_motion_graph.json'
            cache = FilePath(__file__).with_name("precomputed")/cache_name
            try:
                motion_graph = load_motion_graph(cache, layout=layout)
            except (OSError, ValueError):
                motion_graph = compile_motion_graph(layout)
                save_motion_graph(motion_graph, cache)
        else:
            motion_graph = compile_motion_graph(layout)
    paths = {name: Path(name, (layout["control_points"][node],))
             for name, node in robot_starts.items()}
    shape = RectangularFootprint()
    controller = RMFSController(paths, TuplePath(tuple(paths), ()),
        event_release=True, footprints={n: shape for n in paths}, layout=layout,
        rack_positions=rack_positions, motion_graph=motion_graph, seed=seed,
        initial_headings=initial_headings, full_cycle=full_cycle,
        workstations=layout.get("warehouse_stations") if workstations is None else workstations,
        service_duration=service_duration, departure_policy=departure_policy, **controller_options)
    model = RMFS("RMFS_OrderWorkflow" if full_cycle else "RMFS_InitialRackDispatch",
                 collision_check=collision_check)
    robots = tuple(MRobot(name, node, lift_up=False, control_points=layout["control_points"],
                         command_coordinates="metres", stop_on_collision=True,
                         initial_heading_deg=(initial_headings or {}).get(name),
                         lift_duration=lift_duration, lower_duration=lower_duration, passage_push=rolling)
                   for name, node in robot_starts.items())
    sensors = tuple(PSensor(f"S{i+1}", stop_on_collision=True) for i in range(robot_count))
    generator = OrderGenerator(order_count)
    model.add_components(controller, generator, *robots, *sensors)
    model.controller, model.robots, model.sensors = controller, robots, sensors
    model.order_generator = generator
    model.add_couples(((generator, "!orders"), (controller, "?orders")))
    for robot, sensor in zip(robots, sensors):
        model.add_couples(
            ((controller, f"!{robot.name}"), (robot, "?command")),
            ((controller, f"!orient_{robot.name}"), (robot, "?orient")),
            ((robot, "!motion_plan"), (sensor, "?motion_plan")),
            ((sensor, "!sense"), (controller, f"?{robot.name}")),
            ((sensor, "!Collision"), (controller, f"?collision_{robot.name}")),
            ((robot, "!arrived"), (controller, f"?arrived_{robot.name}")),
            ((robot, "!rotation_started"), (controller, f"?rotation_started_{robot.name}")),
            ((robot, "!rotation_finished"), (controller, f"?rotation_finished_{robot.name}")),
        )
        if rolling:
            model.add_couples(
                ((robot, "!passage_request"), (controller, f"?passage_request_{robot.name}")),
                ((controller, f"!extend_{robot.name}"), (robot, "?extend_motion")),
            )
        if full_cycle:
            model.add_couples(
                ((controller, f"!lift_{robot.name}"), (robot, "?lift")),
                ((controller, f"!lower_{robot.name}"), (robot, "?lower")),
                ((robot, "!lifted"), (controller, f"?lifted_{robot.name}")),
                ((robot, "!lowered"), (controller, f"?lowered_{robot.name}")),
            )
    return model


def add_lookahead_options(parser):
    from .WarehouseLookahead import parse_horizon

    parser.add_argument('--lookahead-depth', type=int, choices=(0, 1), default=0)
    parser.add_argument('--lookahead-horizon', type=parse_horizon, default=1 / 3)


def workflow_engine_factory(*, lookahead_depth=0, lookahead_horizon=1 / 3,
                            rolling_passage=None, crossing_stops=None):
    from socs.devs.TESEngin import TESEnginBase

    if not lookahead_depth:
        return TESEnginBase
    if (rolling_passage, crossing_stops) != ('allow', 'allow') or lookahead_depth != 1:
        raise ValueError('Warehouse lookahead requires depth 1 and both crossing policies allowed.')
    from .WarehouseLookahead import WarehouseLookaheadEngine
    return partial(WarehouseLookaheadEngine, horizon=lookahead_horizon)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, required=True)
    parser.add_argument('--orders', type=int, default=100, help='initial orders (default: 100)')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--animate', action='store_true', help='Open the shared 3D viewer')
    parser.add_argument('--departure-policy', choices=('destination-clear', 'advance'),
                        default='destination-clear', help='Whole-leg clearance or safe-prefix progress')
    parser.add_argument('--rolling-passage', choices=('allow', 'forbid'))
    parser.add_argument('--crossing-stops', choices=('allow', 'forbid'))
    add_lookahead_options(parser)
    parser.add_argument('--first-leg-only', action='store_true', help='Stop at the initial rack arrivals')
    parser.add_argument('--collision-check', action='store_true', help='Enable sampled stop-on-collision validation')
    parser.add_argument('--collision-interval', type=float, default=.1)
    args, viewer_args = parser.parse_known_args()
    if viewer_args and not args.animate:
        parser.error('Viewer options require --animate: ' + ' '.join(viewer_args))
    if args.collision_check and (not isfinite(args.collision_interval) or args.collision_interval <= 0):
        parser.error('collision-interval must be positive and finite')
    try:
        engine_factory = workflow_engine_factory(
            lookahead_depth=args.lookahead_depth, lookahead_horizon=args.lookahead_horizon,
            rolling_passage=args.rolling_passage, crossing_stops=args.crossing_stops)
        model = create_model(robot_count=args.robots, order_count=args.orders, seed=args.seed,
                             full_cycle=not args.first_leg_only, collision_check=args.collision_check,
                             departure_policy=args.departure_policy, rolling_passage=args.rolling_passage,
                             crossing_stops=args.crossing_stops)
    except ValueError as error:
        parser.error(str(error))
    if args.animate:
        import sys
        from importlib import import_module
        from .WarehouseLayout import load_layout
        layout = load_layout(home_capacity=args.robots)
        by_position = {tuple(p): key for key, p in layout['control_points'].items()}
        routes = {r.name: (by_position[tuple(model.controller.roadmap.paths[r.name].vertices[0])],)
                  for r in model.robots}
        def build(**options):
            fresh = create_model(robot_count=args.robots, order_count=args.orders, seed=args.seed,
                                 full_cycle=not args.first_leg_only,
                                 departure_policy=args.departure_policy,
                                 rolling_passage=args.rolling_passage, crossing_stops=args.crossing_stops,
                                 collision_check=options.get('collision_check', False),
                                 initial_headings=options.get('initial_headings'))
            return fresh, fresh.robots, fresh.sensors
        original_argv = sys.argv
        sys.argv = [sys.argv[0], *viewer_args]
        try:
            import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
                name='RMFS_OrderWorkflow', grid=layout, robot_paths=routes,
                model_factory=build, engine_factory=engine_factory, collision_check=args.collision_check,
                stop_on_collision=True, hide_point_labels=True, render_fps=10)
        finally:
            sys.argv = original_argv
        return
    engine = engine_factory(model, TimeIntervalAnimation=args.collision_interval if args.collision_check else float('inf'))
    engine.bVerbose = False
    while isfinite(model.tNext):
        engine.execute_one_step()
    controller = model.controller
    print(f"Initial orders received: {controller.received_orders}")
    print(f"Active assignments: {len(controller.assignments)}, pending: {len(controller.pending_orders)}")
    if controller.full_cycle:
        print(f"Shipped: {controller.shipped_orders}; returned: {controller.completed_orders}")
        print(f"Robots home: {sum(s == 'home' for s in controller.robot_status.values())}")
        if controller.completed_orders != controller.received_orders or any(
                s != 'home' for s in controller.robot_status.values()):
            raise RuntimeError(f"Simulation stopped with unfinished jobs: {controller.robot_status}")
        last_return = max((a['time'] for a in controller.finished_assignments), default=0.)
        print(f"Last rack lowered: {last_return:.3f} s")
        print(f"All-home simulation time: {engine.tNow:.3f} s")
        print(f"Per-robot shipments: {controller.robot_counts}")
        if args.collision_check:
            print(f"Detected contacts: {len(model.collision_history)} (stop on collision)")
    else:
        print(f"Robots at rack: {sum(s == 'at_rack' for s in controller.robot_status.values())}")
        print(f"First-leg simulation time: {engine.tNow:.3f} s; completed orders: {controller.completed_orders}")


if __name__ == "__main__":
    main()
