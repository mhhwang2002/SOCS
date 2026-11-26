"""Compare initial unloaded routes for two robots on a 5×5 grid."""

from argparse import ArgumentParser
from importlib import import_module
from json import loads
from math import isfinite

from socs.devs.TESEngin import TESEnginBase

from ..GridPath import make_grid
from ..MotionEnergyCollector import attach_motion_collector
from ..RoadMap import RoadMap
from ..RobotPathPlanner import RobotPathPlanner
from ..motion_graph import compile_motion_graph
from ..path_geometry import Path, PathPosition, Subpath, sweep_2d
from .crossing_stop_comparison import create_model as create_crossing_model
from .run_crossing_energy_comparison import DEFAULT_CONFIG, fleet_energy


STARTS = {"R1": (2, 2), "R2": (1, 1)}
RACKS = {"R1": (0, 3), "R2": (3, 3)}
FOCUS = "R2"


def plan_candidates(grid):
    points = grid["control_points"]
    roadmap = RoadMap({name: Path(name, (points[start],))
                       for name, start in STARTS.items()}, layout=grid)
    planner = RobotPathPlanner(roadmap, compile_motion_graph(grid))

    def plan(name, traffic_routes=()):
        blocked = [points[node] for other, node in STARTS.items() if other != name]
        blocked.extend(points[node] for other, node in RACKS.items() if other != name)
        return planner.plan(name, {"goal_position": points[RACKS[name]]},
                            blocked_positions=blocked, traffic_routes=traffic_routes)

    candidates = {}
    for label in ("fastest", "sweep-aware"):
        paths, traffic = {}, []
        for name in STARTS:
            route = plan(name, traffic if label == "sweep-aware" else ())
            paths[name] = route
            traffic.append(Subpath(Path(name, route), PathPosition(0),
                                   PathPosition(len(route) - 1)))
        candidates[label] = paths
    return candidates


def create_model(grid, paths, **options):
    by_position = {point: node for node, point in grid["control_points"].items()}
    node_paths = {name: tuple(by_position[p] for p in route) for name, route in paths.items()}
    crossings = {node for node, neighbors in grid["adjacency"].items() if len(neighbors) >= 3}
    return create_crossing_model(node_paths, grid, crossings, push_pull=True,
                                 allow_crossing_stops=True, **options)


def evaluate(grid, paths, **options):
    model, robots, _ = create_model(grid, paths, collision_check=True, **options)
    collector = attach_motion_collector(model, robots)
    engine = TESEnginBase(model, TimeIntervalAnimation=.02)
    engine.bVerbose = False
    while isfinite(model.tNext):
        engine.execute_one_step()
    if not model.controller.traffic_manager.done or model.collision_history:
        raise RuntimeError("The route set did not complete without detected contact.")
    config = loads(DEFAULT_CONFIG.read_text(encoding="utf-8-sig"))
    arrivals, energy = fleet_energy(config, collector, paths, loaded=False)
    roadmap = model.controller.roadmap
    sweeps = {name: sweep_2d(Subpath(roadmap.paths[name], PathPosition(0),
                                   PathPosition(len(route) - 1)),
                            roadmap.footprints[name], roadmap.policy)
              for name, route in paths.items()}
    conflicts = tuple(name for name in paths if name != FOCUS and
                      sweeps[FOCUS].intersects(sweeps[name], roadmap.policy.tolerance))
    stops = sum(event["event"] == "arrived"
                and tuple(event["position"]) != paths[event["robot"]][-1]
                for event in roadmap.history)
    return max(arrivals.values()), sum(energy.values()), arrivals, conflicts, stops


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--animate", choices=("fastest", "sweep-aware"))
    args, viewer_args = parser.parse_known_args()
    grid = make_grid(5)
    candidates = plan_candidates(grid)
    if args.animate:
        paths = candidates[args.animate]
        by_position = {point: node for node, point in grid["control_points"].items()}
        routes = {name: tuple(by_position[p] for p in route) for name, route in paths.items()}
        import_module(".01_01_RobotPath_Straight", __package__).main(
            name=f"5×5 unloaded rack dispatch — {args.animate}", grid=grid,
            robot_paths=routes, no_rack=True, collision_check=True, argv=viewer_args,
            model_factory=lambda **options: create_model(grid, paths, **options))
        return
    if viewer_args:
        parser.error("unrecognized arguments: " + " ".join(viewer_args))
    for label, paths in candidates.items():
        makespan, wh, arrivals, conflicts, stops = evaluate(grid, paths)
        print(f"{label}:")
        for name, route in paths.items():
            print(f"  A* generated {name} path {route}")
        times = ", ".join(f"{name}={arrival:.3f} s" for name, arrival in arrivals.items())
        print(f"  overlaps={conflicts}; arrivals: {times}; "
              f"makespan={makespan:.3f} s; fleet energy={wh:.3f} Wh; stops={stops}; contacts=0")


if __name__ == "__main__":
    main()
