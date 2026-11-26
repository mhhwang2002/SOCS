"""Inspect A* routes from warehouse homes to clustered racks."""

from argparse import ArgumentParser
from functools import partial
from importlib import import_module
from itertools import combinations, permutations
from json import dumps
from math import dist
from pathlib import Path as FilePath

from ..RoadMap import RoadMap
from ..RobotPathPlanner import RobotPathPlanner
from ..TrafficManager import TrafficManager
from ..WarehouseLayout import load_layout, shortest_route
from ..motion import EMPTY, StraightProfile
from ..motion_graph import compile_motion_graph
from ..path_geometry import Path, PathPosition, Subpath
from .grid_route_planning import create_model, evaluate
from .incremental_routing import dag_witness


STARTS = {"R1": (2.0, -19.5), "R2": (15.725, -19.5)}
ASSIGNED_RACKS = {"R1": (0, 2, 0), "R2": (1, 2, 1)}
SHARED_AISLE_STARTS = {"R1": STARTS["R1"], "R2": (7.725, -19.5)}
SHARED_AISLE_RACKS = {"R1": ASSIGNED_RACKS["R1"], "R2": (0, 3, 0)}
CROSS_STARTS = {"R1": (0., -19.5), "R2": STARTS["R1"],
                "R3": STARTS["R2"], "R4": (17.925, -19.5)}
CROSS_RACKS = {"R1": (1, 3, 1), "R2": (1, 2, 1), "R3": (0, 3, 0), "R4": (0, 2, 0)}
CROSS_VARIANTS = {
    "fastest": {},
    "sweep-aware": {"sweep_aware": True},
    "no-head-on": {"sweep_aware": True, "avoid_head_on": True},
    "right-first": {"sweep_aware": True, "planning_order": ("R3", "R4", "R1", "R2")},
    "alternating": {"sweep_aware": True, "planning_order": ("R1", "R3", "R2", "R4")},
}


def cluster_layout(*, crossed=False):
    warehouse = load_layout()
    points = {key: point for key, point in warehouse["control_points"].items()
              if 0 <= point[0] <= 18 and -20 <= point[1] <= 0}
    segments = tuple((a, b) for a, b in warehouse["segments"] if a in points and b in points)
    adjacency = {key: tuple(b for b in warehouse["adjacency"][key] if b in points)
                 for key in points}
    if crossed:
        # Add the lower-left home spur drawn in the four-robot layout.
        home, aisle = CROSS_STARTS["R1"], (0., -16.)
        points[home] = home
        segments += ((home, aisle),)
        adjacency[home] = (aisle,)
        adjacency[aisle] += (home,)
    return dict(
        control_points=points,
        segments=segments,
        adjacency=adjacency,
        extent=20.0,
        warehouse_racks={cell: point for cell, point in warehouse["warehouse_racks"].items()
                         if cell[0] in (0, 1)},
        warehouse_stations={},
    )


def plan_routes(grid, starts=STARTS, assigned_racks=ASSIGNED_RACKS, *,
                sweep_aware=False, avoid_head_on=False, planning_order=None):
    roadmap = RoadMap({name: Path(name, (point,)) for name, point in starts.items()}, layout=grid)
    graph = compile_motion_graph(grid)
    planner = RobotPathPlanner(roadmap, graph)
    paths, traffic = {}, []
    reverse_arcs = set()
    for name in assigned_racks if planning_order is None else planning_order:
        cell = assigned_racks[name]
        if avoid_head_on and reverse_arcs:
            edges = []
            for edge in graph["edges"]:
                nodes = tuple(map(tuple, edge["path"]))
                if reverse_arcs.isdisjoint(zip(nodes, nodes[1:])):
                    edges.append(edge)
            planner = RobotPathPlanner(roadmap, {**graph, "edges": edges})
        route = planner.plan(name, {"goal_position": grid["warehouse_racks"][cell]},
                             traffic_routes=traffic)
        paths[name] = route
        reverse_arcs.update(zip(route[1:], route))
        if sweep_aware:
            traffic.append(Subpath(Path(name, route), PathPosition(0),
                                   PathPosition(len(route) - 1)))
    # Keep simulator tie order independent of planning order.
    return {name: paths[name] for name in starts}


def head_on_length(paths):
    """Sum shared opposite-direction aisle lengths over unordered robot pairs."""
    arcs = [set(zip(route, route[1:])) for route in paths.values()]
    return sum(dist(a, b) for first, second in combinations(arcs, 2)
               for a, b in first if (b, a) in second)


def crossed_candidates(grid):
    """Generate distinct A* route bundles in a deterministic order."""
    candidates = {}
    seen = set()
    options = list(CROSS_VARIANTS.items())
    for order in permutations(CROSS_STARTS):
        suffix = "-".join(order)
        options.extend((
            (f"sweep-{suffix}", {"sweep_aware": True, "planning_order": order}),
            (f"no-head-on-{suffix}", {"sweep_aware": True, "avoid_head_on": True,
                                     "planning_order": order}),
        ))
    for label, settings in options:
        try:
            paths = plan_routes(grid, CROSS_STARTS, CROSS_RACKS, **settings)
        except ValueError:
            if not settings.get("avoid_head_on"):
                raise
            continue
        signature = tuple(paths.values())
        if signature not in seen:
            seen.add(signature)
            candidates[label] = paths
    return candidates


def free_flow_total(paths):
    """Sum unloaded, uninterrupted travel times over the fleet."""
    total = 0.0
    for route in paths.values():
        corners = (route[0], *turning_points(route), route[-1])
        total += sum(StraightProfile(dist(a, b), EMPTY.acceleration,
                                     EMPTY.deceleration, EMPTY.speed).duration
                     for a, b in zip(corners, corners[1:]))
        # The crossed layout has axis-aligned runs and a two-second unloaded turn.
        total += (len(corners) - 2) * 90 / EMPTY.yaw_rate
    return total


def screen_crossed_candidates(candidates, slack=0.06):
    """Apply the crossed-example free-flow budget before simulation."""
    times = {label: free_flow_total(paths) for label, paths in candidates.items()}
    budget = min(times.values()) * (1 + slack)
    return {label: paths for label, paths in candidates.items()
            if times[label] <= budget}


def study_crossed_candidates(grid, candidates, retained, report):
    results = {}
    for label, paths in candidates.items():
        row = dict(paths=paths, retained=label in retained,
                   free_flow_total_s=free_flow_total(paths),
                   head_on_pair_m=head_on_length(paths))
        try:
            makespan, wh, arrivals, _, stops = evaluate(grid, paths, **CROSSED_EXECUTION)
            row.update(makespan_s=makespan, energy_wh=wh, arrivals_s=arrivals, stops=stops)
            print(f"{label}: {makespan:.3f} s, {wh:.3f} Wh, {stops} stops", flush=True)
        except (ValueError, RuntimeError) as error:
            row["error"] = str(error)
            print(f"{label}: No validated completion: {error}", flush=True)
        results[label] = row
    if report is not None:
        report.write_text(dumps(results, indent=2) + "\n", encoding="utf-8")


def crossed_witness(solver):
    return dag_witness(solver, {name: set() for name in solver.names})


CROSSED_EXECUTION = {
    "plan_builder": crossed_witness,
    "traffic_manager_type": partial(TrafficManager, allow_crossing_stops=True),
}


def run_crossed_case(grid, label, routes):
    print(f"{label}: {head_on_length(routes):.3f} pair-metres of opposing shared aisle")
    for name, route in routes.items():
        print(f"  {name} corners {turning_points(route)}")
    makespan, wh, arrivals, _, stops = evaluate(grid, routes, **CROSSED_EXECUTION)
    times = ", ".join(f"{name}={arrival:.3f} s" for name, arrival in arrivals.items())
    print(f"  arrivals: {times}; makespan={makespan:.3f} s; "
          f"fleet energy={wh:.3f} Wh; stops={stops}; contacts=0")


def turning_points(route):
    directions = [tuple((y - x) / dist(a, b) for x, y in zip(a, b))
                  for a, b in zip(route, route[1:])]
    return tuple(point for point, a, b in zip(route[1:], directions, directions[1:]) if a != b)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--animate", action="store_true")
    parser.add_argument("--candidate", help="Select a generated label in x-screen or x-study.")
    parser.add_argument("--report", type=FilePath, help="Save the x-study results as JSON.")
    parser.add_argument("--scenario", choices=("separate", "fastest", "sweep-aware", "compare",
                        "x-compare", "x-screen", "x-study",
                        *(f"x-{label}" for label in CROSS_VARIANTS)),
                        default="separate", help="Select a rack assignment and A* routing variant.")
    args, viewer_args = parser.parse_known_args()
    grid = cluster_layout(crossed=args.scenario.startswith("x-"))
    if args.candidate and args.scenario not in ("x-screen", "x-study"):
        parser.error("--candidate requires x-screen or x-study.")
    if args.report and (args.scenario != "x-study" or args.animate or args.candidate):
        parser.error("--report requires a full x-study run.")
    if args.scenario in ("x-screen", "x-study"):
        candidates = crossed_candidates(grid)
        retained = screen_crossed_candidates(candidates)
        selected = candidates if args.scenario == "x-study" else retained
        if args.candidate:
            if args.candidate not in selected:
                parser.error(f"Unknown candidate in {args.scenario}: {args.candidate}")
            routes = selected[args.candidate]
            if args.animate:
                animate_routes(grid, routes, True, viewer_args)
            elif viewer_args:
                parser.error("Viewer options require --animate.")
            else:
                run_crossed_case(grid, args.candidate, routes)
        elif args.animate or viewer_args:
            parser.error("Select --candidate to animate one route set.")
        elif args.scenario == "x-study":
            study_crossed_candidates(grid, candidates, retained, args.report)
        else:
            print(f"Retained {len(retained)} of {len(candidates)} route sets.")
            for label, paths in retained.items():
                print(f"{label}: {free_flow_total(paths):.3f} free-flow robot-s, "
                      f"{head_on_length(paths):.3f} opposing pair-m")
        return
    if args.scenario == "x-compare":
        if args.animate or viewer_args:
            parser.error("Select one x- scenario to animate.")
        seen = set()
        for label, planning_options in CROSS_VARIANTS.items():
            routes = plan_routes(grid, CROSS_STARTS, CROSS_RACKS, **planning_options)
            signature = tuple(routes.values())
            if signature in seen:
                print(f"{label}: duplicate route set")
                continue
            seen.add(signature)
            try:
                run_crossed_case(grid, label, routes)
            except (ValueError, RuntimeError) as error:
                print(f"  No validated completion: {error}")
        return
    if args.scenario == "compare":
        if args.animate or viewer_args:
            parser.error("Select fastest or sweep-aware to animate one shared-aisle route set.")
        for label in ("fastest", "sweep-aware"):
            routes = plan_routes(grid, SHARED_AISLE_STARTS, SHARED_AISLE_RACKS,
                                 sweep_aware=label == "sweep-aware")
            makespan, wh, arrivals, conflicts, stops = evaluate(grid, routes)
            print(f"{label}: R2 corners {turning_points(routes['R2'])}; overlaps={conflicts}")
            print(f"  R1 arrival={arrivals['R1']:.3f} s; R2 arrival={arrivals['R2']:.3f} s; "
                  f"makespan={makespan:.3f} s; fleet energy={wh:.3f} Wh; stops={stops}; contacts=0")
        return
    if args.scenario.startswith("x-"):
        starts, racks = CROSS_STARTS, CROSS_RACKS
        planning_options = CROSS_VARIANTS[args.scenario.removeprefix("x-")]
    elif args.scenario == "separate":
        starts, racks = STARTS, ASSIGNED_RACKS
        planning_options = {}
    else:
        starts, racks = SHARED_AISLE_STARTS, SHARED_AISLE_RACKS
        planning_options = {"sweep_aware": args.scenario == "sweep-aware"}
    routes = plan_routes(grid, starts, racks, **planning_options)
    if not args.animate and viewer_args:
        parser.error("Viewer options require --animate: " + " ".join(viewer_args))
    if args.scenario.startswith("x-") and not args.animate:
        run_crossed_case(grid, args.scenario.removeprefix("x-"), routes)
        return
    for name, route in routes.items():
        shortest = shortest_route(grid, route[0], route[-1])
        route_length = sum(dist(a, b) for a, b in zip(route, route[1:]))
        min_length = sum(dist(a, b) for a, b in zip(shortest, shortest[1:]))
        corners = turning_points(route)
        print(f"{name}: home {route[0]} -> rack {racks[name]} at {route[-1]}; "
              f"corners={corners}; {route_length:.3f} m (distance minimum {min_length:.3f} m)")
    if args.animate:
        animate_routes(grid, routes, args.scenario.startswith("x-"), viewer_args)


def animate_routes(grid, routes, crossed, viewer_args):
    model_options = CROSSED_EXECUTION if crossed else {}
    import_module(".01_01_RobotPath_Straight", __package__).main(
        name="Warehouse clusters — unloaded rack dispatch", grid=grid,
        robot_paths=routes, no_rack=True, collision_check=True,
        hide_point_labels=True, argv=viewer_args,
        model_factory=lambda **options: create_model(grid, routes, **model_options, **options))


if __name__ == "__main__":
    main()
