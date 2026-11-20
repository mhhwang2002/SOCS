"""Interior traffic fixtures, not a new runtime path planner.

Fixed seeds construct seven collision-free simultaneous grid matchings.
Concatenating each object's assigned moves produces its fixed simple route.
The controller executes those routes through a fully validated tuple witness.
This is a constructive demo, not a claim about exhaustive solver scalability.
"""

from math import atan2, degrees
from random import Random

from .GridPath import make_grid
from .controlled import create_controlled_model
from .convoy import simultaneous_witness


SEEDS = {4: 20261119, 10: 20261223, 20: 20261385}


def make_interior_traffic(size):
    grid = make_grid(size)
    rng = Random(SEEDS[size])
    routes = [[p] for p in grid["control_points"] if sum(p) % 2 == 0]
    for _ in range(7):
        candidates = {i: [q for q in grid["adjacency"][route[-1]] if q not in route]
                      for i, route in enumerate(routes)}
        for choices in candidates.values():
            rng.shuffle(choices)
        assigned = {}

        def augment(robot, seen):
            for point in candidates[robot]:
                if point in seen:
                    continue
                seen.add(point)
                if point not in assigned or augment(assigned[point], seen):
                    assigned[point] = robot
                    return True
            return False

        order = list(range(len(routes)))
        rng.shuffle(order)
        if not all(augment(i, set()) for i in order):
            raise ValueError("The fixed interior-traffic fixture cannot be constructed.")
        destinations = {i: point for point, i in assigned.items()}
        for i, route in enumerate(routes):
            route.append(destinations[i])
    paths = {f"R{i+1}": tuple(route) for i, route in enumerate(routes)}
    if not all(any(0 < x < size - 1 and 0 < y < size - 1 for x, y in route)
               for route in paths.values()):
        raise ValueError("Every demo object must visit an interior vertex.")
    headings = {name: degrees(atan2(route[1][1] - route[0][1],
                                   route[1][0] - route[0][0]))
                for name, route in paths.items()}
    return grid, paths, headings


def create_interior_model(size, **options):
    grid, paths, headings = make_interior_traffic(size)
    options.setdefault("initial_headings", headings)
    return create_controlled_model(paths, name=f"InteriorTraffic_{len(paths)}", grid=grid,
                                   plan_builder=simultaneous_witness, **options)


def run_interior(size):
    from importlib import import_module
    grid, paths, headings = make_interior_traffic(size)
    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name=f"InteriorTraffic_{len(paths)}_{size}x{size}", no_rack=True, grid=grid,
        model_factory=lambda **opts: create_interior_model(size, **opts),
        robot_paths=paths, initial_headings=headings,
        collision_check=True, stop_on_collision=True)
