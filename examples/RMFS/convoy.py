"""Constructive, fully validated convoy demo; each route is a simple prefix.

The road perimeter is cyclic, but no individual path revisits any vertex.
This demonstrates a known feasible tuple sequence, not general solver scaling.
"""

from math import atan2, degrees

from .GridPath import make_grid
from .controlled import create_controlled_model
from .path_geometry import TuplePath
from .wait_duplication import SearchResult


def make_convoy(size, steps=5):
    grid = make_grid(size)
    last = size - 1
    ring = (tuple((x, 0) for x in range(last))
            + tuple((last, y) for y in range(last))
            + tuple((x, last) for x in range(last, 0, -1))
            + tuple((0, y) for y in range(last, 0, -1)))
    if not 0 < steps < len(ring):
        raise ValueError("Convoy paths must be nonempty and shorter than the perimeter.")
    routes = {f"R{i+1}": tuple(ring[(start+k) % len(ring)] for k in range(steps+1))
              for i, start in enumerate(range(0, len(ring), 2))}
    headings = {name: degrees(atan2(route[1][1]-route[0][1], route[1][0]-route[0][0]))
                for name, route in routes.items()}
    return grid, routes, headings


def simultaneous_witness(solver):
    lengths = {len(word) for word in solver.words}
    if len(lengths) != 1:
        raise ValueError("Convoy witness requires equal constituent counts.")
    count = lengths.pop()
    rows = tuple(tuple(word[k] for word in solver.words) for k in range(count))
    states = tuple((k,) * len(solver.names) for k in range(count + 1))
    return SearchResult("feasible", TuplePath(solver.names, rows), states,
                        count + 1, count, 0.0,
                        "Constructed synchronized witness; no exhaustive search.")


def create_convoy_model(size, **options):
    grid, routes, headings = make_convoy(size)
    options.setdefault("initial_headings", headings)
    return create_controlled_model(routes, name=f"Convoy_{len(routes)}", grid=grid,
                                   plan_builder=simultaneous_witness, **options)


def run_convoy(size):
    from importlib import import_module
    grid, routes, headings = make_convoy(size)
    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name=f"Convoy_{len(routes)}_{size}x{size}", no_rack=True, grid=grid,
        model_factory=lambda **opts: create_convoy_model(size, **opts),
        robot_paths=routes, initial_headings=headings,
        collision_check=True, stop_on_collision=True)
