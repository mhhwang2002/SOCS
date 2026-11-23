"""Compile/cache directed, uninterrupted straight runs on an existing roadmap.

No DEVS models or traffic policies are changed. Macro edges retain every
underlying vertex: drawing them curved does not make the physical route curved.
"""
import argparse
from dataclasses import asdict
from hashlib import sha256
import json
from math import atan2, degrees, dist, isfinite
from pathlib import Path

from .motion import EMPTY, LOADED, StraightProfile

SCHEMA_VERSION = 1


def _json_id(value):
    if isinstance(value, tuple):
        return [_json_id(v) for v in value]
    if isinstance(value, (str, int)):
        return value
    if isinstance(value, float) and isfinite(value):
        return value
    raise ValueError("Roadmap IDs must be strings, finite numbers or tuples of these.")


def _inputs(layout, profiles):
    points = {key: tuple(float(x) for x in xy)
              for key, xy in layout["control_points"].items()}
    if not points or any(len(xy) != 2 or not all(map(isfinite, xy))
                         for xy in points.values()):
        raise ValueError("A roadmap needs finite 2D vertex positions.")
    order = sorted(points, key=lambda key: json.dumps(_json_id(key)))
    adjacency = layout["adjacency"]
    if set(adjacency) != set(points):
        raise ValueError("Adjacency and vertex IDs must agree.")
    rank = {key: k for k, key in enumerate(order)}
    neighbors = {}
    for key in order:
        values = tuple(adjacency[key])
        if any(v not in points or dist(points[key], points[v]) <= 0 for v in values):
            raise ValueError("Every arc must connect distinct known positions.")
        if any(points[key][0] != points[v][0] and points[key][1] != points[v][1]
               for v in values):
            raise ValueError("Only horizontal or vertical existing layout arcs are supported.")
        neighbors[key] = tuple(sorted(set(values), key=rank.__getitem__))
    if not profiles or any(not isinstance(name, str) or not name for name in profiles):
        raise ValueError("At least one named motion profile is required.")
    parameters = {name: asdict(limits) for name, limits in sorted(profiles.items())}
    if any(not isfinite(v) or v <= 0 for p in parameters.values() for v in p.values()):
        raise ValueError("All motion limits must be positive and finite.")
    payload = dict(nodes=[dict(id=_json_id(k), position_m=points[k]) for k in order],
                   arcs=[[_json_id(k), _json_id(v)] for k in order for v in neighbors[k]],
                   profiles=parameters, schema_version=SCHEMA_VERSION)
    fingerprint = sha256(json.dumps(payload, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return points, order, neighbors, parameters, fingerprint


def compile_motion_graph(layout, profiles=None):
    """Add all forward horizontal/vertical runs, retaining original directed arcs.

    Only existing arcs can be followed. No shortcut across a missing segment,
    a turn, or a prohibited direction is introduced. Fixed relative tolerance
    below is part of schema v1's collinearity policy.
    """
    profiles = {"empty": EMPTY, "loaded": LOADED} if profiles is None else profiles
    points, order, neighbors, parameters, fingerprint = _inputs(layout, profiles)
    edges = []
    for source in order:
        for first in neighbors[source]:
            delta = tuple(b-a for a, b in zip(points[source], points[first]))
            length = dist(points[source], points[first])
            direction = tuple(x/length for x in delta)
            heading = degrees(atan2(direction[1], direction[0]))
            pending = [(first, (source, first), length)]
            while pending:
                target, path, length = pending.pop()
                edges.append(dict(source=_json_id(source), target=_json_id(target),
                                  path=[_json_id(v) for v in path],
                                  intermediate_points=[_json_id(v) for v in path[1:-1]],
                                  kind="original" if len(path) == 2 else "pass_through",
                                  length_m=length, entry_heading_deg=heading,
                                  exit_heading_deg=heading,
                                  travel_time_s={name: StraightProfile(
                                      length, p["acceleration"], p["deceleration"],
                                      p["speed"]).duration for name, p in parameters.items()}))
                for nxt in reversed(neighbors[target]):
                    if nxt in path:
                        continue
                    step = tuple(b-a for a, b in zip(points[target], points[nxt]))
                    d = dist(points[target], points[nxt])
                    unit = tuple(x/d for x in step)
                    if (abs(direction[0]*unit[1]-direction[1]*unit[0]) <= 1e-12
                            and sum(a*b for a, b in zip(direction, unit)) > 0):
                        pending.append((nxt, (*path, nxt), length+d))
    return dict(schema_version=SCHEMA_VERSION, fingerprint=fingerprint,
                units=dict(length="metre", time="second", heading="degree"),
                boundary_condition="rest_to_rest; no stops at intermediate points",
                traffic_delay_included=False, turn_time_included=False,
                turn_time_rule="abs(shortest_heading_change_deg) / profile.yaw_rate",
                profiles=parameters,
                nodes=[dict(id=_json_id(k), position_m=list(points[k])) for k in order],
                edges=edges,
                counts=dict(nodes=len(points), original_arcs=sum(map(len, neighbors.values())),
                            pass_through_arcs=sum(e["kind"] == "pass_through" for e in edges),
                            total_arcs=len(edges)))


def save_motion_graph(graph, filename):
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)
    filename.write_text(json.dumps(graph, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def load_motion_graph(filename, *, layout=None, profiles=None):
    graph = json.loads(Path(filename).read_text(encoding="utf-8"))
    if graph.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported compiled motion graph schema.")
    if layout is not None:
        profiles = {"empty": EMPTY, "loaded": LOADED} if profiles is None else profiles
        fingerprint = _inputs(layout, profiles)[-1]
        if graph.get("fingerprint") != fingerprint:
            raise ValueError("Compiled graph is stale: layout or motion limits changed.")
    return graph


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", choices=("grid", "l-shape", "warehouse"), default="grid")
    parser.add_argument("--points", type=int, default=4, help="Grid points per axis, not cells")
    parser.add_argument("--spacing", type=float, default=2.0, help="Grid spacing in metres")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--force", action="store_true", help="Recompile even a valid cache")
    args = parser.parse_args()
    if args.layout in ("grid", "l-shape"):
        from .GridPath import make_grid
        layout = make_grid(args.points, args.spacing)
        name = f"grid{args.points}_motion_graph.json"
        if args.layout == "l-shape":
            selected = {k for k in layout["control_points"] if 0 in k}
            layout = dict(control_points={k: v for k, v in layout["control_points"].items()
                                         if k in selected},
                          adjacency={k: tuple(v for v in values if v in selected)
                                     for k, values in layout["adjacency"].items() if k in selected})
            name = f"l{args.points}_motion_graph.json"
    else:
        from .WarehouseLayout import load_layout
        layout = load_layout()
        name = "warehouse_motion_graph.json"
    target = args.output or Path(__file__).with_name("precomputed")/name
    reused = False
    if target.exists() and not args.force:
        try:
            graph = load_motion_graph(target, layout=layout)
            reused = True
        except (ValueError, KeyError, TypeError):
            graph = compile_motion_graph(layout)
    else:
        graph = compile_motion_graph(layout)
    if not reused:
        save_motion_graph(graph, target)
    print("Reused" if reused else "Compiled", target.resolve())
    print(json.dumps(graph["counts"]))


if __name__ == "__main__":
    main()
