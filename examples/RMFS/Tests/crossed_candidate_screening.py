"""Compare spatial and temporal screening against recorded X-case DEVS results."""

from argparse import ArgumentParser
from itertools import accumulate, combinations, pairwise
from json import dumps, loads
from math import atan2, dist, pi
from pathlib import Path

from ..MRobot import _EMPTY_YAW_SPEED, _LOADED_YAW_SPEED, _YAW_ACCELERATION
from ..motion import EMPTY, LOADED, StraightProfile
from ..path_geometry import RectangularFootprint


def route_timing(route, *, loaded=False):
    """Map free-flow route distance to time immediately before and after turns."""
    route = (route[0], *(b for a, b in pairwise(route) if dist(a, b) > 1e-9))
    directions = [atan2(b[1] - a[1], b[0] - a[0]) for a, b in pairwise(route)]
    offsets = tuple(accumulate((dist(a, b) for a, b in pairwise(route)), initial=0.0))
    if len(route) == 1:
        return offsets, [], 0.0
    corners = [0, *(i for i in range(1, len(directions))
                    if directions[i] != directions[i - 1]), len(route) - 1]
    runs = []
    elapsed = 0.0
    limits = LOADED if loaded else EMPTY
    yaw_speed = _LOADED_YAW_SPEED if loaded else _EMPTY_YAW_SPEED
    for start, end in pairwise(corners):
        length = offsets[end] - offsets[start]
        profile = StraightProfile(length, limits.acceleration, limits.deceleration, limits.speed)
        runs.append((offsets[start], offsets[end], elapsed, profile))
        elapsed += profile.duration
        if end < len(directions):
            angle = abs((directions[end] - directions[end - 1] + pi) % (2 * pi) - pi)
            elapsed += StraightProfile(angle, _YAW_ACCELERATION, _YAW_ACCELERATION,
                                       yaw_speed).duration
    return offsets, runs, elapsed


def occupancy(offsets, runs, first, last, padding):
    low = max(0.0, offsets[first] - padding)
    high = min(offsets[-1], offsets[last] + padding)
    times = []
    for start, end, elapsed, profile in runs:
        if low <= end and high >= start:
            times.append(elapsed + profile.time_at_distance(max(0.0, low - start)))
            times.append(elapsed + profile.time_at_distance(min(profile.length, high - start)))
    return min(times), max(times)


def overlap(first, second):
    return max(0.0, min(first[1], second[1]) - max(first[0], second[0]))


def bundle_metrics(paths):
    routes = [tuple(map(tuple, route)) for route in paths.values()]
    if any(len(route) < 2 or len(set(route)) != len(route) for route in routes):
        raise ValueError("X-case screening expects simple nonempty travel routes.")
    timings = [route_timing(route) for route in routes]
    padding = RectangularFootprint().width / 2
    crossings = reverse_runs = 0
    reverse_length = crossing_overlap = reverse_overlap = 0.0
    for (a, (a_offsets, a_runs, _)), (b, (b_offsets, b_runs, _)) in combinations(
        zip(routes, timings), 2
    ):
        b_index = {node: i for i, node in enumerate(b)}
        reversed_edges = [i for i, (u, v) in enumerate(pairwise(a))
                          if u in b_index and v in b_index and b_index[u] == b_index[v] + 1]
        groups = []
        for i in reversed_edges:
            if groups and i == groups[-1][1] and (
                a[i - 1][0] == a[i][0]) == (a[i][0] == a[i + 1][0]
            ):
                groups[-1][1] = i + 1
            else:
                groups.append([i, i + 1])
        reverse_vertices = set()
        for first, last in groups:
            reverse_runs += 1
            reverse_length += a_offsets[last] - a_offsets[first]
            reverse_vertices.update(a[first:last + 1])
            reverse_overlap += overlap(
                occupancy(a_offsets, a_runs, first, last, padding),
                occupancy(b_offsets, b_runs, b_index[a[last]], b_index[a[first]], padding),
            )
        for i, node in enumerate(a[1:-1], 1):
            j = b_index.get(node)
            if j is None or j == 0 or j == len(b) - 1 or node in reverse_vertices:
                continue
            a_adjacent = {a[i - 1], a[i + 1]}
            b_adjacent = {b[j - 1], b[j + 1]}
            # Shared approach/departure arcs indicate a merge or following, not a point crossing.
            if a_adjacent.isdisjoint(b_adjacent):
                crossings += 1
                crossing_overlap += overlap(
                    occupancy(a_offsets, a_runs, i, i, padding),
                    occupancy(b_offsets, b_runs, j, j, padding),
                )
    return dict(free_flow_total_s=sum(t[2] for t in timings), crossings=crossings,
                reverse_runs=reverse_runs, reverse_length_m=reverse_length,
                crossing_overlap_s=crossing_overlap, reverse_overlap_s=reverse_overlap,
                occupancy_overlap_s=crossing_overlap + reverse_overlap)


def rank(rows, columns):
    values = {column: sorted(round(row[column], 6) for row in rows.values()) for column in columns}
    return sorted(rows, key=lambda name: (
        sum(values[column].index(round(rows[name][column], 6)) for column in columns),
        name,
    ))


def compare(results):
    rows = {name: {**bundle_metrics(row["paths"]), **{
        key: row[key] for key in ("makespan_s", "energy_wh")
    }} for name, row in results.items() if "error" not in row}
    spatial = ("free_flow_total_s", "crossings", "reverse_runs", "reverse_length_m")
    orders = {"spatial": rank(rows, spatial),
              "spatiotemporal": rank(rows, (*spatial, "occupancy_overlap_s"))}
    objectives = ("makespan_s", "energy_wh")
    reference = {key: sorted(rows, key=lambda name: (rows[name][key], name))
                 for key in objectives}
    comparison = {}
    for method, order in orders.items():
        comparison[method] = {
            str(budget): {
                "candidates": order[:budget],
                "top_five_retained": {key: len(set(order[:budget]) & set(reference[key][:5]))
                                      for key in objectives},
                "best_included": {key: reference[key][0] in order[:budget] for key in objectives},
                "best_result": {key: min(rows[name][key] for name in order[:budget])
                                for key in objectives},
            }
            for budget in (5, 10, 15)
        }
    return dict(metrics=rows, orders=orders, reference=reference, comparison=comparison)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path, help="Recorded x-study JSON results.")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = compare(loads(args.results.read_text(encoding="utf-8")))
    if args.report:
        args.report.write_text(dumps(report, indent=2) + "\n", encoding="utf-8")
    print(dumps(report["comparison"], indent=2))


if __name__ == "__main__":
    main()
