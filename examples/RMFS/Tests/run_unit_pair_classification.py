"""Offline endpoint classification on the existing 4x4, 2 m grid.

Unit here means the common CENTERLINE vertex/edge subgraph is connected
(or empty). This explicit definition is narrower than an unspecified
connected overlap of arbitrary swept bodies. The 1.5 m square footprint
does not overlap nonincident grid edges.

Enumerates all ordered pairs of simple paths up to --max-edges. An independent
vertex-progress DAG tests feasibility; the production wait-duplication solver
cross-checks representative pairs in all 16 classes. No plant or viewer runs.
"""

import argparse
import json
from collections import Counter
from itertools import product
from pathlib import Path as FilePath

from ..GridPath import ADJACENCY, CONTROL_POINTS, SEGMENTS
from ..path_geometry import Path
from ..wait_duplication import WaitDuplicationSolver


def safe_order(flags):
    si, ei, sj, ej = flags
    if not sj and not ei:
        return "i_then_j"
    if not si and not ej:
        return "j_then_i"
    return None


def simple_paths(max_edges):
    vertex_ids = {p: i for i, p in enumerate(sorted(ADJACENCY))}
    edge_ids = {frozenset(e): i for i, e in enumerate(SEGMENTS)}
    rows = []

    def visit(route):
        if len(route) > 1:
            vertices = sum(1 << vertex_ids[p] for p in route)
            edges = sum(1 << edge_ids[frozenset((a, b))] for a, b in zip(route, route[1:]))
            # Integer boxes are exact: grid centers * 8; half side = 3.
            # Scaling coordinates by 4 removes floating-point comparisons.
            wait = [(8*x-3, 8*y-3, 8*x+3, 8*y+3) for x, y in route]
            move = [(min(a[0], b[0])*8-3, min(a[1], b[1])*8-3,
                     max(a[0], b[0])*8+3, max(a[1], b[1])*8+3)
                    for a, b in zip(route, route[1:])]
            rows.append(dict(route=tuple(route), vertices=vertices, edges=edges,
                             start=1 << vertex_ids[route[0]], end=1 << vertex_ids[route[-1]],
                             wait=wait, move=move))
        if len(route) == max_edges + 1:
            return
        for q in ADJACENCY[route[-1]]:
            if q not in route:
                visit([*route, q])

    for p in sorted(ADJACENCY):
        visit([p])
    return rows


def disjoint(a, b):
    return a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1]


def independent_reachability(a, b):
    """Progress DAG; no production geometry, solver or successor functions."""
    m, n = len(a["route"]), len(b["route"])
    if not disjoint(a["wait"][0], b["wait"][0]):
        return False
    reachable = {(0, 0)}
    for x in range(m):
        for y in range(n):
            if (x, y) not in reachable:
                continue
            if (x, y) == (m - 1, n - 1):
                return True
            for dx, dy in ((1, 0), (0, 1), (1, 1)):
                if x + dx >= m or y + dy >= n:
                    continue
                box_a = a["move"][x] if dx else a["wait"][x]
                box_b = b["move"][y] if dy else b["wait"][y]
                if disjoint(box_a, box_b):
                    reachable.add((x + dx, y + dy))
    return False


def run(max_edges=3, sample_limit=20):
    paths = simple_paths(max_edges)
    counts = {flags: Counter() for flags in product((0, 1), repeat=4)}
    samples = {flags: [] for flags in counts}
    checked = rejected_nonunit = rejected_initial = 0
    for a, b in product(paths, repeat=2):
        if a["start"] == b["start"]:
            rejected_initial += 1
            continue
        common = a["vertices"] & b["vertices"]
        shared_edges = a["edges"] & b["edges"]
        # Intersection of two simple paths is a forest: connected iff |E|=|V|-1.
        if common and shared_edges.bit_count() != common.bit_count() - 1:
            rejected_nonunit += 1
            continue
        flags = (int(bool(a["start"] & b["vertices"])),
                 int(bool(a["end"] & b["vertices"])),
                 int(bool(b["start"] & a["vertices"])),
                 int(bool(b["end"] & a["vertices"])))
        feasible = independent_reachability(a, b)
        predicted = safe_order(flags) is not None
        counts[flags]["feasible" if feasible else "infeasible"] += 1
        checked += 1
        if checked % 500000 == 0:
            print(f"Progress: {checked} unit/independent pairs checked", flush=True)
        if feasible != predicted:
            raise AssertionError(("Endpoint rule counterexample", flags, a["route"], b["route"]))
        if len(samples[flags]) < sample_limit:
            metric = {name: Path(name, tuple(CONTROL_POINTS[p] for p in row["route"]))
                      for name, row in (("i", a), ("j", b))}
            solver = WaitDuplicationSolver(metric)
            result = solver.solve()
            assert (result.status == "feasible") == feasible
            if feasible:
                solver.validate(result)
            samples[flags].append(dict(i=a["route"], j=b["route"], feasible=feasible))
    rows = []
    for flags, counter in counts.items():
        row = dict(flags=flags, order=safe_order(flags), counts=dict(counter), samples=samples[flags])
        rows.append(row)
        print(f"{''.join(map(str,flags))}: {dict(counter)}; order={safe_order(flags)}", flush=True)
    return dict(grid_points=4, spacing_m=2, footprint_m=[1.5, 1.5], max_edges=max_edges,
                definition="connected common centerline subpath, empty allowed",
                paths=len(paths), checked_ordered_pairs=checked,
                rejected_initial_overlap=rejected_initial,
                rejected_disconnected_common_path=rejected_nonunit,
                production_crosschecks=sum(len(v) for v in samples.values()), rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-edges", type=int, default=3, choices=range(1, 6))
    parser.add_argument("--output", type=FilePath)
    args = parser.parse_args()
    result = run(args.max_edges)
    print(f"Checked {result['checked_ordered_pairs']} pairs; production cross-checks "
          f"{result['production_crosschecks']}; no disagreements.")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
