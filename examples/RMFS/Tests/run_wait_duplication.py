"""Evaluate all existing two/three-Frogy paths without starting a viewer.

python -m examples.RMFS.Tests.run_wait_duplication --output <report.json>
The original uncontrolled examples are not changed. Coordinates are converted
from grid indices to metres. This validates untimed path tuples, not dynamics.
"""

import argparse
import importlib
import json
from pathlib import Path as FilePath

from ..GridPath import CONTROL_POINTS
from ..path_geometry import Path, PathPosition, RectangularFootprint, interference_2d
from ..wait_duplication import WaitDuplicationSolver


def example_paths(module_name):
    module = importlib.import_module(f"examples.RMFS.Tests.{module_name}")
    return {name: Path(name, tuple(CONTROL_POINTS[p] for p in points))
            for name, points in module.PATHS.items()}


def run_all():
    results = []
    for source in sorted(FilePath(__file__).parent.glob("0[23]_*.py")):
        paths = example_paths(source.stem)
        solver = WaitDuplicationSolver(paths)
        result = solver.solve()
        interference = {}
        endpoints_safe = True
        for name, path in paths.items():
            data = interference_2d(path, RectangularFootprint(),
                                   {key: (other, RectangularFootprint())
                                    for key, other in paths.items() if key != name})
            encode = lambda p: [p.segment, p.ratio]
            interference[name] = [[encode(i.start), encode(i.end)] for i in data.intervals]
            endpoints_safe &= not (data.contains(PathPosition(0)) or
                                   data.contains(PathPosition(len(path.vertices) - 1)))
        if result.status == "feasible":
            solver.validate(result)
        elif endpoints_safe:
            raise AssertionError(f"Sufficient-condition violation: {source.stem}")
        row = dict(example=source.stem, status=result.status,
                   sufficient_condition=endpoints_safe,
                   stages=len(result.tuple_path.stages) if result.tuple_path else None,
                   visited_states=result.visited_states,
                   checked_transitions=result.checked_transitions,
                   elapsed_seconds=result.elapsed_seconds,
                   interference=interference,
                   consumed_counts=result.consumed_counts,
                   tuples=([[dict(start=[p.start.segment, p.start.ratio],
                                  end=[p.end.segment, p.end.ratio], wait=p.is_point)
                             for p in stage] for stage in result.tuple_path.stages]
                           if result.tuple_path else None))
        results.append(row)
        print(f"{source.stem}: {result.status}; stages={row['stages']}; "
              f"states={result.visited_states}; {result.elapsed_seconds:.4f}s")
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=FilePath)
    args = parser.parse_args()
    rows = run_all()
    print(f"Total {len(rows)}, feasible {sum(r['status'] == 'feasible' for r in rows)}, "
          f"infeasible {sum(r['status'] == 'infeasible' for r in rows)}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(dict(
            units="metres", footprint=[1.5, 1.5], contact_is_interference=True,
            tolerance=1e-9, safety_margin=0.0,
            scope="fixed vertex decomposition, wait duplication, untimed disjoint sweeps",
            results=rows), indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
