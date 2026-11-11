"""Compare two-fork pickup policies across controlled inventory layouts."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

from examples.ASRS.aco_optimization import load_instance
from .aco_optimization import TwoForkAntColonyOptimizer
from .timing import DEFAULT_TIMING_PROFILE, TimingProfile


DEFAULT_INSTANCE_DIR = Path(__file__).with_name("instances") / "pairability"
DEFAULT_OUTPUT = DEFAULT_INSTANCE_DIR / "comparison.csv"
SCENARIO_FILES = (
    "pairability_almost-none.json",
    "pairability_low.json",
    "pairability_medium.json",
    "pairability_high.json",
)
MODES = ("single", "double", "sequential", "all")


def compare(instance_dir, output, seeds, ants, iterations, objective, penalty_weight, timing_profile):
    rows = []
    for filename in SCENARIO_FILES:
        path = Path(instance_dir) / filename
        payload = json.loads(path.read_text(encoding="utf-8"))
        metadata = payload["layout_scenario"]
        instance = load_instance(path)
        for seed in seeds:
            for mode in MODES:
                optimizer = TwoForkAntColonyOptimizer(
                    instance,
                    objective,
                    seed,
                    ants=ants,
                    iterations=iterations,
                    penalty_weight=penalty_weight,
                    timing_profile=timing_profile,
                    picking_mode=mode,
                )
                schedule, result, _ = optimizer.run()
                mode_counts = {
                    name: schedule.pickup_modes.count(name)
                    for name in ("single", "simultaneous", "sequential")
                }
                rows.append({
                    "scenario": metadata["name"],
                    "adjacent_pairs": metadata["adjacent_occupied_pairs"],
                    "order_compatible_pairs": metadata["order_compatible_adjacent_pairs"],
                    "seed": seed,
                    "mode": mode,
                    "makespan": result.makespan,
                    "mean_order_departure": result.mean_order_completion_time,
                    "complete_orders": result.complete_order_count,
                    "shortage_units": result.shortage_units,
                    "trips": len(schedule.pickup_groups),
                    **mode_counts,
                })
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    summaries = defaultdict(list)
    for row in rows:
        summaries[(row["scenario"], row["mode"])].append(row)
    print(f"Saved {len(rows)} runs to {output}")
    print("scenario, mode, mean makespan, mean complete, mean shortage, mean trips")
    for (scenario, mode), values in summaries.items():
        count = len(values)
        print(
            f"{scenario}, {mode}, "
            f"{sum(row['makespan'] for row in values) / count:.3f}, "
            f"{sum(row['complete_orders'] for row in values) / count:.2f}, "
            f"{sum(row['shortage_units'] for row in values) / count:.2f}, "
            f"{sum(row['trips'] for row in values) / count:.2f}"
        )
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance-dir", default=str(DEFAULT_INSTANCE_DIR))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--seeds", type=int, nargs="+", default=[42, 43, 44])
    parser.add_argument("--ants", type=int, default=6)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--objective", choices=("makespan", "mean"), default="makespan")
    parser.add_argument("--penalty-weight", type=float, default=100.0)
    parser.add_argument("--timing-profile", default=str(DEFAULT_TIMING_PROFILE))
    args = parser.parse_args(argv)
    compare(
        args.instance_dir,
        args.output,
        args.seeds,
        args.ants,
        args.iterations,
        args.objective,
        args.penalty_weight,
        TimingProfile.load(args.timing_profile),
    )


if __name__ == "__main__":
    main()
