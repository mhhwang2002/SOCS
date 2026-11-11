"""Generate a reproducible factorial set of online AS/RS workloads."""

from __future__ import annotations

import argparse
from pathlib import Path

from .online_workload import generate_arrival_trace


def _label(value: float) -> str:
    return f"{value:g}".replace(".", "p")


def generate_matrix(
    instances_path: Path,
    output_dir: Path,
    cycle_times_path: Path,
    seed: int = 1000,
    replicates: int = 3,
    mean_interarrivals: tuple[float, ...] = (15.0, 60.0, 120.0),
    due_slack_factors: tuple[float, ...] = (1.0, 2.0, 4.0),
    urgency_spread: float = 0.4,
    common_random_numbers: bool = False,
) -> list[Path]:
    if replicates < 1:
        raise ValueError("replicates must be at least 1")
    instance_paths = (
        sorted(instances_path.glob("*.json"))
        if instances_path.is_dir()
        else [instances_path]
    )
    if not instance_paths:
        raise ValueError(f"No JSON instances found at {instances_path}")
    outputs = []
    output_dir.mkdir(parents=True, exist_ok=True)
    for instance_index, instance_path in enumerate(instance_paths):
        for interarrival_index, mean_interarrival in enumerate(mean_interarrivals):
            for slack_index, due_slack_factor in enumerate(due_slack_factors):
                for replicate in range(replicates):
                    if common_random_numbers:
                        trace_seed = seed + instance_index * 100_000 + replicate * 1_000
                    else:
                        trace_seed = (
                            seed
                            + instance_index * 100_000
                            + interarrival_index * 10_000
                            + slack_index * 1_000
                            + replicate
                        )
                    filename = (
                        f"{instance_path.stem}_iat{_label(mean_interarrival)}"
                        f"_slack{_label(due_slack_factor)}_rep{replicate + 1}"
                        f"_seed{trace_seed}.json"
                    )
                    output_path = output_dir / filename
                    outputs.append(generate_arrival_trace(
                        instance_path,
                        output_path,
                        cycle_times_path,
                        seed=trace_seed,
                        mean_interarrival=mean_interarrival,
                        due_slack_factor=due_slack_factor,
                        urgency_spread=urgency_spread,
                    ))
    return outputs


def main(argv=None):
    default_table = Path(__file__).resolve().parent / "data" / "crane_cycle_times.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instances", required=True, type=Path, help="one JSON instance or a directory of instances")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--cycle-times", type=Path, default=default_table)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--replicates", type=int, default=3)
    parser.add_argument("--mean-interarrivals", nargs="+", type=float, default=(15.0, 60.0, 120.0))
    parser.add_argument("--due-slack-factors", nargs="+", type=float, default=(1.0, 2.0, 4.0))
    parser.add_argument("--urgency-spread", type=float, default=0.4)
    parser.add_argument(
        "--common-random-numbers",
        action="store_true",
        help="reuse the same arrival and urgency draws across load and deadline conditions",
    )
    args = parser.parse_args(argv)
    outputs = generate_matrix(
        args.instances,
        args.output_dir,
        args.cycle_times,
        args.seed,
        args.replicates,
        tuple(args.mean_interarrivals),
        tuple(args.due_slack_factors),
        args.urgency_spread,
        args.common_random_numbers,
    )
    print(f"Generated {len(outputs)} reproducible arrival traces in {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
