"""Generate reproducible online order-arrival traces for AS/RS experiments."""

from __future__ import annotations

import argparse
import hashlib
import json
import random
from pathlib import Path
from statistics import median

from .aco_optimization import COLORS, load_instance
from .crane_cycle_times import CraneCycleTimeTable


TRACE_FORMAT = "socs-asrs-online-workload"
TRACE_VERSION = 1


def _instance_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate_arrival_trace(
    instance_path: Path,
    output_path: Path,
    cycle_times_path: Path,
    seed: int = 42,
    mean_interarrival: float = 30.0,
    due_slack_factor: float = 2.0,
    urgency_spread: float = 0.4,
) -> Path:
    """Write a fixed Poisson-arrival trace linked to one inventory/order instance."""
    if mean_interarrival <= 0:
        raise ValueError("mean_interarrival must be positive")
    if due_slack_factor <= 0:
        raise ValueError("due_slack_factor must be positive")
    if not 0.0 <= urgency_spread <= 1.0:
        raise ValueError("urgency_spread must be between 0 and 1")

    instance_path = Path(instance_path)
    output_path = Path(output_path)
    cycle_times_path = Path(cycle_times_path)
    instance = load_instance(instance_path)
    cycle_table = CraneCycleTimeTable.load(cycle_times_path)
    reference_unit_cycle = median(
        duration.ready
        for duration in (*cycle_table.first_cycles.values(), *cycle_table.cycles.values())
    )
    if reference_unit_cycle <= 0:
        raise ValueError("Cycle-time table must contain positive ready durations")

    arrival_rng = random.Random(seed)
    urgency_rng = random.Random(seed ^ 0x5A17)
    urgency_order = list(range(len(instance.order_demands)))
    urgency_rng.shuffle(urgency_order)
    if len(urgency_order) == 1:
        urgency_fractions = [1.0]
    else:
        urgency_fractions = [
            1.0 - urgency_spread / 2.0
            + urgency_spread * rank / (len(urgency_order) - 1)
            for rank in range(len(urgency_order))
        ]
    due_fraction_by_order = [1.0] * len(urgency_order)
    rank_by_order = [0] * len(urgency_order)
    for rank, order_id in enumerate(urgency_order):
        due_fraction_by_order[order_id] = urgency_fractions[rank]
        rank_by_order[order_id] = rank + 1

    arrival_time = 0.0
    orders = []
    for index, demand in enumerate(instance.order_demands):
        if index:
            arrival_time += arrival_rng.expovariate(1.0 / mean_interarrival)
        requested = sum(demand)
        reference_work = max(1, requested) * reference_unit_cycle
        urgency_fraction = due_fraction_by_order[index]
        orders.append({
            "order_id": index + 1,
            "arrival_time": round(arrival_time, 9),
            "due_time": round(
                arrival_time + due_slack_factor * urgency_fraction * reference_work,
                9,
            ),
            "urgency_rank": rank_by_order[index],
            "due_slack_fraction": round(urgency_fraction, 9),
            "requested_quantity": requested,
            "demand_by_sku": {
                sku: demand[sku_index]
                for sku_index, sku in enumerate(COLORS)
                if demand[sku_index]
            },
        })

    trace = {
        "format": TRACE_FORMAT,
        "version": TRACE_VERSION,
        "generation": {
            "seed": seed,
            "arrival_process": "Poisson; exponential inter-arrival times; first arrival at time 0",
            "mean_interarrival_time": mean_interarrival,
            "due_date_rule": "arrival_time + due_slack_factor * urgency_fraction * requested_units * median_DEVS_cycle_ready_time; urgency fractions are assigned by a seeded shuffled rank",
            "due_date_status": "synthetic workload parameter, not fitted to observed customer data",
            "due_slack_factor": due_slack_factor,
            "urgency_spread": urgency_spread,
            "urgency_fraction_range": [1.0 - urgency_spread / 2.0, 1.0 + urgency_spread / 2.0],
            "reference_unit_cycle_time": round(reference_unit_cycle, 9),
            "cycle_time_table": str(cycle_times_path),
            "cycle_time_table_sha256": _instance_digest(cycle_times_path),
        },
        "source_instance": {
            "path": str(instance_path),
            "sha256": _instance_digest(instance_path),
        },
        "orders": orders,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(trace, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return output_path


def main(argv=None):
    default_cycle_times = Path(__file__).resolve().parent / "data" / "crane_cycle_times.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True, type=Path, help="fixed inventory and SKU-demand instance")
    parser.add_argument("--output", required=True, type=Path, help="JSON file to create")
    parser.add_argument("--cycle-times", type=Path, default=default_cycle_times)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--mean-interarrival", type=float, default=30.0)
    parser.add_argument("--due-slack-factor", type=float, default=2.0)
    parser.add_argument("--urgency-spread", type=float, default=0.4)
    args = parser.parse_args(argv)
    output = generate_arrival_trace(
        args.instance,
        args.output,
        args.cycle_times,
        args.seed,
        args.mean_interarrival,
        args.due_slack_factor,
        args.urgency_spread,
    )
    print(f"Saved reproducible online arrival trace: {output.resolve()}")
    print(f"  orders: {len(json.loads(output.read_text(encoding='utf-8'))['orders'])}")
    print(f"  seed: {args.seed}")


if __name__ == "__main__":
    main()
