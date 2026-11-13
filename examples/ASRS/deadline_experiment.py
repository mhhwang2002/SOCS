"""Compare single-crane AS/RS policies under reproducible dispatch deadlines."""

from __future__ import annotations

import argparse
import csv
import json
import random
from math import sqrt
from pathlib import Path
from statistics import mean, stdev

from .aco_optimization import (
    AntColonyOptimizer,
    ASRSInstance,
    COLORS,
    PICK_TIME,
    STATION_TIME,
    Schedule,
    evaluate_schedule,
    load_instance,
    travel_time,
)
from .compare_allocation_methods import _assign_order
from .asrs_devs import ASRSController, Crane, make_asrs_model
from .crane_cycle_times import CraneCycleTimeTable
from socs.devs.TESEngin import ExecutionResult, TESEnginBase


DEFAULT_FACTORS = (0.75, 1.0, 1.25)
POLICIES = ("FCFS", "EDD", "minimum_slack", "nearest_pick", "ACO_makespan")
SUMMARY_METRICS = (
    "complete_on_time", "partial_on_time", "unprocessed_at_deadline",
    "rescue_orders", "full_rescue_orders", "rescue_units",
    "local_units_by_deadline", "makespan", "cycle_table_makespan",
    "time_model_ratio", "mean_order_completion",
)


def _t_critical_95(sample_count: int) -> float:
    values = {
        2: 12.706, 3: 4.303, 4: 3.182, 5: 2.776, 6: 2.571,
        7: 2.447, 8: 2.365, 9: 2.306, 10: 2.262, 11: 2.228,
        12: 2.201, 13: 2.179, 14: 2.160, 15: 2.145, 16: 2.131,
        17: 2.120, 18: 2.110, 19: 2.101, 20: 2.093, 21: 2.086,
        22: 2.080, 23: 2.074, 24: 2.069, 25: 2.064, 26: 2.060,
        27: 2.056, 28: 2.052, 29: 2.048, 30: 2.045,
    }
    return values.get(sample_count - 1, 1.96)


def _mean_interval(values: list[float]) -> tuple[float, float, float, float]:
    average = mean(values)
    if len(values) < 2:
        return average, 0.0, float("nan"), float("nan")
    deviation = stdev(values)
    margin = _t_critical_95(len(values)) * deviation / sqrt(len(values))
    return average, deviation, average - margin, average + margin


def _greedy_schedule(
    instance: ASRSInstance,
    policy: str,
    due_times: tuple[float, ...],
) -> Schedule:
    """Build an executable schedule using a transparent dispatch rule."""
    available = {
        sku: [i for i, product in enumerate(instance.products) if product.color == sku]
        for sku in COLORS
    }
    remaining = set(range(len(instance.order_demands)))
    priority: list[int] = []
    route: list[int] = []
    assignments: list[int] = []
    stations = [0] * len(remaining)
    position = instance.initial_position
    elapsed_estimate = 0.0

    while remaining:
        if policy == "fcfs":
            order_id = min(remaining)
        elif policy == "edd":
            order_id = min(remaining, key=lambda i: (due_times[i], i))
        elif policy == "slack":
            order_id = min(
                remaining,
                key=lambda i: (
                    due_times[i] - elapsed_estimate
                    - _estimated_service_time(instance, i, available, position),
                    due_times[i],
                    i,
                ),
            )
        elif policy == "nearest":
            order_id = min(
                remaining,
                key=lambda i: (_distance_to_order(instance, i, available, position), i),
            )
        else:
            raise ValueError(f"Unknown dispatch policy: {policy}")

        picked, station = _assign_order(instance, order_id, available, None, position)
        stations[order_id] = station
        route.extend(picked)
        assignments.extend([order_id] * len(picked))
        priority.append(order_id)
        remaining.remove(order_id)
        station_position = instance.station_positions[station]
        for product_index in picked:
            product_position = instance.products[product_index].position
            elapsed_estimate += (
                travel_time(position, product_position)
                + PICK_TIME
                + travel_time(product_position, station_position)
                + STATION_TIME
            )
            position = station_position

    return Schedule(tuple(route), tuple(stations), tuple(assignments), tuple(priority))


def _eligible_products(instance: ASRSInstance, order_id: int, available: dict[str, list[int]]):
    chosen = []
    for color_index, sku in enumerate(COLORS):
        count = min(instance.order_demands[order_id][color_index], len(available[sku]))
        chosen.extend(available[sku][:count])
    return chosen


def _distance_to_order(instance: ASRSInstance, order_id: int, available, position) -> float:
    candidates = _eligible_products(instance, order_id, available)
    if not candidates:
        return float("inf")
    return min(travel_time(position, instance.products[i].position) for i in candidates)


def _estimated_service_time(instance: ASRSInstance, order_id: int, available, position) -> float:
    """Estimate order service time, including a station return for every unit."""
    estimates = []
    for station in instance.station_positions:
        local_available = {sku: list(indices) for sku, indices in available.items()}
        demand_remaining = list(instance.order_demands[order_id])
        current = position
        total = 0.0
        while True:
            candidates = [
                product_index
                for sku_index, sku in enumerate(COLORS)
                if demand_remaining[sku_index] > 0
                for product_index in local_available[sku]
            ]
            if not candidates:
                break
            product_index = min(
                candidates,
                key=lambda i: travel_time(current, instance.products[i].position),
            )
            product = instance.products[product_index]
            total += (
                travel_time(current, product.position)
                + PICK_TIME
                + travel_time(product.position, station)
                + STATION_TIME
            )
            current = station
            demand_remaining[COLORS.index(product.color)] -= 1
            local_available[product.color].remove(product_index)
        estimates.append(total)
    return min(estimates, default=0.0)


def _deadline_profile(instance: ASRSInstance, reference_horizon: float, seed: int, factor: float):
    """Create a fixed, reproducible spread of urgent and less-urgent orders."""
    order_ids = list(range(len(instance.order_demands)))
    random.Random(seed).shuffle(order_ids)
    if len(order_ids) == 1:
        fractions = [1.0]
    else:
        fractions = [0.8 + 0.4 * rank / (len(order_ids) - 1) for rank in range(len(order_ids))]
    due_times = [0.0] * len(order_ids)
    for rank, order_id in enumerate(order_ids):
        due_times[order_id] = reference_horizon * factor * fractions[rank]
    return tuple(due_times), tuple(order_ids)


def _evaluate_at_deadlines(instance: ASRSInstance, schedule: Schedule, due_times: tuple[float, ...]):
    """Measure local shipments by deadline by executing the AS/RS DEVS model."""
    assignments = schedule.product_assignments or tuple(
        instance.products[index].order_id for index in schedule.product_order
    )
    inventory = {
        (product.bay + 1, product.level + 1): {
            "box_id": product.product_id,
            "color": product.color,
        }
        for product in instance.products
    }
    retrieval_plan = []
    for product_index, order_id in zip(schedule.product_order, assignments):
        product = instance.products[product_index]
        retrieval_plan.append({
            "cell": f"Cell-B{product.bay + 1:02d}-L{product.level + 1}",
            "product_id": product.product_id,
            "color": product.color,
            "order_id": order_id + 1,
            "station": schedule.order_station[order_id],
        })
    requested = {
        order_id + 1: sum(demand)
        for order_id, demand in enumerate(instance.order_demands)
    }
    model = make_asrs_model(
        inventory=inventory,
        retrieval_plan=retrieval_plan,
        requested_order_items=requested,
    )
    controller = next(child for child in model.children if isinstance(child, ASRSController))
    engine = TESEnginBase(
        model,
        TimeIntervalContinuous=float("inf"),
        TimeIntervalAnimation=float("inf"),
        TimeSimulationEnd=1e9,
    )
    engine.bVerbose = False

    unit_completion_times: dict[str, float] = {}
    observed_counts = {order_id: 0 for order_id in controller.delivered_products_by_order}
    steps = 0
    while controller.phase != "completed":
        result = engine.executeOneStep()
        steps += 1
        if result is ExecutionResult.TimeBoundReached:
            raise RuntimeError("AS/RS DEVS schedule did not complete before the simulation time bound.")
        if steps > 100_000:
            raise RuntimeError("AS/RS DEVS schedule exceeded the event-step safety limit.")
        for order_id, product_ids in controller.delivered_products_by_order.items():
            previous = observed_counts.get(order_id, 0)
            for product_id in product_ids[previous:]:
                unit_completion_times[product_id] = float(engine.tNow)
            observed_counts[order_id] = len(product_ids)

    unit_times = [
        unit_completion_times.get(instance.products[i].product_id, float("inf"))
        for i in schedule.product_order
    ]

    delivered_by_order = [0] * len(instance.order_demands)
    delivered_by_sku = [[0] * len(COLORS) for _ in instance.order_demands]
    completion_by_order: list[float | None] = [None] * len(instance.order_demands)
    for product_index, order_id, completed_at in zip(schedule.product_order, assignments, unit_times):
        if completion_by_order[order_id] is None or completed_at > completion_by_order[order_id]:
            completion_by_order[order_id] = completed_at
        if completed_at <= due_times[order_id]:
            delivered_by_order[order_id] += 1
            delivered_by_sku[order_id][COLORS.index(instance.products[product_index].color)] += 1

    requested_by_order = [sum(demand) for demand in instance.order_demands]
    rescue_units_by_order = [
        max(0, requested_by_order[i] - delivered_by_order[i])
        for i in range(len(requested_by_order))
    ]
    complete_on_time = sum(
        delivered_by_order[i] == requested_by_order[i]
        for i in range(len(requested_by_order))
    )
    partial_on_time = sum(
        0 < delivered_by_order[i] < requested_by_order[i]
        for i in range(len(requested_by_order))
    )
    untouched = sum(count == 0 for count in delivered_by_order)
    rescue_orders = sum(units > 0 for units in rescue_units_by_order)
    full_rescue_orders = sum(
        delivered_by_order[i] == 0 and requested_by_order[i] > 0
        for i in range(len(requested_by_order))
    )
    rescue_by_sku = [0] * len(COLORS)
    for order_id, demand in enumerate(instance.order_demands):
        for sku_index, requested in enumerate(demand):
            rescue_by_sku[sku_index] += max(0, requested - delivered_by_sku[order_id][sku_index])

    return {
        "local_complete_on_time": complete_on_time,
        "local_partial_on_time": partial_on_time,
        "local_unprocessed_at_deadline": untouched,
        "rescue_orders": rescue_orders,
        "full_rescue_orders": full_rescue_orders,
        "rescue_units": sum(rescue_units_by_order),
        "rescue_units_by_sku": {sku: rescue_by_sku[i] for i, sku in enumerate(COLORS)},
        "local_units_by_deadline": sum(delivered_by_order),
        "total_requested_units": sum(requested_by_order),
        "makespan": max(unit_times, default=0.0),
        "mean_order_completion": (
            sum(value for value in completion_by_order if value is not None)
            / max(1, sum(value is not None for value in completion_by_order))
        ),
        "on_time_order_ids": [
            i + 1 for i, count in enumerate(delivered_by_order)
            if count == requested_by_order[i]
        ],
        "rescue_units_by_order": [count for count in rescue_units_by_order],
    }


def run_experiment(
    instance_path: Path,
    output_dir: Path,
    cycle_times_path: Path,
    seed: int = 42,
    repeats: int = 5,
    ants: int = 12,
    iterations: int = 25,
    factors: tuple[float, ...] = DEFAULT_FACTORS,
):
    cycle_time_table = CraneCycleTimeTable.load(cycle_times_path)
    instance_paths = (
        sorted(instance_path.glob("*.json"))
        if instance_path.is_dir()
        else [instance_path]
    )
    if not instance_paths:
        raise ValueError(f"No JSON instances found in {instance_path}.")
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_rows = []
    design = {
        "instances": [str(path) for path in instance_paths],
        "seed": seed,
        "deadline_factors": list(factors),
        "reference_policy": "ascending order ID (fixed input-order proxy; all orders released at time zero)",
        "deadline_definition": "due_i = FCFS_makespan * factor * rank_fraction_i; rank_fraction spans 0.8 to 1.2; urgency rank is shuffled once per instance seed",
        "rescue_definition": "requested SKU units not delivered by the local crane at that order's due time",
        "deadline_ordering": "all orders are available at simulation time zero; deadlines are evaluation cutoffs and do not stop the crane",
        "timing_model": "ASRSController and acceleration-profile Crane DEVS model; parameters are not yet calibrated against physical equipment",
        "optimizer_cycle_table": str(cycle_times_path),
        "optimizer_timing_semantics": "DEVS-measured command-to-box-out and command-to-idle cycles; startup delay charged once",
        "methods": list(POLICIES),
        "aco_searches_per_instance": repeats,
        "aco_seeds": "seed + instance_index * 100 + run_index",
        "replication_unit_for_confidence_intervals": "instance; ACO seeds are averaged within each instance first",
    }
    (output_dir / "experiment_design.json").write_text(
        json.dumps(design, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    for instance_index, current_path in enumerate(instance_paths):
        instance = load_instance(current_path)
        instance_name = current_path.stem
        instance_seed = seed + instance_index * 100
        no_deadlines = tuple([float("inf")] * len(instance.order_demands))
        reference_schedule = _greedy_schedule(instance, "fcfs", no_deadlines)
        reference_horizon = _evaluate_at_deadlines(
            instance, reference_schedule, no_deadlines
        )["makespan"]
        if reference_horizon <= 0:
            raise ValueError(f"Instance {current_path} must contain at least one pickable product.")
        aco_schedules = []
        for run in range(repeats):
            optimizer = AntColonyOptimizer(
                instance,
                "makespan",
                instance_seed + run,
                ants=ants,
                iterations=iterations,
                penalty_weight=100.0,
                cycle_time_table=cycle_time_table,
            )
            schedule, _, _ = optimizer.run()
            aco_schedules.append(schedule)

        for factor in factors:
            due_times, urgency_order = _deadline_profile(
                instance, reference_horizon, seed + instance_index * 1009, factor
            )
            schedules = {
                "FCFS": [_greedy_schedule(instance, "fcfs", due_times)],
                "EDD": [_greedy_schedule(instance, "edd", due_times)],
                "minimum_slack": [_greedy_schedule(instance, "slack", due_times)],
                "nearest_pick": [_greedy_schedule(instance, "nearest", due_times)],
                "ACO_makespan": aco_schedules,
            }
            for method, method_schedules in schedules.items():
                for run, schedule in enumerate(method_schedules):
                    metrics = _evaluate_at_deadlines(instance, schedule, due_times)
                    cycle_table_result = evaluate_schedule(instance, schedule, cycle_time_table)
                    cycle_table_makespan = cycle_table_result.makespan
                    raw_rows.append({
                        "instance": instance_name,
                        "factor": factor,
                        "method": method,
                        "run": run,
                        "seed": instance_seed + run if method == "ACO_makespan" else instance_seed,
                        "reference_horizon": reference_horizon,
                        "complete_on_time": metrics["local_complete_on_time"],
                        "partial_on_time": metrics["local_partial_on_time"],
                        "unprocessed_at_deadline": metrics["local_unprocessed_at_deadline"],
                        "rescue_orders": metrics["rescue_orders"],
                        "full_rescue_orders": metrics["full_rescue_orders"],
                        "rescue_units": metrics["rescue_units"],
                        "local_units_by_deadline": metrics["local_units_by_deadline"],
                        "total_requested_units": metrics["total_requested_units"],
                        "makespan": metrics["makespan"],
                        "cycle_table_makespan": cycle_table_makespan,
                        "time_model_ratio": metrics["makespan"] / cycle_table_makespan
                        if cycle_table_makespan else 0.0,
                        "mean_order_completion": metrics["mean_order_completion"],
                        "urgency_order": ";".join(str(i + 1) for i in urgency_order),
                        "due_times": ";".join(f"{value:.6f}" for value in due_times),
                        "rescue_units_by_sku": json.dumps(metrics["rescue_units_by_sku"], sort_keys=True),
                        "rescue_units_by_order": ";".join(str(value) for value in metrics["rescue_units_by_order"]),
                    })

    raw_path = output_dir / "deadline_policy_raw.csv"
    fields = list(raw_rows[0]) if raw_rows else []
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(raw_rows)

    summary_rows = []
    factors_present = sorted(set(float(row["factor"]) for row in raw_rows))
    instances_present = sorted(set(row["instance"] for row in raw_rows))
    for factor in factors_present:
        fcfs_by_instance = {}
        for instance_name in instances_present:
            fcfs_runs = [row for row in raw_rows if row["instance"] == instance_name
                         and float(row["factor"]) == factor and row["method"] == "FCFS"]
            if fcfs_runs:
                fcfs_by_instance[instance_name] = {
                    metric: mean(float(row[metric]) for row in fcfs_runs)
                    for metric in SUMMARY_METRICS
                }
        for method in POLICIES:
            instance_rows = []
            for instance_name in instances_present:
                selected = [row for row in raw_rows if row["instance"] == instance_name
                            and float(row["factor"]) == factor and row["method"] == method]
                if not selected:
                    continue
                instance_rows.append({
                    "instance": instance_name,
                    **{
                        metric: mean(float(row[metric]) for row in selected)
                        for metric in SUMMARY_METRICS
                    },
                    "seed_sd_rescue_units": stdev(float(row["rescue_units"]) for row in selected)
                    if method == "ACO_makespan" and len(selected) > 1 else 0.0,
                    "seed_sd_complete_on_time": stdev(float(row["complete_on_time"]) for row in selected)
                    if method == "ACO_makespan" and len(selected) > 1 else 0.0,
                })
            if not instance_rows:
                continue
            row = {"factor": factor, "method": method, "instances": len(instance_rows),
                   "aco_runs_per_instance": repeats if method == "ACO_makespan" else 1}
            for metric in SUMMARY_METRICS:
                average, deviation, low, high = _mean_interval(
                    [float(item[metric]) for item in instance_rows]
                )
                row[metric] = average
                row[f"{metric}_between_instance_sd"] = deviation
                row[f"{metric}_ci95_low"] = low
                row[f"{metric}_ci95_high"] = high
            row["rescue_units_mean_seed_sd"] = mean(
                float(item["seed_sd_rescue_units"]) for item in instance_rows
            )
            row["complete_on_time_mean_seed_sd"] = mean(
                float(item["seed_sd_complete_on_time"]) for item in instance_rows
            )
            paired_complete = [
                float(item["complete_on_time"]) - fcfs_by_instance[item["instance"]]["complete_on_time"]
                for item in instance_rows if item["instance"] in fcfs_by_instance
            ]
            paired_rescue = [
                float(item["rescue_units"]) - fcfs_by_instance[item["instance"]]["rescue_units"]
                for item in instance_rows if item["instance"] in fcfs_by_instance
            ]
            for name, values in (("delta_complete_vs_fcfs", paired_complete),
                                 ("delta_rescue_units_vs_fcfs", paired_rescue)):
                average, deviation, low, high = _mean_interval(values)
                row[name] = average
                row[f"{name}_between_instance_sd"] = deviation
                row[f"{name}_ci95_low"] = low
                row[f"{name}_ci95_high"] = high
            summary_rows.append(row)

    summary_path = output_dir / "deadline_policy_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary_rows[0]) if summary_rows else [])
        writer.writeheader()
        writer.writerows(summary_rows)

    print(f"Instances: {len(instance_paths)}; factors: {len(factors)}; ACO seeds/instance: {repeats}")
    print(f"Saved reproducible design: {output_dir / 'experiment_design.json'}")
    print(f"Saved raw results: {raw_path}")
    print(f"Saved instance-level summary and paired 95% t intervals: {summary_path}")
    print("factor | method | n instances | on-time complete | rescue units | delta rescue vs FCFS [95% CI]")
    for row in summary_rows:
        print(
            f"{row['factor']:.2f} | {row['method']} | {row['instances']} | "
            f"{row['complete_on_time']:.2f} | {row['rescue_units']:.2f} | "
            f"{row['delta_rescue_units_vs_fcfs']:.2f} "
            f"[{row['delta_rescue_units_vs_fcfs_ci95_low']:.2f}, "
            f"{row['delta_rescue_units_vs_fcfs_ci95_high']:.2f}]"
        )


def main(argv=None):
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--instance", type=Path,
        default=root / "examples" / "ASRS" / "results" / "allocation_study" / "instances",
        help="one fixed instance JSON or a directory of fixed instance JSON files",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=root / "examples" / "ASRS" / "results" / "deadline_study_16instances",
    )
    parser.add_argument(
        "--cycle-times", type=Path,
        default=root / "examples" / "ASRS" / "data" / "crane_cycle_times.json",
        help="DEVS-measured crane cycle table used by the ACO evaluator",
    )
    parser.add_argument("--seed", type=int, default=42, help="seed for fixed deadline ranks and ACO")
    parser.add_argument("--repeats", type=int, default=5, help="ACO search repetitions per deadline factor")
    parser.add_argument("--ants", type=int, default=12)
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument(
        "--deadline-factors", type=float, nargs="+", default=DEFAULT_FACTORS,
        help="deadline multipliers relative to the FCFS reference makespan",
    )
    args = parser.parse_args(argv)
    run_experiment(
        args.instance, args.output_dir, args.cycle_times, args.seed, args.repeats,
        args.ants, args.iterations, tuple(args.deadline_factors),
    )


if __name__ == "__main__":
    main()
