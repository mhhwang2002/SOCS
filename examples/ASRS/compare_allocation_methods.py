"""Compare random, deterministic, and ACO allocation policies on fixed instances."""

from __future__ import annotations

import argparse
import csv
import random
from dataclasses import dataclass
from pathlib import Path

from .aco_optimization import (
    AntColonyOptimizer,
    ASRSInstance,
    COLORS,
    LEVELS,
    RACK_COLUMNS,
    Product,
    Schedule,
    evaluate_schedule,
    save_instance,
    travel_time,
)
from .crane_cycle_times import CraneCycleTimeTable


@dataclass(frozen=True)
class Scenario:
    sku_count: int
    order_sizes: str
    sku_overlap: str
    shortage_pattern: str

    @property
    def name(self) -> str:
        return f"sku{self.sku_count}_{self.order_sizes}_{self.sku_overlap}_{self.shortage_pattern}"


def scenarios() -> tuple[Scenario, ...]:
    return tuple(
        Scenario(sku_count, sizes, overlap, shortage)
        for sku_count in (3, 5)
        for sizes in ("balanced", "variable")
        for overlap in ("spread", "clustered")
        for shortage in ("spread", "concentrated")
    )


def _order_sizes(profile: str) -> list[int]:
    if profile == "balanced":
        return [6] * 5 + [5] * 5
    return [2, 3, 4, 5, 5, 6, 6, 7, 8, 9]


def _distribute_sku_demand(
    totals: list[int], order_sizes: list[int], profile: str, rng: random.Random
) -> list[list[int]]:
    remaining_capacity = order_sizes[:]
    demand = [[0] * len(COLORS) for _ in order_sizes]
    for sku_index, quantity in enumerate(totals):
        order_ids = list(range(len(order_sizes)))
        rng.shuffle(order_ids)
        if profile == "spread":
            while quantity:
                eligible = [order_id for order_id in order_ids if remaining_capacity[order_id] > 0]
                if not eligible:
                    raise ValueError("SKU demand exceeds total order capacity.")
                order_id = eligible[0]
                demand[order_id][sku_index] += 1
                remaining_capacity[order_id] -= 1
                quantity -= 1
                order_ids.remove(order_id)
                order_ids.append(order_id)
        else:
            for order_id in order_ids:
                assigned = min(quantity, remaining_capacity[order_id])
                demand[order_id][sku_index] += assigned
                remaining_capacity[order_id] -= assigned
                quantity -= assigned
                if quantity == 0:
                    break
    if any(remaining_capacity):
        raise ValueError("SKU totals do not fill all order demand.")
    return demand


def make_scenario_instance(scenario: Scenario, seed: int) -> ASRSInstance:
    """Build and saveable instance with identical total stock and demand across policies."""
    rng = random.Random(seed)
    sku_names = COLORS[: scenario.sku_count]
    quotient, remainder = divmod(50, scenario.sku_count)
    stock_counts = [quotient + (index < remainder) for index in range(scenario.sku_count)]
    order_sizes = _order_sizes(scenario.order_sizes)
    rng.shuffle(order_sizes)

    shortage = [0] * scenario.sku_count
    if scenario.shortage_pattern == "concentrated":
        shortage[rng.randrange(scenario.sku_count)] = 5
    else:
        for index in range(5):
            shortage[index % scenario.sku_count] += 1
    sku_totals = [stock_counts[index] + shortage[index] for index in range(scenario.sku_count)]
    demand = _distribute_sku_demand(sku_totals, order_sizes, scenario.sku_overlap, rng)

    locations = [(bay, level) for level in range(LEVELS) for bay in range(RACK_COLUMNS)]
    rng.shuffle(locations)
    stock_colors = [
        sku_names[index]
        for index, quantity in enumerate(stock_counts)
        for _ in range(quantity)
    ]
    rng.shuffle(stock_colors)

    # Product membership in the instance file is only a reproducible baseline allocation.
    baseline_counts = [0] * len(order_sizes)
    products = []
    order_products: list[list[str]] = [[] for _ in order_sizes]
    product_slots = [order_id for order_id, size in enumerate(order_sizes) for _ in range(size)]
    for index, (location, color) in enumerate(zip(locations, stock_colors)):
        product_id = f"P{index:02d}"
        bay, level = location
        order_id = product_slots[index]
        baseline_counts[order_id] += 1
        order_products[order_id].append(product_id)
        products.append(Product(product_id, color, bay, level, order_id))

    penalties = tuple(size - baseline_counts[index] for index, size in enumerate(order_sizes))
    return ASRSInstance(
        tuple(products),
        tuple(tuple(ids) for ids in order_products),
        penalties,
        tuple(tuple(row) for row in demand),
    )


def _assign_order(
    instance: ASRSInstance,
    order_id: int,
    available: dict[str, list[int]],
    rng: random.Random | None,
    current_position: tuple[float, float],
) -> tuple[list[int], int]:
    chosen: list[int] = []
    for sku_index, sku in enumerate(COLORS):
        needed = instance.order_demands[order_id][sku_index]
        candidates = available[sku]
        if rng is None:
            candidates.sort(key=lambda index: travel_time(current_position, instance.products[index].position))
        else:
            rng.shuffle(candidates)
        for _ in range(min(needed, len(candidates))):
            chosen.append(candidates.pop(0))

    if rng is not None:
        rng.shuffle(chosen)
        station = rng.randrange(2)
        return chosen, station

    routes = []
    for station in (0, 1):
        left = chosen[:]
        position = current_position
        cost = 0.0
        route = []
        while left:
            product_index = min(
                left,
                key=lambda index: travel_time(position, instance.products[index].position),
            )
            product = instance.products[product_index]
            cost += travel_time(position, product.position)
            position = product.position
            left.remove(product_index)
            route.append(product_index)
        cost += travel_time(position, instance.station_positions[station])
        routes.append((cost, station, route))
    _, station, route = min(routes, key=lambda item: (item[0], item[1]))
    return route, station


def random_schedule(instance: ASRSInstance, seed: int) -> Schedule:
    rng = random.Random(seed)
    available = {sku: [i for i, product in enumerate(instance.products) if product.color == sku] for sku in COLORS}
    order_priority = list(range(len(instance.order_demands)))
    rng.shuffle(order_priority)
    route: list[int] = []
    assignments: list[int] = []
    stations = [0] * len(order_priority)
    position = instance.initial_position
    for order_id in order_priority:
        picked, station = _assign_order(instance, order_id, available, rng, position)
        stations[order_id] = station
        route.extend(picked)
        assignments.extend([order_id] * len(picked))
        position = instance.station_positions[station]
    return Schedule(tuple(route), tuple(stations), tuple(assignments), tuple(order_priority))


def heuristic_schedule(instance: ASRSInstance) -> Schedule:
    """Use deterministic feasible-first, scarcity-aware order selection and nearest-cell routing."""
    available = {sku: [i for i, product in enumerate(instance.products) if product.color == sku] for sku in COLORS}
    remaining_orders = set(range(len(instance.order_demands)))
    priority: list[int] = []
    stations = [0] * len(remaining_orders)
    route: list[int] = []
    assignments: list[int] = []
    position = instance.initial_position

    while remaining_orders:
        feasible = []
        for order_id in remaining_orders:
            demand = instance.order_demands[order_id]
            if all(demand[index] <= len(available[sku]) for index, sku in enumerate(COLORS)):
                scarcity = sum(
                    demand[index] / max(1, len(available[sku]))
                    for index, sku in enumerate(COLORS)
                )
                feasible.append((scarcity, sum(demand), order_id))
        if feasible:
            order_id = max(feasible, key=lambda row: (row[0], -row[1], -row[2]))[2]
        else:
            order_id = max(
                remaining_orders,
                key=lambda candidate: (
                    sum(min(instance.order_demands[candidate][index], len(available[sku])) for index, sku in enumerate(COLORS))
                    / max(1, sum(instance.order_demands[candidate])),
                    -candidate,
                ),
            )

        picked, station = _assign_order(instance, order_id, available, None, position)
        stations[order_id] = station
        route.extend(picked)
        assignments.extend([order_id] * len(picked))
        priority.append(order_id)
        remaining_orders.remove(order_id)
        position = instance.station_positions[station]

    return Schedule(tuple(route), tuple(stations), tuple(assignments), tuple(priority))


def _metrics(
    instance: ASRSInstance,
    schedule: Schedule,
    completion_priority: float,
    penalty_weight: float,
    cycle_time_table: CraneCycleTimeTable | None = None,
) -> dict[str, float | int]:
    result = evaluate_schedule(instance, schedule, cycle_time_table)
    partial_orders = sum(
        0 < shipped < requested
        for shipped, requested in zip(result.shipped_by_order, result.requested_by_order)
    )
    unprocessed_orders = sum(shipped == 0 for shipped in result.shipped_by_order)
    incomplete_order_penalty = completion_priority * (partial_orders + unprocessed_orders)
    shortage_penalty = penalty_weight * result.shortage_units
    return {
        "complete_orders": result.complete_order_count,
        "partial_orders": partial_orders,
        "unprocessed_orders": unprocessed_orders,
        "M": completion_priority,
        "incomplete_order_penalty": incomplete_order_penalty,
        "shortage_units": result.shortage_units,
        "weighted_shortage_penalty": shortage_penalty,
        "shipped_units": sum(result.shipped_by_order),
        "makespan": result.makespan,
        "mean_order_completion": result.mean_order_completion_time,
        "objective_score": incomplete_order_penalty + shortage_penalty + result.makespan,
    }


def compare(seed: int = 42, repeats: int = 3, ants: int = 12, iterations: int = 25, output_dir: Path | None = None):
    root = Path(__file__).resolve().parents[2]
    instance_dir = (output_dir or root / "examples" / "ASRS" / "results" / "allocation_study") / "instances"
    result_dir = output_dir or root / "examples" / "ASRS" / "results" / "allocation_study"
    instance_dir.mkdir(parents=True, exist_ok=True)
    result_dir.mkdir(parents=True, exist_ok=True)
    cycle_times_path = Path(__file__).resolve().parent / "data" / "crane_cycle_times.json"
    cycle_time_table = (
        CraneCycleTimeTable.load(cycle_times_path) if cycle_times_path.is_file() else None
    )
    rows = []

    for scenario_index, scenario in enumerate(scenarios()):
        instance_seed = seed + scenario_index * 1009
        instance = make_scenario_instance(scenario, instance_seed)
        save_instance(instance, instance_dir / f"{scenario.name}.json", seed=instance_seed)
        score_reference = AntColonyOptimizer(
            instance,
            "makespan",
            seed + scenario_index * 100,
            ants=ants,
            iterations=iterations,
            penalty_weight=100.0,
            cycle_time_table=cycle_time_table,
        )
        completion_priority = score_reference.completion_priority
        methods = [
            ("deterministic", [heuristic_schedule(instance)]),
            ("random", [random_schedule(instance, seed + scenario_index * 100 + run) for run in range(repeats)]),
        ]
        for method, schedules in methods:
            for run_index, schedule in enumerate(schedules):
                rows.append({
                    "scenario": scenario.name,
                    "method": method,
                    "run": run_index,
                    **_metrics(instance, schedule, completion_priority, 100.0, cycle_time_table),
                })
        for run_index in range(repeats):
            optimizer = AntColonyOptimizer(
                instance,
                "makespan",
                seed + scenario_index * 100 + run_index,
                ants=ants,
                iterations=iterations,
                penalty_weight=100.0,
                cycle_time_table=cycle_time_table,
            )
            schedule, _, _ = optimizer.run()
            rows.append({
                "scenario": scenario.name,
                "method": "aco",
                "run": run_index,
                **_metrics(instance, schedule, completion_priority, 100.0, cycle_time_table),
            })

    raw_path = result_dir / "allocation_methods_raw.csv"
    fields = [
        "scenario", "method", "run", "complete_orders", "partial_orders", "unprocessed_orders",
        "M", "incomplete_order_penalty", "shortage_units", "weighted_shortage_penalty",
        "shipped_units", "makespan", "mean_order_completion", "objective_score",
    ]
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    summary = []
    for scenario in scenarios():
        for method in ("random", "deterministic", "aco"):
            selected = [row for row in rows if row["scenario"] == scenario.name and row["method"] == method]
            if not selected:
                continue
            summary.append({
                "scenario": scenario.name,
                "method": method,
                "runs": len(selected),
                **{
                    field: sum(float(row[field]) for row in selected) / len(selected)
                    for field in (
                        "complete_orders", "partial_orders", "unprocessed_orders", "M",
                        "incomplete_order_penalty", "shortage_units", "weighted_shortage_penalty",
                        "shipped_units", "makespan", "mean_order_completion", "objective_score",
                    )
                },
            })
    summary_path = result_dir / "allocation_methods_summary.csv"
    summary_fields = [
        "scenario", "method", "runs", "complete_orders", "partial_orders", "unprocessed_orders",
        "M", "incomplete_order_penalty", "shortage_units", "weighted_shortage_penalty",
        "shipped_units", "makespan", "mean_order_completion", "objective_score",
    ]
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summary)

    print(f"Saved raw runs to {raw_path}")
    print(f"Saved scenario/method means to {summary_path}")
    print("Scenario | method | complete | partial | unprocessed | priority penalty | shortage penalty | objective | makespan")
    for row in summary:
        print(
            f"{row['scenario']} | {row['method']} | {row['complete_orders']:.2f} | "
            f"{row['partial_orders']:.2f} | {row['unprocessed_orders']:.2f} | "
            f"{row['incomplete_order_penalty']:.3f} | {row['weighted_shortage_penalty']:.3f} | "
            f"{row['objective_score']:.3f} | {row['makespan']:.3f}"
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42, help="master seed for scenario generation and search")
    parser.add_argument("--repeats", type=int, default=3, help="random and ACO repetitions per fixed scenario")
    parser.add_argument("--ants", type=int, default=12)
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument("--output-dir", type=Path, help="defaults to examples/ASRS/results/allocation_study")
    args = parser.parse_args(argv)
    compare(args.seed, args.repeats, args.ants, args.iterations, args.output_dir)


if __name__ == "__main__":
    main()
