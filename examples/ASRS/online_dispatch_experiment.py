"""Compare non-preemptive online dispatch rules on fixed arrival traces."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import random
from math import ceil, sqrt
from pathlib import Path
from statistics import mean, stdev
from time import perf_counter

from .aco_optimization import (
    ASRSInstance,
    AntColonyOptimizer,
    COLORS,
    Product,
    load_instance,
)
from .crane_cycle_times import CraneCycleTimeTable


POLICIES = (
    "FCFS", "Random", "EDD", "minimum_slack", "preemptive_minimum_slack",
    "nearest_pick", "rolling_ACO",
)
METRICS = (
    "complete_orders_by_deadline", "partial_orders_by_deadline",
    "unprocessed_orders_by_deadline", "rescue_orders", "rescue_units",
    "partial_orders_at_departure", "partial_shortage_units_at_departure",
    "partial_shortage_sku_types_at_departure", "weighted_partial_penalty",
    "shipped_units_by_deadline", "mean_response_time", "p90_response_time",
    "p95_response_time", "full_run_makespan", "maximum_queue", "optimizer_seconds",
    "preemption_events",
)


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[max(0, ceil(percentile * len(ordered)) - 1)]


def _t_critical_95(n: int) -> float:
    table = {
        2: 12.706, 3: 4.303, 4: 3.182, 5: 2.776, 6: 2.571, 7: 2.447,
        8: 2.365, 9: 2.306, 10: 2.262, 11: 2.228, 12: 2.201,
        13: 2.179, 14: 2.160, 15: 2.145, 16: 2.131, 17: 2.120,
        18: 2.110, 19: 2.101, 20: 2.093, 21: 2.086, 22: 2.080,
        23: 2.074, 24: 2.069, 25: 2.064, 26: 2.060, 27: 2.056,
        28: 2.052, 29: 2.048, 30: 2.045,
    }
    return table.get(n - 1, 1.96)


def _mean_ci(values: list[float]) -> tuple[float, float, float, float]:
    average = mean(values)
    if len(values) < 2:
        return average, 0.0, float("nan"), float("nan")
    deviation = stdev(values)
    margin = _t_critical_95(len(values)) * deviation / sqrt(len(values))
    return average, deviation, average - margin, average + margin


def _trace_read(trace_path: Path, instance_path: Path) -> tuple[dict, ASRSInstance]:
    trace = json.loads(trace_path.read_text(encoding="utf-8"))
    if trace.get("format") != "socs-asrs-online-workload" or trace.get("version") != 1:
        raise ValueError(f"Unsupported online workload trace: {trace_path}")
    digest = hashlib.sha256(instance_path.read_bytes()).hexdigest()
    if trace.get("source_instance", {}).get("sha256") != digest:
        raise ValueError(f"Trace {trace_path} was generated for a different instance file.")
    return trace, load_instance(instance_path)


def _build_queue_instance(instance, product_indices, order_ids, trace_orders):
    product_map = []
    products = []
    for original_index in product_indices:
        product = instance.products[original_index]
        product_map.append(original_index)
        products.append(Product(product.product_id, product.color, product.bay, product.level, 0))
    demands = []
    for order_id in order_ids:
        record = trace_orders[order_id]
        demands.append(tuple(record["demand_by_sku"].get(sku, 0) for sku in COLORS))
    temporary = ASRSInstance(
        tuple(products),
        tuple(() for _ in order_ids),
        tuple(0 for _ in order_ids),
        tuple(demands),
        instance.initial_position,
        instance.station_positions,
    )
    return temporary, product_map


def _greedy_order_plan(instance, available, order, demand, current_station, table):
    best = None
    for station in (0, 1):
        remaining = list(demand)
        stock = {sku: list(available[sku]) for sku in COLORS}
        route = []
        current = current_station
        total = 0.0
        while any(remaining):
            candidates = [
                index
                for sku_index, sku in enumerate(COLORS)
                if remaining[sku_index] > 0
                for index in stock[sku]
            ]
            if not candidates:
                break
            product_index = min(
                candidates,
                key=lambda i: table.duration(
                    current,
                    instance.products[i].bay,
                    instance.products[i].level,
                    station,
                ).ready,
            )
            product = instance.products[product_index]
            duration = table.duration(current, product.bay, product.level, station)
            route.append((product_index, station))
            total += duration.ready
            current = station
            remaining[COLORS.index(product.color)] -= 1
            stock[product.color].remove(product_index)
        candidate = (total, route, station)
        if best is None or candidate[0] < best[0]:
            best = candidate
    return best


def simulate_online_policy(
    instance,
    trace,
    table,
    policy: str,
    seed: int = 42,
    ants: int = 8,
    iterations: int = 12,
    capture_log: bool = False,
    partial_penalty_kappa: float = 60.0,
    partial_penalty_epsilon: float = 1.0,
    partial_penalty_cap: float = 5.0,
):
    """Simulate order arrivals and non-preemptive order-at-a-time crane service."""
    trace_orders = {int(record["order_id"]) - 1: record for record in trace["orders"]}
    order_count = len(trace_orders)
    if set(trace_orders) != set(range(len(instance.order_demands))):
        raise ValueError("Trace must contain exactly the orders in its source instance.")
    if partial_penalty_kappa < 0 or partial_penalty_epsilon <= 0 or partial_penalty_cap < 1:
        raise ValueError("Partial-penalty kappa must be nonnegative, epsilon positive, and cap at least 1.")
    available = {sku: [] for sku in COLORS}
    for index, product in enumerate(instance.products):
        available[product.color].append(index)
    unresolved = set(trace_orders)
    now = table.startup_delay
    current_station = None
    rng = random.Random(seed)
    completion_times = [None] * order_count
    delivered_by_deadline = [0] * order_count
    requested = [sum(trace_orders[i]["demand_by_sku"].values()) for i in range(order_count)]
    maximum_queue = 0
    optimizer_seconds = 0.0
    dispatch_number = 0
    dispatch_log = []
    pick_log = []
    order_log = []

    while unresolved:
        waiting = [
            order_id for order_id in unresolved
            if float(trace_orders[order_id]["arrival_time"]) <= now
        ]
        if not waiting:
            now = max(now, min(float(trace_orders[i]["arrival_time"]) for i in unresolved))
            waiting = [
                order_id for order_id in unresolved
                if float(trace_orders[order_id]["arrival_time"]) <= now
            ]
        maximum_queue = max(maximum_queue, len(waiting))

        chosen_order = None
        chosen_route = None
        chosen_station = None
        if policy == "rolling_ACO":
            started = perf_counter()
            order_ids = sorted(waiting)
            product_indices = [i for sku in COLORS for i in available[sku]]
            temporary, product_map = _build_queue_instance(
                instance, product_indices, order_ids, trace_orders
            )
            optimizer = AntColonyOptimizer(
                temporary,
                "makespan",
                seed + dispatch_number * 1009,
                ants=ants,
                iterations=iterations,
                penalty_weight=100.0,
                cycle_time_table=table,
                initial_station=current_station,
            )
            schedule, _, _ = optimizer.run()
            optimizer_seconds += perf_counter() - started
            if schedule.order_priority:
                local_order = schedule.order_priority[0]
                chosen_order = order_ids[local_order]
                chosen_station = schedule.order_station[local_order]
                chosen_route = [
                    (product_map[product_index], chosen_station)
                    for product_index, assigned_order in zip(
                        schedule.product_order, schedule.product_assignments
                    )
                    if assigned_order == local_order
                ]
        else:
            plans = {
                order_id: _greedy_order_plan(
                    instance,
                    available,
                    order_id,
                    tuple(trace_orders[order_id]["demand_by_sku"].get(sku, 0) for sku in COLORS),
                    current_station,
                    table,
                )
                for order_id in waiting
            }
            if policy == "Random":
                chosen_order = rng.choice(waiting)
            elif policy == "FCFS":
                chosen_order = min(waiting, key=lambda i: (trace_orders[i]["arrival_time"], i))
            elif policy == "EDD":
                chosen_order = min(waiting, key=lambda i: (trace_orders[i]["due_time"], trace_orders[i]["arrival_time"], i))
            elif policy == "minimum_slack":
                chosen_order = min(
                    waiting,
                    key=lambda i: (
                        float(trace_orders[i]["due_time"]) - now - plans[i][0],
                        float(trace_orders[i]["due_time"]),
                        i,
                    ),
                )
            elif policy == "nearest_pick":
                def first_pick_cycle(order_id):
                    route = plans[order_id][1]
                    if not route:
                        return float("inf")
                    product_index, station = route[0]
                    product = instance.products[product_index]
                    return table.duration(
                        current_station, product.bay, product.level, station
                    ).ready

                chosen_order = min(waiting, key=lambda i: (first_pick_cycle(i), i))
            else:
                raise ValueError(f"Unknown policy: {policy}")
            _, chosen_route, chosen_station = plans[chosen_order]

        if chosen_order is None:
            raise RuntimeError(f"Policy {policy} failed to select an available order.")
        dispatch_log.append({"order_id": chosen_order + 1, "time": now})
        due = float(trace_orders[chosen_order]["due_time"])
        box_times = []
        for product_index, station in chosen_route:
            product = instance.products[product_index]
            duration = table.duration(current_station, product.bay, product.level, station)
            box_out_at = now + duration.box_out
            box_times.append(box_out_at)
            if box_out_at <= due:
                delivered_by_deadline[chosen_order] += 1
            pick_log.append({
                "order_id": chosen_order + 1,
                "box": product.product_id,
                "cell": f"Cell-B{product.bay + 1:02d}-L{product.level + 1}",
                "color": product.color,
                "station": station,
                "dispatch_time": now,
                "completed_at": box_out_at,
            })
            now += duration.ready
            current_station = station
            available[product.color].remove(product_index)
        completion_times[chosen_order] = box_times[-1] if box_times else now
        demand = trace_orders[chosen_order]["demand_by_sku"]
        picked_by_sku = {sku: 0 for sku in COLORS}
        for product_index, _ in chosen_route:
            picked_by_sku[instance.products[product_index].color] += 1
        shortage_by_sku = {
            sku: max(0, int(demand.get(sku, 0)) - picked_by_sku[sku])
            for sku in COLORS
        }
        shortage_units = sum(shortage_by_sku.values())
        shortage_sku_types = sum(quantity > 0 for quantity in shortage_by_sku.values())
        departure_time = float(completion_times[chosen_order])
        slack_at_departure = float(trace_orders[chosen_order]["due_time"]) - departure_time
        urgency_factor = min(
            partial_penalty_cap,
            1.0 + partial_penalty_kappa / (max(slack_at_departure, 0.0) + partial_penalty_epsilon),
        )
        partial_penalty = shortage_units * urgency_factor
        order_log.append({
            "order_id": chosen_order + 1,
            "dispatch_time": float(dispatch_log[-1]["time"]),
            "departure_time": departure_time,
            "due_time": float(trace_orders[chosen_order]["due_time"]),
            "slack_at_departure": slack_at_departure,
            "requested_units": requested[chosen_order],
            "locally_shipped_units": len(chosen_route),
            "shortage_units": shortage_units,
            "shortage_sku_types": shortage_sku_types,
            "urgency_factor": urgency_factor,
            "weighted_partial_penalty": partial_penalty,
        })
        unresolved.remove(chosen_order)
        dispatch_number += 1

    complete = sum(delivered_by_deadline[i] >= requested[i] for i in range(order_count))
    partial = sum(0 < delivered_by_deadline[i] < requested[i] for i in range(order_count))
    unprocessed = sum(delivered_by_deadline[i] == 0 for i in range(order_count))
    rescue_by_order = [requested[i] - delivered_by_deadline[i] for i in range(order_count)]
    response_times = [
        float(completion_times[i]) - float(trace_orders[i]["arrival_time"])
        for i in range(order_count)
    ]
    result = {
        "complete_orders_by_deadline": complete,
        "partial_orders_by_deadline": partial,
        "unprocessed_orders_by_deadline": unprocessed,
        "rescue_orders": sum(value > 0 for value in rescue_by_order),
        "rescue_units": sum(rescue_by_order),
        "partial_orders_at_departure": sum(row["shortage_units"] > 0 for row in order_log),
        "partial_shortage_units_at_departure": sum(row["shortage_units"] for row in order_log),
        "partial_shortage_sku_types_at_departure": sum(row["shortage_sku_types"] for row in order_log),
        "weighted_partial_penalty": sum(row["weighted_partial_penalty"] for row in order_log),
        "shipped_units_by_deadline": sum(delivered_by_deadline),
        "mean_response_time": mean(response_times) if response_times else 0.0,
        "p90_response_time": _percentile(response_times, 0.90),
        "p95_response_time": _percentile(response_times, 0.95),
        "full_run_makespan": max(completion_times, default=0.0),
        "maximum_queue": maximum_queue,
        "optimizer_seconds": optimizer_seconds,
        "preemption_events": 0,
    }
    if capture_log:
        result["dispatch_log"] = dispatch_log
        result["pick_log"] = pick_log
        result["order_log"] = order_log
    return result


def _greedy_order_plan_at_station(instance, available, demand, current_station, station, table):
    remaining = list(demand)
    stock = {sku: list(available[sku]) for sku in COLORS}
    route = []
    current = current_station
    total = 0.0
    while any(remaining):
        candidates = [
            index
            for sku_index, sku in enumerate(COLORS)
            if remaining[sku_index] > 0
            for index in stock[sku]
        ]
        if not candidates:
            break
        product_index = min(
            candidates,
            key=lambda i: table.duration(
                current, instance.products[i].bay, instance.products[i].level, station
            ).ready,
        )
        product = instance.products[product_index]
        duration = table.duration(current, product.bay, product.level, station)
        route.append((product_index, station))
        total += duration.ready
        current = station
        remaining[COLORS.index(product.color)] -= 1
        stock[product.color].remove(product_index)
    return total, route, station


def simulate_preemptive_minimum_slack(
    instance,
    trace,
    table,
    partial_penalty_kappa: float = 60.0,
    partial_penalty_epsilon: float = 1.0,
    partial_penalty_cap: float = 5.0,
    capture_log: bool = False,
):
    """Reconsider the next order after every complete pick-and-delivery cycle."""
    trace_orders = {int(record["order_id"]) - 1: record for record in trace["orders"]}
    order_count = len(trace_orders)
    if set(trace_orders) != set(range(len(instance.order_demands))):
        raise ValueError("Trace must contain exactly the orders in its source instance.")
    if partial_penalty_kappa < 0 or partial_penalty_epsilon <= 0 or partial_penalty_cap < 1:
        raise ValueError("Partial-penalty kappa must be nonnegative, epsilon positive, and cap at least 1.")

    available = {sku: [] for sku in COLORS}
    for index, product in enumerate(instance.products):
        available[product.color].append(index)
    remaining = {
        order_id: [int(record["demand_by_sku"].get(sku, 0)) for sku in COLORS]
        for order_id, record in trace_orders.items()
    }
    requested = [sum(remaining[i]) for i in range(order_count)]
    unresolved = set(trace_orders)
    assigned_station = {}
    completion_times = [None] * order_count
    delivered_by_deadline = [0] * order_count
    now = table.startup_delay
    current_station = None
    active_order = None
    preemption_events = 0
    maximum_queue = 0
    dispatch_log = []
    pick_log = []
    order_log = []

    def has_pickable_unit(order_id):
        return any(
            remaining[order_id][sku_index] > 0 and available[sku]
            for sku_index, sku in enumerate(COLORS)
        )

    def close_order(order_id, departure_time):
        nonlocal active_order
        shortage_by_sku = {
            sku: remaining[order_id][sku_index]
            for sku_index, sku in enumerate(COLORS)
        }
        shortage_units = sum(shortage_by_sku.values())
        shortage_sku_types = sum(quantity > 0 for quantity in shortage_by_sku.values())
        due = float(trace_orders[order_id]["due_time"])
        slack_at_departure = due - departure_time
        urgency_factor = min(
            partial_penalty_cap,
            1.0 + partial_penalty_kappa / (max(slack_at_departure, 0.0) + partial_penalty_epsilon),
        )
        completion_times[order_id] = departure_time
        order_log.append({
            "order_id": order_id + 1,
            "dispatch_time": float(first_dispatch[order_id]),
            "departure_time": departure_time,
            "due_time": due,
            "slack_at_departure": slack_at_departure,
            "requested_units": requested[order_id],
            "locally_shipped_units": requested[order_id] - shortage_units,
            "shortage_units": shortage_units,
            "shortage_sku_types": shortage_sku_types,
            "urgency_factor": urgency_factor,
            "weighted_partial_penalty": shortage_units * urgency_factor,
        })
        unresolved.remove(order_id)
        assigned_station.pop(order_id, None)
        if active_order == order_id:
            active_order = None

    first_dispatch = {}
    while unresolved:
        waiting = sorted(
            order_id for order_id in unresolved
            if float(trace_orders[order_id]["arrival_time"]) <= now
        )
        if not waiting:
            now = max(now, min(float(trace_orders[i]["arrival_time"]) for i in unresolved))
            waiting = sorted(
                order_id for order_id in unresolved
                if float(trace_orders[order_id]["arrival_time"]) <= now
            )
        maximum_queue = max(maximum_queue, len(waiting))

        for order_id in waiting:
            if not has_pickable_unit(order_id):
                first_dispatch.setdefault(order_id, now)
                close_order(order_id, now)
        waiting = [order_id for order_id in waiting if order_id in unresolved]
        if not waiting:
            continue

        plans = {}
        for order_id in waiting:
            demand = remaining[order_id]
            if order_id in assigned_station:
                plan = _greedy_order_plan_at_station(
                    instance, available, demand, current_station,
                    assigned_station[order_id], table,
                )
            else:
                plan = _greedy_order_plan(
                    instance, available, order_id, demand, current_station, table
                )
            plans[order_id] = plan

        chosen_order = min(
            waiting,
            key=lambda order_id: (
                float(trace_orders[order_id]["due_time"]) - now - plans[order_id][0],
                float(trace_orders[order_id]["due_time"]),
                order_id,
            ),
        )
        _, route, station = plans[chosen_order]
        if not route:
            first_dispatch.setdefault(chosen_order, now)
            close_order(chosen_order, now)
            continue
        if active_order is not None and active_order != chosen_order and has_pickable_unit(active_order):
            preemption_events += 1
        first_dispatch.setdefault(chosen_order, now)
        assigned_station.setdefault(chosen_order, station)
        product_index, _ = route[0]
        product = instance.products[product_index]
        duration = table.duration(
            current_station, product.bay, product.level, assigned_station[chosen_order]
        )
        box_out_at = now + duration.box_out
        if box_out_at <= float(trace_orders[chosen_order]["due_time"]):
            delivered_by_deadline[chosen_order] += 1
        dispatch_log.append({"order_id": chosen_order + 1, "time": now})
        pick_log.append({
            "order_id": chosen_order + 1,
            "box": product.product_id,
            "cell": f"Cell-B{product.bay + 1:02d}-L{product.level + 1}",
            "color": product.color,
            "station": assigned_station[chosen_order],
            "dispatch_time": now,
            "completed_at": box_out_at,
        })
        available[product.color].remove(product_index)
        remaining[chosen_order][COLORS.index(product.color)] -= 1
        now += duration.ready
        current_station = assigned_station[chosen_order]
        active_order = chosen_order
        if not any(remaining[chosen_order]) or not has_pickable_unit(chosen_order):
            close_order(chosen_order, box_out_at)

    complete = sum(delivered_by_deadline[i] >= requested[i] for i in range(order_count))
    partial = sum(0 < delivered_by_deadline[i] < requested[i] for i in range(order_count))
    unprocessed = sum(delivered_by_deadline[i] == 0 for i in range(order_count))
    rescue_by_order = [requested[i] - delivered_by_deadline[i] for i in range(order_count)]
    response_times = [
        float(completion_times[i]) - float(trace_orders[i]["arrival_time"])
        for i in range(order_count)
    ]
    result = {
        "complete_orders_by_deadline": complete,
        "partial_orders_by_deadline": partial,
        "unprocessed_orders_by_deadline": unprocessed,
        "rescue_orders": sum(value > 0 for value in rescue_by_order),
        "rescue_units": sum(rescue_by_order),
        "partial_orders_at_departure": sum(row["shortage_units"] > 0 for row in order_log),
        "partial_shortage_units_at_departure": sum(row["shortage_units"] for row in order_log),
        "partial_shortage_sku_types_at_departure": sum(row["shortage_sku_types"] for row in order_log),
        "weighted_partial_penalty": sum(row["weighted_partial_penalty"] for row in order_log),
        "shipped_units_by_deadline": sum(delivered_by_deadline),
        "mean_response_time": mean(response_times) if response_times else 0.0,
        "p90_response_time": _percentile(response_times, 0.90),
        "p95_response_time": _percentile(response_times, 0.95),
        "full_run_makespan": max(completion_times, default=0.0),
        "maximum_queue": maximum_queue,
        "optimizer_seconds": 0.0,
        "preemption_events": preemption_events,
    }
    if capture_log:
        result["dispatch_log"] = dispatch_log
        result["pick_log"] = pick_log
        result["order_log"] = order_log
    return result


def run_experiment(instance_path: Path, trace_dir: Path, table_path: Path, output_dir: Path,
                   seed: int = 42, repeats: int = 3, ants: int = 8, iterations: int = 12,
                   partial_penalty_kappa: float = 60.0,
                   partial_penalty_epsilon: float = 1.0,
                   partial_penalty_cap: float = 5.0):
    instance_paths = (
        sorted(instance_path.glob("*.json"))
        if instance_path.is_dir()
        else [instance_path]
    )
    if not instance_paths:
        raise ValueError(f"No JSON instances found in {instance_path}")
    instances_by_hash = {
        hashlib.sha256(path.read_bytes()).hexdigest(): (path, load_instance(path))
        for path in instance_paths
    }
    table = CraneCycleTimeTable.load(table_path)
    trace_paths = sorted(trace_dir.glob("*.json"))
    if not trace_paths:
        raise ValueError(f"No JSON arrival traces found in {trace_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    raw = []
    order_rows = []
    for trace_index, trace_path in enumerate(trace_paths):
        trace_payload = json.loads(trace_path.read_text(encoding="utf-8"))
        instance_hash = trace_payload.get("source_instance", {}).get("sha256")
        if instance_hash not in instances_by_hash:
            raise ValueError(f"Trace {trace_path} does not match an allowed source instance.")
        expected_cycle_hash = trace_payload.get("generation", {}).get("cycle_time_table_sha256")
        actual_cycle_hash = hashlib.sha256(table_path.read_bytes()).hexdigest()
        if expected_cycle_hash != actual_cycle_hash:
            raise ValueError(f"Trace {trace_path} was generated with a different cycle-time table.")
        source_path, instance = instances_by_hash[instance_hash]
        trace, _ = _trace_read(trace_path, source_path)
        scenario = source_path.stem
        trace_id = trace_path.stem
        for policy in POLICIES:
            policy_repeats = repeats if policy in {"Random", "rolling_ACO"} else 1
            for run in range(policy_repeats):
                run_seed = seed + trace_index * 10007 + run
                if policy == "preemptive_minimum_slack":
                    metrics = simulate_preemptive_minimum_slack(
                        instance, trace, table,
                        partial_penalty_kappa=partial_penalty_kappa,
                        partial_penalty_epsilon=partial_penalty_epsilon,
                        partial_penalty_cap=partial_penalty_cap,
                        capture_log=True,
                    )
                else:
                    metrics = simulate_online_policy(
                        instance, trace, table, policy, run_seed, ants, iterations,
                        capture_log=True,
                        partial_penalty_kappa=partial_penalty_kappa,
                        partial_penalty_epsilon=partial_penalty_epsilon,
                        partial_penalty_cap=partial_penalty_cap,
                    )
                for order_result in metrics.pop("order_log"):
                    order_rows.append({
                        "scenario": scenario,
                        "trace": trace_id,
                        "policy": policy,
                        "run": run,
                        "seed": run_seed,
                        **order_result,
                    })
                raw.append({
                    "scenario": scenario,
                    "trace": trace_id,
                    "policy": policy,
                    "run": run,
                    "seed": run_seed,
                    "mean_interarrival": trace["generation"]["mean_interarrival_time"],
                    "due_slack_factor": trace["generation"]["due_slack_factor"],
                    "urgency_spread": trace["generation"]["urgency_spread"],
                    **metrics,
                })

    raw_path = output_dir / "online_policy_raw.csv"
    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(raw[0]))
        writer.writeheader()
        writer.writerows(raw)

    summary = []
    paired_metrics = (
        "complete_orders_by_deadline", "rescue_units", "weighted_partial_penalty",
        "mean_response_time", "p95_response_time", "preemption_events",
    )
    conditions = sorted({
        (float(row["mean_interarrival"]), float(row["due_slack_factor"]))
        for row in raw
    })
    for mean_interarrival, due_slack_factor in conditions:
        scenarios = sorted({
            row["scenario"] for row in raw
            if float(row["mean_interarrival"]) == mean_interarrival
            and float(row["due_slack_factor"]) == due_slack_factor
        })
        scenario_values = {}
        for policy in POLICIES:
            by_scenario = []
            for scenario in scenarios:
                rows = [
                    row for row in raw
                    if row["scenario"] == scenario
                    and row["policy"] == policy
                    and float(row["mean_interarrival"]) == mean_interarrival
                    and float(row["due_slack_factor"]) == due_slack_factor
                ]
                if rows:
                    by_scenario.append({
                        "scenario": scenario,
                        "traces": len({row["trace"] for row in rows}),
                        **{metric: mean(float(row[metric]) for row in rows) for metric in METRICS},
                    })
            if not by_scenario:
                continue
            scenario_values[policy] = {row["scenario"]: row for row in by_scenario}
            record = {
                "mean_interarrival": mean_interarrival,
                "due_slack_factor": due_slack_factor,
                "urgency_spread": mean(
                    float(row["urgency_spread"]) for row in raw
                    if float(row["mean_interarrival"]) == mean_interarrival
                    and float(row["due_slack_factor"]) == due_slack_factor
                ),
                "policy": policy,
                "scenarios": len(by_scenario),
                "mean_traces_per_scenario": mean(float(row["traces"]) for row in by_scenario),
            }
            for metric in METRICS:
                avg, deviation, low, high = _mean_ci([float(row[metric]) for row in by_scenario])
                record[metric] = avg
                record[f"{metric}_between_scenario_sd"] = deviation
                record[f"{metric}_ci95_low"] = low
                record[f"{metric}_ci95_high"] = high
            fcfs = scenario_values.get("FCFS", {})
            minimum_slack = scenario_values.get("minimum_slack", {})
            for metric in paired_metrics:
                deltas = (
                    [
                        float(scenario_values[policy][scenario][metric]) - float(fcfs[scenario][metric])
                        for scenario in scenario_values[policy]
                        if scenario in fcfs
                    ]
                    if policy != "FCFS"
                    else [0.0 for _ in by_scenario]
                )
                avg, _, low, high = _mean_ci(deltas)
                record[f"{metric}_delta_vs_fcfs"] = avg
                record[f"{metric}_delta_vs_fcfs_ci95_low"] = low
                record[f"{metric}_delta_vs_fcfs_ci95_high"] = high
                min_slack_deltas = (
                    [
                        float(scenario_values[policy][scenario][metric])
                        - float(minimum_slack[scenario][metric])
                        for scenario in scenario_values[policy]
                        if scenario in minimum_slack
                    ]
                    if policy == "preemptive_minimum_slack"
                    else [0.0 for _ in by_scenario]
                )
                avg, _, low, high = _mean_ci(min_slack_deltas)
                record[f"{metric}_delta_vs_minimum_slack"] = avg
                record[f"{metric}_delta_vs_minimum_slack_ci95_low"] = low
                record[f"{metric}_delta_vs_minimum_slack_ci95_high"] = high
            summary.append(record)
    with (output_dir / "online_policy_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)

    with (output_dir / "partial_order_penalty_raw.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(order_rows[0]))
        writer.writeheader()
        writer.writerows(order_rows)

    design = {
        "source_instances": [
            {"path": str(path), "sha256": digest}
            for digest, (path, _) in instances_by_hash.items()
        ],
        "arrival_trace_files": [str(path) for path in trace_paths],
        "arrival_trace_generation_parameters": [
            {
                "file": path.name,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "seed": json.loads(path.read_text(encoding="utf-8"))["generation"]["seed"],
                "mean_interarrival_time": json.loads(path.read_text(encoding="utf-8"))["generation"]["mean_interarrival_time"],
                "due_slack_factor": json.loads(path.read_text(encoding="utf-8"))["generation"]["due_slack_factor"],
                "urgency_spread": json.loads(path.read_text(encoding="utf-8"))["generation"]["urgency_spread"],
            }
            for path in trace_paths
        ],
        "cycle_time_table": str(table_path),
        "cycle_time_table_sha256": hashlib.sha256(table_path.read_bytes()).hexdigest(),
        "python_version": platform.python_version(),
        "experiment_seed": seed,
        "policies": list(POLICIES),
        "crane_service": "most policies are non-preemptive and serve an order to its available SKU limit; preemptive_minimum_slack reselects an order after each complete pick-and-delivery cycle",
        "preemption": {
            "policy": "preemptive_minimum_slack",
            "decision_boundary": "after box-out and crane-ready at the unloading station; no mid-motion interruption",
            "station_rule": "each order retains its unloading station after its first pick",
            "staging_rule": "partially fulfilled orders wait in unlimited per-order staging",
            "event_count": "switching away from a still-pickable active order",
            "interruption_or_resumption_overhead": "no extra fixed overhead; travel from current station and cell is included in DEVS cycle durations",
        },
        "inventory_rule": "SKU stock is reserved and removed when the selected order's retrieval cycle executes; no replenishment",
        "online_information": "dispatch rules see only released, unresolved orders and current remaining inventory",
        "ACO": {
            "ants": ants,
            "iterations": iterations,
            "repeats": repeats,
            "penalty_weight": 100.0,
            "objective": "complete-order priority, then shortage, then makespan",
            "run_seed_rule": "experiment_seed + trace_index * 10007 + run_index",
            "replan_seed_rule": "run_seed + dispatch_number * 1009",
        },
        "partial_order_penalty": {
            "formula": "shortage_units * min(cap, 1 + kappa / (max(due_time - local_departure_time, 0) + epsilon))",
            "kappa_time_units": partial_penalty_kappa,
            "epsilon_time_units": partial_penalty_epsilon,
            "urgency_factor_cap": partial_penalty_cap,
            "shortage_basis": "missing SKU units when the order's local service ends; distinct missing SKU types are reported separately but are not penalized in this pilot",
            "scope": "post-hoc evaluation only; dispatch policies and ACO objective are unchanged",
        },
        "statistics": "ACO and Random repeats and multiple arrival traces are averaged within each source instance; confidence intervals use source instances as the replication unit",
        "caveat": "synthetic Poisson arrivals and due dates; DEVS-derived cycle table, not hardware-calibrated; no backup-center queue/capacity",
    }
    (output_dir / "online_experiment_design.json").write_text(
        json.dumps(design, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"Saved {len(raw)} online-policy observations from {len(trace_paths)} fixed traces to {output_dir.resolve()}")
    for row in summary:
        print(
            f"{row['policy']}: complete={row['complete_orders_by_deadline']:.2f}, "
            f"rescue_units={row['rescue_units']:.2f}, mean_response={row['mean_response_time']:.2f}, "
            f"p95={row['p95_response_time']:.2f}, "
            f"preemptions={row['preemption_events']:.2f}"
        )


def main(argv=None):
    default_table = Path(__file__).resolve().parent / "data" / "crane_cycle_times.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True, type=Path)
    parser.add_argument("--trace-dir", required=True, type=Path)
    parser.add_argument("--cycle-times", type=Path, default=default_table)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--ants", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=12)
    parser.add_argument("--partial-penalty-kappa", type=float, default=60.0)
    parser.add_argument("--partial-penalty-epsilon", type=float, default=1.0)
    parser.add_argument("--partial-penalty-cap", type=float, default=5.0)
    args = parser.parse_args(argv)
    run_experiment(
        args.instance, args.trace_dir, args.cycle_times, args.output_dir,
        args.seed, args.repeats, args.ants, args.iterations,
        args.partial_penalty_kappa, args.partial_penalty_epsilon,
        args.partial_penalty_cap,
    )


if __name__ == "__main__":
    main()
