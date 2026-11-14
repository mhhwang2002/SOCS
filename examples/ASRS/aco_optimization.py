"""ACO plus SOCS simulation for a single-crane AS/RS order set."""

from __future__ import annotations

import argparse
import json
import random
from dataclasses import dataclass
from math import inf
from pathlib import Path

from socs.devs.DEVS import DEVS
from socs.devs.TESEngin import ExecutionResult, TESEnginBase

from .crane_cycle_times import CraneCycleTimeTable


COLORS = ("red", "blue", "green", "yellow", "white")
CELL_COUNT = 90
PRODUCT_COUNT_PER_COLOR = 10
ORDER_COUNT = 10
PRODUCTS_PER_ORDER = 5
RACK_COLUMNS = 15
LEVELS = 6
CELL_WIDTH = 1.0
LEVEL_PITCH = 0.75
X_SPEED = 2.0
Z_SPEED = 2.0
PICK_TIME = 0.5
STATION_TIME = 1.0


@dataclass(frozen=True)
class Product:
    product_id: str
    color: str
    bay: int
    level: int
    order_id: int

    @property
    def position(self) -> tuple[float, float]:
        crane_z = self.level * LEVEL_PITCH + 0.27 - 0.21 + 0.08
        return ((self.bay + 0.5) * CELL_WIDTH, crane_z)


@dataclass(frozen=True)
class ASRSInstance:
    products: tuple[Product, ...]
    order_products: tuple[tuple[str, ...], ...]
    order_penalties: tuple[int, ...]
    order_demands: tuple[tuple[int, ...], ...] = ()
    initial_position: tuple[float, float] = (-0.5, LEVEL_PITCH / 2)
    station_positions: tuple[tuple[float, float], ...] = (
        (-0.5, LEVEL_PITCH / 2),
        (RACK_COLUMNS * CELL_WIDTH + 0.5, LEVEL_PITCH / 2),
    )


@dataclass(frozen=True)
class Schedule:
    product_order: tuple[int, ...]
    order_station: tuple[int, ...]
    product_assignments: tuple[int, ...] = ()
    order_priority: tuple[int, ...] = ()


@dataclass(frozen=True)
class ScheduleResult:
    makespan: float
    order_completion_times: tuple[float | None, ...]
    shipped_by_order: tuple[int, ...]
    requested_by_order: tuple[int, ...]
    shortage_units: int
    complete_order_count: int

    @property
    def mean_order_completion_time(self) -> float:
        completed = [time for time in self.order_completion_times if time is not None]
        return sum(completed) / len(completed) if completed else 0.0


def generate_instance(seed: int = 42) -> ASRSInstance:
    """Generate 50 color-balanced products and a feasible random order set."""
    rng = random.Random(seed)
    locations = [(bay, level) for level in range(LEVELS) for bay in range(RACK_COLUMNS)]
    rng.shuffle(locations)
    colors = [color for color in COLORS for _ in range(PRODUCT_COUNT_PER_COLOR)]
    rng.shuffle(colors)
    product_orders = [index for index in range(ORDER_COUNT) for _ in range(PRODUCTS_PER_ORDER)]
    rng.shuffle(product_orders)

    products = []
    order_products = [[] for _ in range(ORDER_COUNT)]
    for index, ((bay, level), color, order_id) in enumerate(
        zip(locations[: len(colors)], colors, product_orders)
    ):
        product_id = f"P{index:02d}"
        products.append(Product(product_id, color, bay, level, order_id))
        order_products[order_id].append(product_id)
    partial_order_count = rng.randint(3, 5)
    partial_order_ids = set(rng.sample(range(ORDER_COUNT), partial_order_count))
    order_penalties = tuple(
        rng.randint(1, 2) if order_id in partial_order_ids else 0
        for order_id in range(ORDER_COUNT)
    )
    color_index = {color: index for index, color in enumerate(COLORS)}
    order_demands = [[0] * len(COLORS) for _ in range(ORDER_COUNT)]
    for product in products:
        order_demands[product.order_id][color_index[product.color]] += 1
    for order_id, shortage in enumerate(order_penalties):
        for _ in range(shortage):
            order_demands[order_id][color_index[rng.choice(COLORS)]] += 1
    return ASRSInstance(
        tuple(products), tuple(tuple(items) for items in order_products), order_penalties,
        tuple(tuple(demand) for demand in order_demands),
    )


def save_instance(instance: ASRSInstance, path, seed: int | None = None) -> Path:
    """Save an AS/RS problem instance as readable, versioned JSON."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    products_by_order = {
        order_id: list(product_ids)
        for order_id, product_ids in enumerate(instance.order_products)
    }
    payload = {
        "format": "socs-asrs-instance",
        "version": 2,
        "generation_seed": seed,
        "rack": {"columns": RACK_COLUMNS, "levels": LEVELS},
        "products": [
            {
                "product_id": product.product_id,
                "color": product.color,
                "bay": product.bay + 1,
                "level": product.level + 1,
                "order_id": product.order_id + 1,
            }
            for product in instance.products
        ],
        "orders": [
            {
                "order_id": order_id + 1,
                "product_ids": product_ids,
                "requested_quantity": len(product_ids) + instance.order_penalties[order_id],
                "shortage_quantity": instance.order_penalties[order_id],
                "demand_by_color": {
                    color: instance.order_demands[order_id][index]
                    for index, color in enumerate(COLORS)
                },
            }
            for order_id, product_ids in products_by_order.items()
        ],
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def load_instance(path) -> ASRSInstance:
    """Load and validate a saved AS/RS problem instance."""
    path = Path(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("format") != "socs-asrs-instance" or payload.get("version") not in {1, 2}:
        raise ValueError(f"Unsupported AS/RS instance format or version in {path}.")
    rack = payload.get("rack", {})
    if rack.get("columns") != RACK_COLUMNS or rack.get("levels") != LEVELS:
        raise ValueError(f"Instance rack must be {RACK_COLUMNS} columns by {LEVELS} levels.")

    order_records = payload.get("orders")
    product_records = payload.get("products")
    if not isinstance(order_records, list) or not order_records:
        raise ValueError("Instance must contain a non-empty orders list.")
    if not isinstance(product_records, list):
        raise ValueError("Instance must contain a products list.")
    order_records = sorted(order_records, key=lambda order: order.get("order_id", 0))
    expected_order_ids = list(range(1, len(order_records) + 1))
    if [order.get("order_id") for order in order_records] != expected_order_ids:
        raise ValueError("Order IDs must be consecutive, starting at 1.")

    order_products = []
    order_penalties = []
    for order in order_records:
        product_ids = order.get("product_ids")
        requested = order.get("requested_quantity")
        shortage = order.get("shortage_quantity")
        if not isinstance(product_ids, list) or not all(isinstance(item, str) for item in product_ids):
            raise ValueError(f"Order {order['order_id']} must list product_ids as strings.")
        if len(set(product_ids)) != len(product_ids):
            raise ValueError(f"Order {order['order_id']} contains duplicate product IDs.")
        if not isinstance(requested, int) or not isinstance(shortage, int) or shortage < 0:
            raise ValueError(f"Order {order['order_id']} has invalid quantity fields.")
        if requested != len(product_ids) + shortage:
            raise ValueError(
                f"Order {order['order_id']} requested_quantity must equal fulfillable items plus shortage_quantity."
            )
        order_products.append(tuple(product_ids))
        order_penalties.append(shortage)

    products = []
    seen_ids = set()
    seen_cells = set()
    for record in product_records:
        product_id = record.get("product_id")
        color = record.get("color")
        bay, level, order_id = record.get("bay"), record.get("level"), record.get("order_id")
        if not isinstance(product_id, str) or product_id in seen_ids:
            raise ValueError(f"Invalid or duplicate product ID: {product_id!r}.")
        if color not in COLORS:
            raise ValueError(f"Product {product_id} has unsupported color {color!r}.")
        if not isinstance(bay, int) or not 1 <= bay <= RACK_COLUMNS:
            raise ValueError(f"Product {product_id} bay must be between 1 and {RACK_COLUMNS}.")
        if not isinstance(level, int) or not 1 <= level <= LEVELS:
            raise ValueError(f"Product {product_id} level must be between 1 and {LEVELS}.")
        if not isinstance(order_id, int) or not 1 <= order_id <= len(order_records):
            raise ValueError(f"Product {product_id} has invalid order_id {order_id!r}.")
        cell = (bay, level)
        if cell in seen_cells:
            raise ValueError(f"Multiple products occupy cell B{bay}/L{level}.")
        if product_id not in order_products[order_id - 1]:
            raise ValueError(f"Product {product_id} is not listed in order {order_id}.")
        seen_ids.add(product_id)
        seen_cells.add(cell)
        products.append(Product(product_id, color, bay - 1, level - 1, order_id - 1))

    listed_ids = [product_id for order in order_products for product_id in order]
    if len(listed_ids) != len(set(listed_ids)) or set(listed_ids) != seen_ids:
        raise ValueError("Each product must belong to exactly one order and appear in products.")
    products_by_id = {product.product_id: product for product in products}
    demand_rng = random.Random(payload.get("generation_seed", 42))
    order_demands = []
    for order_id, record in enumerate(order_records):
        demand_record = record.get("demand_by_color")
        if demand_record is None:
            demand = [0] * len(COLORS)
            for product_id in order_products[order_id]:
                demand[COLORS.index(products_by_id[product_id].color)] += 1
            for _ in range(order_penalties[order_id]):
                demand[COLORS.index(demand_rng.choice(COLORS))] += 1
        else:
            if not isinstance(demand_record, dict) or set(demand_record) - set(COLORS):
                raise ValueError(f"Order {order_id + 1} has invalid demand_by_color.")
            demand = [demand_record.get(color, 0) for color in COLORS]
            if any(not isinstance(value, int) or value < 0 for value in demand):
                raise ValueError(f"Order {order_id + 1} has invalid color demand quantities.")
            if sum(demand) != len(order_products[order_id]) + order_penalties[order_id]:
                raise ValueError(f"Order {order_id + 1} color demand does not match requested quantity.")
        order_demands.append(tuple(demand))
    return ASRSInstance(
        tuple(products), tuple(order_products), tuple(order_penalties), tuple(order_demands)
    )


def travel_time(start: tuple[float, float], end: tuple[float, float]) -> float:
    """Estimate simultaneous X/Z travel at constant axis speeds."""
    dx = abs(end[0] - start[0]) / X_SPEED
    dz = abs(end[1] - start[1]) / Z_SPEED
    return max(dx, dz)


class ScheduleSimulation(DEVS):
    """Execute one candidate schedule as SOCS internal DEVS events."""

    def __init__(self, instance: ASRSInstance, schedule: Schedule):
        super().__init__("ScheduleSimulation")
        self.instance = instance
        self.schedule = schedule
        self.assignments = schedule.product_assignments or tuple(
            instance.products[index].order_id for index in schedule.product_order
        )
        self.delta_times = self._build_task_durations()
        self.delta_order_completions = self._build_completion_marks()
        self.set_initial_state()

    def _build_task_durations(self) -> tuple[float, ...]:
        position = self.instance.initial_position
        durations = []
        for product_index, order_id in zip(self.schedule.product_order, self.assignments):
            product = self.instance.products[product_index]
            station = self.instance.station_positions[self.schedule.order_station[order_id]]
            durations.append(
                travel_time(position, product.position)
                + PICK_TIME
                + travel_time(product.position, station)
                + STATION_TIME
            )
            position = station
        return tuple(durations)

    def _build_completion_marks(self) -> tuple[int | None, ...]:
        last_task = {
            order_id: index for index, order_id in enumerate(self.assignments)
        }
        return tuple(
            order_id if last_task.get(order_id) == index else None
            for index, order_id in enumerate(self.assignments)
        )

    def set_initial_state(self):
        self.task_index = 0
        self.elapsed = 0.0
        self.order_completion_times = [None] * len(self.instance.order_products)

    def tau(self):
        if self.task_index >= len(self.delta_times):
            return inf
        return self.delta_times[self.task_index]

    def delta_x(self, event):
        return False

    def delta_y(self):
        self.elapsed = self.tNext
        completed_order = self.delta_order_completions[self.task_index]
        if completed_order is not None:
            self.order_completion_times[completed_order] = self.elapsed
        self.task_index += 1
        return None

    def get_state(self):
        return self.task_index, self.elapsed, tuple(self.order_completion_times)

    def set_state(self, state):
        self.task_index, self.elapsed, completions = state
        self.order_completion_times = list(completions)


def evaluate_schedule(
    instance: ASRSInstance,
    schedule: Schedule,
    cycle_time_table: CraneCycleTimeTable | None = None,
    initial_station: int | None = None,
) -> ScheduleResult:
    """Evaluate a schedule with a DEVS-measured cycle table or the legacy estimate."""
    if cycle_time_table is not None:
        assignments = schedule.product_assignments or tuple(
            instance.products[index].order_id for index in schedule.product_order
        )
        order_count = len(instance.order_products)
        completion_marks = {
            order_id: index for index, order_id in enumerate(assignments)
        }
        completion_times: list[float | None] = [None] * order_count
        shipped_values = [0] * order_count
        ready_time = cycle_time_table.startup_delay
        last_box_out = 0.0
        previous_station = initial_station
        for index, (product_index, order_id) in enumerate(zip(schedule.product_order, assignments)):
            product = instance.products[product_index]
            station = schedule.order_station[order_id]
            duration = cycle_time_table.duration(
                previous_station, product.bay, product.level, station
            )
            box_out_time = ready_time + duration.box_out
            ready_time += duration.ready
            last_box_out = box_out_time
            shipped_values[order_id] += 1
            if completion_marks[order_id] == index:
                completion_times[order_id] = box_out_time
            previous_station = station

        shipped = tuple(shipped_values)
        requested = tuple(
            sum(instance.order_demands[order_id])
            if instance.order_demands
            else len(items) + instance.order_penalties[order_id]
            for order_id, items in enumerate(instance.order_products)
        )
        shortage_units = sum(
            max(0, requested[order_id] - shipped[order_id])
            for order_id in range(order_count)
        )
        complete_count = sum(
            shipped[order_id] >= requested[order_id]
            for order_id in range(order_count)
        )
        return ScheduleResult(
            last_box_out,
            tuple(completion_times),
            shipped,
            requested,
            shortage_units,
            complete_count,
        )

    model = ScheduleSimulation(instance, schedule)
    engine = TESEnginBase(
        model,
        TimeIntervalContinuous=inf,
        TimeIntervalAnimation=inf,
        TimeSimulationEnd=1e9,
    )
    engine.bVerbose = False
    while model.task_index < len(model.delta_times):
        result = engine.execute_one_step()
        if result is ExecutionResult.TimeBoundReached:
            break
    times = tuple(
        None if value is None else float(value)
        for value in model.order_completion_times
    )
    shipped_values = [0] * len(instance.order_products)
    for order_id in model.assignments:
        shipped_values[order_id] += 1
    shipped = tuple(shipped_values)
    requested = tuple(
        sum(instance.order_demands[order_id])
        if instance.order_demands
        else len(items) + instance.order_penalties[order_id]
        for order_id, items in enumerate(instance.order_products)
    )
    shortage_units = sum(
        max(0, requested[order_id] - shipped[order_id])
        for order_id in range(len(requested))
    )
    complete_count = sum(
        shipped[order_id] >= requested[order_id]
        for order_id in range(len(requested))
    )
    return ScheduleResult(model.elapsed, times, shipped, requested, shortage_units, complete_count)


class AntColonyOptimizer:
    """Search product routes, order priority, and color-compatible allocation."""

    def __init__(
        self,
        instance: ASRSInstance,
        objective: str,
        seed: int,
        ants: int = 12,
        iterations: int = 25,
        alpha: float = 1.0,
        beta: float = 3.0,
        evaporation: float = 0.25,
        penalty_weight: float = 1.0,
        cycle_time_table: CraneCycleTimeTable | None = None,
        initial_station: int | None = None,
    ):
        if objective not in {"makespan", "mean"}:
            raise ValueError("objective must be 'makespan' or 'mean'")
        if penalty_weight < 0:
            raise ValueError("penalty_weight must be non-negative")
        if not instance.order_demands:
            raise ValueError("Instance must include per-order color demand.")
        self.instance = instance
        self.objective = objective
        self.rng = random.Random(seed)
        self.ants = ants
        self.iterations = iterations
        self.alpha = alpha
        self.beta = beta
        self.evaporation = evaporation
        self.penalty_weight = penalty_weight
        self.cycle_time_table = cycle_time_table
        if initial_station not in (None, 0, 1):
            raise ValueError("initial_station must be None (home), 0 (left), or 1 (right)")
        self.initial_station = initial_station
        order_count = len(instance.order_products)
        product_count = len(instance.products)
        self.task_pheromone = [[1.0 for _ in range(product_count)] for _ in range(product_count + 1)]
        self.station_pheromone = [[1.0, 1.0] for _ in range(order_count)]
        self.order_pheromone = [[1.0 for _ in range(order_count)] for _ in range(order_count + 1)]
        requested = [sum(demand) for demand in instance.order_demands]
        if cycle_time_table is None:
            points = [instance.initial_position, *instance.station_positions]
            points.extend(product.position for product in instance.products)
            max_leg = max(travel_time(a, b) for a in points for b in points)
            self.max_makespan = len(instance.products) * (
                2.0 * max_leg + PICK_TIME + STATION_TIME
            )
        else:
            max_cycle = max(
                duration.ready
                for duration in (*cycle_time_table.first_cycles.values(), *cycle_time_table.cycles.values())
            )
            self.max_makespan = cycle_time_table.startup_delay + len(instance.products) * max_cycle
        self.completion_priority = (
            self.penalty_weight * sum(requested) + self.max_makespan + 1.0
        )
        self.best_score = inf

    def _construct(self) -> Schedule:
        products = self.instance.products
        colors = {color: index for index, color in enumerate(COLORS)}
        available = {color: [] for color in COLORS}
        for product_index, product in enumerate(products):
            available[product.color].append(product_index)
        remaining_counts = {color: len(indices) for color, indices in available.items()}
        remaining_orders = set(range(len(self.instance.order_products)))
        order_priority = []
        previous_order = len(remaining_orders)

        while remaining_orders:
            candidates = sorted(remaining_orders)
            weights = []
            for order_id in candidates:
                demand = self.instance.order_demands[order_id]
                fully_feasible = all(
                    demand[color_index] <= remaining_counts[color]
                    for color, color_index in colors.items()
                )
                demand_size = max(sum(demand), 1)
                feasibility = 2.0 if fully_feasible else 1.0
                heuristic = feasibility / demand_size
                pheromone = self.order_pheromone[previous_order][order_id]
                weights.append((pheromone**self.alpha) * (heuristic**self.beta))
            order_id = self.rng.choices(candidates, weights=weights, k=1)[0]
            order_priority.append(order_id)
            remaining_orders.remove(order_id)
            for color, color_index in colors.items():
                remaining_counts[color] = max(
                    0,
                    remaining_counts[color]
                    - self.instance.order_demands[order_id][color_index],
                )
            previous_order = order_id

        remaining_counts = {color: len(indices) for color, indices in available.items()}
        route = []
        assignments = []
        station_by_order: list[int | None] = [None] * len(self.instance.order_products)
        current_position = self.instance.initial_position
        current_station = self.initial_station
        previous_product = len(products)

        for order_id in order_priority:
            demand_remaining = list(self.instance.order_demands[order_id])
            while True:
                candidates = [
                    product_index
                    for color_index, color in enumerate(COLORS)
                    if demand_remaining[color_index] > 0
                    for product_index in available[color]
                ]
                if not candidates:
                    break

                actions = []
                weights = []
                possible_stations = (0, 1) if station_by_order[order_id] is None else (station_by_order[order_id],)
                for product_index in candidates:
                    product = products[product_index]
                    for station_index in possible_stations:
                        station = self.instance.station_positions[station_index]
                        if self.cycle_time_table is None:
                            duration = (
                                travel_time(current_position, product.position)
                                + PICK_TIME
                                + travel_time(product.position, station)
                                + STATION_TIME
                            )
                        else:
                            duration = self.cycle_time_table.duration(
                                current_station, product.bay, product.level, station_index
                            ).ready
                        pheromone = self.task_pheromone[previous_product][product_index]
                        if station_by_order[order_id] is None:
                            pheromone *= self.station_pheromone[order_id][station_index]
                        heuristic = 1.0 / max(duration, 1e-9)
                        actions.append((product_index, station_index, duration))
                        weights.append((pheromone**self.alpha) * (heuristic**self.beta))

                product_index, station_index, _ = self.rng.choices(actions, weights=weights, k=1)[0]
                product = products[product_index]
                if station_by_order[order_id] is None:
                    station_by_order[order_id] = station_index
                route.append(product_index)
                assignments.append(order_id)
                available[product.color].remove(product_index)
                remaining_counts[product.color] -= 1
                demand_remaining[colors[product.color]] -= 1
                current_station = station_by_order[order_id]
                current_position = self.instance.station_positions[current_station]
                previous_product = product_index

        return Schedule(
            tuple(route),
            tuple(0 if value is None else int(value) for value in station_by_order),
            tuple(assignments),
            tuple(order_priority),
        )

    def _score(self, schedule: Schedule) -> tuple[float, ScheduleResult]:
        result = evaluate_schedule(
            self.instance,
            schedule,
            self.cycle_time_table,
            initial_station=self.initial_station,
        )
        time_value = (
            result.makespan
            if self.objective == "makespan"
            else result.mean_order_completion_time
        )
        incomplete_orders = len(result.requested_by_order) - result.complete_order_count
        score = (
            self.completion_priority * incomplete_orders
            + self.penalty_weight * result.shortage_units
            + time_value
        )
        return score, result

    def run(self) -> tuple[Schedule, ScheduleResult, tuple[float, ...]]:
        best_schedule = None
        best_result = None
        best_score = inf
        history = []

        for _ in range(self.iterations):
            iteration_solutions = []
            for _ in range(self.ants):
                schedule = self._construct()
                score, result = self._score(schedule)
                iteration_solutions.append((score, schedule))
                if score < best_score:
                    best_score, best_schedule, best_result = score, schedule, result

            for row in self.task_pheromone:
                for index in range(len(row)):
                    row[index] *= 1.0 - self.evaporation
            for row in self.station_pheromone:
                row[0] *= 1.0 - self.evaporation
                row[1] *= 1.0 - self.evaporation
            for row in self.order_pheromone:
                for index in range(len(row)):
                    row[index] *= 1.0 - self.evaporation

            elite_count = max(1, self.ants // 4)
            ranked = sorted(iteration_solutions, key=lambda item: item[0])[:elite_count]
            for rank, (_, schedule) in enumerate(ranked):
                deposit = 1.0 / (rank + 1)
                previous = len(self.instance.products)
                for product_index in schedule.product_order:
                    self.task_pheromone[previous][product_index] += deposit
                    previous = product_index
                previous_order = len(self.instance.order_products)
                for order_id in schedule.order_priority:
                    self.order_pheromone[previous_order][order_id] += deposit
                    previous_order = order_id
                for order_id in set(schedule.product_assignments):
                    self.station_pheromone[order_id][schedule.order_station[order_id]] += deposit

            history.append(best_score)

        assert best_schedule is not None and best_result is not None
        self.best_score = best_score
        return best_schedule, best_result, tuple(history)


def _report(instance: ASRSInstance, optimizer: AntColonyOptimizer, schedule: Schedule, result: ScheduleResult, history):
    print("\nCombined objective result")
    print(f"  score: {optimizer.best_score:.3f}")
    print(f"  complete orders: {result.complete_order_count}/{len(result.requested_by_order)}")
    print(f"  unshipped items: {result.shortage_units}; weighted penalty: {optimizer.penalty_weight * result.shortage_units:.3f}")
    print(f"  makespan: {result.makespan:.3f} simulation time units")
    print(f"  mean order departure: {result.mean_order_completion_time:.3f}")
    print(f"  completion priority coefficient: {optimizer.completion_priority:.3f}")
    for order_id, requested in enumerate(result.requested_by_order):
        shipped = result.shipped_by_order[order_id]
        status = "complete" if shipped == requested else ("unprocessed" if shipped == 0 else "partial")
        actual = [
            sum(1 for product_index, assigned_order in zip(schedule.product_order, schedule.product_assignments)
                if assigned_order == order_id and instance.products[product_index].color == color)
            for color in COLORS
        ]
        demand = instance.order_demands[order_id]
        color_text = ", ".join(
            f"{color}:{actual[index]}/{demand[index]}"
            for index, color in enumerate(COLORS) if demand[index]
        )
        print(f"  order {order_id + 1:02d}: {status}; shipped {shipped}/{requested}; {color_text}")
    print(f"  order priority: {[order_id + 1 for order_id in schedule.order_priority]}")
    print(f"  station by order (L=0, R=1): {list(schedule.order_station)}")
    route = []
    for product_index, order_id in zip(schedule.product_order, schedule.product_assignments):
        product = instance.products[product_index]
        station = "L" if schedule.order_station[order_id] == 0 else "R"
        route.append(
            f"{product.product_id}({product.color},B{product.bay + 1}/L{product.level + 1},"
            f"O{order_id + 1}->{station})"
        )
    print("  product route and order allocation: " + " -> ".join(route))
    print(f"  best score by iteration: {[round(value, 3) for value in history]}")


def main(argv=None):
    default_cycle_times = Path(__file__).resolve().parent / "data" / "crane_cycle_times.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True, help="saved JSON problem instance")
    parser.add_argument(
        "--cycle-times", type=Path, default=default_cycle_times,
        help="DEVS-measured crane cycle table (default: examples/ASRS/data/crane_cycle_times.json)",
    )
    parser.add_argument("--seed", type=int, default=42, help="ACO search random seed")
    parser.add_argument("--ants", type=int, default=12)
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument("--time-objective", choices=("makespan", "mean"), default="makespan")
    parser.add_argument(
        "--penalty-weight", type=float, default=50000.0,
        help="lambda cost per missing color-specific product",
    )
    args = parser.parse_args(argv)

    instance = load_instance(args.instance)
    cycle_time_table = (
        CraneCycleTimeTable.load(args.cycle_times)
        if args.cycle_times.is_file()
        else None
    )
    print("AS/RS ACO + SOCS experiment")
    print(f"  instance: {Path(args.instance).resolve()}")
    print(f"  ACO search seed: {args.seed}")
    print(f"  objective: completion priority, then lambda-weighted shortage, then {args.time_objective}")
    print(f"  lambda: {args.penalty_weight} simulation time units per missing product")
    print(f"  occupied cells: {len(instance.products)}/{CELL_COUNT}")
    print(f"  products by color: { {color: sum(p.color == color for p in instance.products) for color in COLORS} }")
    print(f"  orders: {len(instance.order_products)}")
    print(f"  total requested by color: { {color: sum(demand[index] for demand in instance.order_demands) for index, color in enumerate(COLORS)} }")
    print(f"  ACO: {args.ants} ants x {args.iterations} iterations")
    print(
        f"  travel model: DEVS-measured crane cycle table ({args.cycle_times})"
        if cycle_time_table is not None
        else "  travel model: legacy constant-speed X/Z estimate (cycle table not found)"
    )

    optimizer = AntColonyOptimizer(
        instance,
        args.time_objective,
        args.seed,
        ants=args.ants,
        iterations=args.iterations,
        penalty_weight=args.penalty_weight,
        cycle_time_table=cycle_time_table,
    )
    schedule, result, history = optimizer.run()
    _report(instance, optimizer, schedule, result, history)


if __name__ == "__main__":
    main()
