"""ACO schedule search with paired-cell retrievals for a two-fork crane."""

from __future__ import annotations

import argparse
import random
from dataclasses import dataclass
from itertools import combinations
from math import inf
from pathlib import Path

from socs.devs.DEVS import DEVS
from socs.devs.TESEngin import ExecutionResult, TESEnginBase

from examples.ASRS.aco_optimization import (
    COLORS,
    ASRSInstance,
    AntColonyOptimizer,
    ScheduleResult,
    load_instance,
    travel_time,
)
from .timing import DEFAULT_TIMING_PROFILE, TimingProfile
from .asrs_devs import STARTUP_DELAY

CELL_WIDTH = 1.0
LEVEL_PITCH = 0.75
FORK_SPACING = 1.0


@dataclass(frozen=True)
class TwoForkSchedule:
    """A product route plus explicit single, simultaneous, or sequential picks."""

    product_order: tuple[int, ...]
    pickup_groups: tuple[tuple[int, ...], ...]
    pickup_modes: tuple[str, ...]
    order_station: tuple[int, ...]
    product_assignments: tuple[int, ...]
    order_priority: tuple[int, ...]


class TwoForkScheduleSimulation(DEVS):
    """Evaluate grouped pickups with the precomputed crane kinematic timings."""

    def __init__(
        self,
        instance: ASRSInstance,
        schedule: TwoForkSchedule,
        timing_profile: TimingProfile | None = None,
    ):
        super().__init__("TwoForkScheduleSimulation")
        self.instance = instance
        self.schedule = schedule
        self.timing_profile = timing_profile or TimingProfile.load()
        self.batch_orders = self._batch_order_ids()
        self.batch_durations = self._batch_durations()
        if self.batch_durations:
            self.batch_durations = (self.batch_durations[0] + STARTUP_DELAY,) + self.batch_durations[1:]
        self.completion_marks = self._completion_marks()
        self.set_initial_state()

    def _batch_order_ids(self):
        orders = []
        offset = 0
        for group in self.schedule.pickup_groups:
            assigned = self.schedule.product_assignments[offset : offset + len(group)]
            if not assigned or len(set(assigned)) != 1:
                raise ValueError("A dual-fork pickup group must belong to one order.")
            orders.append(assigned[0])
            offset += len(group)
        if offset != len(self.schedule.product_order):
            raise ValueError("Pickup groups do not cover the product route exactly.")
        return tuple(orders)

    def _pickup_position(self, group):
        products = [self.instance.products[index] for index in group]
        if len(products) == 1:
            x = products[0].position[0] - FORK_SPACING / 2.0
            return x, products[0].position[1]
        return (
            sum(product.position[0] for product in products) / 2.0,
            sum(product.position[1] for product in products) / 2.0,
        )

    def _fork2_pickup_position(self, product_index):
        product = self.instance.products[product_index]
        return product.position[0] + FORK_SPACING / 2.0, product.position[1]

    def _group_duration(self, start_station, group, mode, destination_station):
        if mode != "sequential":
            pickup = self._pickup_position(group)
            return (
                self.timing_profile.station_to_pickup(start_station, pickup)
                + self.timing_profile.approach_time
                + self.timing_profile.fork_lift_to_down(pickup, destination_station)
            )

        first_pose = self._pickup_position((group[0],))
        second_pose = self._fork2_pickup_position(group[1])
        return (
            self.timing_profile.station_to_pickup(start_station, first_pose)
            + self.timing_profile.approach_time
            + self.timing_profile.data["fork_lift_withdraw_time"]
            + self.timing_profile.between_pickups(first_pose, second_pose)
            + self.timing_profile.approach_time
            + self.timing_profile.data["fork_lift_withdraw_time"]
            + self.timing_profile.pickup_to_station(second_pose, destination_station)
            + self.timing_profile.data["unload_service_time"]
        )

    def _batch_durations(self):
        current_station = next(
            (index for index, station in enumerate(self.instance.station_positions)
             if station == self.instance.initial_position),
            0,
        )
        durations = []
        for group, mode, order_id in zip(
            self.schedule.pickup_groups,
            self.schedule.pickup_modes,
            self.batch_orders,
        ):
            station_index = self.schedule.order_station[order_id]
            durations.append(self._group_duration(current_station, group, mode, station_index))
            current_station = station_index
        return tuple(durations)

    def _completion_marks(self):
        last_batch = {
            order_id: index for index, order_id in enumerate(self.batch_orders)
        }
        return tuple(
            order_id if last_batch.get(order_id) == index else None
            for index, order_id in enumerate(self.batch_orders)
        )

    def set_initial_state(self):
        self.task_index = 0
        self.elapsed = 0.0
        self.order_completion_times = [None] * len(self.instance.order_products)

    def tau(self):
        return self.batch_durations[self.task_index] if self.task_index < len(self.batch_durations) else inf

    def delta_x(self, event):
        return False

    def delta_y(self):
        self.elapsed = self.tNext
        completed = self.completion_marks[self.task_index]
        if completed is not None:
            self.order_completion_times[completed] = self.elapsed
        self.task_index += 1
        return None

    def get_state(self):
        return self.task_index, self.elapsed, tuple(self.order_completion_times)

    def set_state(self, state):
        self.task_index, self.elapsed, completion_times = state
        self.order_completion_times = list(completion_times)


def evaluate_twofork_schedule(
    instance: ASRSInstance,
    schedule: TwoForkSchedule,
    timing_profile: TimingProfile | None = None,
) -> ScheduleResult:
    """Run one grouped schedule through a SOCS DEVS atomic evaluator."""
    model = TwoForkScheduleSimulation(instance, schedule, timing_profile)
    engine = TESEnginBase(
        model,
        TimeIntervalContinuous=inf,
        TimeIntervalAnimation=inf,
        TimeSimulationEnd=1e9,
    )
    engine.bVerbose = False
    while model.task_index < len(model.batch_durations):
        result = engine.execute_one_step()
        if result is ExecutionResult.TimeBoundReached:
            raise RuntimeError("Two-fork schedule exceeded the evaluator time bound.")

    completion_times = tuple(
        None if value is None else float(value)
        for value in model.order_completion_times
    )
    shipped_counts = [0] * len(instance.order_products)
    for order_id in schedule.product_assignments:
        shipped_counts[order_id] += 1
    shipped = tuple(shipped_counts)
    requested = tuple(
        sum(demand) for demand in instance.order_demands
    )
    shortage = sum(
        max(0, requested[order_id] - shipped[order_id])
        for order_id in range(len(requested))
    )
    complete = sum(
        shipped[order_id] >= requested[order_id]
        for order_id in range(len(requested))
    )
    return ScheduleResult(model.elapsed, completion_times, shipped, requested, shortage, complete)


class TwoForkAntColonyOptimizer(AntColonyOptimizer):
    """Reuse the same ACO pheromones and score, adding feasible paired picks."""

    def __init__(
        self,
        *args,
        timing_profile=None,
        station_policy="first-pickup",
        picking_mode="double",
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        if picking_mode not in {"single", "double", "sequential", "all"}:
            raise ValueError("picking_mode must be 'single', 'double', 'sequential', or 'all'.")
        if station_policy not in {"first-pickup", "whole-order"}:
            raise ValueError("station_policy must be 'first-pickup' or 'whole-order'.")
        self.station_policy = station_policy
        self.picking_mode = picking_mode
        self.timing_profile = (
            timing_profile
            if isinstance(timing_profile, TimingProfile)
            else TimingProfile.load(timing_profile or DEFAULT_TIMING_PROFILE)
        )
        max_leg = max(
            value
            for station in self.timing_profile.data["travel_from_station"].values()
            for value in station.values()
        )
        max_cycle = max(
            value
            for pose in self.timing_profile.data["fork_lift_to_down_cycle"].values()
            for value in pose.values()
        )
        max_between = max(
            value
            for source in self.timing_profile.data["travel_between_pickups"].values()
            for value in source.values()
        )
        max_sequential = (
            2 * max_leg
            + 2 * self.timing_profile.approach_time
            + 2 * self.timing_profile.data["fork_lift_withdraw_time"]
            + max_between
            + self.timing_profile.data["unload_service_time"]
        )
        max_batch_time = max(
            max_leg + self.timing_profile.approach_time + max_cycle,
            max_sequential,
        )
        self.max_makespan = len(self.instance.products) * max_batch_time + STARTUP_DELAY
        requested = sum(sum(demand) for demand in self.instance.order_demands)
        self.completion_priority = (
            self.penalty_weight * requested + self.max_makespan + 1.0
        )

    def _pickup_position(self, group):
        selected = [self.instance.products[index] for index in group]
        if len(selected) == 1:
            return selected[0].position[0] - FORK_SPACING / 2.0, selected[0].position[1]
        return (
            sum(product.position[0] for product in selected) / 2.0,
            sum(product.position[1] for product in selected) / 2.0,
        )

    def _fork2_pickup_position(self, product_index):
        product = self.instance.products[product_index]
        return product.position[0] + FORK_SPACING / 2.0, product.position[1]

    def _group_duration(self, start_station, group, mode, destination_station):
        if mode != "sequential":
            pickup = self._pickup_position(group)
            return (
                self.timing_profile.station_to_pickup(start_station, pickup)
                + self.timing_profile.approach_time
                + self.timing_profile.fork_lift_to_down(pickup, destination_station)
            )
        first_pose = self._pickup_position((group[0],))
        second_pose = self._fork2_pickup_position(group[1])
        return (
            self.timing_profile.station_to_pickup(start_station, first_pose)
            + self.timing_profile.approach_time
            + self.timing_profile.data["fork_lift_withdraw_time"]
            + self.timing_profile.between_pickups(first_pose, second_pose)
            + self.timing_profile.approach_time
            + self.timing_profile.data["fork_lift_withdraw_time"]
            + self.timing_profile.pickup_to_station(second_pose, destination_station)
            + self.timing_profile.data["unload_service_time"]
        )

    def _order_duration(self, start_station, pickup_groups, pickup_modes, destination_station):
        """Estimate a complete order route when it uses one unloading station."""
        current_station = start_station
        duration = 0.0
        for group, mode in zip(pickup_groups, pickup_modes):
            duration += self._group_duration(current_station, group, mode, destination_station)
            current_station = destination_station
        return duration

    def _construct(self) -> TwoForkSchedule:
        products = self.instance.products
        color_index = {color: index for index, color in enumerate(COLORS)}
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
                    demand[color_index[color]] <= remaining_counts[color]
                    for color in COLORS
                )
                heuristic = (2.0 if fully_feasible else 1.0) / max(sum(demand), 1)
                pheromone = self.order_pheromone[previous_order][order_id]
                weights.append((pheromone**self.alpha) * (heuristic**self.beta))
            selected = self.rng.choices(candidates, weights=weights, k=1)[0]
            order_priority.append(selected)
            remaining_orders.remove(selected)
            for color in COLORS:
                color_demand = self.instance.order_demands[selected][color_index[color]]
                remaining_counts[color] = max(0, remaining_counts[color] - color_demand)
            previous_order = selected

        route = []
        assignments = []
        pickup_groups = []
        pickup_modes = []
        station_by_order: list[int | None] = [None] * len(self.instance.order_products)
        current_station = next(
            (index for index, station in enumerate(self.instance.station_positions)
             if station == self.instance.initial_position),
            0,
        )
        previous_product = len(products)

        for order_id in order_priority:
            demand_remaining = list(self.instance.order_demands[order_id])
            order_groups = []
            order_modes = []
            order_start_station = current_station
            while any(demand_remaining):
                candidates = [
                    product_index
                    for color in COLORS
                    for product_index in available[color]
                    if demand_remaining[color_index[color]] > 0
                ]
                if not candidates:
                    break

                actions = []
                weights = []
                if self.station_policy == "first-pickup":
                    possible_stations = (
                        (0, 1)
                        if station_by_order[order_id] is None
                        else (station_by_order[order_id],)
                    )
                else:
                    possible_stations = (None,)
                single_groups = [((index,), "single") for index in candidates]
                simultaneous_groups = []
                if self.picking_mode in {"double", "all"}:
                    for first, second in combinations(candidates, 2):
                        product_a = products[first]
                        product_b = products[second]
                        if abs(product_a.bay - product_b.bay) != 1 or product_a.level != product_b.level:
                            continue
                        pair_demand = [0] * len(COLORS)
                        pair_demand[color_index[product_a.color]] += 1
                        pair_demand[color_index[product_b.color]] += 1
                        if any(pair_demand[i] > demand_remaining[i] for i in range(len(COLORS))):
                            continue
                        simultaneous_groups.append((
                            tuple(sorted((first, second), key=lambda i: products[i].bay)),
                            "simultaneous",
                        ))
                sequential_groups = []
                if self.picking_mode in {"sequential", "all"}:
                    for first, second in combinations(candidates, 2):
                        pair_demand = [0] * len(COLORS)
                        pair_demand[color_index[products[first].color]] += 1
                        pair_demand[color_index[products[second].color]] += 1
                        if any(
                            pair_demand[index] > demand_remaining[index]
                            for index in range(len(COLORS))
                        ):
                            continue
                        sequential_groups.append(((first, second), "sequential"))
                        sequential_groups.append(((second, first), "sequential"))

                if self.picking_mode == "sequential":
                    groups = sequential_groups or single_groups
                elif self.picking_mode == "all":
                    groups = simultaneous_groups or sequential_groups or single_groups
                elif self.picking_mode == "double":
                    groups = simultaneous_groups or single_groups
                else:
                    groups = single_groups

                for group, mode in groups:
                    pickup = self._pickup_position(group if mode != "sequential" else (group[0],))
                    pheromone = self.task_pheromone[previous_product][group[0]]
                    if len(group) == 2:
                        pheromone = (
                            pheromone * self.task_pheromone[group[0]][group[1]]
                        ) ** 0.5
                    for station_index in possible_stations:
                        if station_index is None:
                            candidate_prefix = (*order_groups, group)
                            duration = min(
                                self._order_duration(
                                    order_start_station,
                                    candidate_prefix,
                                    (*order_modes, mode),
                                    candidate_station,
                                )
                                for candidate_station in (0, 1)
                            )
                        else:
                            duration = self._group_duration(
                                current_station, group, mode, station_index
                            )
                        if not pickup_groups:
                            duration += STARTUP_DELAY
                        if station_index is not None and station_by_order[order_id] is None:
                            pheromone_value = pheromone * self.station_pheromone[order_id][station_index]
                        else:
                            pheromone_value = pheromone
                        productivity = len(group) / max(duration, 1e-9)
                        actions.append((group, mode, station_index))
                        weights.append(
                            (pheromone_value**self.alpha)
                            * (productivity**self.beta)
                        )

                group, mode, station_index = self.rng.choices(actions, weights=weights, k=1)[0]
                if station_index is not None and station_by_order[order_id] is None:
                    station_by_order[order_id] = station_index
                order_groups.append(group)
                order_modes.append(mode)
                pickup_groups.append(group)
                pickup_modes.append(mode)
                route.extend(group)
                assignments.extend([order_id] * len(group))
                for product_index in group:
                    product = products[product_index]
                    available[product.color].remove(product_index)
                    demand_remaining[color_index[product.color]] -= 1
                if self.station_policy == "first-pickup":
                    current_station = station_by_order[order_id]
                previous_product = group[-1]

            if self.station_policy == "whole-order" and order_groups:
                station_weights = []
                for station_index in (0, 1):
                    duration = self._order_duration(
                        order_start_station, order_groups, order_modes, station_index
                    )
                    pheromone = self.station_pheromone[order_id][station_index]
                    station_weights.append(
                        (pheromone**self.alpha)
                        * ((1.0 / max(duration, 1e-9)) ** self.beta)
                    )
                station_by_order[order_id] = self.rng.choices(
                    (0, 1), weights=station_weights, k=1
                )[0]
                current_station = station_by_order[order_id]
            elif not order_groups:
                station_by_order[order_id] = current_station

        return TwoForkSchedule(
            tuple(route),
            tuple(pickup_groups),
            tuple(pickup_modes),
            tuple(0 if station is None else int(station) for station in station_by_order),
            tuple(assignments),
            tuple(order_priority),
        )

    def _pickup_position(self, group):
        products = [self.instance.products[index] for index in group]
        if len(products) == 1:
            return products[0].position[0] - FORK_SPACING / 2.0, products[0].position[1]
        return (
            sum(product.position[0] for product in products) / 2.0,
            sum(product.position[1] for product in products) / 2.0,
        )

    def _score(self, schedule: TwoForkSchedule):
        result = evaluate_twofork_schedule(self.instance, schedule, self.timing_profile)
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


def build_retrieval_plan(instance: ASRSInstance, schedule: TwoForkSchedule):
    """Convert ACO pickup groups to controller commands for Fork1 and Fork2."""
    plan = []
    route_offset = 0
    for group, mode in zip(schedule.pickup_groups, schedule.pickup_modes):
        order_id = schedule.product_assignments[route_offset]
        items = []
        if mode == "simultaneous":
            left_to_right = sorted(group, key=lambda index: instance.products[index].bay)
            fork_by_product = {left_to_right[0]: "Fork2", left_to_right[1]: "Fork1"}
        elif mode == "sequential":
            fork_by_product = {group[0]: "Fork1", group[1]: "Fork2"}
        else:
            fork_by_product = {group[0]: "Fork1"}
        for product_index in group:
            product = instance.products[product_index]
            if schedule.product_assignments[route_offset] != order_id:
                raise ValueError("One pickup group cannot contain products for different orders.")
            items.append(
                {
                    "cell": f"Cell-B{product.bay + 1:02d}-L{product.level + 1}",
                    "product_id": product.product_id,
                    "color": product.color,
                    "order_id": order_id + 1,
                    "fork": fork_by_product[product_index],
                }
            )
            route_offset += 1
        plan.append(
            {
                "items": items,
                "order_id": order_id + 1,
                "station": schedule.order_station[order_id],
                "mode": mode,
            }
        )
    return plan


def report(instance, optimizer, schedule, result, history):
    simultaneous_count = sum(mode == "simultaneous" for mode in schedule.pickup_modes)
    sequential_count = sum(mode == "sequential" for mode in schedule.pickup_modes)
    single_count = sum(mode == "single" for mode in schedule.pickup_modes)
    print("\nTwo-fork ACO result")
    print(f"  score: {optimizer.best_score:.3f}")
    print(f"  complete orders: {result.complete_order_count}/{len(result.requested_by_order)}")
    print(
        f"  unshipped items: {result.shortage_units}; weighted penalty: "
        f"{optimizer.penalty_weight * result.shortage_units:.3f}"
    )
    print(f"  makespan: {result.makespan:.3f} simulation time units")
    print(f"  mean order departure: {result.mean_order_completion_time:.3f}")
    print(
        f"  retrieval trips: {len(schedule.pickup_groups)} "
        f"({single_count} single, {simultaneous_count} simultaneous, "
        f"{sequential_count} move-then-pick) for {len(schedule.product_order)} products"
    )
    print(f"  completion priority coefficient: {optimizer.completion_priority:.3f}")
    print(f"  unloading-station policy: {optimizer.station_policy}")
    print(f"  picking mode: {optimizer.picking_mode}")
    print(f"  order priority: {[order_id + 1 for order_id in schedule.order_priority]}")
    print(f"  station by order (L=0, R=1): {list(schedule.order_station)}")
    print(f"  best score by iteration: {[round(value, 3) for value in history]}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True, help="saved JSON problem instance")
    parser.add_argument("--seed", type=int, default=42, help="ACO search random seed")
    parser.add_argument("--ants", type=int, default=12)
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument("--time-objective", choices=("makespan", "mean"), default="makespan")
    parser.add_argument(
        "--penalty-weight",
        type=float,
        default=50000.0,
        help="lambda cost per missing color-specific product",
    )
    parser.add_argument(
        "--timing-profile",
        default=str(DEFAULT_TIMING_PROFILE),
        help="DEVS-calibrated movement and fork-cycle time JSON",
    )
    parser.add_argument(
        "--station-policy",
        choices=("first-pickup", "whole-order"),
        default="first-pickup",
        help="choose L/R on the first pickup or after evaluating the whole order route",
    )
    parser.add_argument(
        "--picking-mode",
        choices=("single", "double", "sequential", "all"),
        default="double",
        help="single: one pick; double: force simultaneous pairs when feasible, else single; sequential: force sequential pairs when feasible; all: strict simultaneous > sequential > single priority",
    )
    args = parser.parse_args(argv)
    instance = load_instance(args.instance)
    print("AS/RS two-fork ACO + SOCS experiment")
    print(f"  instance: {Path(args.instance).resolve()}")
    print(f"  ACO seed: {args.seed}; ants: {args.ants}; iterations: {args.iterations}")
    print(f"  objective: complete orders > lambda shortage > {args.time_objective}")
    print(f"  lambda: {args.penalty_weight} simulation time units per missing product")
    print("  pairing rules: adjacent simultaneous or same-order sequential Fork1 then Fork2")
    print(f"  unloading-station policy: {args.station_policy}")
    print(f"  picking mode: {args.picking_mode}")
    print(f"  timing profile: {Path(args.timing_profile).resolve()}")
    optimizer = TwoForkAntColonyOptimizer(
        instance,
        args.time_objective,
        args.seed,
        ants=args.ants,
        iterations=args.iterations,
        penalty_weight=args.penalty_weight,
        timing_profile=TimingProfile.load(args.timing_profile),
        station_policy=args.station_policy,
        picking_mode=args.picking_mode,
    )
    schedule, result, history = optimizer.run()
    report(instance, optimizer, schedule, result, history)


if __name__ == "__main__":
    main()
