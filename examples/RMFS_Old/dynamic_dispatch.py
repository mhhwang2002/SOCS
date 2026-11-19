"""Offline comparison of route choices at live RMFS dispatch events."""

from argparse import ArgumentParser
from collections import Counter
from dataclasses import replace
from heapq import heapify, heappop, heappush
from itertools import islice, pairwise
from math import dist, inf
from time import perf_counter

from socs.devs.DEVS import DEVS
from socs.devs.TESEngin import TESEnginBase
from socs.devs.TESBase import PortValue

from .fleet import LIFT, PICK, Reservations, ScheduledOrder, occupied_intervals, parking_fleet
from .fleet import remaining_order_routes, replace_routes
from .model import OrderAllocator
from .motion import MissionMotion
from .traffic_layout import ADJACENCY, RACK_POSITIONS, STATION_APPROACHES, shortest_route

EPS = 1e-8
STRATEGIES = ("A*", "A*k-X", "A*k-L", "A*k-LD")


def tied_routes(start, goal, k, *, blocked_edges=(), blocked_nodes=()):
    """Enumerate shortest paths from a DAG with at most k parents per node."""
    if start in blocked_nodes or goal in blocked_nodes:
        return
    costs = {start: 0.0}
    parents = {}
    queue = [(dist(start, goal), 0.0, start)]
    while queue and queue[0][0] <= costs.get(goal, inf) + EPS:
        _, cost, node = heappop(queue)
        if cost > costs[node] + EPS or node == goal:
            continue
        for neighbor in ADJACENCY[node]:
            if neighbor in blocked_nodes or frozenset((node, neighbor)) in blocked_edges:
                continue
            new = cost + dist(node, neighbor)
            old = costs.get(neighbor, inf)
            if new < old - EPS:
                costs[neighbor] = new
                parents[neighbor] = [node]
                heappush(queue, (new + dist(neighbor, goal), new, neighbor))
            elif abs(new - old) <= EPS and node not in parents[neighbor]:
                if len(parents[neighbor]) < k:
                    parents[neighbor].append(node)

    def paths(node):
        if node == start:
            yield (start,)
        for parent in parents.get(node, ()):
            for prefix in paths(parent):
                yield (*prefix, node)

    yield from paths(goal)


class Dispatch(DEVS):
    """Track committed mission trajectories and rack-return/home events."""

    def __init__(self, robots, staged=False):
        super().__init__("Dispatch")
        self.robots = robots
        self.staged = staged
        self.add_input_ports("commit")

    def set_initial_state(self):
        self.active = {}
        self.events = [(0.0, i, "home") for i in range(len(self.robots))]
        self.available = []
        self.home_arrivals = {}
        self.headings = {}
        self.returned = 0

    def get_state(self):
        return self.active, self.events, self.available, self.home_arrivals, self.headings, self.returned

    def set_state(self, state):
        self.active, self.events, self.available, self.home_arrivals, self.headings, self.returned = state

    def tau(self):
        return max(0.0, self.events[0][0] - self.tNow) if self.events else inf

    def delta_x(self, event):
        index, order, occupancy = event.value
        self.events[:] = [item for item in self.events if item[1] != index]
        heapify(self.events)
        self.active[index] = order, occupancy
        self.home_arrivals.pop(index, None)
        travel = order.motion.travel_times
        returned_at = order.finish - travel["ReturnToStandby"]
        if returned_at > self.tNow + EPS:
            heappush(self.events, (returned_at, index, "return"))
        if self.staged:
            pickup = order.start + travel["TravelToRack"] + LIFT
            shipped = pickup + travel["TravelToWorkstation"] + PICK
            for time, kind in ((pickup, "pickup"), (shipped, "shipped")):
                if time > self.tNow + EPS:
                    heappush(self.events, (time, index, kind))
        heappush(self.events, (order.finish, index, "home"))
        return True

    def delta_y(self):
        while self.events and self.events[0][0] <= self.tNow + EPS:
            time, index, kind = heappop(self.events)
            self.available.append((index, kind))
            if kind == "return":
                self.returned += 1
            elif kind == "home":
                mission = self.active.pop(index, None)
                if mission is not None:
                    self.headings[index] = mission[0].motion.end_heading
                self.home_arrivals[index] = time


def candidates(base, strategy, k, limit, only_phase=None):
    yield base
    if strategy == "A*":
        return
    for phase, route in base.routes.items():
        if only_phase is not None and phase != only_phase:
            continue
        seen = {route}
        tied = tied_routes(route[0], route[-1], k)
        for path in islice(tied, limit + 1):
            if path not in seen:
                seen.add(path)
                yield replace_routes(base, phase, path)
        if strategy == "A*k-LD":
            edges = sorted(pairwise(route), key=lambda edge: -dist(*edge))
            for edge in edges[:3]:
                path = next(tied_routes(route[0], route[-1], k,
                                        blocked_edges=(frozenset(edge),)), None)
                if path and path not in seen:
                    seen.add(path)
                    yield replace_routes(base, phase, path)


def shared_resources(motion, nodes, edges):
    return sum(len(set(route) & nodes) +
               len({frozenset(edge) for edge in pairwise(route)} & edges)
               for route in motion.routes.values())


def commit(engine, model, index, order, occupancy):
    engine.XBuffer.append((model, PortValue("commit", (index, order, occupancy))))
    engine.execute_one_step()


def experiment(strategy, *, k=2, limit=8, count=100, robot_count=10, seed=42,
               staged=False, first_leg_only=None, replay_branches=True):
    began = perf_counter()
    allocator = OrderAllocator(seed)
    assignments = [allocator.assign() for _ in range(count)]
    model = Dispatch(parking_fleet(robot_count), staged)
    engine = TESEnginBase(model)
    engine.bVerbose = False
    stats = Counter()
    if first_leg_only is None:
        first_leg_only = staged
    branch_times = []
    decision_times = []
    number = 0
    while model.events:
        engine.execute_one_step()
        available, model.available = model.available, []
        for index, kind in available:
            phase = {"pickup": "TravelToWorkstation", "shipped": "ReturnRack"}.get(kind)
            if phase is None and number == count:
                if not staged or kind != "return":
                    continue
                phase = "ReturnToStandby"
            decision_started = perf_counter()
            now = engine.tNow
            robot = model.robots[index]
            active = model.active.get(index)
            if phase is not None:
                previous = active[0]
                assignment, base = previous.assignment, previous.motion
                cutoff = max(0.0, now - previous.start)
            elif kind == "return":
                previous = active[0]
                origin = previous.motion.routes["ReturnRack"][-1]
                heading = previous.motion.start_headings["ReturnToStandby"]
            else:
                origin = robot.position
                heading = model.headings.get(index, -90)
            if phase is None:
                assignment = assignments[number]
                slot = RACK_POSITIONS[assignment.rack]
                station = STATION_APPROACHES[assignment.workstation]
                base = MissionMotion(slot, station, shortest_route(origin, slot),
                                     home=robot.position, heading=heading)
            reservations = Reservations()
            remaining = []
            for other, (order, occupancy) in model.active.items():
                if other != index:
                    reservations.reserve(order.start, occupancy)
                    remaining.extend(remaining_order_routes(order, now))
            nodes = {node for route in remaining for node in route}
            edges = {frozenset(edge) for route in remaining for edge in pairwise(route)}
            only_phase = phase or ("TravelToRack" if first_leg_only else None)
            motions = list(candidates(base, strategy, k, limit, only_phase))
            if strategy != "A*":
                motions[1:] = sorted(motions[1:], key=lambda m: shared_resources(m, nodes, edges))
            if strategy == "A*k-X":
                motions.sort(key=lambda m: shared_resources(m, nodes, edges))
            lookahead = strategy in {"A*k-L", "A*k-LD"}
            snapshot = engine.capture_snapshot() if lookahead and replay_branches else None
            best = None
            for motion in motions[:limit]:
                started = perf_counter()
                duration, occupancy = occupied_intervals(motion, assignment)
                if phase is None:
                    start = reservations.earliest_start(now, occupancy)
                    order = ScheduledOrder(number + 1, assignment, robot, motion, now, start,
                                           start + duration)
                    blocked = kind == "return" and start > now + EPS
                else:
                    suffix = [(cell, max(0.0, begin - cutoff), end - cutoff)
                              for cell, begin, end in occupancy if end > cutoff + EPS]
                    blocked = reservations.earliest_start(now, suffix) > now + EPS
                    order = replace(previous, motion=motion, finish=previous.start + duration)
                if blocked:
                    stats["blocked"] += 1
                    if lookahead:
                        branch_times.append(("blocked", perf_counter() - started))
                    continue
                finish = order.finish
                if snapshot is not None:
                    stats["simulated"] += 1
                    try:
                        commit(engine, model, index, order, occupancy)
                        while index not in model.home_arrivals:
                            engine.execute_one_step()
                        finish = model.home_arrivals[index]
                    finally:
                        engine.restore_snapshot(snapshot)
                    branch_times.append(("feasible", perf_counter() - started))
                stats["feasible"] += 1
                if best is None or finish < best[0] - EPS:
                    best = finish, order, occupancy
                if not lookahead:
                    break
            if best is None:
                stats["fallback"] += 1
            else:
                _, order, occupancy = best
                commit(engine, model, index, order, occupancy)
                if phase is None:
                    stats["return_dispatch" if kind == "return" else "home_dispatch"] += 1
                    number += 1
                else:
                    stats["stage_decision"] += 1
                    stats["stage_change"] += order.motion.routes != base.routes
                if order.motion.routes != base.routes:
                    stats["alternative"] += 1
                    stats["longer"] += any(
                        sum(dist(*edge) for edge in pairwise(route)) >
                        sum(dist(*edge) for edge in pairwise(base.routes[phase])) + EPS
                        for phase, route in order.motion.routes.items()
                    )
            decision_times.append(perf_counter() - decision_started)
    assert model.returned == count
    return engine.tNow, perf_counter() - began, stats, branch_times, decision_times


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--k", type=int, default=2)
    parser.add_argument("--ell", type=int, default=8)
    parser.add_argument("--orders", type=int, default=100)
    parser.add_argument("--robots", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--staged", action="store_true")
    parser.add_argument("--ablation", action="store_true",
                        help="isolate dispatch candidate scope and stage replanning")
    args = parser.parse_args()
    if min(args.k, args.ell, args.orders, args.robots) < 1:
        parser.error("counts must be positive")
    if args.ablation:
        for strategy in STRATEGIES[1:]:
            for first_leg_only, staged in ((False, False), (True, False),
                                          (False, True), (True, True)):
                result, _, stats, _, _ = experiment(
                    strategy, k=args.k, limit=args.ell, count=args.orders,
                    robot_count=args.robots, seed=args.seed, staged=staged,
                    first_leg_only=first_leg_only, replay_branches=False,
                )
                print(f"{strategy}: first_leg_only={first_leg_only} staged={staged} "
                      f"makespan={result:.2f}s return_dispatch={stats['return_dispatch']} "
                      f"stage_changes={stats['stage_change']}", flush=True)
        return
    for strategy in STRATEGIES:
        makespan, elapsed, stats, branches, decisions = experiment(
            strategy, k=args.k, limit=args.ell, count=args.orders,
            robot_count=args.robots, seed=args.seed, staged=args.staged
        )
        simulated = [seconds for status, seconds in branches if status == "feasible"]
        print(f"{strategy}: makespan={makespan:.2f}s runtime={elapsed:.3f}s "
              f"stats={dict(stats)} candidate_checks={len(branches)} "
              f"simulation_mean_ms={1000*sum(simulated)/max(1,len(simulated)):.2f} "
              f"simulation_max_ms={1000*max(simulated, default=0):.2f} "
              f"decision_max_ms={1000*max(decisions):.2f}", flush=True)


if __name__ == "__main__":
    main()
