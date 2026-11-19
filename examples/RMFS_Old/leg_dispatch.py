"""Compare next-leg routing policies on a fixed FIFO order workload."""

from argparse import ArgumentParser
from collections import Counter, deque
from dataclasses import dataclass
from functools import cache
from math import dist, inf, isfinite
from itertools import islice, pairwise
from random import Random

from socs.devs.DEVS import DEVS
from socs.devs.TESEngin import TESEnginBase
from socs.devs.TESBase import PortValue

from .dynamic_dispatch import EPS, STRATEGIES, tied_routes
from .fleet import CELL, LIFT, LOWER, PICK, Reservations, parking_fleet, swept_cells
from .motion import EMPTY, LOADED, build_actions, final_heading, heading_between
from .traffic_layout import RACK_POSITIONS, STATION_APPROACHES, shortest_route

STAGES = ("rack", "station", "return", "home")


@dataclass(frozen=True)
class Leg:
    route: tuple
    start: float
    finish: float
    heading: float
    occupancy: tuple


class LegEvents(DEVS):
    def __init__(self, homes):
        super().__init__("LegEvents")
        self.homes = homes
        self.add_input_ports("depart")

    def set_initial_state(self):
        self.active = {}
        self.points = list(self.homes)
        self.headings = [-90] * len(self.homes)

    def get_state(self):
        return self.active, self.points, self.headings

    def set_state(self, state):
        self.active, self.points, self.headings = state

    def tau(self):
        return max(0.0, min((leg.finish for leg in self.active.values()), default=inf)
                   - self.tNow)

    def delta_x(self, event):
        index, leg = event.value
        self.active[index] = leg
        return True

    def delta_y(self):
        for index, leg in tuple(self.active.items()):
            if leg.finish <= self.tNow + EPS:
                self.points[index] = leg.route[-1]
                self.headings[index] = leg.heading
                del self.active[index]


def dispatch_leg(engine, model, index, leg):
    engine.XBuffer.append((model, PortValue("depart", (index, leg))))
    engine.execute_one_step()


def cell_windows(actions, goal, dwell, loaded):
    """Reserve each swept cell from entry to rear clearance without time ticks."""
    elapsed = 0.0
    spans = {}
    radius = .75 if loaded else .82
    for action in actions:
        for cell in swept_cells(action.start, action.end, radius):
            first, last = 0.0, 1.0
            for axis, (start, end) in enumerate(zip(action.start, action.end)):
                if start == end:
                    continue
                low = (cell[axis] * CELL - radius - start) / (end - start)
                high = ((cell[axis] + 1) * CELL + radius - start) / (end - start)
                first = max(first, min(low, high))
                last = min(last, max(low, high))
            if first > last:
                continue
            straight = action.straight
            begin, end = (0.0, action.duration) if straight is None else (
                straight.time_at_distance(first * straight.length),
                straight.time_at_distance(last * straight.length))
            windows = spans.setdefault(cell, [])
            begin, end = elapsed + begin, elapsed + end
            if windows and windows[-1][1] >= begin - EPS:
                windows[-1] = windows[-1][0], end
            else:
                windows.append((begin, end))
        elapsed += action.duration
    if dwell:
        for cell in swept_cells(goal, goal, .75):
            spans.setdefault(cell, []).append((elapsed, elapsed + dwell))
    return elapsed + dwell, tuple((cell, a, b) for cell, windows in spans.items()
                                   for a, b in windows)


def choose_leg(engine, model, index, stage, goal, strategy, k, limit, routes_override=None):
    now = engine.tNow
    origin = model.points[index]
    heading = model.headings[index]
    active_nodes, active_edges = set(), set()
    reservations = Reservations()
    parked = []
    for other, point in enumerate(model.points):
        if other == index:
            continue
        leg = model.active.get(other)
        if leg is not None:
            reservations.reserve(leg.start, leg.occupancy)
            reservations.reserve(now, [(cell, 0, leg.start - now)
                                        for cell in swept_cells(point, point, .82)]
                                 if leg.start > now else [])
            point, arrival = leg.route[-1], leg.finish
            active_nodes.update(leg.route)
            active_edges.update(frozenset(edge) for edge in pairwise(leg.route))
        else:
            arrival = now
            if routes_override is not None or stage == "lift":
                parked.append(point)
                continue
        reservations.reserve(arrival, [(cell, 0, inf)
                                       for cell in swept_cells(point, point, .82)])
    routes = (route_candidates(origin, goal, strategy, k, limit, active_nodes, active_edges)
              if routes_override is None else routes_override)
    lookahead = strategy in {"A*k-L", "A*k-LD"}
    snapshot = engine.capture_snapshot() if lookahead else None
    loaded = stage in ("station", "return", "transit_loaded")
    dwell = {"lift": LIFT, "station": PICK, "return": LOWER}.get(stage, 0)
    best = None
    for route in routes:
        if any(blocks_route(route, point) for point in parked):
            continue
        actions = build_actions(route, heading, LOADED if loaded else EMPTY)
        duration, occupancy = cell_windows(actions, goal, dwell, loaded)
        terminal = [(cell, duration, inf) for cell in swept_cells(goal, goal, .82)]
        start = reservations.earliest_start(now, (*occupancy, *terminal))
        if not isfinite(start):
            continue
        if start > now + EPS:
            wait = [(cell, 0.0, start - now) for cell in swept_cells(origin, origin, .82)]
            if reservations.earliest_start(now, wait) > now + EPS:
                continue
        leg = Leg(route, start, start + duration, final_heading(route, heading), tuple(occupancy))
        if not lookahead:
            return leg
        try:
            dispatch_leg(engine, model, index, leg)
            while index in model.active:
                engine.execute_one_step()
            finish = engine.tNow
        finally:
            engine.restore_snapshot(snapshot)
        if best is None or finish < best.finish - EPS:
            best = leg
    return best


def route_candidates(origin, goal, strategy, k, limit, active_nodes, active_edges):
    direct = shortest_route(origin, goal)
    routes = [direct]
    if strategy != "A*":
        routes.extend(route for route in islice(tied_routes(origin, goal, k), limit)
                      if route != direct)
    if strategy == "A*k-LD":
        edges = sorted(pairwise(direct), key=lambda edge: -dist(*edge))
        for edge in edges[:3]:
            path = next(tied_routes(origin, goal, k, blocked_edges=(frozenset(edge),)), None)
            if path and path not in routes:
                routes.append(path)
    def conflicts(route):
        return len(set(route) & active_nodes) + sum(
            frozenset(edge) in active_edges for edge in pairwise(route)
        )
    routes.sort(key=conflicts)
    return routes[:limit]


@cache
def parked_cells(point):
    return frozenset(swept_cells(point, point, .82))


def blocks_route(route, point):
    """Check stationary clearance before rounding positions into grid cells."""
    clearance = 2 * .82
    x, y = point
    return any(min(a[0], b[0]) - clearance <= x <= max(a[0], b[0]) + clearance
               and min(a[1], b[1]) - clearance <= y <= max(a[1], b[1]) + clearance
               for a, b in pairwise(route if len(route) > 1 else route * 2))


@cache
def reserved_cells(route):
    return frozenset(cell for a, b in pairwise(route)
                     for cell in swept_cells(a, b, .82))


def next_block(route, others):
    """Reserve through opposing traffic and up to a stopping point outside it."""
    reverse = {edge[::-1] for other in others for edge in pairwise(other)}
    edges = tuple(pairwise(route))
    if not edges or not any(edge in reverse for edge in edges[:2]):
        return route[:2]
    end = 1
    while end < len(edges) and edges[end] in reverse:
        end += 1
    conflict = reserved_cells(route[:end + 1])
    while end < len(edges) and parked_cells(route[end]) & conflict:
        end += 1
    return route[:end + 1]


def departure_blocks(route, others):
    """Try longer continuous movement first, without splitting a conflict block."""
    minimum = next_block(route, others)
    ends = [len(minimum)]
    while ends[-1] < len(route):
        end = ends[-1]
        if end < 2 or heading_between(*route[end - 2:end]) != heading_between(*route[end - 1:end + 1]):
            break
        ends.append(end + len(next_block(route[end - 1:], others)) - 1)
    return (route[:end] for end in reversed(ends))


def completion_order_exists(routes, checkpoints, limit=256):
    """Find a reachable stage event, where future routes can be reassessed."""
    budget = limit

    def moves(remaining):
        for j, route in enumerate(remaining):
            if len(route) == 1:
                continue
            for prefix in departure_blocks(route, remaining[:j] + remaining[j + 1:]):
                blockers = tuple(k for k, other in enumerate(remaining)
                                 if k != j and blocks_route(prefix, other[0]))
                yield j, prefix, blockers

    dependencies = {j: blockers for j, _, blockers in moves(routes)}
    def has_cycle(j, ancestors):
        return j in ancestors or any(has_cycle(k, ancestors | {j})
                                     for k in dependencies.get(j, ()))

    if any(has_cycle(j, set()) for j in dependencies):
        return False

    @cache
    def search(indices):
        nonlocal budget
        if any(indices[j] == len(routes[j]) - 1 for j in checkpoints):
            return True
        if all(i == len(route) - 1 for i, route in zip(indices, routes)):
            return True
        if budget == 0:
            return False
        budget -= 1
        remaining = tuple(route[i:] for i, route in zip(indices, routes))
        for j, prefix, blockers in moves(remaining):
            if blockers:
                continue
            successor = indices[:j] + (indices[j] + len(prefix) - 1,) + indices[j + 1:]
            if search(successor):
                return True
        return False

    return search((0,) * len(routes))


def choose_waypoint(engine, model, index, stage, goal, strategy, k, limit,
                    routes, pending, jobs, homes):
    projected = []
    for j, point in enumerate(model.points):
        if j in model.active:
            point = model.active[j].route[-1]
        task = pending.get(j, "")
        if j == index:
            projected.append((point,))
        elif j in routes and len(routes[j]) > 1:
            projected.append(routes[j])
        elif task in {"next_rack", "next_station", "next_return", "next_home"}:
            if task == "next_home":
                target = homes[j]
            else:
                rack, station = jobs[j]
                target = STATION_APPROACHES[station] if task == "next_station" else RACK_POSITIONS[rack]
            projected.append(shortest_route(point, target))
        else:
            projected.append((point,))
    others = tuple(projected[:index] + projected[index + 1:])
    nodes = {point for route in others if len(route) > 1 for point in route}
    edges = {frozenset(edge) for route in others for edge in pairwise(route)}
    candidates = ([routes[index]] if index in routes else
                  route_candidates(model.points[index], goal, strategy, k, limit, nodes, edges))
    best = None
    remainder = None
    for route in candidates:
        for prefix in departure_blocks(route, others):
            final = len(prefix) == len(route)
            motion_stage = stage if final else (
                "transit_loaded" if stage in {"station", "return"} else "rack")
            leg = choose_leg(engine, model, index, motion_stage, prefix[-1], strategy,
                             k, limit, (prefix,))
            if leg is None or leg.start > engine.tNow + EPS:
                continue
            tail = route[len(prefix) - 1:]
            projected[index] = tail
            checkpoints = tuple(j for j, candidate in enumerate(projected) if len(candidate) > 1)
            if not completion_order_exists(tuple(projected), checkpoints):
                continue
            if best is None or leg.finish < best.finish - EPS:
                best, remainder = leg, tail
            break
        if best is not None and strategy in {"A*", "A*k-X"}:
            break
    if best is not None:
        routes[index] = remainder
    return best


def admission_blockers(points, robot, rack, station):
    """Identify idle footprints that would block the incoming order."""
    rack_point = RACK_POSITIONS[rack]
    station_point = STATION_APPROACHES[station]
    footprint = set()
    for start, goal in ((points[robot], rack_point), (rack_point, station_point),
                        (station_point, rack_point)):
        footprint.update(reserved_cells(shortest_route(start, goal)))
        footprint.update(parked_cells(goal))
    return tuple(index for index, point in enumerate(points)
                 if index != robot and footprint & parked_cells(point))


def experiment(strategy, count=100, robot_count=10, seed=42, k=2, limit=8,
               recovery_limit=0, waypoint_control=False, serial_admission=False, trace=None,
               snapshot_at=None):
    if serial_admission:
        if strategy != "A*":
            raise ValueError("Serial admission is an A* control experiment")
        waypoint_control = True
    if waypoint_control and recovery_limit:
        raise ValueError("Waypoint control currently runs without rollback")
    homes = tuple(robot.position for robot in parking_fleet(robot_count))
    model = LegEvents(homes)
    engine = TESEnginBase(model)
    engine.bVerbose = False
    rack_preferences = tuple(Random(seed + order).sample(tuple(RACK_POSITIONS), len(RACK_POSITIONS))
                             for order in range(count))
    workstation_counts = Counter()
    workstations = []
    for _ in range(count):
        station = min(STATION_APPROACHES, key=lambda key: (workstation_counts[key], key))
        workstations.append(station)
        workstation_counts[station] += 1
    shipments = Counter()
    jobs = {}
    pending = {}
    routes = {}
    assigned = returned = 0
    history = deque(maxlen=32)
    recoveries = 0
    deferred = None

    def record(kind, index, *data):
        if trace is not None:
            trace.append((engine.tNow, kind, index, *data))

    while True:
        if snapshot_at is not None and engine.tNow >= snapshot_at:
            snapshot = engine.capture_snapshot()
            controller = (jobs.copy(), pending.copy(), routes.copy(), shipments.copy(),
                          assigned, returned)
            engine.restore_snapshot(snapshot)
            jobs, pending, routes, shipments, assigned, returned = controller
            record("snapshot", -1)
            snapshot_at = None
        for index, stage in tuple(pending.items()):
            if index in model.active or stage.startswith("next_"):
                continue
            if index in routes and len(routes[index]) > 1:
                pending[index] = "next_" + stage
                continue
            routes.pop(index, None)
            del pending[index]
            if stage == "rack":
                pending[index] = "next_lift"
            elif stage == "lift":
                record("lift", index, jobs[index][0])
                pending[index] = "next_station"
            elif stage == "station":
                shipments[index] += 1
                record("shipped", index)
                pending[index] = "next_return"
            elif stage == "return":
                returned += 1
                record("returned", index, jobs[index][0])
                del jobs[index]

        ready = sorted((i for i in range(robot_count) if i not in jobs and i not in pending),
                       key=lambda i: (shipments[i], i))
        for index in ready:
            if serial_admission and (jobs or pending):
                break
            if assigned == count:
                if model.points[index] != homes[index]:
                    pending[index] = "next_home"
                continue
            used = {rack for rack, _ in jobs.values()}
            rack = next(rack for rack in rack_preferences[assigned] if rack not in used)
            if serial_admission:
                blockers = admission_blockers(model.points, index, rack, workstations[assigned])
                if blockers:
                    # A direct blocker may itself need a stationary robot to
                    # move first, so give available clearing moves a second pass.
                    for other in (*blockers, *(i for i in ready if i not in blockers)):
                        if model.points[other] == homes[other]:
                            continue
                        move = choose_leg(engine, model, other, "home", homes[other],
                                          strategy, k, limit)
                        if move is not None:
                            pending[other] = "home"
                            record("move", other, "home", move, model.headings[other])
                            dispatch_leg(engine, model, other, move)
                            break
                    if not model.active:
                        return None, returned
                    break
            jobs[index] = rack, workstations[assigned]
            record("assigned", index)
            pending[index] = "next_rack"
            assigned += 1

        if not pending:
            return engine.tNow, returned

        for index, task in tuple(pending.items()):
            if not task.startswith("next_") or index == deferred:
                continue
            stage = task[5:]
            if stage == "home":
                goal = homes[index]
            else:
                rack, station = jobs[index]
                goal = STATION_APPROACHES[station] if stage == "station" else RACK_POSITIONS[rack]
            if stage == "lift":
                station_point = STATION_APPROACHES[station]
                station_cells = set(swept_cells(station_point, station_point, .82))
                occupied = False
                for other, (_, target_station) in jobs.items():
                    if other == index or target_station != station:
                        continue
                    task = pending[other]
                    active_leg = model.active.get(other)
                    if task in {"lift", "next_station", "station", "next_return"}:
                        occupied = True
                    elif task == "return" and active_leg is not None:
                        elapsed = engine.tNow - active_leg.start
                        occupied = elapsed < 0 or any(
                            cell in station_cells and begin <= elapsed < end
                            for cell, begin, end in active_leg.occupancy
                        )
                    if occupied:
                        break
                if occupied:
                    continue
            if not waypoint_control or stage == "lift":
                leg = choose_leg(engine, model, index, stage, goal, strategy, k, limit)
            else:
                leg = choose_waypoint(engine, model, index, stage, goal, strategy,
                                      k, limit, routes, pending, jobs, homes)
            if leg is not None:
                if recovery_limit and model.active:
                    history.append((engine.capture_snapshot(), pending.copy(), jobs.copy(),
                                    shipments.copy(), assigned, returned, index))
                pending[index] = stage
                record("move", index, stage, leg, model.headings[index])
                dispatch_leg(engine, model, index, leg)
        if not model.active:
            if not history or recoveries >= recovery_limit:
                record("stalled", -1)
                return None, returned
            checkpoint, pending, jobs, shipments, assigned, returned, deferred = history.pop()
            engine.restore_snapshot(checkpoint)
            recoveries += 1
            continue
        deferred = None
        engine.execute_one_step()


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--strategy", choices=STRATEGIES)
    parser.add_argument("--waypoint-control", action="store_true")
    parser.add_argument("--serial-admission", action="store_true",
                        help="run only A* with gated order admission and waypoint control")
    parser.add_argument("--recovery-limit", type=int, default=0,
                        help="maximum departure decisions to retry after deadlock")
    args = parser.parse_args()
    if args.serial_admission and (args.strategy not in (None, "A*") or args.recovery_limit):
        parser.error("--serial-admission requires A* and no rollback")
    strategies = ("A*",) if args.serial_admission else (
        (args.strategy,) if args.strategy else STRATEGIES)
    for strategy in strategies:
        makespan, completed = experiment(strategy, recovery_limit=args.recovery_limit,
                                         waypoint_control=args.waypoint_control,
                                         serial_admission=args.serial_admission)
        print(f"{strategy}: makespan={makespan}, returned={completed}/100", flush=True)


if __name__ == "__main__":
    main()
