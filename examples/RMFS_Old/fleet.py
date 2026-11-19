"""Plan fleet missions with space-time reservations and makespan search."""

from argparse import ArgumentParser
from bisect import bisect_right, insort
from dataclasses import dataclass, replace
from itertools import pairwise
from math import cos, dist, floor, hypot, radians, sin

from random import Random
from time import perf_counter

from socs.devs import CoupledDEVS
from socs.devs.TESBase import PortValue

from .aabb_tree import candidate_pairs, footprint_bounds

from .model import OrderAllocator, RMFS, RobotStatus, select_robot
from .motion import MissionMotion, final_heading
from .traffic_layout import PARKING, RACK_POSITIONS, STATION_APPROACHES, shortest_route

LIFT = 1.4
PICK = 3.0
LOWER = 1.4
CELL = .5
STEP = .5


@dataclass(frozen=True)
class ScheduledOrder:
    number: int
    assignment: object
    robot: RobotStatus
    motion: MissionMotion
    assigned: float
    start: float
    finish: float
    route_intersections: tuple = ()


def dynamic_route(motion, phase, elapsed):
    """Keep a segment until the entire vehicle has cleared its endpoint."""
    route = motion.routes[phase]
    if len(route) < 2:
        return route
    point, heading = motion.pose(phase, elapsed)
    carrying = phase in LOADED_PHASES
    x0, y0, x1, y1 = footprint_bounds((point, heading, carrying))
    progress = 0.0
    distance_before = 0.0
    for a, b in pairwise(route):
        length = dist(a, b)
        if not length:
            continue
        dx, dy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
        projection = (point[0] - a[0]) * dx + (point[1] - a[1]) * dy
        if -1e-8 <= projection <= length + 1e-8 and dist(
            point, (a[0] + projection * dx, a[1] + projection * dy)
        ) < 1e-8:
            progress = max(progress, distance_before + projection)
        distance_before += length
    end = 0.0
    margin = max((x1 - x0) / 2, (y1 - y0) / 2)
    for index, (a, b) in enumerate(pairwise(route)):
        end += dist(a, b)
        if end + margin >= progress:
            return route[index:]
    return route[-1:]


def remaining_order_routes(order, time):
    elapsed = time - order.start
    for phase, dwell in (("TravelToRack", LIFT), ("TravelToWorkstation", PICK),
                         ("ReturnRack", LOWER), ("ReturnToStandby", 0)):
        duration = order.motion.travel_times[phase]
        if elapsed <= duration:
            yield dynamic_route(order.motion, phase, max(0, elapsed))
        elapsed -= duration + dwell


def route_intersections(motion, orders, time, robot):
    """Locate shared points and collinear overlaps on the axis-aligned graph."""
    proposed = [edge for route in motion.routes.values() for edge in pairwise(route)]
    intersections = set()
    for order in orders:
        if order.robot == robot or time >= order.finish:
            continue
        for route in remaining_order_routes(order, time):
            for a, b in pairwise(route):
                for c, d in proposed:
                    low = tuple(max(min(u, v), min(w, z)) for u, v, w, z in zip(a, b, c, d))
                    high = tuple(min(max(u, v), max(w, z)) for u, v, w, z in zip(a, b, c, d))
                    if all(u <= v for u, v in zip(low, high)):
                        intersections.add((order.robot.name, low, high))
    return tuple(sorted(intersections))


def timeline(motion):
    """Yield local times and sampling functions for all mission stages."""
    for phase, dwell in (
        ("TravelToRack", LIFT), ("TravelToWorkstation", PICK),
        ("ReturnRack", LOWER), ("ReturnToStandby", 0),
    ):
        loaded = phase in {"TravelToWorkstation", "ReturnRack"}
        for action in motion.actions[phase]:
            yield action.duration, action.pose, loaded
        if dwell:
            point = motion.routes[phase][-1]
            yield dwell, lambda _, p=point: (p, 0), True


def swept_cells(start, end, radius):
    """Cover each swept rectangle, including the robot's rotational envelope."""
    low_x, high_x = sorted((start[0], end[0]))
    low_y, high_y = sorted((start[1], end[1]))
    for x in range(floor((low_x-radius)/CELL), floor((high_x+radius)/CELL)+1):
        for y in range(floor((low_y-radius)/CELL), floor((high_y+radius)/CELL)+1):
            yield x, y


def occupied_intervals(motion, assignment):
    now, intervals = occupancy_for_timeline(timeline(motion))
    return now, [*intervals, (("rack", assignment.rack), 0.0, now)]


def occupancy_for_timeline(stages):
    occupancy = {}
    now = 0.0
    for duration, pose, loaded in stages:
        elapsed = 0.0
        while elapsed < duration:
            end = min(duration, elapsed + STEP)
            point, _ = pose(elapsed)
            next_point, _ = pose(end)
            radius = .75 if loaded else .82
            for cell in swept_cells(point, next_point, radius):
                spans = occupancy.setdefault(cell, [])
                if spans and spans[-1][1] >= now+elapsed-1e-8:
                    spans[-1] = (spans[-1][0], now+end)
                else:
                    spans.append((now+elapsed, now+end))
            elapsed = end
        now += duration
    return now, [(cell, begin, end) for cell, spans in occupancy.items()
                 for begin, end in spans]


class Reservations:
    def __init__(self):
        self.cells = {}

    def earliest_start(self, earliest, intervals):
        start = earliest
        while True:
            retry = start
            for cell, begin, end in intervals:
                reservations = self.cells.get(cell)
                if not reservations:
                    continue
                index = bisect_right(reservations, (start+begin+1e-8, float("inf")))
                for conflict in (index-1, index):
                    if 0 <= conflict < len(reservations):
                        other_start, other_end = reservations[conflict]
                        if start+begin < other_end-1e-8 and other_start < start+end-1e-8:
                            retry = max(retry, other_end-begin)
            if retry <= start+1e-8:
                return start
            start = retry

    def reserve(self, start, intervals):
        for cell, begin, end in intervals:
            insort(self.cells.setdefault(cell, []), (start+begin, start+end))


def parking_fleet(robot_count):
    if not 1 <= robot_count <= len(PARKING):
        raise ValueError(f"robot_count must be between 1 and {len(PARKING)}")
    return tuple(RobotStatus(f"MiR600-{i:02}", point)
                 for i, point in enumerate(PARKING[:robot_count], 1))


def schedule_orders(count=100, seed=42, *, robot_count=10, trials=1):
    """Try reservation priority orders while keeping the same order assignments."""
    if trials < 1:
        raise ValueError("trials must be positive")
    planned, last = _schedule(count, seed, robot_count)
    chains = {}
    for order in planned:
        chains.setdefault(order.robot, []).append(order)

    prepared = {}
    for robot, chain in chains.items():
        missions = []
        for order in chain:
            motion = order.motion
            if order.number in last:
                motion = MissionMotion(
                    RACK_POSITIONS[order.assignment.rack],
                    STATION_APPROACHES[order.assignment.workstation],
                    motion.routes["TravelToRack"], home=robot.position,
                    heading=motion.start_headings["TravelToRack"],
                )
            duration, occupancy = occupied_intervals(motion, order.assignment)
            origin = motion.routes["TravelToRack"][0]
            missions.append((order, motion, duration, occupancy, tuple(swept_cells(origin, origin, .82))))

        prepared[robot] = missions

    priority = tuple(prepared)
    best = baseline = fit_chains(prepared, priority)
    if trials == 1 or len(priority) < 2:
        return baseline

    longest_first = tuple(sorted(priority, key=lambda robot: -sum(
        mission[2] for mission in prepared[robot]
    )))
    rng = Random(seed)
    seen = {priority}
    for attempt in range(1, trials):
        if attempt == 1:
            candidate = longest_first
        elif attempt == 2:
            candidate = priority[::-1]
        else:
            changed = list(priority if attempt % 2 else longest_first)
            a, b = rng.sample(range(len(changed)), 2)
            changed[a], changed[b] = changed[b], changed[a]
            candidate = tuple(changed)
        if candidate in seen:
            continue
        seen.add(candidate)
        result = fit_chains(prepared, candidate)
        if makespan(result) < makespan(best):
            best = result
            priority = candidate
    return best


def makespan(orders):
    return max(order.finish for order in orders)


def fit_chains(prepared, priority):
    reservations = Reservations()
    scheduled = []
    for robot in priority:
        missions = prepared[robot]
        # Fit orders individually. If a planned wait at a rack would overlap
        # another robot, defer the preceding arrival and fit the suffix again.
        lower_bounds = [0.0] * len(missions)
        timings = []
        index = 0
        while index < len(missions):
            _, _, duration, occupancy, waiting_cells = missions[index]
            ready = timings[-1][1] if timings else 0.0
            start = reservations.earliest_start(max(ready, lower_bounds[index]), occupancy)
            if index and start > ready + 1e-8:
                clear = reservations.earliest_start(
                    ready, [(cell, 0.0, start-ready) for cell in waiting_cells]
                )
                if clear > ready + 1e-8:
                    index -= 1
                    lower_bounds[index] = timings[-1][0] + clear - ready
                    timings.pop()
                    continue
            timings.append((start, start + duration))
            index += 1

        ready = 0.0
        for (order, motion, _, occupancy, waiting_cells), (start, finish) in zip(missions, timings):
            if start > ready + 1e-8:
                reservations.reserve(ready, [(cell, 0.0, start-ready) for cell in waiting_cells])
            reservations.reserve(start, occupancy)
            scheduled.append(replace(order, motion=motion, assigned=ready, start=start, finish=finish))
            ready = finish
    return sorted(scheduled, key=lambda order: order.number)


def _schedule(count, seed, robot_count):
    if count < 1:
        raise ValueError("count must be positive")
    allocator = OrderAllocator(seed)
    robots = parking_fleet(robot_count)
    ready = dict.fromkeys(robots, 0.0)
    positions = {robot: robot.position for robot in robots}
    headings = dict.fromkeys(robots, -90)
    last_order = {}
    reservations = Reservations()
    orders = []
    for number in range(1, count+1):
        assignment = allocator.assign()
        slot = RACK_POSITIONS[assignment.rack]
        station = STATION_APPROACHES[assignment.workstation]
        earliest = min(ready.values())
        available = [robot for robot in robots if ready[robot] <= earliest+1e-8]
        by_status = {RobotStatus(robot.name, positions[robot]): robot for robot in available}
        status, approach = select_robot(by_status, slot)
        robot = by_status[status]
        last_order[robot] = number
        motion = MissionMotion(
            slot, station, approach, home=robot.position,
            return_home=False, heading=headings[robot],
        )
        duration, intervals = occupied_intervals(motion, assignment)
        start = reservations.earliest_start(earliest, intervals)
        reservations.reserve(start, intervals)
        finish = start+duration
        intersections = route_intersections(motion, orders, earliest, robot)
        orders.append(ScheduledOrder(number, assignment, robot, motion, earliest, start, finish,
                                     intersections))
        positions[robot] = slot
        headings[robot] = final_heading(motion.routes["ReturnRack"])
        ready[robot] = finish - motion.travel_times["ReturnToStandby"]
    return orders, set(last_order.values())


LOADED_PHASES = {"TravelToWorkstation", "PickBin", "ReturnRack", "LowerRack"}
LOADED_RADIUS = hypot(1.5, 1.5) / 2


def footprints_overlap(a, b):
    """Test the robot or carried rack footprint on the floor plane."""
    def rectangle(pose):
        center, heading, carrying = pose
        angle = radians(0 if carrying else heading)
        c, s = cos(angle), sin(angle)
        return center, ((c, s), (-s, c)), (.75, .75) if carrying else (.675, .455)

    def dot(u, v):
        return u[0] * v[0] + u[1] * v[1]

    center_a, axes_a, half_a = rectangle(a)
    center_b, axes_b, half_b = rectangle(b)
    offset = (center_b[0] - center_a[0], center_b[1] - center_a[1])
    for axis in axes_a + axes_b:
        reach_a = sum(size * abs(dot(axis, edge)) for edge, size in zip(axes_a, half_a))
        reach_b = sum(size * abs(dot(axis, edge)) for edge, size in zip(axes_b, half_b))
        if abs(dot(offset, axis)) > reach_a + reach_b + 1e-9:
            return False
    return True


def mission_pose(mission, time):
    robot = mission.robot
    motion = mission.motion
    phase = robot.phase
    if phase == "Collided":
        return robot.collision_pose
    if phase in motion.routes:
        start = robot.tNext - motion.travel_times[phase]
        point, heading = motion.pose(phase, time - start)
    else:
        route, index = {
            "Idle": ("TravelToRack", 0),
            "LiftRack": ("TravelToWorkstation", 0),
            "PickBin": ("ReturnRack", 0),
            "LowerRack": ("ReturnToStandby", 0),
            "Done": ("ReturnToStandby", -1),
        }[phase]
        point = motion.routes[route][index]
        heading = motion.end_heading if phase == "Done" else motion.start_headings[route]
    return point, heading, phase in LOADED_PHASES


class CoupledRMFS(CoupledDEVS):
    def __init__(self):
        super().__init__("RMFSFleet")
        self.blocked_robots = set()

    def get_snapshot(self):
        return frozenset(self.blocked_robots)

    def set_snapshot(self, state):
        self.blocked_robots.clear()
        self.blocked_robots.update(state)

    def set_init_total_state(self, t):
        self.blocked_robots.clear()
        super().set_init_total_state(t)

    def Delta_x(self, x, t):
        rescheduled = super().Delta_x(x, t)
        if x is None:
            rescheduled = self.check_collisions(t) or rescheduled
        return rescheduled

    def check_collisions(self, t):
        # Missions are ordered by dispatch sequence. Select one per physical robot.
        priority = {"Done": 0, "Idle": 1, "Collided": 3}
        current = {}
        for mission in self.children:
            robot = mission.robot
            rank = priority.get(robot.phase, 2)
            previous = current.get(robot.name)
            if previous is None or rank > previous[0] or rank == previous[0] == 0:
                current[robot.name] = rank, mission
        poses = {name: mission_pose(mission, t) for name, (_, mission) in current.items()}
        collisions = {}
        for a, b in candidate_pairs(poses):
            if footprints_overlap(poses[a], poses[b]):
                collisions.setdefault(a, []).append(b)
                collisions.setdefault(b, []).append(a)
        newly_collided = collisions.keys() - self.blocked_robots
        self.blocked_robots.update(newly_collided)
        for name in sorted(newly_collided):
            mission = current[name][1]
            mission.robot.collision_pose = poses[name]
            mission.robot.Delta_x(PortValue("?collision", sorted(collisions[name])), t)
            mission.update_tn()
        if newly_collided:
            self.update_tn()
        return bool(newly_collided)


def make_fleet_model(orders):
    model = CoupledRMFS()
    for order in orders:
        mission = RMFS(
            order.assignment, (order.robot,), name=f"Order{order.number:03}",
            dispatch_time=order.start, approach=order.motion.routes["TravelToRack"], motion=order.motion,
        )
        mission.robot.blocked_robots = model.blocked_robots
        model.add_components(mission)
    return model








def schedule_home_dispatch(count, seed, robot_count, *, alternatives=True, candidate_times=None):
    """Compare mission routes against the same reservations before dispatch."""
    if count < 1:
        raise ValueError("count must be positive")
    allocator = OrderAllocator(seed)
    robots = parking_fleet(robot_count)
    ready = dict.fromkeys(robots, 0.0)
    headings = dict.fromkeys(robots, -90)
    reservations = Reservations()
    orders = []
    candidate_counts = []
    chosen_detours = 0
    for number in range(1, count + 1):
        assignment = allocator.assign()
        slot = RACK_POSITIONS[assignment.rack]
        station = STATION_APPROACHES[assignment.workstation]
        earliest = min(ready.values())
        robot, approach = select_robot(
            (robot for robot in robots if ready[robot] <= earliest + 1e-8), slot
        )
        base = MissionMotion(slot, station, approach, home=robot.position,
                             heading=headings[robot])
        candidates = [(False, base)]
        if alternatives:
            for phase, route in base.routes.items():
                edges = sorted(pairwise(route), key=lambda edge: -dist(*edge))
                seen = {route}
                for edge in edges[:3]:
                    try:
                        alternate = shortest_route(route[0], route[-1],
                                                   blocked_edges=(frozenset(edge),))
                    except ValueError:
                        continue  # Removing a bridge can disconnect the endpoints.
                    if alternate in seen:
                        continue
                    seen.add(alternate)
                    motion = replace_routes(base, phase, alternate)
                    candidates.append((True, motion))

        best = None
        for detour, motion in candidates:
            started = perf_counter()
            duration, intervals = occupied_intervals(motion, assignment)
            start = reservations.earliest_start(earliest, intervals)
            finish = start + duration
            if candidate_times is not None:
                candidate_times.append(perf_counter() - started)
            if best is None or finish < best[0] - 1e-8:
                best = finish, start, intervals, motion, detour

        finish, start, intervals, motion, detour = best
        reservations.reserve(start, intervals)
        orders.append(ScheduledOrder(number, assignment, robot, motion, earliest, start, finish))
        ready[robot] = finish
        headings[robot] = motion.end_heading
        candidate_counts.append(len(candidates))
        chosen_detours += detour
    return orders, chosen_detours, candidate_counts


def replace_routes(base, phase, alternate):
    motion = MissionMotion.__new__(MissionMotion)
    motion.routes = {**base.routes, phase: alternate}
    motion._build_profiles(base.start_headings["TravelToRack"])
    return motion


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=10)
    parser.add_argument("--orders", type=int, default=100)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--trials", type=int, default=24)
    parser.add_argument("--home-route-stats", action="store_true",
                        help="compare route choices with waiting restricted to home bays")
    args = parser.parse_args()
    if args.home_route_stats:
        started = perf_counter()
        baseline, _, _ = schedule_home_dispatch(args.orders, args.seed, args.robots,
                                                alternatives=False)
        baseline_seconds = perf_counter() - started
        times = []
        started = perf_counter()
        orders, detours, counts = schedule_home_dispatch(args.orders, args.seed, args.robots,
                                                        candidate_times=times)
        search_seconds = perf_counter() - started
        print(f"Planning runtime: single={baseline_seconds:.4f} s, search={search_seconds:.4f} s")
        print(f"Reservation candidate check: mean={sum(times)/len(times):.4f} s, "
              f"max={max(times):.4f} s")
        print(f"A* only: {len(orders) - detours}/{len(orders)} orders")
        print(f"Alternative route: {detours}/{len(orders)} orders")
        longer = sum(
            sum(dist(*edge) for edge in pairwise(route)) >
            sum(dist(*edge) for edge in pairwise(shortest_route(route[0], route[-1]))) + 1e-8
            for order in orders for route in order.motion.routes.values()
        )
        print(f"Equal-distance alternative: {detours - longer}/{len(orders)} orders")
        print(f"Longer alternative: {longer}/{len(orders)} orders")
        print(f"Candidates: {sum(counts)} total, {sum(counts) / len(counts):.2f} per order")
        print(f"A* baseline makespan: {makespan(baseline):.2f} s")
        print(f"Route comparison makespan: {makespan(orders):.2f} s")
        return
    settings = dict(robot_count=args.robots)
    baseline = schedule_orders(args.orders, args.seed, **settings)
    best = schedule_orders(args.orders, args.seed, trials=args.trials, **settings)
    print(f"Baseline makespan {makespan(baseline):.2f} s")
    print(f"Best makespan     {makespan(best):.2f} s")


if __name__ == "__main__":
    main()
