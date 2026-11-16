"""Compare yielding policies with five or ten loaded robots at warehouse crossings."""

from argparse import ArgumentParser
from copy import deepcopy
from itertools import combinations

from .bidirectional_experiment import STEP, Truck, swept_area_is_clear
from .fleet import footprints_overlap
from .traffic_layout import RACK_POSITIONS, VERTICAL_LANES, shortest_route
from .warehouse_snapshot import B_START, OPPOSING, START, Snapshot, candidates, split_at_conflict
from .warehouse_three import C_ROUTE, FORK_TICK, frame, permit, remaining

# Adjacent northbound lanes cross the same east-west aisle used by A and B.
CROSSING_LANES = VERTICAL_LANES[4:11]


def routes_for(robot_count):
    # Extend each northern exit past the east-west lane at y=15.
    return tuple(
        (*shortest_route((x, -14), (x, 15)), (x, 17))
        for x in CROSSING_LANES[:robot_count - 3]
    )


def evaluate(robot_count, *, check_collisions=False):
    options = tuple(candidates(Snapshot(0)))
    crossing_routes = routes_for(robot_count)
    routes = (OPPOSING, C_ROUTE, *crossing_routes)
    assert all(swept_area_is_clear(route, RACK_POSITIONS.values()) for route in routes)
    goals = (options[0].first[-1], *(route[-1] for route in routes))
    trucks = (Truck(START, 0), Truck(B_START, -90), Truck(C_ROUTE[0], 180),
              *(Truck(route[0], 90) for route in crossing_routes))
    trucks[2].start(C_ROUTE)
    history = []
    for tick in range(FORK_TICK):
        history.append(frame(tick, trucks, goals))
        trucks[2].advance(STEP)
    return [
        (option, *simulate(deepcopy(trucks), history.copy(), goals, option, crossing_routes,
                           check_collisions=check_collisions))
        for option in options
    ]


def simulate(trucks, frames, goals, option, crossing_routes, *, check_collisions=False):
    a, b, c, *crossers = trucks
    approach, onward, opposing = option.first, option.onward, OPPOSING
    yielded = False
    delayed_entries = set()
    crossing_remaining = list(crossing_routes)
    pairs = tuple(combinations(trucks, 2)) if check_collisions else ()
    for tick in range(FORK_TICK, 15001):
        for index, (truck, route) in enumerate(zip(crossers, crossing_remaining)):
            if len(route) == 1 or tick < FORK_TICK + index * 50:
                continue
            obstacles = tuple(remaining(other) for other in trucks if other is not truck)
            if onward and len(opposing) > 1:
                obstacles += (opposing,)
            if onward and not yielded:
                # Preserve access to the selected bay while A approaches it.
                obstacles += (approach,)
            crossing_remaining[index] = permit(truck, route, *obstacles)
            if not truck.actions:
                delayed_entries.add(index)
        traffic = tuple(remaining(truck) for truck in (c, *crossers))
        if onward and not yielded:
            approach = permit(a, approach, *traffic, remaining(b))
            yielded = len(approach) == 1 and not a.actions
        if yielded or not onward:
            opposing = permit(b, opposing, *traffic, remaining(a))
            b_path = remaining(b) if len(opposing) == 1 else OPPOSING
            if yielded:
                onward = permit(a, onward, *traffic, b_path)
            else:
                approach = permit(a, approach, *traffic, b_path)
        frames.append(frame(tick, trucks, goals))
        if any(footprints_overlap((x.position, x.heading, True), (y.position, y.heading, True))
               for x, y in pairs):
            return "collision", None, frames, len(delayed_entries)
        if all(truck.position == goal and not truck.actions for truck, goal in zip(trucks, goals)):
            return "complete", tick * STEP, frames, len(delayed_entries)
        if tick >= FORK_TICK + (len(crossers) - 1) * 50 and not any(
            truck.actions for truck in trucks
        ):
            return "stalled", None, frames, len(delayed_entries)
        # Advance only after granting all permits for this sample.
        for truck in trucks:
            truck.advance(STEP)
    return "timeout", None, frames, len(delayed_entries)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, choices=(5, 10), required=True)
    parser.add_argument("--check-collisions", action="store_true",
                        help="check robot footprints at each simulation sample")
    args = parser.parse_args()
    for option, status, makespan, _, delayed in evaluate(
        args.robots, check_collisions=args.check_collisions
    ):
        duration = "-" if makespan is None else f"{makespan:.2f} s"
        print(f"{option.name:20} {status:9} {duration:10} delayed entries: {delayed}")


if __name__ == "__main__":
    main()
