"""Fork a warehouse experiment after the third loaded robot has started."""

from copy import deepcopy
from itertools import combinations

from .bidirectional_experiment import STEP, Truck, swept_area_is_clear
from .fleet import footprints_overlap
from .traffic_layout import INNER_VERTICAL_LANES, RACK_POSITIONS, shortest_route
from .warehouse_snapshot import (
    B_START, LOWER, OPPOSING, START, Snapshot,
    candidates, split_at_conflict,
)

FORK_TICK = 250
CASES = ("C clears near bay early", "C holds near bay")
C_ROUTE = (
    shortest_route((INNER_VERTICAL_LANES[1], LOWER), (2., LOWER))
    + shortest_route((2., LOWER), (2., -14))[1:]
)


def remaining(truck):
    route = (truck.position, *(action.end for action in truck.actions))
    return route if truck.actions else route * 2


def permit(truck, route, *obstacles):
    prefix = route
    for obstacle in obstacles:
        prefix, _ = split_at_conflict(prefix, obstacle)
    if len(prefix) > 1 and truck.extend(prefix):
        return route[len(prefix) - 1:]
    return route


def frame(tick, trucks, goals):
    poses = []
    for truck, goal in zip(trucks, goals):
        status = ""
        if truck.actions and truck.actions[0].turn:
            status = "R"
        elif not truck.actions and truck.position != goal:
            status = "W"
        poses.append((truck.position, truck.heading, status))
    return tick * STEP, tuple(poses)


def evaluate(case, *, check_collisions=False):
    options = tuple(candidates(Snapshot(0)))
    assert swept_area_is_clear(C_ROUTE, RACK_POSITIONS.values())
    goals = (options[0].first[-1], OPPOSING[-1], C_ROUTE[-1])
    snapshot = (Truck(START, 0), Truck(B_START, -90), Truck(C_ROUTE[0], 180))
    release_tick = 0 if case == 0 else 2500
    if release_tick == 0:
        snapshot[2].start(C_ROUTE)
    history = []
    for tick in range(FORK_TICK):
        history.append(frame(tick, snapshot, goals))
        snapshot[2].advance(STEP)
    return [
        (option, *simulate(deepcopy(snapshot), history.copy(), goals, option, release_tick,
                           check_collisions=check_collisions))
        for option in options
    ]


def simulate(trucks, frames, goals, option, release_tick, *, check_collisions=False):
    a, b, c = trucks
    approach = option.first
    onward = option.onward
    opposing = OPPOSING
    yielded = False
    for tick in range(FORK_TICK, 15001):
        if tick == release_tick:
            c.start(C_ROUTE)
        c_path = remaining(c) if c.actions or c.position == goals[2] else C_ROUTE
        if onward:
            if not yielded:
                approach = permit(a, approach, c_path, remaining(b))
                yielded = len(approach) == 1 and not a.actions
            if yielded:
                # Keep B's corridor clear until the complete rack is in the bay.
                opposing = permit(b, opposing, c_path, remaining(a))
                onward = permit(a, onward, c_path, remaining(b) if len(opposing) == 1 else OPPOSING)
        else:
            opposing = permit(b, opposing, c_path, remaining(a))
            approach = permit(a, approach, c_path, remaining(b) if len(opposing) == 1 else OPPOSING)
        frames.append(frame(tick, trucks, goals))
        if check_collisions and any(footprints_overlap((x.position, x.heading, True), (y.position, y.heading, True))
               for x, y in combinations(trucks, 2)):
            return "collision", None, frames
        if all(truck.position == goal and not truck.actions for truck, goal in zip(trucks, goals)):
            return "complete", tick * STEP, frames
        for truck in trucks:
            truck.advance(STEP)
    return "timeout", None, frames


def main():
    from argparse import ArgumentParser

    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--check-collisions", action="store_true")
    args = parser.parse_args()
    for case, label in enumerate(CASES):
        print(label)
        for option, status, makespan, _ in evaluate(case, check_collisions=args.check_collisions):
            print(f"  {option.name:20} {status:9} {makespan if makespan is not None else '-'}")


if __name__ == "__main__":
    main()
