"""Compare yielding policies with five or ten loaded robots at warehouse crossings."""

from argparse import ArgumentParser
from itertools import combinations
import json
from pathlib import Path

from .bidirectional_experiment import STEP, Truck, swept_area_is_clear
from .fleet import footprints_overlap
from .motion import Action, LOADED, Straight
from .traffic_layout import RACK_POSITIONS, VERTICAL_LANES, shortest_route
from .warehouse_snapshot import B_START, OPPOSING, START, Snapshot, candidates, split_at_conflict
from .warehouse_three import C_ROUTE, FORK_TICK, frame, permit, remaining

# Adjacent northbound lanes cross the same east-west aisle used by A and B.
CROSSING_LANES = VERTICAL_LANES[4:11]
SNAPSHOT_VERSION = 1


def save_state(trucks):
    """Encode the state immediately before the first policy decision."""
    return json.dumps({
        "version": SNAPSHOT_VERSION,
        "tick": FORK_TICK,
        "robots": [
            [truck.position, truck.heading, truck.elapsed, [
                [action.start, action.end, action.heading, action.turn, action.duration,
                 action.straight.length if action.straight else None]
                for action in truck.actions
            ]]
            for truck in trucks
        ],
    }, separators=(",", ":"))


def restore_state(snapshot, robot_count):
    state = json.loads(snapshot)
    if (state["version"], state["tick"], len(state["robots"])) != (
        SNAPSHOT_VERSION, FORK_TICK, robot_count
    ):
        raise ValueError("Snapshot version, decision tick or robot count does not match this scenario")
    trucks = []
    for position, heading, elapsed, actions in state["robots"]:
        truck = Truck(tuple(position), heading)
        truck.elapsed = elapsed
        truck.actions = [
            Action(tuple(start), tuple(end), angle, turn, duration,
                   Straight(length, LOADED) if length is not None else None)
            for start, end, angle, turn, duration, length in actions
        ]
        trucks.append(truck)
    return tuple(trucks)


def routes_for(robot_count):
    # Extend each northern exit past the east-west lane at y=15.
    return tuple(
        (*shortest_route((x, -14), (x, 15)), (x, 17))
        for x in CROSSING_LANES[:robot_count - 3]
    )


def evaluate(robot_count, *, check_collisions=False, record_frames=True,
             snapshot_in=None, snapshot_out=None):
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
        if record_frames:
            history.append(frame(tick, trucks, goals))
        trucks[2].advance(STEP)
    snapshot = save_state(trucks)
    if snapshot_in is not None:
        snapshot = snapshot_in.read_text(encoding="utf-8")
    if snapshot_out is not None:
        snapshot_out.write_text(snapshot, encoding="utf-8")
    return [
        (option, *simulate(restore_state(snapshot, robot_count), history.copy(), goals,
                           option, crossing_routes, check_collisions=check_collisions,
                           record_frames=record_frames))
        for option in options
    ]


def simulate(trucks, frames, goals, option, crossing_routes, *, check_collisions=False,
             record_frames=True):
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
        if record_frames:
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
    snapshots = parser.add_mutually_exclusive_group()
    snapshots.add_argument("--save-snapshot", type=Path)
    snapshots.add_argument("--load-snapshot", type=Path)
    args = parser.parse_args()
    for option, status, makespan, _, delayed in evaluate(
        args.robots, check_collisions=args.check_collisions, record_frames=False,
        snapshot_in=args.load_snapshot, snapshot_out=args.save_snapshot,
    ):
        duration = "-" if makespan is None else f"{makespan:.2f} s"
        print(f"{option.name:20} {status:9} {duration:10} delayed entries: {delayed}")


if __name__ == "__main__":
    main()
