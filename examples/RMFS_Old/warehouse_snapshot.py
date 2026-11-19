"""Compare fixed-route policies from independent copies of one warehouse state."""

from dataclasses import dataclass
from itertools import pairwise
from math import dist

from .bidirectional_experiment import STEP, Truck, swept_area_is_clear
from .fleet import footprints_overlap
from .traffic_layout import ADJACENCY, HORIZONTAL_LANES, INNER_VERTICAL_LANES, LANE_SEGMENTS, RACK_POSITIONS, shortest_route

ROW = HORIZONTAL_LANES[3]
UPPER = HORIZONTAL_LANES[4]
LOWER = HORIZONTAL_LANES[2]
START = (2., ROW)
GOAL = (INNER_VERTICAL_LANES[-1], ROW)
WEST = INNER_VERTICAL_LANES[0]
EAST = INNER_VERTICAL_LANES[5]
B_START = (EAST, UPPER)
B_GOAL = (WEST, -3.1)
DIRECT = shortest_route(START, GOAL)
OPPOSING = (shortest_route(B_START, (EAST, ROW)) +
            shortest_route((EAST, ROW), (WEST, ROW))[1:] +
            shortest_route((WEST, ROW), B_GOAL)[1:])


@dataclass(frozen=True)
class Snapshot:
    opposing_delay: float
    bypass_closed: bool = False


@dataclass(frozen=True)
class Candidate:
    name: str
    first: tuple
    onward: tuple = ()


def candidates(snapshot):
    rack_nodes = RACK_POSITIONS.values()
    contested = {frozenset((a, b)) for a, b in pairwise(DIRECT) if WEST <= a[0] < EAST}
    # A temporary closure across alternate rows leaves the main aisle open.
    closed = {frozenset((s.start, s.end)) for s in LANE_SEGMENTS
              if s.start[0] < 21 < s.end[0] and s.start[1] != ROW} if snapshot.bypass_closed else set()

    def route(start, goal, extra=()):
        return shortest_route(start, goal, blocked_edges=closed.union(extra), blocked_nodes=rack_nodes)

    yield Candidate("Wait", DIRECT)
    if not snapshot.bypass_closed:
        for row, label in ((LOWER, "South"), (UPPER, "North")):
            detour = (route(START, (START[0], row)) +
                      route((START[0], row), (GOAL[0], row), contested)[1:] +
                      route((GOAL[0], row), GOAL)[1:])
            yield Candidate(f"Detour {label}", detour)
        conflicting_edges = {
            frozenset((s.start, s.end)) for s in LANE_SEGMENTS
            if len(split_at_conflict((s.start, s.end), OPPOSING)[1]) > 1
        }
        yield Candidate("Avoid B's Path", route(START, GOAL, conflicting_edges))
    for index in (1, 3, 4):
        x = INNER_VERTICAL_LANES[index]
        junction = (x, ROW)
        # Use the first connected point on the southern branch with room to stop.
        approach = route(junction, (x, LOWER))
        bay = approach[-1]
        first = route(START, junction) + approach[1:]
        onward = route(bay, GOAL, [frozenset(approach[-2:])])
        yield Candidate(f"Yield at {x:.3f} m", first, onward)


def split_at_conflict(route, other_route):
    """Split at the last control point before two rack sweeps can touch."""
    other_edges = tuple(pairwise(other_route))
    for index, edge in enumerate(pairwise(route)):
        if any(
            all(max(min(u, v), min(w, z)) - min(max(u, v), max(w, z)) <= 1.5
                for u, v, w, z in zip(*edge, *other))
            for other in other_edges
        ):
            return route[:index + 1], route[index:]
    return route, route[-1:]


def simulate(snapshot, candidate, *, check_collisions=False):
    a, b = Truck(START, 0), Truck(B_START, -90)
    opposing_prefix, opposing_suffix = OPPOSING[:1], OPPOSING
    approach, continuation = candidate.first, candidate.onward
    if candidate.onward:
        opposing_prefix, opposing_suffix = split_at_conflict(OPPOSING, candidate.first)
    else:
        approach, continuation = split_at_conflict(candidate.first, OPPOSING)
    frames = []
    a_arrival = b_arrival = None
    b_approaching = b_started = False
    a_stage = 0
    for tick in range(10001):
        time = tick * STEP
        if check_collisions and footprints_overlap((a.position, a.heading, True), (b.position, b.heading, True)):
            frames.append((time, a.position, a.heading, b.position, b.heading, False, False))
            return "collision", None, frames
        if not b_started and time >= snapshot.opposing_delay:
            if candidate.onward and not b_approaching:
                if len(opposing_prefix) > 1:
                    b.start(opposing_prefix)
                b_approaching = True
            if not candidate.onward or (a_stage == 1 and not a.actions):
                b_started = b.extend(opposing_suffix)
        if b_started and not b.actions and b_arrival is None:
            b_arrival = time
        if a_stage == 0:
            a.start(approach)
            a_stage = 1
        if a_stage == 1 and (not candidate.onward or b_started):
            remaining_b = (b.position, *(action.end for action in b.actions)) if b_started else OPPOSING
            if len(remaining_b) == 1:
                remaining_b *= 2  # A stopped rack still occupies its footprint.
            available, blocked = split_at_conflict(continuation, remaining_b)
            if len(available) > 1 and a.extend(available):
                continuation = blocked
            if len(continuation) == 1:
                a_stage = 2
        if a.position == GOAL and not a.actions and a_arrival is None:
            a_arrival = time
        waiting_a = a_arrival is None and not a.actions
        waiting_b = not b_started and not b.actions
        frames.append((time, a.position, a.heading, b.position, b.heading, waiting_a, waiting_b))
        if a_arrival is not None and b_arrival is not None:
            return "complete", a_arrival, frames
        a.advance(STEP)
        b.advance(STEP)
    return "timeout", None, frames


def evaluate(snapshot, *, check_collisions=False):
    results = []
    for candidate in candidates(snapshot):
        for route in (candidate.first, candidate.onward):
            assert all(b in ADJACENCY[a] for a, b in pairwise(route))
        if not swept_area_is_clear(candidate.first, RACK_POSITIONS.values()) or not swept_area_is_clear(
            candidate.onward, RACK_POSITIONS.values()
        ):
            results.append((candidate, "blocked", None, []))
            continue
        results.append((candidate, *simulate(snapshot, candidate, check_collisions=check_collisions)))
    return results


SCENARIOS = (
    ("Bypass open", Snapshot(0)),
    ("Bypass closed", Snapshot(0, True)),
    ("Bypass closed, opposing robot delayed", Snapshot(45, True)),
    ("Bypass open, opposing robot delayed", Snapshot(45)),
)


def main():
    from argparse import ArgumentParser

    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--check-collisions", action="store_true")
    args = parser.parse_args()
    for label, snapshot in SCENARIOS:
        print(label)
        results = evaluate(snapshot, check_collisions=args.check_collisions)
        for candidate, status, eta, frames in results:
            print(f"  {candidate.name:20} {status:9} "
                  f"A={eta if eta is not None else '-'}  end={frames[-1][0] if frames else '-'}")
        winner = min((row for row in results if row[2] is not None), key=lambda row: row[2])
        print(f"  Best A arrival: {winner[0].name}")


if __name__ == "__main__":
    main()
