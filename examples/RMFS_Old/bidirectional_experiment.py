"""Deterministic tests for head-on traffic in a single physical aisle."""

from dataclasses import dataclass
from itertools import combinations
from math import ceil

from .fleet import footprints_overlap
from .motion import LOADED, build_actions, heading_between

STEP = .02
WEST = (-6., 0.)
EAST_GATE = (6., 0.)
EAST_WAIT = (6., -2.5)
WEST_EXIT = (-8., 0.)
EAST_EXIT = (8., 0.)
BAY = (0., 2.5)
MERGE = (2.5, 0.)
RACK_HALF = .75


@dataclass(frozen=True)
class Case:
    name: str
    bay_available: bool = True
    blocked: tuple = ()
    reserved: frozenset = frozenset()
    delay: float = 0
    east_exit_release: float | None = None


CASES = (
    Case("clear bay"),
    Case("no bay", bay_available=False),
    Case("bay occupied", blocked=(BAY,)),
    Case("bay reserved", reserved=frozenset({"bay"})),
    Case("bay approach blocked", blocked=((0., 1.6),)),
    Case("merge reserved", reserved=frozenset({"merge"})),
    Case("east exit reserved", reserved=frozenset({"east_exit"})),
    Case("delayed arrival at bay", delay=4),
    Case("east exit opens later", reserved=frozenset({"east_exit"}), east_exit_release=5),
)

TO_BAY = (WEST, (0., 0.), BAY)
FROM_BAY = (BAY, (2.5, 2.5), MERGE, EAST_EXIT)
EASTBOUND = (WEST, EAST_EXIT)
WESTBOUND = (EAST_WAIT, EAST_GATE, WEST_EXIT)


def swept_area_is_clear(route, stationary):
    """Exact sweep against stationary racks for axis-aligned translation."""
    return all(
        x < min(a[0], b[0]) - 2 * RACK_HALF
        or x > max(a[0], b[0]) + 2 * RACK_HALF
        or y < min(a[1], b[1]) - 2 * RACK_HALF
        or y > max(a[1], b[1]) + 2 * RACK_HALF
        for a, b in zip(route, route[1:]) for x, y in stationary
    )


class Truck:
    def __init__(self, position, heading):
        self.position = position
        self.heading = heading
        self.actions = []
        self.elapsed = 0.

    def start(self, route):
        assert route[0] == self.position and not self.actions
        self.actions = list(build_actions(route, self.heading, LOADED))

    def extend(self, route):
        if not self.actions:
            self.start(route)
            return True
        last = self.actions[-1]
        assert last.end == route[0]
        extended = list(build_actions((last.start, *route), last.heading, LOADED))
        if len(self.actions) == 1 and extended[0] != last:
            braking_start = last.duration - last.straight.peak / LOADED.deceleration
            if self.elapsed >= braking_start:
                return False
        self.actions[-1:] = extended
        return True

    def advance(self, dt):
        while self.actions:
            action = self.actions[0]
            increment = min(dt, action.duration - self.elapsed)
            self.elapsed += increment
            self.position, self.heading = action.pose(self.elapsed)
            if self.elapsed < action.duration - 1e-9:
                return
            self.position = action.end
            self.actions.pop(0)
            self.elapsed = 0.
            dt -= increment
            if dt <= 0:
                return


def run(case, *, uncontrolled=False, force_exclusive=False, record=None, check_collisions=True):
    a = Truck(WEST, 0)
    b = Truck(EAST_WAIT, heading_between(EAST_WAIT, EAST_GATE))
    bay_resources = {"bay", "bay_approach", "merge", "east_exit", "west_exit"}
    use_bay = (
        not force_exclusive and
        case.bay_available and not case.reserved.intersection(bay_resources)
        and swept_area_is_clear(TO_BAY, case.blocked)
        and swept_area_is_clear(FROM_BAY, case.blocked)
    )
    policy = "uncontrolled" if uncontrolled else "bay" if use_bay else "exclusive"
    events = []
    stage = "request"
    for tick in range(ceil(120 / STEP) + 1):
        now = tick * STEP
        poses = [(truck.position, truck.heading, True) for truck in (a, b)]
        poses.extend((point, 0, True) for point in case.blocked)
        if record is not None:
            record(now, a.position, b.position)
        if check_collisions and any(footprints_overlap(p, q) for p, q in combinations(poses, 2)):
            return policy, "collision", now, events
        if stage == "request":
            if policy == "uncontrolled":
                a.start(EASTBOUND)
                b.start(WESTBOUND)
                stage = "both"
            elif use_bay:
                events.append((now, "A reserves bay, approach and merge"))
                stage = "to bay"
            elif (
                ("east_exit" not in case.reserved
                 or case.east_exit_release is not None and now >= case.east_exit_release)
                and swept_area_is_clear(EASTBOUND, (*case.blocked, b.position))
            ):
                a.start(EASTBOUND)
                events.append((now, "A reserves whole aisle"))
                stage = "exclusive A"
            elif case.east_exit_release is None:
                return policy, "waiting for clear exit", now, events
        if stage == "to bay" and now >= case.delay:
            a.start(TO_BAY)
            stage = "yielding"
        if stage == "yielding" and not a.actions:
            assert a.position[1] > 2 * RACK_HALF
            if "west_exit" in case.reserved or not swept_area_is_clear(WESTBOUND, case.blocked):
                return policy, "waiting for clear exit", now, events
            b.start(WESTBOUND)
            events.append((now, "A clears main lane; B enters"))
            stage = "passing"
        if stage == "passing" and b.position[0] + RACK_HALF < MERGE[0] - RACK_HALF:
            a.start(FROM_BAY)
            events.append((now, "B clears merge; A rejoins"))
            stage = "both"
        if stage == "exclusive A" and a.position[0] - RACK_HALF > EAST_GATE[0]:
            if "west_exit" in case.reserved or not swept_area_is_clear(WESTBOUND, case.blocked):
                return policy, "waiting for clear exit", now, events
            b.start(WESTBOUND)
            events.append((now, "A releases whole aisle; B enters"))
            stage = "both"
        if stage == "both" and not a.actions and not b.actions:
            assert a.position == EAST_EXIT and b.position == WEST_EXIT
            return policy, "complete", now, events
        a.advance(STEP)
        b.advance(STEP)
    return policy, "timeout", 120., events


def main():
    expected = ("complete",) * 6 + ("waiting for clear exit", "complete", "complete")
    for case, outcome in zip(CASES, expected):
        policy, result, time, events = run(case)
        print(f"{case.name:23} {policy:10} {result:23} t={time:.2f}")
        for when, event in events:
            print(f"  {when:6.2f}  {event}")
        assert result == outcome, (case.name, result)
        assert policy == ("bay" if case.name in {"clear bay", "delayed arrival at bay"} else "exclusive")
    _, result, time, _ = run(CASES[0], uncontrolled=True)
    assert result == "collision"
    print(f"uncontrolled baseline: {result} at {time:.2f} s")
    _, result, time, _ = run(CASES[0], force_exclusive=True)
    assert result == "complete"
    print(f"clear-bay geometry with exclusive control: {time:.2f} s")


if __name__ == "__main__":
    main()
