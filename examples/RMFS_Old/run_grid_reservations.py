"""Event-driven reservation experiment for the three-robot PPT grid."""

import argparse
from dataclasses import dataclass
from functools import cache

from .motion import LOADED, build_actions, heading_between


@dataclass(frozen=True)
class Case:
    routes: tuple
    no_stop: tuple


CASES = {
    "same": Case(((2, 8, 14, 20, 26), (7, 8, 14, 20, 21),
                  (9, 8, 14, 20, 19)), ((), (), ())),
    "reverse": Case(((2, 8, 14, 15, 16, 22), (10, 16, 15, 14, 13, 19),
                     (19, 13, 7, 8, 9)),
                    ((8, 14, 15, 16), (16, 15, 14, 13), (13, 7, 8))),
}

# Assumed geometry for the numbered diagram, in metres.
SPACING = 3.0
HALF = 0.85


def point(node):
    row, col = divmod(node - 1, 6)
    return col * SPACING, -row * SPACING


def sweep(a, b):
    x, y = point(a)
    u, v = point(b)
    return min(x, u) - HALF, min(y, v) - HALF, max(x, u) + HALF, max(y, v) + HALF


def overlaps(a, b):
    return a[0] <= b[2] and b[0] <= a[2] and a[1] <= b[3] and b[1] <= a[3]


def experiment(case):
    routes = case.routes
    robots = range(len(routes))
    goals = tuple(len(route) - 1 for route in routes)

    @cache
    def request(robot, index):
        route = routes[robot]
        end = index + 1
        while route[end] in case.no_stop[robot] and end < goals[robot]:
            end += 1
        path = route[index:end + 1]
        return end, tuple(sweep(a, b) for a, b in zip(path, path[1:]))

    def clear(area, obstacles):
        return all(not overlaps(a, b) for a in area for b in obstacles)

    @cache
    def can_finish(state):
        """Search serial continuations from a snapshot of waypoint indices."""
        if state == goals:
            return True
        for robot in robots:
            if state[robot] == goals[robot]:
                continue
            end, area = request(robot, state[robot])
            stationary = [sweep(node, node) for j in robots if j != robot
                          for node in (routes[j][state[j]],)]
            if clear(area, stationary):
                successor = state[:robot] + (end,) + state[robot + 1:]
                if can_finish(successor):
                    return True
        return False

    time = 0.
    state = [0 for _ in robots]
    heading = [heading_between(point(route[0]), point(route[1])) for route in routes]
    active = {}
    log = []
    while tuple(state) != goals or active:
        for robot in robots:
            if robot in active or state[robot] == goals[robot]:
                continue
            end, area = request(robot, state[robot])
            obstacles = []
            for j in robots:
                if j == robot:
                    continue
                if j in active:
                    obstacles.extend(active[j][2])
                else:
                    node = routes[j][state[j]]
                    obstacles.append(sweep(node, node))
            projected = [active[j][1] if j in active else state[j] for j in robots]
            projected[robot] = end
            if not clear(area, obstacles) or not can_finish(tuple(projected)):
                continue
            path = routes[robot][state[robot]:end + 1]
            actions = build_actions(tuple(map(point, path)), heading[robot], LOADED)
            finish = time + sum(action.duration for action in actions)
            heading[robot] = heading_between(point(path[-2]), point(path[-1]))
            active[robot] = (finish, end, area)
            log.append((time, robot + 1, "depart", path))
        if not active:
            raise RuntimeError(f"No feasible departure at waypoint indices {state}")
        time = min(move[0] for move in active.values())
        for robot, (finish, end, _) in list(active.items()):
            if finish == time:
                state[robot] = end
                del active[robot]
                log.append((time, robot + 1, "release", (routes[robot][end],)))
    return time, log


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=(*CASES, "all"), nargs="?", default="all")
    args = parser.parse_args()
    for name in CASES if args.case == "all" else (args.case,):
        makespan, log = experiment(CASES[name])
        print(f"[{name}]")
        for time, robot, action, path in log:
            print(f"{time:8.3f}  R{robot} {action:7} {' -> '.join(map(str, path))}")
        print(f"All three arrived. Makespan: {makespan:.3f} s")


if __name__ == "__main__":
    main()
