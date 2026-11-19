"""Check routes between dedicated home bays outside the warehouse lanes."""

from itertools import permutations

from .bidirectional_experiment import RACK_HALF, Truck, swept_area_is_clear
from .motion import heading_between
from .traffic_layout import ADJACENCY, RACK_POSITIONS, STATION_APPROACHES, shortest_route

MIN_SPACING = 2 * RACK_HALF


def spaced(points, axis):
    selected = []
    for point in sorted(points, key=lambda point: point[axis]):
        if not selected or point[axis] - selected[-1][axis] > MIN_SPACING:
            selected.append(point)
    return selected


HOME_BAYS = tuple(
    (anchor, (anchor[0] + dx, anchor[1] + dy))
    for anchors, axis, dx, dy in (
        ((point for point in ADJACENCY if point[1] == -16), 0, 0, -3.5),
        ((point for point in ADJACENCY if point[0] == 0), 1, -3.5, 0),
        ((point for point in ADJACENCY if point[0] == 42), 1, 3.5, 0),
        ((point for point in ADJACENCY if point[1] == 15 and all(
            abs(point[0] - x) > MIN_SPACING for x, _ in STATION_APPROACHES.values()
        )), 0, 0, 5.5),
    )
    for anchor in spaced(anchors, axis)
)


def home_route(source, target):
    entry, start = HOME_BAYS[source]
    exit_point, goal = HOME_BAYS[target]
    return (start, *shortest_route(entry, exit_point, blocked_nodes=RACK_POSITIONS.values()), goal)


def main():
    homes = tuple(home for _, home in HOME_BAYS)
    robots = [Truck(home, heading_between(home, anchor)) for anchor, home in HOME_BAYS]
    count = 0
    for source, target in permutations(range(len(robots)), 2):
        route = home_route(source, target)
        stationary = (*RACK_POSITIONS.values(), *(
            home for index, home in enumerate(homes) if index not in (source, target)
        ))
        if not swept_area_is_clear(route, stationary):
            raise RuntimeError(f"Home route {source + 1} -> {target + 1} crosses an occupied bay or rack")
        robot = robots[source]
        robot.start(route)
        robot.advance(sum(action.duration for action in robot.actions) + 1e-8)
        if robot.position != homes[target] or robot.actions:
            raise RuntimeError(f"Home route {source + 1} -> {target + 1} did not finish")
        robots[source] = Truck(homes[source], heading_between(*route[:2]))
        count += 1
    print(f"{len(robots)} home bays, {len(robots)} robots, {count} directed routes verified")


if __name__ == "__main__":
    main()
