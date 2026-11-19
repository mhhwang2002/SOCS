"""Bidirectional RMFS lane graph and workstation approach paths."""

from dataclasses import dataclass
from itertools import pairwise
from heapq import heappop, heappush
from math import dist, inf

CLUSTERS = 10
WORKSTATIONS = 10


@dataclass(frozen=True)
class LaneSegment:
    start: tuple[float, float]
    end: tuple[float, float]

    @property
    def arcs(self):
        return ((self.start, self.end), (self.end, self.start))


INNER_VERTICAL_LANES = tuple(
    8.825 + 8 * aisle + offset
    for aisle in range(4)
    for offset in (-1.10, 1.10)
)
VERTICAL_LANES = (0, 2, *INNER_VERTICAL_LANES, 40, 42)
HORIZONTAL_LANES = (-16, -14, -1.325, 0.875, 13, 15)
STATION_APPROACHES = {
    f"W{index + 1:02}": (1 + 4 * index, 18)
    for index in range(WORKSTATIONS)
}

RACK_POSITIONS = {
    (cluster, row, col): (
        4 + cluster % 5 * 8 + col * 1.65,
        round(-11.35 + cluster // 5 * 14 + row * 1.65, 3),
    )
    for cluster in range(CLUSTERS)
    for row in range(6)
    for col in range(2)
}
RACK_SLOT = RACK_POSITIONS[2, 2, 1]
STANDBY = (INNER_VERTICAL_LANES[4], HORIZONTAL_LANES[3])

rack_connections = set()
lane_splits = {x: set(HORIZONTAL_LANES) for x in VERTICAL_LANES}
for (cluster, _, col), (x, y) in RACK_POSITIONS.items():
    aisle = cluster % 5 + col
    left, right = VERTICAL_LANES[2 * aisle:2 * aisle + 2]
    lane_splits[left].add(y)
    lane_splits[right].add(y)
    points = sorted((x, left, right))
    rack_connections.update(LaneSegment((a, y), (b, y)) for a, b in pairwise(points))

RACK_CONNECTIONS = tuple(sorted(rack_connections, key=lambda segment: (segment.start, segment.end)))
LANE_SEGMENTS = tuple(
    LaneSegment((x, start), (x, end))
    for x, splits in lane_splits.items()
    for start, end in pairwise(sorted(splits))
) + tuple(
    LaneSegment((start, y), (end, y))
    for y in HORIZONTAL_LANES
    for start, end in pairwise(
        sorted((*VERTICAL_LANES, *(x for x, _ in STATION_APPROACHES.values())))
        if y == HORIZONTAL_LANES[-1] else VERTICAL_LANES
    )
) + tuple(
    LaneSegment((x, HORIZONTAL_LANES[-1]), approach)
    for approach in STATION_APPROACHES.values()
    for x, _ in (approach,)
)
PARKING_LINKS = tuple(
    LaneSegment((x, HORIZONTAL_LANES[0]), (x, -19.5))
    for x in VERTICAL_LANES[1:-1]
) + tuple(
    LaneSegment((x, y), (x + offset, y))
    for x, offset in ((VERTICAL_LANES[0], -3.5), (VERTICAL_LANES[-1], 3.5))
    for y in HORIZONTAL_LANES[1:]
)
PARKING = tuple(segment.end for segment in PARKING_LINKS)
LANE_SEGMENTS += RACK_CONNECTIONS + PARKING_LINKS
# Ordinary links are at most 3.5 m. Bisect the longer links so their
# reservation units match the existing layout without adding junctions.
MID_SEGMENT_CONTROL_POINTS = tuple(
    tuple(round((a + b) / 2, 6) for a, b in zip(segment.start, segment.end))
    for segment in LANE_SEGMENTS if dist(segment.start, segment.end) > 3.5
)
control_points = iter(MID_SEGMENT_CONTROL_POINTS)
split_segments = []
for segment in LANE_SEGMENTS:
    if dist(segment.start, segment.end) > 3.5:
        midpoint = next(control_points)
        split_segments.extend((LaneSegment(segment.start, midpoint), LaneSegment(midpoint, segment.end)))
    else:
        split_segments.append(segment)
LANE_SEGMENTS = tuple(split_segments)
DIRECTED_ARCS = tuple(arc for segment in LANE_SEGMENTS for arc in segment.arcs)
ADJACENCY = {}
for source, destination in DIRECTED_ARCS:
    ADJACENCY.setdefault(source, []).append(destination)



def shortest_route(start, goal, *, blocked_edges=(), blocked_nodes=()):
    """Find the shortest centerline path using A* and Euclidean distance."""
    blocked_edges = set(blocked_edges)
    blocked_nodes = set(blocked_nodes)
    costs = {start: 0.0}
    previous = {}
    queue = [(dist(start, goal), 0.0, start)]
    while queue:
        _, cost, node = heappop(queue)
        if cost != costs[node]:
            continue
        if node == goal:
            path = [goal]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return tuple(reversed(path))
        for neighbor in ADJACENCY[node]:
            if neighbor in blocked_nodes or frozenset((node, neighbor)) in blocked_edges:
                continue
            candidate = cost + dist(node, neighbor)
            if candidate < costs.get(neighbor, inf):
                costs[neighbor] = candidate
                previous[neighbor] = node
                heappush(queue, (candidate + dist(neighbor, goal), candidate, neighbor))
    raise ValueError(f"No lane route from {start} to {goal}")




