"""Ordinary controller component: heading-aware A* on compiled straight runs."""
from heapq import heappop, heappush
from itertools import count
from math import dist, inf

from .path_geometry import sweep_2d


def _key(value):
    return tuple(map(_key, value)) if isinstance(value, list) else value


def _overlap_length(bounds, regions, padding):
    """Length of a straight run inside the other robot's expanded sweep."""
    axis = 0 if bounds[0] != bounds[1] else 1
    low, high = bounds[2 * axis:2 * axis + 2]
    across = bounds[2 * (1 - axis)]
    intervals = []
    for region in regions:
        if region.lower[1 - axis] - padding[1 - axis] <= across <= region.upper[1 - axis] + padding[1 - axis]:
            start = max(low, region.lower[axis] - padding[axis])
            end = min(high, region.upper[axis] + padding[axis])
            if start < end:
                intervals.append((start, end))
    length, covered = 0.0, low
    for start, end in sorted(intervals):
        length += max(0.0, end - max(start, covered))
        covered = max(covered, end)
    return length


class RobotPathPlanner:
    def __init__(self, roadmap, graph):
        self.roadmap = roadmap  # The exact same blackboard used by TrafficManager.
        self.graph = graph
        self.points = {_key(n["id"]): tuple(n["position_m"]) for n in graph["nodes"]}
        self.by_position = {p: k for k, p in self.points.items()}
        self.outgoing = {k: [] for k in self.points}
        for edge in graph["edges"]:
            self.outgoing[_key(edge["source"])].append(edge)

    def plan(self, robot, assignment, *, blocked_positions=(), loaded=False,
             traffic_routes=()):
        """Return a metre-coordinate path to a rack or explicit goal_position.

        This first dispatch mode protects every other start and assigned rack
        endpoint. TrafficManager still controls conflicts between moving paths.
        Each overlapping metre of a moving robot's sweep adds one metre
        of nominal travel time to the route score.
        """
        start = tuple(self.roadmap.robots[robot]["reported_position"])
        goal = tuple(assignment.get("goal_position", assignment.get("rack_position")))
        if start not in self.by_position or goal not in self.by_position:
            raise ValueError("Robot and destination must be on the supplied roadmap.")
        footprint = self.roadmap.footprints[robot]
        hx, hy = footprint.width/2, footprint.depth/2
        margin = self.roadmap.policy.safety_margin
        # All participants use the configured common envelope in this mode.
        obstacles = tuple({(x-hx-margin, x+hx+margin, y-hy-margin, y+hy+margin)
                           for x, y in blocked_positions})
        tolerance = self.roadmap.policy.tolerance
        clear_edges = {}
        traffic_costs = {}
        traffic_sweeps = [
            sweep_2d(part, self.roadmap.footprints[part.path.name], self.roadmap.policy).regions
            for part in traffic_routes if not part.is_point
        ]

        def safe(bounds):
            if bounds not in clear_edges:
                left, right, bottom, top = bounds
                x_lo, x_hi = left-hx-margin, right+hx+margin
                y_lo, y_hi = bottom-hy-margin, top+hy+margin
                clear_edges[bounds] = not any(
                    x_lo <= x1 + tolerance and x0 <= x_hi + tolerance
                    and y_lo <= y1 + tolerance and y0 <= y_hi + tolerance
                    for x0, x1, y0, y1 in obstacles)
            return clear_edges[bounds]

        if not safe((start[0], start[0], start[1], start[1])) or not safe(
                (goal[0], goal[0], goal[1], goal[1])):
            raise ValueError("No route with clear start and destination.")
        if start == goal:
            return (start,)
        mode = "loaded" if loaded else "empty"
        limits = self.graph["profiles"][mode]
        padding = (hx + margin + tolerance, hy + margin + tolerance)
        initial = (self.by_position[start], self.roadmap.robots[robot].get("heading_deg"))
        target = self.by_position[goal]
        costs, previous = {initial: 0.0}, {}
        serial = count()
        queue = [(dist(start, goal)/limits["speed"], 0.0, next(serial), initial)]
        while queue:
            _, cost, _, state = heappop(queue)
            if cost != costs[state]:
                continue
            vertex, heading = state
            if vertex == target:
                edges = []
                while state != initial:
                    state, edge = previous[state]
                    edges.append(edge)
                path = [start]
                for edge in reversed(edges):
                    path.extend(self.points[_key(k)] for k in edge["path"][1:])
                return tuple(path)
            for edge in self.outgoing[vertex]:
                nxt = _key(edge["target"])
                a, b = self.points[vertex], self.points[nxt]
                bounds = (min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]))
                if not safe(bounds):
                    continue
                if bounds not in traffic_costs:
                    traffic_costs[bounds] = sum(
                        _overlap_length(bounds, regions, padding) for regions in traffic_sweeps
                    ) / limits["speed"]
                direction = edge["entry_heading_deg"]
                turn = 0 if heading is None else abs((direction-heading+180) % 360-180)
                value = cost+edge["travel_time_s"][mode]+turn/limits["yaw_rate"]+traffic_costs[bounds]
                successor = (nxt, edge["exit_heading_deg"])
                if value < costs.get(successor, inf):
                    costs[successor] = value
                    previous[successor] = (state, edge)
                    heappush(queue, (value+dist(self.points[nxt], goal)/limits["speed"],
                                     value, next(serial), successor))
        raise ValueError("No route avoiding the protected endpoints.")
