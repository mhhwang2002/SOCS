"""Ordinary controller component: heading-aware A* on compiled straight runs."""
from heapq import heappop, heappush
from itertools import count
from math import dist, inf

from .path_geometry import AABB


def _key(value):
    return tuple(map(_key, value)) if isinstance(value, list) else value


class RobotPathPlanner:
    def __init__(self, roadmap, graph):
        self.roadmap = roadmap  # The exact same blackboard used by TrafficManager.
        self.graph = graph
        self.points = {_key(n["id"]): tuple(n["position_m"]) for n in graph["nodes"]}
        self.by_position = {p: k for k, p in self.points.items()}
        self.outgoing = {k: [] for k in self.points}
        for edge in graph["edges"]:
            self.outgoing[_key(edge["source"])].append(edge)

    def plan(self, robot, assignment, *, blocked_positions=(), loaded=False):
        """Return a metre-coordinate path to a rack or explicit goal_position.

        This first dispatch mode protects every other start and assigned rack
        endpoint. TrafficManager still controls conflicts between moving paths.
        Travel estimates contain no traffic waits; no lookahead is performed.
        """
        start = tuple(self.roadmap.robots[robot]["reported_position"])
        goal = tuple(assignment.get("goal_position", assignment.get("rack_position")))
        if start not in self.by_position or goal not in self.by_position:
            raise ValueError("Robot and destination must be on the supplied roadmap.")
        footprint = self.roadmap.footprints[robot]
        hx, hy = footprint.width/2, footprint.depth/2
        margin = self.roadmap.policy.safety_margin
        # All participants use the configured common envelope in this mode.
        obstacles = [AABB((x-hx-margin, y-hy-margin), (x+hx+margin, y+hy+margin))
                     for x, y in blocked_positions]

        def safe(a, b):
            box = AABB((min(a[0], b[0])-hx-margin, min(a[1], b[1])-hy-margin),
                       (max(a[0], b[0])+hx+margin, max(a[1], b[1])+hy+margin))
            return all(not box.intersects(other, self.roadmap.policy.tolerance)
                       for other in obstacles)

        if not safe(start, start) or not safe(goal, goal):
            raise ValueError("No route with clear start and destination.")
        if start == goal:
            return (start,)
        mode = "loaded" if loaded else "empty"
        limits = self.graph["profiles"][mode]
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
                if not safe(self.points[vertex], self.points[nxt]):
                    continue
                direction = edge["entry_heading_deg"]
                turn = 0 if heading is None else abs((direction-heading+180) % 360-180)
                value = cost+edge["travel_time_s"][mode]+turn/limits["yaw_rate"]
                successor = (nxt, edge["exit_heading_deg"])
                if value < costs.get(successor, inf):
                    costs[successor] = value
                    previous[successor] = (state, edge)
                    heappush(queue, (value+dist(self.points[nxt], goal)/limits["speed"],
                                     value, next(serial), successor))
        raise ValueError("No route avoiding the protected endpoints.")
