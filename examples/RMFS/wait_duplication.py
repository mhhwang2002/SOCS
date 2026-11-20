"""Complete finite wait-duplication search on fixed path constituents.

This is a standalone test object, not a DEVS model or a TrafficManager change.
No timed predictions, time stepping, rerouting or sweep merging are performed.
"""

from collections import deque
from dataclasses import dataclass
from itertools import combinations
from time import perf_counter

from .path_geometry import (GeometryPolicy, PathPosition, RectangularFootprint,
                            Subpath, TuplePath, sweep_2d)


@dataclass(frozen=True)
class SearchResult:
    status: str  # feasible / infeasible (exhaustion or a geometric impossibility certificate)
    tuple_path: TuplePath | None
    consumed_counts: tuple[tuple[int, ...], ...]
    visited_states: int
    checked_transitions: int
    elapsed_seconds: float
    reason: str | None = None


class WaitDuplicationSolver:
    def __init__(self, paths, footprints=None, *, policy=GeometryPolicy()):
        self.names = tuple(paths)
        if not self.names:
            raise ValueError("At least one object is required.")
        self.paths = tuple(paths[name] for name in self.names)
        footprints = footprints or {}
        self.footprints = tuple(footprints.get(name, RectangularFootprint())
                                for name in self.names)
        self.policy = policy
        self.words = tuple(path.constituents() for path in self.paths)
        self.goal = tuple(len(word) for word in self.words)
        self.sweeps = tuple(tuple(sweep_2d(part, shape, policy) for part in word)
                            for word, shape in zip(self.words, self.footprints))
        self.waits = tuple(tuple(path.point(k) for k in range(len(path.vertices)))
                          for path in self.paths)
        self.wait_sweeps = tuple(tuple(sweep_2d(part, shape, policy) for part in points)
                                 for points, shape in zip(self.waits, self.footprints))

    def _position(self, i, consumed):
        return self.words[i][consumed - 1].end if consumed else PathPosition(0)

    def _choices(self, state, moving):
        parts, sweeps = [], []
        for i, count in enumerate(state):
            if i in moving:
                parts.append(self.words[i][count])
                sweeps.append(self.sweeps[i][count])
            else:
                vertex = self._position(i, count).segment
                parts.append(self.waits[i][vertex])
                sweeps.append(self.wait_sweeps[i][vertex])
        return tuple(parts), tuple(sweeps)

    def _safe(self, sweeps):
        return all(not a.intersects(b, self.policy.tolerance)
                   for a, b in combinations(sweeps, 2))

    def solve(self):
        started = perf_counter()
        for label, vertex in (("initial", 0), ("final", -1)):
            if not self._safe(tuple(sweeps[vertex] for sweeps in self.wait_sweeps)):
                return SearchResult("infeasible", None, (), 1, 0,
                                    perf_counter() - started,
                                    f"Overlapping {label} footprints: all objects must coexist there.")
        for i, j in combinations(range(len(self.paths)), 2):
            a, b = self.paths[i].vertices, self.paths[j].vertices
            for axis in (0, 1):
                fixed = 1 - axis
                if (all(p[fixed] == a[0][fixed] for p in (*a, *b)) and
                        (a[0][axis] - b[0][axis]) * (a[-1][axis] - b[-1][axis]) < 0):
                    return SearchResult("infeasible", None, (), 1, 0,
                                        perf_counter() - started,
                                        f"Reversed order on a shared straight line: "
                                        f"{self.names[i]} and {self.names[j]} cannot pass.")
        initial = (0,) * len(self.paths)
        queue = deque([initial])
        parent = {initial: None}
        checked = 0
        # Every mask consumes at least one original constituent. Waiting copies
        # do not change progress, so an all-wait self-loop adds no reachability.
        while queue:
            state = queue.popleft()
            if state == self.goal:
                states, rows = [state], []
                while parent[state] is not None:
                    previous, row = parent[state]
                    rows.append(row)
                    states.append(previous)
                    state = previous
                return SearchResult("feasible", TuplePath(self.names, tuple(reversed(rows))),
                                    tuple(reversed(states)), len(parent), checked,
                                    perf_counter() - started)
            active = [i for i, count in enumerate(state) if count < self.goal[i]]
            for size in range(1, len(active) + 1):
                for moving in combinations(active, size):
                    checked += 1
                    parts, sweeps = self._choices(state, moving)
                    if not self._safe(sweeps):
                        continue
                    next_state = tuple(count + (i in moving) for i, count in enumerate(state))
                    if next_state not in parent:
                        parent[next_state] = (state, parts)
                        queue.append(next_state)
        return SearchResult("infeasible", None, (), len(parent), checked,
                            perf_counter() - started)

    def validate(self, result):
        """Rebuild and check the witness, including original point constituents.

        Does not trust the search's precomputed sweep cache.
        """
        if result.status != "feasible" or result.tuple_path is None:
            raise ValueError("No feasible witness to validate.")
        rows, states = result.tuple_path.stages, result.consumed_counts
        if (result.tuple_path.object_names != self.names or len(states) != len(rows) + 1
                or states[0] != (0,) * len(self.paths) or states[-1] != self.goal):
            raise ValueError("Invalid witness endpoints or object order.")
        for before, after, row in zip(states, states[1:], rows):
            if len(row) != len(self.paths):
                raise ValueError("Wrong tuple width.")
            deltas = tuple(b - a for a, b in zip(before, after))
            if not any(deltas) or any(delta not in (0, 1) for delta in deltas):
                raise ValueError("Each stage must consume original items without skips.")
            for i, (part, delta) in enumerate(zip(row, deltas)):
                expected = (self.words[i][before[i]] if delta else
                            self.waits[i][self._position(i, before[i]).segment])
                if part != expected:
                    raise ValueError("Witness changed a path or duplicated a non-point.")
            if not self._safe(tuple(sweep_2d(part, shape, self.policy)
                                    for part, shape in zip(row, self.footprints))):
                raise ValueError("Interfering tuple in witness.")
        return True
