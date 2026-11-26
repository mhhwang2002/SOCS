"""Crossing admission policies for dynamically assigned warehouse orders."""

from .TrafficManager import TrafficManager
from .path_geometry import PathPosition, Subpath, sweep_2d


class WarehouseTrafficManager(TrafficManager):
    def __init__(self, roadmap, tuple_path, *, stopping_allowed, **options):
        self.stopping_allowed = stopping_allowed
        layout = roadmap.layout
        self.crossings = frozenset(layout['control_points'][point]
                                   for point, neighbours in layout['adjacency'].items()
                                   if len(neighbours) >= 3)
        super().__init__(roadmap, tuple_path, allow_crossing_stops=stopping_allowed, **options)

    def _combine_initial_prefix(self):
        return False

    def _prefix_parts(self, part):
        # The full-cycle model records already segmented tasks in its snapshots.
        return (part,)

    def _advance_parts(self, path):
        last = len(path.vertices) - 1
        limits = {0, last}
        for index in range(1, last):
            if path.vertices[index] not in self.crossings:
                continue
            if path.vertices[index - 1] not in self.crossings:
                limits.add(index - 1)
            if self.stopping_allowed:
                limits.add(index)
            if path.vertices[index + 1] not in self.crossings:
                limits.add(index + 1)
        positions = tuple(PathPosition(index) for index in sorted(limits))
        return tuple(Subpath(path, first, last) for first, last in zip(positions, positions[1:]))

    def reset(self, time, **options):
        self.pending_push = None
        self.held_until_stop = set()
        super().reset(time, **options)

    lookahead_enabled = False

    def passage_request(self, name, stop, time):
        if self.lookahead_enabled and self._has_push_choice(name, stop):
            self.pending_push = (name, tuple(stop))
            return
        super().passage_request(name, stop, time)

    def _has_push_choice(self, name, stop):
        if name not in self.active_tasks:
            return False
        approved = self.roadmap.robots[name]['approved_path']
        if approved.coordinates()[-1] != tuple(stop):
            return False
        first = next((i for i, (_, owner, _) in enumerate(self.tasks)
                      if owner == name and i not in self.issued_tasks), None)
        extension = None if first is None else self._released_extension(name, first)
        if extension is None:
            return False
        path = Subpath(approved.path, approved.end, extension[0].end)
        sweep = sweep_2d(path, self.footprints[name], self.policy)
        return any(other != name and path.start != path.end
                   and len(remaining.coordinates()) > 1
                   and sweep.intersects(sweep_2d(remaining, self.footprints[other], self.policy),
                                        self.policy.tolerance)
                   for other in self.roadmap.paths
                   for remaining in (self.roadmap.remaining_subpath(other),))

    def _released_extension(self, name, first):
        if name in self.held_until_stop:
            return None
        return super()._released_extension(name, first)

    def choose_push(self, issue, time):
        name, stop = self.pending_push
        self.pending_push = None
        if issue:
            super().passage_request(name, stop, time)
        else:
            self.held_until_stop.add(name)

    def arrived(self, name, position, time):
        self.held_until_stop.discard(name)
        super().arrived(name, position, time)

    def get_snapshot(self):
        return dict(super().get_snapshot(), pending_push=self.pending_push,
                    held_until_stop=set(self.held_until_stop),
                    lookahead_enabled=self.lookahead_enabled)
