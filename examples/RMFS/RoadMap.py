"""Controller-owned blackboard: approval is distinct from plant observation."""

from copy import deepcopy
from math import dist

from .path_geometry import GeometryPolicy, PathPosition, RectangularFootprint, Subpath, sweep_2d


class RoadMap:
    def __init__(self, paths, *, footprints=None, policy=GeometryPolicy(), layout=None):
        self.layout = layout  # Static shared geometry; excluded from snapshots.
        self.paths = dict(paths)
        self.footprints = {name:(footprints or {}).get(name,RectangularFootprint())
                           for name in self.paths}
        self.policy = policy
        self.reset(0.0)

    def reset(self, time):
        self.robots = {
            name: dict(reported_position=path.vertices[0], reported_at=time,
                       motion="Wait", command_state="idle", approved_path=None,
                       command_id=None, stage=None, last_leaving=None,
                       released_progress=PathPosition(0))
            for name, path in self.paths.items()
        }
        self.history = []
        self.interference_edges = ()
        self.components = ()
        self._sweep_cache = {}
        self.update_groups(time,record=False)

    def update_groups(self, time, *, record=True, changed_names=None):
        """Update pairs incident to a changed residual sweep, then components.

        This graph describes geometric interference, NOT a wait-for graph.
        Final point footprints remain occupied. No controller polling is used.
        """
        names = tuple(self.paths)
        dirty = set()
        changed_names = names if changed_names is None or not self._sweep_cache else changed_names
        for name in changed_names:
            key = (self.paths[name],self.footprints[name],self.policy,
                   self.robots[name]['released_progress'])
            cached = self._sweep_cache.get(name)
            if cached is None or cached[0] != key:
                self._sweep_cache[name] = (key,sweep_2d(
                    self.remaining_subpath(name),self.footprints[name],self.policy))
                dirty.add(name)
        previous = set(self.interference_edges)
        edges = []
        for i,a in enumerate(names):
            for b in names[i+1:]:
                if a in dirty or b in dirty:
                    intersects = self._sweep_cache[a][1].intersects(
                        self._sweep_cache[b][1],self.policy.tolerance)
                else:
                    intersects = (a,b) in previous
                if intersects:
                    edges.append((a,b))
        edges = tuple(edges)
        adjacency = {name:set() for name in names}
        for a,b in edges:
            adjacency[a].add(b)
            adjacency[b].add(a)
        unseen = set(names)
        components = []
        for name in names:
            if name not in unseen:
                continue
            stack,members = [name],set()
            unseen.remove(name)
            while stack:
                node = stack.pop()
                members.add(node)
                for neighbor in adjacency[node]:
                    if neighbor in unseen:
                        unseen.remove(neighbor)
                        stack.append(neighbor)
            components.append(tuple(n for n in names if n in members))
        components = tuple(components)
        changed = edges != self.interference_edges or components != self.components
        self.interference_edges,self.components = edges,components
        if changed and record:
            self._record("groups_updated",time,edges=edges,components=components)

    @property
    def interference_groups(self):
        return tuple(group for group in self.components if len(group)>1)

    @property
    def independent_objects(self):
        return tuple(group[0] for group in self.components if len(group)==1)

    def _record(self, event, time, name=None, **values):
        self.history.append(dict(event=event, time=time, robot=name, **values))

    def approve(self, name, part, command_id, stage, time):
        record = self.robots[name]
        if record["command_state"] in ("approved", "issued"):
            raise ValueError(f"{name} already has an outstanding command.")
        if dist(record["reported_position"], part.path.at(part.start)) > 1e-8:
            raise ValueError(f"{name}: approved path does not start at reported position.")
        record.update(command_state="approved", approved_path=part,
                      command_id=command_id, stage=stage)
        # Do not modify reported_position or motion at approval.
        self._record("approved", time, name, command_id=command_id, stage=stage)

    def replace_paths(self, paths, time):
        """Install plans only before commands are outstanding; retain observations."""
        if set(paths) != set(self.paths):
            raise ValueError("A plan replacement must preserve the physical fleet.")
        for name, path in paths.items():
            record = self.robots[name]
            if record["command_state"] in ("approved", "issued"):
                raise ValueError("Cannot replace an outstanding plant command.")
            if dist(record["reported_position"], path.vertices[0]) > 1e-8:
                raise ValueError("New path must begin at the reported robot position.")
        self.paths = dict(paths)
        for record in self.robots.values():
            record.update(released_progress=PathPosition(0), approved_path=None,
                          command_state="idle", last_leaving=None)
        self._sweep_cache = {}
        self._record("paths_installed", time)
        self.update_groups(time)

    def replace_robot_path(self, name, path, time):
        record = self.robots[name]
        if record["command_state"] in ("approved", "issued"):
            raise ValueError("Cannot replace an outstanding plant command.")
        if dist(record["reported_position"], path.vertices[0]) > 1e-8:
            raise ValueError("New path must begin at the reported robot position.")
        self.paths[name] = path
        record.update(released_progress=PathPosition(0), approved_path=None,
                      command_state="idle", last_leaving=None)
        self._record("path_installed", time, name)
        self.update_groups(time, changed_names=(name,))

    def issued(self, name, command_id, time):
        record = self.robots[name]
        if record["command_state"] != "approved" or record["command_id"] != command_id:
            raise ValueError("A command must be approved before it can be issued.")
        record.update(command_state="issued", motion="Commanded")
        self._record("issued", time, name, command_id=command_id, stage=record["stage"])

    def extend_approval(self, name, part, time):
        """Reserve a longer issued path without inventing a plant observation."""
        record = self.robots[name]
        previous = record['approved_path']
        if (record['command_state'] != 'issued' or previous is None
                or part.path != previous.path or part.start != previous.start
                or part.end <= previous.end):
            raise ValueError('Only a forward extension of an issued approval is allowed.')
        record['approved_path'] = part
        self._record('extension_reserved', time, name, before=previous.end, after=part.end)

    def arrival(self, name, position, time):
        record = self.robots[name]
        if record["command_state"] != "issued":
            raise ValueError(f"Unexpected arrival from {name}.")
        target = record["approved_path"].path.at(record["approved_path"].end)
        if dist(position, target) > 1e-8:
            raise ValueError(f"{name} reported the wrong arrival position.")
        record.update(reported_position=tuple(position), reported_at=time,
                      motion="Wait", command_state="completed", approved_path=None,
                      released_progress=record["approved_path"].end)
        self._record("arrived", time, name, command_id=record["command_id"],
                     stage=record["stage"], position=tuple(position))
        self.update_groups(time,changed_names=(name,))

    def report(self, name, kind, position, time):
        record = self.robots[name]
        if len(position) != 2:
            raise ValueError("Plant reports require 2D positions.")
        progress = self._locate_forward(name,position) if kind == "leaving" else None
        record.update(reported_position=tuple(position), reported_at=time)
        if kind == "leaving":
            record["last_leaving"] = tuple(position)
            before = record["released_progress"]
            record["released_progress"] = progress
            self._record("released", time, name, before=before,
                         after=record["released_progress"], position=tuple(position))
        record["motion"] = "RotateTo" if kind == "rotation_started" else "MoveTo"
        self._record(kind, time, name, position=tuple(position), stage=record["stage"])
        if kind == "leaving":
            self.update_groups(time,changed_names=(name,))

    def _locate_forward(self, name, position):
        """Map a physical report to original path progress, never backwards.

        Reports carry coordinates only. Search the currently approved part to
        disambiguate paths which revisit the same coordinates.
        """
        path = self.paths[name]
        record = self.robots[name]
        lower = record["released_progress"]
        upper = record["approved_path"].end if record["approved_path"] else lower
        for index in range(lower.segment, min(upper.segment + 1, len(path.vertices)-1)):
            a,b = path.vertices[index:index+2]
            delta = tuple(y-x for x,y in zip(a,b))
            length2 = sum(x*x for x in delta)
            if not length2:
                continue
            ratio = sum((p-x)*d for p,x,d in zip(position,a,delta))/length2
            if not -1e-8 <= ratio <= 1+1e-8:
                continue
            progress = PathPosition(index,min(1.,max(0.,ratio)))
            if lower <= progress <= upper and dist(path.at(progress),position) <= 1e-8:
                return progress
        if dist(path.at(lower),position) <= 1e-8:
            return lower
        raise ValueError(f"{name}: leaving report is outside the approved forward path.")

    def remaining_subpath(self, name, end=None):
        """Conservative unpassed centre path, including the current footprint.

        Only sensor reports/arrivals advance this boundary; the viewer does not.
        """
        path = self.paths[name]
        start = self.robots[name]["released_progress"]
        return Subpath(path,start,PathPosition(len(path.vertices)-1) if end is None else end)

    def get_snapshot(self):
        return deepcopy(dict(paths=self.paths, robots=self.robots, history=self.history,
                             interference_edges=self.interference_edges,components=self.components))

    def set_snapshot(self, state):
        state = deepcopy(state)
        self.paths = state.get("paths", self.paths)
        self.robots, self.history = state["robots"], state["history"]
        self.interference_edges,self.components = state["interference_edges"],state["components"]
        # Derived geometry is not snapshot state. Rebuild lazily after restore.
        self._sweep_cache = {}
