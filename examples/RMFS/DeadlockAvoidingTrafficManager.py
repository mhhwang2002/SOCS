"""Execute fixed paths while preserving a sweep-safe completion witness."""

from .TrafficManager import TrafficManager
from .path_geometry import PathPosition, Subpath
from .wait_duplication import completion_after_approval


class DeadlockAvoidingTrafficManager(TrafficManager):
    """FIFO admission with a constructive fallback at a quiescent state."""

    def __init__(self, roadmap, tuple_path, *, event_release=True, **options):
        if (not event_release or options.get("flexible_crossing")
                or options.get("command_blocks") or options.get("crossing_policy", "fifo") != "fifo"):
            raise ValueError("Deadlock avoidance requires fixed paths and event-driven FIFO.")
        super().__init__(roadmap, tuple_path, event_release=True, **options)

    def _build_tasks(self):
        self.tasks = tuple(
            (k, name, Subpath(path, PathPosition(k), PathPosition(k + 1)))
            for name, path in self.roadmap.paths.items()
            for k in range(len(path.vertices) - 1)
        )
        self.edge_index = {(name, part.start): index
                           for index, (_, name, part) in enumerate(self.tasks)}

    def append_route(self, name, path, time, *, advance=False):
        raise ValueError("Deadlock avoidance requires a fixed fleet route set.")

    def reset(self, time, *, reset_roadmap=True):
        self.completion_witness = None
        super().reset(time, reset_roadmap=reset_roadmap)

    def _reservations(self):
        return {
            name: self.roadmap.remaining_subpath(
                name, record["approved_path"].end if record["approved_path"] is not None
                else record["released_progress"])
            for name, record in self.roadmap.robots.items()
        }

    def _issue(self, name, index, time, *, requester=None):
        stage, _, edge = self.tasks[index]
        approval = self.roadmap.robots[name]["approved_path"]
        if approval is None:
            command_id = self.next_command_id
            self.next_command_id += 1
            self.roadmap.approve(name, edge, command_id, stage, time)
            self.command_buffer.append(dict(robot=name, path=edge.coordinates(),
                                            command_id=command_id, stage=stage))
            self.active_tasks[name] = index
            self.command_tasks[name] = (index,)
            self.awaiting.add(name)
        else:
            extended = Subpath(edge.path, approval.start, edge.end)
            self.roadmap.extend_approval(name, extended, time)
            self.command_buffer.append(dict(robot=name, extension=dict(path=extended.coordinates())))
            self.command_tasks[name] += (index,)
            if requester == name:
                self.roadmap._record("push_extension", time, name)
        self.issued_tasks.add(index)
        self.task_wait_since.pop(index, None)
        self.stage = max(self.stage, stage)

    def reconsider(self, time, *, requester=None, cleared_candidates=None):
        if self.halted:
            return
        self.done = len(self.finished_tasks) == len(self.tasks)
        if self.done:
            return
        reservations = self._reservations()
        if self.completion_witness is None:
            result = completion_after_approval(reservations, self.footprints, policy=self.policy)
            if result.tuple_path is None:
                raise ValueError("The fixed routes have no sweep-safe completion plan.")
            self.completion_witness = result.tuple_path.stages

        candidates = []
        for name, reservation in reservations.items():
            index = self.edge_index.get((name, reservation.end))
            if index is None or (requester is not None and requester != name):
                continue
            if name in self.awaiting and (not self.motion_extension
                    or self.roadmap.robots[name]["command_state"] != "issued"):
                continue
            candidates.append((self.task_wait_since.setdefault(index, time), index, name))
        for _, index, name in sorted(candidates):
            edge = self.tasks[index][2]
            tentative = Subpath(edge.path, reservations[name].start, edge.end)
            result = completion_after_approval(
                {**reservations, name: tentative}, self.footprints, policy=self.policy)
            if result.tuple_path is None:
                continue
            self._issue(name, index, time, requester=requester)
            reservations[name] = tentative
            self.completion_witness = result.tuple_path.stages

        if not self.awaiting:
            # At rest the witness starts from the reported positions. Point
            # stages consume no movement, so dispatch its first moving row.
            for row_index, row in enumerate(self.completion_witness):
                moving = [name for name, part in zip(reservations, row) if not part.is_point]
                if moving:
                    for name in moving:
                        self._issue(name, self.edge_index[name, reservations[name].end], time)
                    self.completion_witness = self.completion_witness[row_index + 1:]
                    break
            else:
                raise RuntimeError("An unfinished fleet has no movement in its completion witness.")

    def get_snapshot(self):
        return {**super().get_snapshot(), "completion_witness": self.completion_witness}
