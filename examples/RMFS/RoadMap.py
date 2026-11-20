"""Controller-owned blackboard: approval is distinct from plant observation."""

from copy import deepcopy
from math import dist


class RoadMap:
    def __init__(self, paths):
        self.paths = dict(paths)
        self.reset(0.0)

    def reset(self, time):
        self.robots = {
            name: dict(reported_position=path.vertices[0], reported_at=time,
                       motion="Wait", command_state="idle", approved_path=None,
                       command_id=None, stage=None, last_leaving=None)
            for name, path in self.paths.items()
        }
        self.history = []

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

    def issued(self, name, command_id, time):
        record = self.robots[name]
        if record["command_state"] != "approved" or record["command_id"] != command_id:
            raise ValueError("A command must be approved before it can be issued.")
        record.update(command_state="issued", motion="Commanded")
        self._record("issued", time, name, command_id=command_id, stage=record["stage"])

    def arrival(self, name, position, time):
        record = self.robots[name]
        if record["command_state"] != "issued":
            raise ValueError(f"Unexpected arrival from {name}.")
        target = record["approved_path"].path.at(record["approved_path"].end)
        if dist(position, target) > 1e-8:
            raise ValueError(f"{name} reported the wrong arrival position.")
        record.update(reported_position=tuple(position), reported_at=time,
                      motion="Wait", command_state="completed", approved_path=None)
        self._record("arrived", time, name, command_id=record["command_id"],
                     stage=record["stage"], position=tuple(position))

    def report(self, name, kind, position, time):
        record = self.robots[name]
        if len(position) != 2:
            raise ValueError("Plant reports require 2D positions.")
        record.update(reported_position=tuple(position), reported_at=time)
        if kind == "leaving":
            record["last_leaving"] = tuple(position)
        record["motion"] = "RotateTo" if kind == "rotation_started" else "MoveTo"
        self._record(kind, time, name, position=tuple(position), stage=record["stage"])

    def get_snapshot(self):
        return deepcopy(dict(robots=self.robots, history=self.history))

    def set_snapshot(self, state):
        state = deepcopy(state)
        self.robots, self.history = state["robots"], state["history"]
