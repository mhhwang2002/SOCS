"""Execute a validated tuple path; only RMFSController emits DEVS commands."""

from collections import deque
from copy import deepcopy


class TrafficManager:
    def __init__(self, roadmap, tuple_path):
        self.roadmap = roadmap
        self.tuple_path = tuple_path
        if tuple(roadmap.paths) != tuple_path.object_names:
            raise ValueError("Tuple path object order differs from RoadMap.")
        self.reset(0.0)

    def reset(self, time):
        self.roadmap.reset(time)
        self.command_buffer = deque()
        self.stage = -1
        self.awaiting = set()
        self.completed_stages = []
        self.stage_history = []
        self.next_command_id = 0
        self.done = False
        self.halted = False
        self._start_next(time)

    def _start_next(self, time):
        while not self.awaiting and not self.done:
            self.stage += 1
            if self.stage == len(self.tuple_path.stages):
                self.done = True
                return
            row = self.tuple_path.stages[self.stage]
            self.stage_history.append(dict(stage=self.stage, event="started", time=time))
            # Point-only stages consume original point items immediately. A wait
            # never needs a plant command; physical position remains unchanged.
            for name, part in zip(self.tuple_path.object_names, row):
                if part.is_point:
                    continue
                command_id = self.next_command_id
                self.next_command_id += 1
                self.roadmap.approve(name, part, command_id, self.stage, time)
                command = dict(robot=name, path=part.coordinates(),
                               command_id=command_id, stage=self.stage)
                self.command_buffer.append(command)  # approval ALWAYS precedes buffer insertion
                self.awaiting.add(name)
            if self.awaiting:
                return
            self._complete_stage(time)

    def _complete_stage(self, time):
        self.completed_stages.append(self.stage)
        self.stage_history.append(dict(stage=self.stage, event="completed", time=time))

    def emit_commands(self, time):
        if self.halted:
            return ()
        commands = tuple(self.command_buffer)
        for command in commands:
            self.roadmap.issued(command["robot"], command["command_id"], time)
        self.command_buffer.clear()
        return commands

    def arrived(self, name, position, time):
        if name not in self.awaiting:
            raise ValueError(f"Arrival from {name} is not expected in this stage.")
        self.roadmap.arrival(name, position, time)
        self.awaiting.remove(name)
        if not self.awaiting and not self.halted:
            self._complete_stage(time)
            self._start_next(time)

    def halt(self, name, partners, time):
        self.halted = True
        self.command_buffer.clear()
        self.roadmap._record("collision", time, name, partners=tuple(partners))

    def get_snapshot(self):
        return deepcopy(dict(command_buffer=self.command_buffer, stage=self.stage,
                             awaiting=self.awaiting, completed_stages=self.completed_stages,
                             stage_history=self.stage_history, next_command_id=self.next_command_id,
                             done=self.done, halted=self.halted, roadmap=self.roadmap.get_snapshot()))

    def set_snapshot(self, state):
        state = deepcopy(state)
        self.roadmap.set_snapshot(state.pop("roadmap"))
        for name, value in state.items():
            setattr(self, name, value)
