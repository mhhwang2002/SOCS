"""Execute a validated tuple path; only RMFSController emits DEVS commands."""

from collections import deque
from copy import deepcopy

from .path_geometry import GeometryPolicy, RectangularFootprint, Subpath, sweep_2d


class TrafficManager:
    def __init__(self, roadmap, tuple_path, *, event_release=False, footprints=None,
                 command_blocks=None):
        self.roadmap = roadmap
        self.tuple_path = tuple_path
        if tuple(roadmap.paths) != tuple_path.object_names:
            raise ValueError("Tuple path object order differs from RoadMap.")
        self.event_release = event_release
        self.command_blocks = command_blocks
        self.task_control = event_release or command_blocks is not None
        self.footprints = {name:(footprints or {}).get(name,RectangularFootprint())
                           for name in tuple_path.object_names}
        self.policy = GeometryPolicy()
        self.tasks = tuple((stage,name,part)
                           for stage,row in enumerate(tuple_path.stages)
                           for name,part in zip(tuple_path.object_names,row) if not part.is_point)
        self.task_sweeps = tuple(sweep_2d(part,self.footprints[name],self.policy)
                                 for _,name,part in self.tasks)
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
        self.issued_tasks = set()
        self.finished_tasks = set()
        self.active_tasks = {}
        self.command_tasks = {}
        if self.task_control:
            self.reconsider(time)
        else:
            self._start_next(time)

    def _remaining_sweep(self, name, part):
        progress = self.roadmap.robots[name]["released_progress"]
        return sweep_2d(Subpath(part.path,max(part.start,progress),part.end),
                        self.footprints[name],self.policy)

    def _can_start(self, index):
        stage,name,part = self.tasks[index]
        candidate = self.task_sweeps[index]
        # Preserve the reference plan's order for potentially interfering
        # tasks. Earlier active tasks may release their passed prefix; future
        # earlier tasks retain their full reservation until actually passed.
        for other_index,(other_stage,other,other_part) in enumerate(self.tasks):
            if other == name or other_stage >= stage or other_index in self.finished_tasks:
                continue
            if not self.event_release:
                return False  # original stage barrier, independent of command grouping
            occupied = (self._remaining_sweep(other,other_part)
                        if other_index in self.issued_tasks else self.task_sweeps[other_index])
            if candidate.intersects(occupied,self.policy.tolerance):
                return False
        # Physical occupancy includes idle objects and final parked bodies.
        # Later active tasks are also checked; approval never teleports them.
        for other,record in self.roadmap.robots.items():
            if other == name:
                continue
            if other in self.active_tasks:
                occupied = self._remaining_sweep(other,record["approved_path"])
            else:
                progress = record["released_progress"]
                occupied = sweep_2d(Subpath(self.roadmap.paths[other],progress,progress),
                                    self.footprints[other],self.policy)
            if candidate.intersects(occupied,self.policy.tolerance):
                return False
        return True

    def reconsider(self, time):
        """Event-driven dispatch only; no periodic control polling."""
        if not self.task_control or self.halted:
            return
        for index,(stage,name,part) in enumerate(self.tasks):
            if index in self.issued_tasks or name in self.active_tasks:
                continue
            if any(k not in self.finished_tasks for k,(_,owner,_) in
                   enumerate(self.tasks[:index]) if owner == name):
                continue
            if not self._can_start(index):
                continue
            indices = [index]
            # The first primitive is already permitted. A blocked later
            # primitive truncates the command, NEVER delays that first move.
            if self.command_blocks is not None:
                block_end = next(end for start,end in self.command_blocks if start <= stage < end)
                for later,(later_stage,owner,later_part) in enumerate(self.tasks[index+1:],index+1):
                    if owner != name:
                        continue
                    if later_stage >= block_end or later in self.issued_tasks:
                        break
                    if later_part.start != self.tasks[indices[-1]][2].end or not self._can_start(later):
                        break
                    indices.append(later)
            part = Subpath(part.path,part.start,self.tasks[indices[-1]][2].end)
            command_id = self.next_command_id
            self.next_command_id += 1
            self.roadmap.approve(name,part,command_id,stage,time)
            self.command_buffer.append(dict(robot=name,path=part.coordinates(),
                                            command_id=command_id,stage=stage))
            self.issued_tasks.update(indices)
            self.active_tasks[name] = index
            self.command_tasks[name] = tuple(indices)
            self.awaiting.add(name)
            self.stage = max(self.stage,stage)
        self.completed_stages = [stage for stage in range(len(self.tuple_path.stages))
                                 if all(k in self.finished_tasks for k,(s,_,_) in
                                        enumerate(self.tasks) if s == stage)]
        self.done = len(self.finished_tasks) == len(self.tasks)

    def leaving(self, name, position, time):
        self.roadmap.report(name,"leaving",position,time)
        if self.task_control:
            progress = self.roadmap.robots[name]["released_progress"]
            self.finished_tasks.update(index for index in self.command_tasks.get(name,())
                                       if self.tasks[index][2].end <= progress)
        before = self.next_command_id
        if self.event_release:
            self.reconsider(time)
        if self.next_command_id > before:
            self.roadmap._record("release_dispatch",time,name,
                                 command_ids=tuple(range(before,self.next_command_id)))

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
        if self.task_control:
            self.active_tasks.pop(name)
            self.finished_tasks.update(self.command_tasks.pop(name))
            self.reconsider(time)
            return
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
                             done=self.done, halted=self.halted,
                             issued_tasks=self.issued_tasks,finished_tasks=self.finished_tasks,
                             active_tasks=self.active_tasks,command_tasks=self.command_tasks,
                             roadmap=self.roadmap.get_snapshot()))

    def set_snapshot(self, state):
        state = deepcopy(state)
        self.roadmap.set_snapshot(state.pop("roadmap"))
        for name, value in state.items():
            setattr(self, name, value)
