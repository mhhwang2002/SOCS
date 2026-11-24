"""Execute a validated tuple path; only RMFSController emits DEVS commands."""

from collections import deque
from copy import deepcopy
from math import atan2, degrees, radians, cos, sin, dist

from .path_geometry import (GeometryPolicy, RectangularFootprint, Subpath, PathPosition,
                            TuplePath, sweep_2d, include_path_points)


class TrafficManager:
    def __init__(self, roadmap, tuple_path, *, event_release=False, footprints=None,
                 command_blocks=None, preorient=False, safe_prefix=False, motion_extension=True,
                 flexible_crossing=False, crossing_policy='fifo'):
        self.roadmap = roadmap
        self.tuple_path = tuple_path
        if tuple(roadmap.paths) != tuple_path.object_names:
            raise ValueError("Tuple path object order differs from RoadMap.")
        self.event_release = event_release
        self.command_blocks = command_blocks
        self.preorient = preorient
        self.safe_prefix = safe_prefix
        self.motion_extension = motion_extension
        self.flexible_crossing = flexible_crossing
        if crossing_policy not in ('fifo', 'lp') or (crossing_policy == 'lp' and not flexible_crossing):
            raise ValueError('LP is available for the validated flexible crossing experiment only.')
        self.crossing_policy = crossing_policy
        self.task_control = event_release or command_blocks is not None
        self.footprints = {name:(footprints or {}).get(name,RectangularFootprint())
                           for name in tuple_path.object_names}
        self.policy = GeometryPolicy()
        self._build_tasks()
        self.reset(0.0)

    def _build_tasks(self):
        tuple_path = self.tuple_path
        original = tuple((stage,name,part)
                           for stage,row in enumerate(tuple_path.stages)
                           for name,part in zip(tuple_path.object_names,row) if not part.is_point)
        # Refine execution units without replanning or changing the witness's
        # cross-robot precedence. An independent prefix can then start now even
        # while a later unit waits for an earlier conflicting task.
        self.tasks = tuple((stage,name,piece) for stage,name,part in original
                           for piece in self._prefix_parts(part))
        self.task_sweeps = tuple(sweep_2d(part,self.footprints[name],self.policy)
                                 for _,name,part in self.tasks)
        if self.flexible_crossing:
            self._validate_flexible_crossing()

    def _validate_flexible_crossing(self):
        # This policy admits a single crossing between two independent approaches.
        names = self.tuple_path.object_names
        if len(names) != 2 or not self.safe_prefix or not self.event_release or self.command_blocks:
            raise ValueError('Flexible crossing requires two robots and separate approach commands.')
        self.crossing_approaches = {}
        self.crossing_passage_lengths = {}
        for name, other in (names, names[::-1]):
            indices = [i for i, (_, owner, _) in enumerate(self.tasks) if owner == name]
            parts = [self.tasks[i][2] for i in indices]
            if len(parts) != 2 or parts[0].end != parts[1].start:
                raise ValueError('Flexible crossing requires one approach and one passage per robot.')
            self.crossing_approaches[name] = indices[0]
            points = parts[1].coordinates()
            self.crossing_passage_lengths[indices[1]] = sum(dist(a, b) for a, b in zip(points, points[1:]))
            for index, (_, owner, part) in enumerate(self.tasks):
                if owner == other:
                    goal = Subpath(parts[1].path, parts[1].end, parts[1].end)
                    for clear in (parts[0], goal):
                        if sweep_2d(clear,self.footprints[name],self.policy).intersects(
                                self.task_sweeps[index],self.policy.tolerance):
                            raise ValueError('Flexible crossing requires clear approaches and goals.')

    def _prefix_parts(self, part):
        if not self.safe_prefix:
            return (part,)
        points = (self.roadmap.layout or {}).get('control_points',{}).values()
        coordinates = include_path_points(part.coordinates(),points)
        positions = [part.start]
        for point in coordinates[1:-1]:
            for k in range(positions[-1].segment, min(part.end.segment+1,len(part.path.vertices)-1)):
                a,b = part.path.vertices[k:k+2]
                delta = tuple(y-x for x,y in zip(a,b))
                length2 = sum(d*d for d in delta)
                ratio = sum((p-x)*d for p,x,d in zip(point,a,delta))/length2
                if not 0 <= ratio <= 1:
                    continue
                at = PathPosition(k,ratio)
                if positions[-1] < at < part.end and all(
                        abs(x-y)<1e-8 for x,y in zip(part.path.at(at),point)):
                    positions.append(at)
                    break
            else:
                raise ValueError('Cannot map a safe-prefix point to path progress.')
        positions.append(part.end)
        return tuple(Subpath(part.path,a,b) for a,b in zip(positions,positions[1:]))

    @property
    def has_commands(self):
        return bool(self.command_buffer or self.orientation_buffer)

    def install_plan(self, paths, tuple_path, time):
        if self.awaiting or self.command_buffer:
            raise ValueError("Cannot install a plan while commands are outstanding.")
        if tuple(paths) != tuple_path.object_names:
            raise ValueError("Tuple path and fleet order differ.")
        self.roadmap.replace_paths(paths, time)
        self.tuple_path = tuple_path
        self._build_tasks()
        self.reset(time, reset_roadmap=False)

    def _combine_initial_prefix(self):
        return True

    def _advance_parts(self, path):
        return tuple(Subpath(path, PathPosition(i), PathPosition(i + 1))
                     for i in range(len(path.vertices) - 1))

    def append_route(self, name, path, time, *, advance=False):
        """Append a new leg without resetting other robots or their commands."""
        if not self.event_release or name in self.awaiting:
            raise ValueError("A new leg requires an available robot and event release.")
        if len(path.vertices) < 2:
            raise ValueError("A movement leg needs at least two points.")
        self.roadmap.replace_robot_path(name, path, time)
        start_stage = len(self.tuple_path.stages)
        parts = (self._advance_parts(path) if advance else
                 (Subpath(path, PathPosition(0), PathPosition(len(path.vertices)-1)),))
        rows = []
        for part in parts:
            rows.append(tuple(part if other == name else
                        Subpath(p, self.roadmap.robots[other]["released_progress"],
                                self.roadmap.robots[other]["released_progress"])
                        for other, p in self.roadmap.paths.items()))
        self.tuple_path = TuplePath(self.tuple_path.object_names, (*self.tuple_path.stages, *rows))
        for offset, part in enumerate(parts):
            self.tasks = (*self.tasks, (start_stage+offset, name, part))
            self.task_sweeps = (*self.task_sweeps, sweep_2d(part, self.footprints[name], self.policy))
        if advance and self._combine_initial_prefix():
            self.command_blocks = (*(self.command_blocks or ()),
                                   (start_stage, start_stage+len(parts)))
        self.done = False
        self.reconsider(time)

    def reset(self, time, *, reset_roadmap=True):
        if reset_roadmap:
            self.roadmap.reset(time)
        self.command_buffer = deque()
        self.orientation_buffer = deque()
        self.orienting = {}
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
        self.task_wait_since = {}
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
            if (self.flexible_crossing or other == name or other_stage >= stage
                    or other_index in self.finished_tasks):
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

    def _released_extension(self, name, first):
        """A collinear extension authorized by present clearance, never prediction."""
        record = self.roadmap.robots[name]
        approved = record['approved_path']
        if not self.safe_prefix or record['command_state'] != 'issued' or approved is None:
            return None
        points = approved.coordinates()
        a,b = points[-2:]
        length = sum((y-x)**2 for x,y in zip(a,b))**.5
        direction = tuple((y-x)/length for x,y in zip(a,b))
        def straight(points):
            for a,b in zip(points,points[1:]):
                length = sum((y-x)**2 for x,y in zip(a,b))**.5
                if not length or any(abs((y-x)/length-d)>1e-8 for x,y,d in zip(a,b,direction)):
                    return False
            return True
        if not straight(points):
            return None  # Commands containing turns retain their planned stops.
        end,indices = approved.end,[]
        for index in range(first,len(self.tasks)):
            _,owner,part = self.tasks[index]
            if owner != name:
                continue
            if (index in self.issued_tasks or part.path != approved.path or part.start != end
                    or not straight(part.coordinates()) or not self._can_start(index)):
                break
            indices.append(index)
            end = part.end
            point = sweep_2d(Subpath(part.path,end,end),self.footprints[name],self.policy)
            if self._extension_endpoint_clear(name, point):
                return Subpath(approved.path,approved.start,end),tuple(indices)
        return None

    def _extension_endpoint_clear(self, name, point):
        return all(other == name or not point.intersects(
                    sweep_2d(self.roadmap.remaining_subpath(other),self.footprints[other],self.policy),
                    self.policy.tolerance) for other in self.roadmap.paths)

    def passage_request(self, name, stop, time):
        approved = self.roadmap.robots[name]['approved_path']
        if (name in self.active_tasks and approved.coordinates()[-1] == tuple(stop)
                and any(owner == name and index not in self.issued_tasks
                        for index, (_, owner, _) in enumerate(self.tasks))):
            self.roadmap._record('passage_request', time, name, position=tuple(stop))
            self.reconsider(time, requester=name)

    def _blocked_extensions(self, reporter):
        if not self.flexible_crossing or not self.motion_extension:
            return None
        return {name for index, (_, name, _) in enumerate(self.tasks)
                if name != reporter and name in self.active_tasks and index not in self.issued_tasks
                and self._released_extension(name,index) is None}

    def reconsider(self, time, *, requester=None, cleared_candidates=None):
        """Event-driven dispatch only; no periodic control polling."""
        if not self.task_control or self.halted:
            return
        pending = []
        blocked_owners = set()
        for index, (_, name, _) in enumerate(self.tasks):
            if index not in self.issued_tasks and name not in blocked_owners:
                self.task_wait_since.setdefault(index, time)
                pending.append(index)
            if index not in self.finished_tasks and index not in self.command_tasks.get(name, ()):
                blocked_owners.add(name)
        # FIFO orders only candidates; the validated precedence and physical
        # occupancy guard remain mandatory. Ties follow tuple object order.
        object_rank = {name:i for i,name in enumerate(self.tuple_path.object_names)}
        pending.sort(key=lambda i:(self.task_wait_since[i],object_rank[self.tasks[i][1]],i))
        if self.crossing_policy == 'lp':
            pending.sort(key=lambda i: -self.crossing_passage_lengths.get(i, 0))
        for index in pending:
            stage,name,part = self.tasks[index]
            passage = self.flexible_crossing and index in self.crossing_passage_lengths
            if passage and self.crossing_policy == 'lp' and any(
                    other_index not in self.issued_tasks and length > self.crossing_passage_lengths[index]
                    and self.active_tasks.get(self.tasks[other_index][1]) ==
                    self.crossing_approaches[self.tasks[other_index][1]]
                    for other_index, length in self.crossing_passage_lengths.items()):
                continue  # The longer robot already has a finite, independently safe approach.
            if name in self.active_tasks:
                if name != requester and (requester is not None or not self.motion_extension):
                    continue
                if self.flexible_crossing and requester is None and name not in (cleared_candidates or ()):
                    continue
                extension = self._released_extension(name,index)
                if extension is not None:
                    part,indices = extension
                    self.roadmap.extend_approval(name,part,time)
                    if requester is not None:
                        self.roadmap._record('push_extension', time, name)
                    if passage:
                        self.roadmap._record('crossing_granted', time, name,
                                             trigger='push' if requester is not None else 'pull')
                    self.command_buffer.append(dict(robot=name,extension=dict(path=part.coordinates())))
                    self.command_tasks[name] = (*self.command_tasks[name],*indices)
                    self.issued_tasks.update(indices)
                    for admitted in indices:
                        self.task_wait_since.pop(admitted,None)
                continue
            if name in self.orienting:
                continue
            if not self._can_start(index):
                if self.preorient:
                    self._prepare_wait_heading(name, part, time)
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
            if passage:
                self.roadmap._record('crossing_granted', time, name, trigger='after_stop')
            self.command_buffer.append(dict(robot=name,path=part.coordinates(),
                                            command_id=command_id,stage=stage))
            self.issued_tasks.update(indices)
            for admitted in indices:
                self.task_wait_since.pop(admitted,None)
            self.active_tasks[name] = index
            self.command_tasks[name] = tuple(indices)
            self.awaiting.add(name)
            self.stage = max(self.stage,stage)
        incomplete = {stage for i, (stage, _, _) in enumerate(self.tasks)
                      if i not in self.finished_tasks}
        self.completed_stages = [stage for stage in range(len(self.tuple_path.stages))
                                 if stage not in incomplete]
        self.done = len(self.finished_tasks) == len(self.tasks)

    def _prepare_wait_heading(self, name, part, time):
        heading = self.roadmap.robots[name].get('heading_deg')
        if heading is None:
            return  # An uninitialized robot auto-aligns to its first move.
        a,b = part.coordinates()[:2]
        target = degrees(atan2(b[1]-a[1], b[0]-a[0]))
        if abs((target-heading+180)%360-180) < 1e-8:
            return
        self.orienting[name] = target
        direction = (cos(radians(target)), sin(radians(target)))
        self.orientation_buffer.append(dict(robot=name, orientation=direction))
        self.roadmap._record('orientation_approved', time, name, heading_deg=target)

    def rotation_report(self, name, kind, position, time):
        self.roadmap.report(name, kind, position, time)
        if kind == 'rotation_finished' and name in self.orienting:
            self.roadmap.robots[name]['heading_deg'] = self.orienting.pop(name)
            self.roadmap.robots[name]['motion'] = 'Wait'
            self.reconsider(time)

    def leaving(self, name, position, time):
        blocked = self._blocked_extensions(name)
        self.roadmap.report(name,"leaving",position,time)
        if self.task_control:
            progress = self.roadmap.robots[name]["released_progress"]
            self.finished_tasks.update(index for index in self.command_tasks.get(name,())
                                       if self.tasks[index][2].end <= progress)
        before = self.next_command_id
        if self.event_release:
            if blocked is None:
                self.reconsider(time)
            else:
                self.reconsider(time, cleared_candidates=blocked)
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
            if 'extension' not in command:
                self.roadmap.issued(command["robot"], command["command_id"], time)
        self.command_buffer.clear()
        orientations = tuple(self.orientation_buffer)
        self.orientation_buffer.clear()
        return (*commands, *orientations)

    def arrived(self, name, position, time):
        if name not in self.awaiting:
            raise ValueError(f"Arrival from {name} is not expected in this stage.")
        coordinates = self.roadmap.robots[name]['approved_path'].coordinates()
        a,b = coordinates[-2:]
        self.roadmap.robots[name]['heading_deg'] = degrees(atan2(b[1]-a[1],b[0]-a[0]))
        blocked = self._blocked_extensions(name)
        self.roadmap.arrival(name, position, time)
        self.awaiting.remove(name)
        if self.task_control:
            self.active_tasks.pop(name)
            self.finished_tasks.update(self.command_tasks.pop(name))
            if blocked is None:
                self.reconsider(time)
            else:
                self.reconsider(time, cleared_candidates=blocked)
            return
        if not self.awaiting and not self.halted:
            self._complete_stage(time)
            self._start_next(time)

    def halt(self, name, partners, time):
        self.halted = True
        self.command_buffer.clear()
        self.orientation_buffer.clear()
        self.roadmap._record("collision", time, name, partners=tuple(partners))

    def get_snapshot(self):
        return deepcopy(dict(tuple_path=self.tuple_path, command_blocks=self.command_blocks,
                             command_buffer=self.command_buffer, stage=self.stage,
                             awaiting=self.awaiting, completed_stages=self.completed_stages,
                             stage_history=self.stage_history, next_command_id=self.next_command_id,
                             done=self.done, halted=self.halted,
                             issued_tasks=self.issued_tasks,finished_tasks=self.finished_tasks,
                             active_tasks=self.active_tasks,command_tasks=self.command_tasks,
                             task_wait_since=self.task_wait_since,
                             orientation_buffer=self.orientation_buffer, orienting=self.orienting,
                             roadmap=self.roadmap.get_snapshot()))

    def set_snapshot(self, state):
        state = deepcopy(state)
        self.roadmap.set_snapshot(state.pop("roadmap"))
        self.task_wait_since = state.pop("task_wait_since",{})
        for name, value in state.items():
            setattr(self, name, value)
        self._build_tasks()
