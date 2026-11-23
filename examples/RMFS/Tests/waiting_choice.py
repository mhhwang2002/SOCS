"""Experimental waiting-contention controller; production TrafficManager unchanged.

Own-path order and approved physical reservations remain mandatory. Original
reference-stage order is NOT used to fix as-yet-uncommitted priorities. Thus
this experiment is not covered automatically by the reference-tuple proof.
It may stall; only completed rollouts are accepted. A pending choice is exposed
at a stable DEVS boundary and resolved by a test-only external input adapter.
"""

from copy import deepcopy
from math import dist
from types import MethodType

from socs.devs.TESBase import PortValue
from ..TrafficManager import TrafficManager
from ..RMFSController import RMFSController
from ..path_geometry import Subpath, sweep_2d, GeometryPolicy, RectangularFootprint


class WaitingChoiceManager(TrafficManager):
    def __init__(self, roadmap, tuple_path, *, lookahead=True, alternative_plans=None,
                 selection_rule='fifo', **kwargs):
        if selection_rule not in ('fifo','longest_remaining_distance','shortest_remaining_distance'):
            raise ValueError('Unknown selection rule: '+str(selection_rule))
        self.initial_selection_rule = selection_rule
        self.lookahead_enabled = lookahead
        self.roadmap = roadmap
        self.safe_prefix = kwargs.get('safe_prefix',False)
        tasks = tuple((stage,name,piece) for stage,row in enumerate(tuple_path.stages)
                      for name,part in zip(tuple_path.object_names,row) if not part.is_point
                      for piece in self._prefix_parts(part))
        identities = {(name,part.start,part.end):i for i,(_,name,part) in enumerate(tasks)}
        shapes = kwargs.get('footprints') or {}
        policy = GeometryPolicy()
        sweeps = [sweep_2d(part,shapes.get(name,RectangularFootprint()),policy) for _,name,part in tasks]
        conflicts = [(i,j) for i,(_,name,_) in enumerate(tasks) for j,(_,other,_) in enumerate(tasks)
                     if name!=other and sweeps[i].intersects(sweeps[j],policy.tolerance)]
        self.plan_predecessors = []
        for plan in alternative_plans or (tuple_path,):
            ranks = {}
            for stage,row in enumerate(plan.stages):
                for name,part in zip(plan.object_names,row):
                    if not part.is_point:
                        for piece in self._prefix_parts(part):
                            ranks[identities[name,piece.start,piece.end]] = stage
            if len(ranks)!=len(tasks):
                raise ValueError('Alternative must retain the same original movement tasks.')
            predecessors = [set() for _ in tasks]
            for i,j in conflicts:
                if ranks[j]<ranks[i]:
                    predecessors[i].add(j)
            self.plan_predecessors.append(tuple(frozenset(row) for row in predecessors))
        super().__init__(roadmap,tuple_path,**kwargs)

    def reset(self,time):
        self.wait_since = {}
        self.pending_choice = ()
        self.choice_history = []
        self.viable_plans = set(range(len(self.plan_predecessors)))
        self.selection_rule = self.initial_selection_rule
        super().reset(time)

    def _can_start(self,index):
        """Committed reservations are hard constraints; unissued priority is free."""
        _,name,_ = self.tasks[index]
        candidate = self.task_sweeps[index]
        for other,record in self.roadmap.robots.items():
            if other == name:
                continue
            if other in self.active_tasks:
                occupied = self._remaining_sweep(other,record['approved_path'])
            else:
                point = record['released_progress']
                occupied = sweep_2d(Subpath(self.roadmap.paths[other],point,point),
                                    self.footprints[other],self.policy)
            if candidate.intersects(occupied,self.policy.tolerance):
                return False
        return bool(self._supporting_plans(index))

    def _supporting_plans(self,index):
        candidate = self.task_sweeps[index]
        supported = set()
        for plan in self.viable_plans:
            for prior in self.plan_predecessors[plan][index]:
                if prior in self.finished_tasks:
                    continue
                _,name,part = self.tasks[prior]
                reserved = (self._remaining_sweep(name,part) if prior in self.issued_tasks
                            else self.task_sweeps[prior])
                if candidate.intersects(reserved,self.policy.tolerance):
                    break
            else:
                supported.add(plan)
        return supported

    def _next_tasks(self):
        result = []
        for index,(_,name,_) in enumerate(self.tasks):
            if index in self.issued_tasks or name in self.active_tasks:
                continue
            if any(k not in self.finished_tasks for k,(_,owner,_) in
                   enumerate(self.tasks[:index]) if owner == name):
                continue
            result.append(index)
        return result

    def _fifo(self,index):
        name = self.tasks[index][1]
        return self.wait_since.get(name,float('inf')),name,index

    def select_by_rule(self,choices,rule=None):
        """A fixed rollout rule, used at every subsequent waiting contention."""
        rule = self.selection_rule if rule is None else rule
        if rule not in ('fifo','longest_remaining_distance','shortest_remaining_distance'):
            raise ValueError('Unknown selection rule: '+str(rule))
        def key(index):
            if rule=='fifo':
                return self._fifo(index)
            name = self.tasks[index][1]
            points = self.roadmap.remaining_subpath(name).coordinates()
            remaining = sum(dist(a,b) for a,b in zip(points,points[1:]))
            return ((-remaining if rule=='longest_remaining_distance' else remaining),*self._fifo(index))
        return min(choices,key=key)

    def _issue(self,index,time):
        stage,name,part = self.tasks[index]
        indices = [index]
        supported = self._supporting_plans(index)
        if not supported:
            raise ValueError('No validated reference supports this departure.')
        if self.command_blocks is not None:
            end = next(b for a,b in self.command_blocks if a<=stage<b)
            for later,(later_stage,owner,later_part) in enumerate(self.tasks[index+1:],index+1):
                if owner!=name:
                    continue
                if later_stage>=end or later in self.issued_tasks:
                    break
                if later_part.start!=self.tasks[indices[-1]][2].end or not self._can_start(later):
                    break
                compatible = supported & self._supporting_plans(later)
                if not compatible:
                    break
                supported = compatible
                indices.append(later)
        self.viable_plans = supported
        part = Subpath(part.path,part.start,self.tasks[indices[-1]][2].end)
        cid = self.next_command_id
        self.next_command_id += 1
        self.roadmap.approve(name,part,cid,stage,time)
        self.command_buffer.append(dict(robot=name,path=part.coordinates(),command_id=cid,stage=stage))
        self.issued_tasks.update(indices)
        self.active_tasks[name] = index
        self.command_tasks[name] = tuple(indices)
        self.awaiting.add(name)
        self.wait_since.pop(name,None)
        self.stage = max(self.stage,stage)

    def reconsider(self,time):
        if self.halted or self.pending_choice:
            return
        while True:
            pending = self._next_tasks()
            ready = []
            for index in pending:
                name = self.tasks[index][1]
                if self._can_start(index):
                    ready.append(index)
                else:
                    self.wait_since.setdefault(name,time)
            if not ready:
                break
            waiting = [i for i in ready if self.tasks[i][1] in self.wait_since]
            # A real lookahead decision requires at least two already waiting,
            # individually eligible objects whose candidate sweeps intersect.
            conflict = next(((a,b) for k,a in enumerate(waiting) for b in waiting[k+1:]
                             if self.task_sweeps[a].intersects(self.task_sweeps[b],self.policy.tolerance)),None)
            if conflict:
                group = set(conflict)
                changed = True
                while changed:
                    previous = len(group)
                    group.update(i for i in waiting if any(self.task_sweeps[i].intersects(
                        self.task_sweeps[j],self.policy.tolerance) for j in tuple(group)))
                    changed = len(group)!=previous
                choices = tuple(sorted(group,key=self._fifo))
                if self.lookahead_enabled:
                    self.pending_choice = choices
                    break
                selected = self.select_by_rule(choices)
                self.choice_history.append(dict(time=time,candidates=[self.tasks[i][1] for i in choices],
                                                selected=self.tasks[selected][1],policy=self.selection_rule))
                self._issue(selected,time)
            else:
                # No competing waiting candidates: grant without lookahead.
                self._issue(min(ready,key=self._fifo),time)
        self.done = len(self.finished_tasks)==len(self.tasks)

    def choose(self,index,time):
        if index not in self.pending_choice or not self._can_start(index):
            raise ValueError('Choice must be an eligible waiting contender.')
        choices = self.pending_choice
        self.pending_choice = ()
        self.choice_history.append(dict(time=time,candidates=[self.tasks[i][1] for i in choices],
                                        selected=self.tasks[index][1],policy='lookahead_choice'))
        self._issue(index,time)
        self.reconsider(time)

    def get_snapshot(self):
        state = super().get_snapshot()
        state.update(wait_since=deepcopy(self.wait_since),pending_choice=self.pending_choice,
                     choice_history=deepcopy(self.choice_history),lookahead_enabled=self.lookahead_enabled,
                     viable_plans=set(self.viable_plans),selection_rule=self.selection_rule)
        return state


class StableImminents(set):
    """Test-only deterministic selection of simultaneous DEVS events."""
    def pop(self):
        selected=min(self,key=lambda model:model.name)
        self.remove(selected)
        return selected


def stable_output(model,time):
    model.IMM=StableImminents(model.IMM)
    return type(model).Delta_y(model,time)


def priority_input(controller,event):
    """Existing Controller, extra experimental external input; no new DEVS class."""
    if event is not None and event.port=='?priority':
        controller.traffic_manager.choose(event.value,controller.tNow)
        controller.phase = 'Command' if controller.traffic_manager.command_buffer else 'Wait'
        return True
    return RMFSController.delta_x(controller,event)


def attach(model,lookahead,selection_rule='fifo'):
    controller = model.controller
    original = controller.traffic_manager
    controller.traffic_manager = WaitingChoiceManager(
        controller.roadmap,original.tuple_path,event_release=True,
        footprints=original.footprints,command_blocks=original.command_blocks,lookahead=lookahead,
        alternative_plans=getattr(model,'waiting_reference_plans',None),selection_rule=selection_rule)
    controller.add_input_ports('?priority')
    controller.delta_x = MethodType(priority_input,controller)
    model.Delta_y = MethodType(stable_output,model)
    return controller


def commit(engine,controller,index):
    # Use the framework's external-input execution path to reschedule parents.
    engine.execute_devs_x(engine.tNow,controller,PortValue('?priority',index))
