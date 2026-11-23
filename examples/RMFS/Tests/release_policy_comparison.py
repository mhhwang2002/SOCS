"""Test-only reuse of the earlier FIFO/LP policies with actual-release extension.

The production TrafficManager remains FIFO. Alternative priorities may only
choose a departure supported by a retained complete coordination certificate.
"""
from functools import reduce
from math import dist
from random import Random
from .run_generated_policy import ResidualPolicyManager,automatic_gates
from .waiting_choice import WaitingChoiceManager
from .run_priority_lookahead import ordered_witness
from ..wait_duplication import WaitDuplicationSolver


class ReleasePolicyComparison(ResidualPolicyManager):
    def _can_start(self,index):
        name = self.tasks[index][1]
        if name in self.active_tasks:
            return WaitingChoiceManager._can_start(self,index)
        return super()._can_start(index)

    def reconsider(self,time):
        super().reconsider(time)
        if self.halted or self.pending_wait or self.pending_choice or not self.motion_extension:
            return
        names = sorted(self.active_tasks,key=lambda name:(
            -self.remaining_length(name) if self.control_mode=='length' else 0,
            self.wait_since.get(name,time),name))
        for name in names:
            first = next((i for i,(_,owner,_) in enumerate(self.tasks)
                          if owner==name and i not in self.issued_tasks),None)
            if first is None:
                continue
            candidate = self._released_extension(name,first)
            if candidate is None:
                continue
            part,indices = candidate
            supported = reduce(set.intersection,(self._supporting_plans(i) for i in indices),
                               set(self.viable_plans))
            if not supported:
                continue
            self.viable_plans = supported
            self.roadmap.extend_approval(name,part,time)
            self.command_buffer.append(dict(robot=name,extension=dict(path=part.coordinates())))
            self.command_tasks[name] = (*self.command_tasks[name],*indices)
            self.issued_tasks.update(indices)


def certificate_portfolio(model,budget=12):
    manager = model.controller.traffic_manager
    paths = model.controller.roadmap.paths
    solver = WaitDuplicationSolver(paths,manager.footprints)
    plans = [manager.tuple_path]
    seen = {repr(manager.tuple_path)}
    requirements = getattr(model,'routing_experiment',{}).get('requires',{})
    # Derive only necessary endpoint orders if this fixture supplies no DAG.
    if not requirements:
        from .generate_path_data import measure
        metrics = measure(paths,next(iter(manager.footprints.values())),manager.policy)
        requirements = {n:set() for n in paths}
        for pair in metrics['pairs']:
            a,b = pair['i'],pair['j']
            si,ei,sj,ej = pair['endpoint_flags']
            if sj or ei: requirements[b].add(a)
            if si or ej: requirements[a].add(b)
    rng = Random(20261121)
    lengths = {n:sum(dist(a,b) for a,b in zip(p.vertices,p.vertices[1:])) for n,p in paths.items()}
    for attempt in range(budget*2):
        pending,order = set(paths),[]
        while pending:
            ready = [n for n in paths if n in pending and set(requirements[n])<=set(order)]
            if not ready:
                break
            n = (ready[0] if attempt==0 else min(ready,key=lambda n:(-lengths[n],n))
                 if attempt==1 else rng.choice(ready))
            order.append(n);pending.remove(n)
        if pending:
            continue
        try:
            result = ordered_witness(solver,order)
            solver.validate(result)
        except ValueError:
            continue
        if result.tuple_path is None:
            continue
        key = repr(result.tuple_path)
        if key not in seen:
            plans.append(result.tuple_path);seen.add(key)
        if len(plans)>=budget:
            break
    return solver,tuple(plans)


def attach_policy(model,policy,geometry,cache=None):
    original = model.controller.traffic_manager
    key = repr((model.controller.roadmap.paths,original.tuple_path))
    if cache is not None and key in cache:
        plans,gates = cache[key]
    else:
        solver,plans = certificate_portfolio(model)
        gates = automatic_gates(solver,geometry)
        if cache is not None:
            cache[key] = plans,gates
    model.controller.traffic_manager = ReleasePolicyComparison(
        model.controller.roadmap,original.tuple_path,pairs=(),
        residual_gates=gates,mode='fifo' if policy=='fifo' else 'length',
        alternative_plans=plans,event_release=True,footprints=original.footprints,
        command_blocks=original.command_blocks,safe_prefix=True,
        motion_extension=original.motion_extension)
    return len(plans)
