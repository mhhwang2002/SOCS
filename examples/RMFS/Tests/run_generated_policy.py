"""Evaluate saved generated paths; no path or production-controller changes.

Both optional waiting for an approved approach and existing-waiter arbitration
are evaluated. Safe waiting vertices are derived from the fixed path geometry.
All policies use the same gate stopping boundaries. The experiment does not
claim a globally optimal schedule or enumerate every possible waiting place.
"""
import argparse
import json
from copy import deepcopy
from math import atan2, degrees, dist
from pathlib import Path
from random import Random
from types import MethodType

from ..controlled import create_controlled_model
from ..GridPath import make_grid
from ..path_geometry import PathPosition, Subpath, sweep_2d
from .generate_path_data import load_witness
from .run_priority_lookahead import ordered_witness
from . import run_unified_policy as unified
from .waiting_choice import stable_output, WaitingChoiceManager
from .run_wait_or_go import SEND, WAIT


def automatic_gates(solver, metrics):
    """Nearest original safe vertex before each pairwise interference interval.

    These are potential stopping boundaries shared by ALL compared policies.
    A gate is used for optional waiting only if the other Plant has a finite,
    already-approved approach to its gate. Residual sweeps are checked again.
    """
    paths = dict(zip(solver.names,solver.paths))
    shapes = dict(zip(solver.names,solver.footprints))
    full = {n:sweep_2d(Subpath(p,PathPosition(0),PathPosition(len(p.vertices)-1)),shapes[n],solver.policy)
            for n,p in paths.items()}
    def gates(name,other,intervals):
        found = set()
        for interval in intervals:
            k,ratio = interval['start']
            k = k if ratio else k-1
            while k >= 0:
                point = sweep_2d(paths[name].point(k),shapes[name],solver.policy)
                if not point.intersects(full[other],solver.policy.tolerance):
                    found.add(PathPosition(k)); break
                k -= 1
        return sorted(found)
    result=[]
    for pair in metrics['pairs']:
        a,b=pair['i'],pair['j']
        # A forced whole-path dependency is not an optional order choice.
        if any(pair['endpoint_flags']):
            continue
        for ga in gates(a,b,pair['i_intervals']):
            for gb in gates(b,a,pair['j_intervals']):
                sa=sweep_2d(Subpath(paths[a],ga,PathPosition(ga.segment+1)),shapes[a],solver.policy)
                sb=sweep_2d(Subpath(paths[b],gb,PathPosition(gb.segment+1)),shapes[b],solver.policy)
                if sa.intersects(sb,solver.policy.tolerance):
                    result.extend(((a,b,ga,gb),(b,a,gb,ga)))
    return tuple(result)


class ResidualPolicyManager(unified.UnifiedPolicyManager):
    """Test-only generalization of the hand-specified optional waiting gates."""
    def __init__(self,*args,residual_gates,**kwargs):
        self.residual_gates=residual_gates
        self.gates_by_name={n:sorted({ga for a,b,ga,gb in residual_gates if a==n})
                            for n in args[0].paths}
        super().__init__(*args,**kwargs)

    def reset(self,time):
        self.residual_resolved=set()
        self.residual_holds={}
        super().reset(time)

    def wait_action(self):
        if self.wait_rule=='length':
            row=self.pending_wait
            later=self._remaining_at(row['b'],row['gb'])
            return WAIT if later>self.remaining_length(row['a']) else SEND
        return super().wait_action()

    def _remaining_at(self,name,position):
        points=Subpath(self.roadmap.paths[name],position,
                      PathPosition(len(self.roadmap.paths[name].vertices)-1)).coordinates()
        return sum(dist(a,b) for a,b in zip(points,points[1:]))

    def _can_start(self,index):
        _,name,part=self.tasks[index]
        progress=self.roadmap.robots[name]['released_progress']
        if name in self.residual_holds:
            return False
        if any(progress<gate<=part.start for gate in self.gates_by_name[name]):
            return False
        return WaitingChoiceManager._can_start(self,index)

    def _approach_option(self,index,time):
        _,a,part=self.tasks[index]
        progress=self.roadmap.robots[a]['released_progress']
        if a in self.active_tasks or progress!=part.start:
            return None
        edges={frozenset(edge) for edge in self.roadmap.interference_edges}
        options=[]
        for aa,b,ga,gb in self.residual_gates:
            key=(a,ga,b,gb)
            if aa!=a or ga!=progress or key in self.residual_resolved:
                continue
            if frozenset((a,b)) not in edges or b not in self.active_tasks:
                continue
            record=self.roadmap.robots[b]
            approach=record['approved_path']
            if approach.end!=gb or record['released_progress']>=gb:
                continue
            qb=next((i for i,(_,owner,p) in enumerate(self.tasks)
                     if owner==b and p.start==gb and i not in self.issued_tasks),None)
            if qb is None or not self._supporting_plans(qb):
                continue
            # A's point is disjoint from B's full sweep by gate construction;
            # B's committed approach therefore completes even while A waits.
            options.append(dict(a=a,b=b,ga=ga,gb=gb,key=key,time=time,actions=(SEND,WAIT)))
        if not options:
            return None
        if self.wait_rule=='length':
            return max(options,key=lambda o:self._remaining_at(o['b'],o['gb']))
        return options[0]

    def reconsider(self,time):
        if self.halted or self.pending_wait or self.pending_choice:
            return
        while True:
            ready=[]
            for index in self._next_tasks():
                name=self.tasks[index][1]
                if self._can_start(index):
                    ready.append(index)
                else:
                    self.wait_since.setdefault(name,time)
            if not ready:
                break
            waiting=[i for i in ready if self.tasks[i][1] in self.wait_since]
            conflict=next(((a,b) for k,a in enumerate(waiting) for b in waiting[k+1:]
                           if self.task_sweeps[a].intersects(self.task_sweeps[b],self.policy.tolerance)),None)
            if conflict:
                group=set(conflict)
                while True:
                    expanded=group|{i for i in waiting if any(self.task_sweeps[i].intersects(
                        self.task_sweeps[j],self.policy.tolerance) for j in group)}
                    if expanded==group: break
                    group=expanded
                choices=tuple(sorted(group,key=self._fifo))
                if self.lookahead_enabled:
                    self.pending_choice=choices; return
                selected=self.select_by_rule(choices)
                self.choice_history.append(dict(time=time,candidates=[self.tasks[i][1] for i in choices],
                                                selected=self.tasks[selected][1],policy=self.selection_rule))
            else:
                selected=min(ready,key=self._fifo)
                option=self._approach_option(selected,time)
                if option:
                    self.pending_wait=option
                    if self.control_mode!='lookahead':
                        self.resolve_wait(self.wait_action(),time)
                    return
            self._issue(selected,time)
        self.done=len(self.finished_tasks)==len(self.tasks)

    def resolve_wait(self,action,time):
        if not self.pending_wait or action not in (SEND,WAIT):
            raise ValueError('No optional residual approach decision.')
        row=self.pending_wait
        self.pending_wait=None
        self.residual_resolved.add(row['key'])
        if action==WAIT:
            self.residual_holds[row['a']]=(row['b'],row['gb'])
            self.wait_since.setdefault(row['a'],time)
        b_points=Subpath(self.roadmap.paths[row['b']],row['gb'],
                        PathPosition(len(self.roadmap.paths[row['b']].vertices)-1)).coordinates()
        self.wait_actions.append(dict(time=time,a=row['a'],b=row['b'],action=action,
                                     remaining_a_m=self.remaining_length(row['a']),
                                     remaining_b_at_gate_m=sum(dist(x,y) for x,y in zip(b_points,b_points[1:])),
                                     gate_a=(row['ga'].segment,row['ga'].ratio),
                                     gate_b=(row['gb'].segment,row['gb'].ratio)))
        self.reconsider(time)

    def arrived(self,name,position,time):
        for a,(b,gate) in tuple(self.residual_holds.items()):
            if name==b and tuple(position)==self.roadmap.paths[b].at(gate):
                del self.residual_holds[a]
                self.wait_since.setdefault(b,time)
        super().arrived(name,position,time)

    def get_snapshot(self):
        state=super().get_snapshot()
        state.update(residual_resolved=deepcopy(self.residual_resolved),
                     residual_holds=deepcopy(self.residual_holds))
        return state


def factory(data, sample_index, budget=12):
    solver, reference = load_witness(data, sample_index)
    sample = next(s for s in data['samples'] if s['index'] == sample_index)
    routes = {n: tuple(map(tuple, sample['routes'][n])) for n in sample['object_names']}
    cfg = data['config']
    if cfg['safety_margin_m'] != 0 or cfg['tolerance_m'] != 1e-9:
        raise ValueError('Plant adapter currently requires default geometry policy.')
    requires = {n: set(v) for n,v in sample['requirements'].items()}
    lengths = {n: sum(((b[0]-a[0])**2+(b[1]-a[1])**2)**.5
                     for a,b in zip(routes[n],routes[n][1:])) for n in routes}
    rng = Random(sample['seed'])
    orders = []
    for attempt in range(budget*4):
        pending = set(routes); order = []
        while pending:
            ready = [n for n in routes if n in pending and requires[n] <= set(order)]
            selected = (min(ready) if attempt == 0 else
                        min(ready,key=lambda n:(-lengths[n],n)) if attempt == 1 else rng.choice(ready))
            order.append(selected); pending.remove(selected)
        if tuple(order) not in orders:
            orders.append(tuple(order))
        if len(orders) >= budget:
            break
    portfolio = [reference]
    seen = {repr(reference.tuple_path)}
    for order in orders:
        plan = ordered_witness(solver, order)
        key = repr(plan.tuple_path)
        if key not in seen:
            portfolio.append(plan); seen.add(key)
    headings = {n: degrees(atan2(p[1][1]-p[0][1],p[1][0]-p[0][0])) for n,p in routes.items()}
    gates=automatic_gates(solver,sample['metrics'])

    def build(case, policy, seed=0, collision=False):
        model,_,_ = create_controlled_model(
            routes, grid=make_grid(cfg['grid_size'],cfg['spacing_m']),
            rack_size=(cfg['width_m'],cfg['depth_m']), initial_headings=headings,
            merge_sweeps=True,event_release=True,collision_check=collision,
            stop_on_collision=True,plan_builder=lambda _:reference)
        controller = model.controller
        original = controller.traffic_manager
        controller.traffic_manager = ResidualPolicyManager(
            controller.roadmap, original.tuple_path, pairs=(),
            residual_gates=gates,
            mode='lookahead' if policy.startswith('lookahead_') else policy, seed=seed,
            alternative_plans=tuple(p.tuple_path for p in portfolio), event_release=True,
            footprints=original.footprints,command_blocks=original.command_blocks)
        controller.add_input_ports('?priority','?wait_action')
        controller.delta_x = MethodType(unified.decision_input,controller)
        model.Delta_y = MethodType(stable_output,model)
        return model
    return build, len(portfolio)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data',type=Path,required=True)
    parser.add_argument('--samples',type=int,nargs='+',help='One-based sample numbers; default all.')
    parser.add_argument('--random-repetitions',type=int,default=20)
    parser.add_argument('--portfolio-budget',type=int,default=12)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if min(args.random_repetitions,args.portfolio_budget)<1:
        parser.error('Budgets must be positive.')
    data = json.loads(args.data.read_text(encoding='utf-8'))
    samples = args.samples or [s['index']+1 for s in data['samples']]
    rows = []
    args.output.parent.mkdir(parents=True,exist_ok=True)
    for number in samples:
        build,count = factory(data,number-1,args.portfolio_budget)
        row = unified.experiment(f'sample_{number}',args.random_repetitions,
                                 model_factory=build,progress=True)
        sample = next(s for s in data['samples'] if s['index']==number-1)
        row.update(sample_number=number,path_hash=sample['path_hash'],geometry=sample['metrics'],
                   reference_candidates=count,scope='both_decision_types; automatic_safe_gates; finite_validated_portfolio')
        rows.append(row)
        args.output.write_text(json.dumps(dict(dataset_hash=data['dataset_hash'],results=rows,
            status='complete' if len(rows)==len(samples) else 'running',
            decision_conditions=['optional_wait_for_committed_approach','released_resource_multiple_waiters'],
            shared_gate_stopping_boundaries=True,
            policy_runtime_excludes_common_portfolio_preparation=True),indent=2),encoding='utf-8')
    print(args.output,flush=True)


if __name__=='__main__':
    main()
