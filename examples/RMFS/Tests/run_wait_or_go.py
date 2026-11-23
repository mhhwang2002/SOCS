"""Optional intentional waiting; all ordinary waiting contention uses LP.

Test-only objects/adapters. No timeout, production manager, Plant, or framework
changes. Waiting is offered only while B already has an approved finite
approach ending at its waiting point and both passage orders remain supported
by validated complete reference tuples. Real lookahead branches on send/wait;
rollouts never nest and send immediately at subsequent optional-wait decisions.
All later ordinary contention uses longest remaining distance, FIFO on ties.
"""

import argparse
from copy import deepcopy
from dataclasses import replace
from itertools import product
import json
from math import inf,isclose,isfinite
from pathlib import Path
from random import Random
from statistics import mean,pstdev
from time import perf_counter
from types import MethodType

from socs.devs.TESEngin import TESEnginBase
from socs.devs.TESBase import PortValue
from ..controlled import create_controlled_model
from ..GridPath import make_grid
from ..path_geometry import PathPosition
from ..RMFSController import RMFSController
from ..wait_duplication import WaitDuplicationSolver
from .run_priority_lookahead import ordered_witness
from .run_waiting_lookahead import parallel_group_result,outcome
from .waiting_choice import WaitingChoiceManager,stable_output


SEND='send_now'
WAIT='wait_for_B'
POLICIES=('fifo','length','random','lookahead_makespan','lookahead_mean')


class WaitOrGoManager(WaitingChoiceManager):
    def __init__(self,*args,pairs,**kwargs):
        self.pairs=tuple(pairs)
        super().__init__(*args,lookahead=False,selection_rule='longest_remaining_distance',**kwargs)

    def reset(self,time):
        self.resolved_waits=set()
        self.held_until={}
        self.pending_wait=None
        self.auto_wait_action=None
        self.wait_actions=[]
        super().reset(time)

    def _can_start(self,index):
        _,name,part=self.tasks[index]
        progress=self.roadmap.robots[name]['released_progress']
        for a,b,ga,gb in self.pairs:
            gate=ga if name==a else gb if name==b else None
            if gate is not None and progress<gate and part.start>=gate:
                return False  # approach ends at an explicit common waiting point
            if name==a and part.start>=ga:
                if a not in self.resolved_waits:
                    return False
                if a in self.held_until and self.roadmap.robots[b]['released_progress']<gb:
                    return False
        return super()._can_start(index)

    def _first_after_gate(self,name,gate):
        return next((i for i,(_,owner,part) in enumerate(self.tasks)
                     if owner==name and part.start==gate and i not in self.issued_tasks),None)

    def reconsider(self,time):
        if self.pending_wait or self.halted:
            return
        super().reconsider(time)
        for a,b,ga,gb in self.pairs:
            if a in self.resolved_waits or a in self.active_tasks:
                continue
            if self.roadmap.robots[a]['released_progress']!=ga:
                continue
            qa,qb=self._first_after_gate(a,ga),self._first_after_gate(b,gb)
            record=self.roadmap.robots[b]
            # A wait has a constructive finite-arrival certificate: B is
            # committed only to an unobstructed approach ending at gb.
            approaching=(b in self.active_tasks and record['approved_path'].end==gb
                         and record['released_progress']<gb)
            optional=(qa is not None and qb is not None and approaching
                      and WaitingChoiceManager._can_start(self,qa)
                      and self._supporting_plans(qb))
            if optional:
                self.pending_wait=dict(a=a,b=b,time=time,actions=(SEND,WAIT))
                if self.auto_wait_action is not None:
                    self.resolve_wait(self.auto_wait_action,time)
                return
            # No certified option: do not invent a wait or bypass mandatory order.
            self.resolved_waits.add(a)
            self.reconsider(time)
            return

    def resolve_wait(self,action,time):
        if not self.pending_wait or action not in (SEND,WAIT):
            raise ValueError('No such optional waiting decision.')
        a,b=self.pending_wait['a'],self.pending_wait['b']
        self.pending_wait=None
        self.resolved_waits.add(a)
        if action==WAIT:
            self.held_until[a]=b
        self.wait_actions.append(dict(time=time,a=a,b=b,action=action))
        self.reconsider(time)

    def arrived(self,name,position,time):
        for a,b,ga,gb in self.pairs:
            if name==b and a in self.held_until and tuple(position)==self.roadmap.paths[b].at(gb):
                # Both are now waiting at the same resource; LP decides, not
                # a forced B priority when A actually has the longer remainder.
                self.wait_since.setdefault(b,time)
        super().arrived(name,position,time)

    def get_snapshot(self):
        result=super().get_snapshot()
        result.update(resolved_waits=set(self.resolved_waits),held_until=dict(self.held_until),
                      pending_wait=deepcopy(self.pending_wait),auto_wait_action=self.auto_wait_action,
                      wait_actions=deepcopy(self.wait_actions))
        return result


def wait_input(controller,event):
    if event is not None and event.port=='?wait_action':
        controller.traffic_manager.resolve_wait(event.value,controller.tNow)
        controller.phase='Command' if controller.traffic_manager.command_buffer else 'Wait'
        return True
    return RMFSController.delta_x(controller,event)


def fixture(case):
    # Same 2 m geometry, footprints and motion assumptions across all policies.
    if case in ('short_A','medium_A','long_A'):
        goal={'short_A':3,'medium_A':8,'long_A':9}[case]
        routes={'R1':tuple((x,8) for x in range(1,goal+1)),
                'R2':tuple((2,y) for y in range(10,-1,-1))}
        return routes,make_grid(12),(('R1','R2',PathPosition(0),PathPosition(1)),),{}
    if case=='mixed_four':
        routes={'R1':tuple((x,14) for x in range(0,4)),
                'R2':tuple((2,y) for y in range(17,-1,-1)),
                'R3':((13,7),(13,8))+tuple((x,8) for x in range(14,23)),
                'R4':tuple((14,y) for y in range(11,-1,-1))}
        pairs=(('R1','R2',PathPosition(1),PathPosition(2)),
               ('R3','R4',PathPosition(1),PathPosition(2)))
        return routes,make_grid(24),pairs,{}
    raise ValueError('Unknown fixture: '+case)


def build(case,collision=False):
    routes,grid,pairs,headings=fixture(case)
    alternatives=[]
    def builder(solver):
        alternatives.clear()
        for a,b,_,_ in pairs:
            names=(a,b)
            local=WaitDuplicationSolver({name:solver.paths[solver.names.index(name)] for name in names},
                                        {name:solver.footprints[solver.names.index(name)] for name in names})
            options=[ordered_witness(local,order) for order in ((a,b),(b,a))]
            alternatives.append(options)
        return parallel_group_result(solver,[group[0] for group in alternatives])
    model,robots,sensors=create_controlled_model(
        routes,grid=grid,initial_headings=headings,merge_sweeps=True,event_release=True,
        collision_check=collision,stop_on_collision=True,plan_builder=builder)
    controller=model.controller
    original=controller.traffic_manager
    solver=WaitDuplicationSolver(controller.roadmap.paths,original.footprints)
    plans=tuple(parallel_group_result(solver,choice).tuple_path for choice in product(*alternatives))
    controller.traffic_manager=WaitOrGoManager(
        controller.roadmap,original.tuple_path,event_release=True,pairs=pairs,
        footprints=original.footprints,command_blocks=original.command_blocks,alternative_plans=plans)
    controller.add_input_ports('?wait_action')
    controller.delta_x=MethodType(wait_input,controller)
    model.Delta_y=MethodType(stable_output,model)
    return model


def apply(engine,action):
    engine.execute_devs_x(engine.tNow,engine.model.controller,PortValue('?wait_action',action))


def result(engine,steps,reason=None):
    row=outcome(engine,engine.model,steps,reason)
    row['completion_std']=pstdev(row['completion_times'].values()) if row['completed'] else None
    return row


def simulate_remaining(engine):
    manager=engine.model.controller.traffic_manager
    steps=0
    while isfinite(engine.model.tNext):
        if manager.pending_wait:
            raise AssertionError('Nested lookahead is forbidden.')
        engine.execute_one_step()
        steps+=1
        if steps>=200000:
            return result(engine,steps,'event_budget')
    return result(engine,steps)


def run(case,policy,seed=0,collision=False):
    if policy not in POLICIES:
        raise ValueError('Unknown policy.')
    engine=TESEnginBase(build(case,collision),TimeIntervalContinuous=inf,
                        TimeIntervalAnimation=.01 if collision else inf)
    engine.bVerbose=False
    manager=engine.model.controller.traffic_manager
    rng=Random(seed)
    decisions=[]
    steps=0
    started=perf_counter()
    while isfinite(engine.model.tNext) or manager.pending_wait:
        if manager.pending_wait:
            pending=dict(manager.pending_wait)
            a,b=pending['a'],pending['b']
            branches=[]
            decision_started=perf_counter()
            if policy.startswith('lookahead'):
                snapshot=engine.capture_snapshot()
                virtual=replace(snapshot,t_next_continuous=inf,t_next_animation=inf)
                try:
                    engine.model.collision_check=False
                    engine.TimeIntervalAnimation=inf
                    for action in (SEND,WAIT):
                        branch_started=perf_counter()
                        engine.restore_snapshot(virtual)
                        manager.auto_wait_action=SEND
                        apply(engine,action)
                        outcome_row=simulate_remaining(engine)
                        branches.append(dict(action=action,wall_seconds=perf_counter()-branch_started,
                                             later_actions=deepcopy(manager.wait_actions[len(decisions)+1:]),
                                             **outcome_row))
                finally:
                    engine.model.collision_check=collision
                    engine.TimeIntervalAnimation=.01 if collision else inf
                    engine.restore_snapshot(snapshot)
                feasible=[row for row in branches if row['completed']]
                metric='makespan' if policy=='lookahead_makespan' else 'mean_completion'
                if not feasible:
                    row=result(engine,steps,'no_completed_rollout')
                    break
                action=min(feasible,key=lambda row:(row[metric],row['action']!=SEND))['action']
            elif policy=='fifo':
                action=SEND
            elif policy=='random':
                action=rng.choice((SEND,WAIT))
            else:
                def length(name):
                    from math import dist
                    points=manager.roadmap.remaining_subpath(name).coordinates()
                    return sum(dist(x,y) for x,y in zip(points,points[1:]))
                action=SEND if length(a)>length(b) else WAIT
            decisions.append(dict(time=engine.tNow,a=a,b=b,action=action,branches=branches,
                                  wall_seconds=perf_counter()-decision_started))
            apply(engine,action)
        else:
            engine.execute_one_step()
            steps+=1
            if steps>=200000:
                row=result(engine,steps,'event_budget')
                break
    else:
        row=result(engine,steps)
    row.update(case=case,policy=policy,seed=seed,decisions=decisions,
               fixed_contention_rule='LP_remaining_distance_FIFO_tie',
               rollout_later_optional_wait_rule=SEND,nested_lookahead=False,
               choice_history=manager.choice_history,
               lookahead_calls=sum(bool(d['branches']) for d in decisions),
               rollouts=sum(len(d['branches']) for d in decisions),
               wall_seconds=perf_counter()-started,
               decision_seconds=sum(d['wall_seconds'] for d in decisions if d['branches']))
    return row


def validate(reference):
    row=run(reference['case'],reference['policy'],reference['seed'],True)
    assert row['completed']==reference['completed'] and not row['contacts']
    for key in ('makespan','mean_completion','completion_std'):
        assert isclose(row[key],reference[key],rel_tol=0,abs_tol=1e-9),(key,row[key],reference[key])
    assert [d['action'] for d in row['decisions']]==[d['action'] for d in reference['decisions']]
    return dict(completed=row['completed'],contacts=row['contacts'],interval=.01,stop_on_collision=True,
                makespan=row['makespan'],mean_completion=row['mean_completion'],completion_std=row['completion_std'])


def experiment(case,repetitions=20):
    rows=[]
    for policy in POLICIES:
        repeats=range(repetitions) if policy=='random' else (0,)
        trials=[run(case,policy,seed) for seed in repeats]
        for trial in trials:
            assert trial['completed'],trial
        # Deterministic replay signatures avoid validating the same trajectory
        # repeatedly; every distinct Random action sequence is checked.
        checked={}
        for trial in trials:
            signature=tuple(d['action'] for d in trial['decisions'])
            if signature not in checked:
                checked[signature]=validate(trial)
            trial['validation']=checked[signature]
        metrics={key:dict(mean=mean(t[key] for t in trials),across_run_std=pstdev(t[key] for t in trials))
                 for key in ('makespan','mean_completion','completion_std')}
        rows.append(dict(policy=policy,repetitions=len(trials),metrics=metrics,trials=trials,
                         distinct_validated_action_sequences=len(checked)))
    return dict(case=case,policies=rows,completion_std_definition='population standard deviation across objects per run',
                across_run_std_definition='population standard deviation across random seeds',
                scope='certified_wait_for_one_arriving_object; all later contention LP; no timeout')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',nargs='+',choices=('short_A','medium_A','long_A','mixed_four'),
                        default=('short_A','medium_A','long_A','mixed_four'))
    parser.add_argument('--random-repetitions',type=int,default=20)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results'/'wait_or_go.json')
    args=parser.parse_args()
    if args.random_repetitions<1:
        parser.error('random-repetitions must be positive')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rows=[]
    for case in args.cases:
        row=experiment(case,args.random_repetitions)
        rows.append(row)
        args.output.write_text(json.dumps(rows,indent=2),encoding='utf-8')
        for policy in row['policies']:
            print(case,policy['policy'],{k:round(v['mean'],6) for k,v in policy['metrics'].items()},flush=True)


if __name__=='__main__':
    main()
