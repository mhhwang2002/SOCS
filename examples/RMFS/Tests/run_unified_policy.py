"""Test-only comparison: one policy for intentional wait AND waiting priority.

FIFO: send on arrival, FIFO among existing waiters.
Length: send if A has the longer remainder, otherwise wait; LP among waiters.
Random: random admissible action/eligible contender at both decision types.
Lookahead: independently combine the CURRENT action, future optional-wait
rule (immediate/length), and future waiting priority (FIFO/LP). This includes
the previous fixed-LP rollout: future immediate sending plus waiting LP.
Real execution applies only the current action and reevaluates next time.
Production TrafficManager and the earlier LP-fixed experiment are unchanged.
"""

import argparse
from copy import deepcopy
from dataclasses import replace
import json
from itertools import product
from math import dist,inf,isclose,isfinite
from pathlib import Path
from random import Random
from statistics import mean,pstdev
from time import perf_counter
from types import MethodType

from socs.devs.TESEngin import TESEnginBase
from socs.devs.TESBase import PortValue
from . import run_wait_or_go as old
from .waiting_choice import stable_output


MODES=('fifo','length','random','lookahead')


class UnifiedPolicyManager(old.WaitOrGoManager):
    def __init__(self,*args,mode,seed=0,reference_predecessors=None,**kwargs):
        self.control_mode=mode
        self.initial_mode=mode
        self.seed=seed
        self.rng=Random(seed)
        super().__init__(*args,**kwargs)
        if reference_predecessors is not None:
            self.plan_predecessors=reference_predecessors
            self.reset(0.)

    def set_mode(self,mode,wait_rule=None):
        if mode not in MODES:
            raise ValueError('Unknown unified policy: '+str(mode))
        self.control_mode=mode
        self.wait_rule=mode if wait_rule is None else wait_rule
        if self.wait_rule not in MODES:
            raise ValueError('Unknown optional-wait rule.')
        self.lookahead_enabled=mode=='lookahead'
        self.selection_rule=('longest_remaining_distance' if mode=='length'
                             else 'random' if mode=='random' else 'fifo')
        self.initial_selection_rule=self.selection_rule

    def reset(self,time):
        self.rng=Random(self.seed)
        self.set_mode(self.initial_mode)
        super().reset(time)

    def select_by_rule(self,choices,rule=None):
        if rule is not None:
            return super().select_by_rule(choices,rule)
        if self.control_mode=='random':
            return self.rng.choice(tuple(choices))
        return super().select_by_rule(choices,
                'longest_remaining_distance' if self.control_mode=='length' else 'fifo')

    def remaining_length(self,name):
        points=self.roadmap.remaining_subpath(name).coordinates()
        return sum(dist(a,b) for a,b in zip(points,points[1:]))

    def wait_action(self):
        if not self.pending_wait:
            raise ValueError('No optional wait to resolve.')
        if self.wait_rule=='fifo':
            return old.SEND
        if self.wait_rule=='random':
            return self.rng.choice((old.SEND,old.WAIT))
        if self.wait_rule=='length':
            a,b=self.pending_wait['a'],self.pending_wait['b']
            return old.SEND if self.remaining_length(a)>self.remaining_length(b) else old.WAIT
        raise ValueError('Lookahead requires external branch evaluation.')

    def reconsider(self,time):
        super().reconsider(time)
        if self.control_mode!='lookahead' and self.pending_wait and not self.pending_choice:
            self.resolve_wait(self.wait_action(),time)

    def get_snapshot(self):
        row=super().get_snapshot()
        row.update(control_mode=self.control_mode,wait_rule=self.wait_rule,random_state=self.rng.getstate())
        return row

    def set_snapshot(self,state):
        state=deepcopy(state)
        random_state=state.pop('random_state')
        super().set_snapshot(state)
        self.rng.setstate(random_state)


def decision_input(controller,event):
    if event is not None and event.port=='?priority':
        controller.traffic_manager.choose(event.value,controller.tNow)
        controller.phase='Command' if controller.traffic_manager.command_buffer else 'Wait'
        return True
    return old.wait_input(controller,event)


def build(case,policy,seed=0,collision=False):
    model=old.build(case,collision)
    controller=model.controller
    original=controller.traffic_manager
    mode='lookahead' if policy.startswith('lookahead_') else policy
    controller.traffic_manager=UnifiedPolicyManager(
        controller.roadmap,original.tuple_path,pairs=original.pairs,mode=mode,seed=seed,
        reference_predecessors=original.plan_predecessors,event_release=True,
        footprints=original.footprints,command_blocks=original.command_blocks)
    if '?priority' not in controller.IP:
        controller.add_input_ports('?priority')
    controller.delta_x=MethodType(decision_input,controller)
    model.Delta_y=MethodType(stable_output,model)
    return model


def pending(manager):
    if manager.pending_choice:
        return 'waiting_priority'
    if manager.pending_wait:
        return 'wait_or_go'
    return None


def current_action(manager,kind):
    return (manager.select_by_rule(manager.pending_choice) if kind=='waiting_priority'
            else manager.wait_action())


def apply(engine,kind,action):
    port='?priority' if kind=='waiting_priority' else '?wait_action'
    engine.execute_devs_x(engine.tNow,engine.model.controller,PortValue(port,action))


def action_name(manager,kind,action):
    return manager.tasks[action][1] if kind=='waiting_priority' else action


def trace(manager):
    return dict(wait_actions=deepcopy(manager.wait_actions),
                waiting_choices=deepcopy(manager.choice_history))


def finish(engine,budget=200000):
    manager=engine.model.controller.traffic_manager
    steps=0
    while isfinite(engine.model.tNext):
        if pending(manager):
            raise AssertionError('Nested lookahead is forbidden.')
        engine.execute_one_step()
        steps+=1
        if steps>=budget:
            return old.result(engine,steps,'event_budget')
    return old.result(engine,steps)


def run(case,policy,seed=0,collision=False,model_factory=None):
    if policy not in old.POLICIES:
        raise ValueError('Unknown comparison policy.')
    setup_started=perf_counter()
    factory=build if model_factory is None else model_factory
    engine=TESEnginBase(factory(case,policy,seed,collision),TimeIntervalContinuous=inf,
                        TimeIntervalAnimation=.01 if collision else inf)
    engine.bVerbose=False
    manager=engine.model.controller.traffic_manager
    setup_seconds=perf_counter()-setup_started
    steps=0
    decisions=[]
    started=perf_counter()
    while isfinite(engine.model.tNext) or pending(manager):
        kind=pending(manager)
        if kind:
            if not policy.startswith('lookahead_'):
                raise AssertionError('Fixed policy left an unresolved decision.')
            decision_started=perf_counter()
            snapshot=engine.capture_snapshot()
            virtual=replace(snapshot,t_next_continuous=inf,t_next_animation=inf)
            branches=[]
            if kind=='wait_or_go':
                actions=(old.SEND,old.WAIT)
            else:
                actions=tuple(dict.fromkeys(manager.select_by_rule(manager.pending_choice,rule)
                              for rule in ('fifo','longest_remaining_distance')))
            try:
                engine.model.collision_check=False
                engine.TimeIntervalAnimation=inf
                for action,wait_rule,mode in product(actions,('fifo','length'),('fifo','length')):
                    branch_started=perf_counter()
                    engine.restore_snapshot(virtual)
                    manager.set_mode(mode,wait_rule=wait_rule)
                    label=action_name(manager,kind,action)
                    apply(engine,kind,action)
                    outcome=finish(engine)
                    branches.append(dict(mode=mode,wait_rule=wait_rule,action=action,action_name=label,
                                         fixed_LP_continuation=(wait_rule=='fifo' and mode=='length'),
                                         wall_seconds=perf_counter()-branch_started,
                                         execution_trace=trace(manager),**outcome))
            finally:
                engine.model.collision_check=collision
                engine.TimeIntervalAnimation=.01 if collision else inf
                engine.restore_snapshot(snapshot)
            feasible=[b for b in branches if b['completed']]
            if not feasible:
                row=old.result(engine,steps,'no_completed_rollout')
                break
            metric='makespan' if policy=='lookahead_makespan' else 'mean_completion'
            chosen=min(feasible,key=lambda b:(b[metric],actions.index(b['action']),
                                             b['wait_rule']!='fifo',b['mode']!='fifo'))
            decisions.append(dict(time=engine.tNow,kind=kind,selected_mode=chosen['mode'],
                                  selected_wait_rule=chosen['wait_rule'],
                                  action_name=chosen['action_name'],action=chosen['action'],
                                  branches=branches,wall_seconds=perf_counter()-decision_started))
            apply(engine,kind,chosen['action'])
        else:
            engine.execute_one_step()
            steps+=1
            if steps>=200000:
                row=old.result(engine,steps,'event_budget')
                break
    else:
        row=old.result(engine,steps)
    row.update(case=case,policy=policy,seed=seed,decisions=decisions,execution_trace=trace(manager),
               nested_lookahead=False,rollout_horizon='system_completion',
               rollout_policy='independent_current_action_future_wait_rule_and_waiting_priority',
               includes_previous_fixed_LP_continuation=True,
               lookahead_calls=len(decisions),rollouts=sum(len(d['branches']) for d in decisions),
               wait_decisions=sum(d['kind']=='wait_or_go' for d in decisions),
               priority_decisions=sum(d['kind']=='waiting_priority' for d in decisions),
               wall_seconds=perf_counter()-started,
               setup_seconds=setup_seconds,total_seconds=perf_counter()-setup_started,
               decision_seconds=sum(d['wall_seconds'] for d in decisions))
    return row


def signature(row):
    state=row['execution_trace']
    return (tuple((v['time'],v['a'],v['action']) for v in state['wait_actions']),
            tuple((v['time'],tuple(v['candidates']),v['selected']) for v in state['waiting_choices']))


def validate(reference,model_factory=None):
    checked=run(reference['case'],reference['policy'],reference['seed'],True,model_factory=model_factory)
    assert checked['completed']==reference['completed'] and checked['contacts']==0
    for key in ('makespan','mean_completion','completion_std'):
        assert isclose(checked[key],reference[key],rel_tol=0,abs_tol=1e-9),(key,checked[key],reference[key])
    assert signature(checked)==signature(reference)
    assert [(d['kind'],d['selected_mode'],d['selected_wait_rule'],d['action_name']) for d in checked['decisions']]==[
        (d['kind'],d['selected_mode'],d['selected_wait_rule'],d['action_name']) for d in reference['decisions']]
    return dict(completed=checked['completed'],contacts=checked['contacts'],interval=.01,
                stop_on_collision=True,signature_matches=True)


def experiment(case,repetitions=20,model_factory=None,progress=False):
    policies=[]
    for policy in old.POLICIES:
        if progress:
            print(case,policy,'starting',flush=True)
        trials=[run(case,policy,seed,model_factory=model_factory) for seed in (range(repetitions) if policy=='random' else (0,))]
        checked={}
        for trial in trials:
            assert trial['completed'],trial
            key=signature(trial)
            if key not in checked:
                checked[key]=validate(trial,model_factory=model_factory)
            trial['validation']=checked[key]
        metrics={key:dict(mean=mean(t[key] for t in trials),across_run_std=pstdev(t[key] for t in trials))
                 for key in ('makespan','mean_completion','completion_std','wall_seconds','setup_seconds','total_seconds','decision_seconds')}
        policies.append(dict(policy=policy,repetitions=len(trials),metrics=metrics,trials=trials,
                             distinct_validated_trajectories=len(checked)))
        if progress:
            print(case,policy,'completed',round(metrics['makespan']['mean'],6),
                  'runtime_s',round(metrics['total_seconds']['mean'],6),flush=True)
    return dict(case=case,policies=policies,
                scope='expanded_product_including_previous_fixed_LP; old_paired_rule_results_preserved',
                completion_std_definition='population std across objects within each run',
                across_run_std_definition='population std across random seeds')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',nargs='+',choices=('short_A','medium_A','long_A','mixed_four'),
                        default=('short_A','medium_A','long_A','mixed_four'))
    parser.add_argument('--random-repetitions',type=int,default=20)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results'/'unified_policy_expanded.json')
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
            print(case,policy['policy'],{k:round(v['mean'],6) for k,v in policy['metrics'].items()},
                  'decisions',[(t['wait_decisions'],t['priority_decisions']) for t in policy['trials'][:1]],flush=True)


if __name__=='__main__':
    main()
