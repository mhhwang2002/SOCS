"""Event-time lookahead ONLY for competing eligible waiting objects.

Real execution repeatedly looks ahead; each virtual execution uses its chosen
fixed rule at every contention and never nests lookahead. Each rollout goes
to system completion, with no horizon cut.
Prototype uses a separate ordinary manager, not production TrafficManager.
"""

import argparse
from copy import deepcopy
from itertools import product
from dataclasses import replace
from importlib import import_module
import json
from math import inf,isfinite,isclose
from pathlib import Path
from time import perf_counter

from socs.devs.TESEngin import TESEnginBase
from ..controlled import create_controlled_model
from .waiting_choice import attach,commit
from .run_lookahead_comparison import make_candidates
from ..wait_duplication import WaitDuplicationSolver
from ..wait_duplication import SearchResult
from ..path_geometry import TuplePath
from ..GridPath import make_grid


PATHS = {
    'R1': ((0,1),(1,1),(2,1),(3,1),(3,0)),
    'R2': ((1,0),(1,1),(0,1),(0,2)),
    'R3': ((1,3),(1,2),(1,1),(2,1),(2,0)),
}

RULES=('fifo','longest_remaining_distance','shortest_remaining_distance')
DEFAULT_RULES=('fifo','longest_remaining_distance')


def rule_contrast_paths():
    return {**PATHS,'R3':((1,3),(1,2),(1,1),(2,1),(2,0),(1,0),(0,0))}


def parallel_group_result(solver,groups):
    """Combine independently validated disjoint groups, padding with point waits."""
    length=max(len(result.tuple_path.stages) for result in groups)
    parts={name:(result,i) for result in groups
           for i,name in enumerate(result.tuple_path.object_names)}
    rows=[]
    states=[]
    for k in range(length+1):
        states.append(tuple(parts[name][0].consumed_counts[min(k,len(parts[name][0].tuple_path.stages))][parts[name][1]]
                            for name in solver.names))
        if k==length:
            break
        row=[]
        for name,path in zip(solver.names,solver.paths):
            result,i=parts[name]
            row.append(result.tuple_path.stages[k][i] if k<len(result.tuple_path.stages)
                       else path.point(len(path.vertices)-1))
        rows.append(tuple(row))
    result=SearchResult('feasible',TuplePath(solver.names,tuple(rows)),tuple(states),
                        len(states),len(rows),0.,'Combined disjoint three-object witnesses.')
    solver.validate(result)
    return result


def build_six(collision):
    local=rule_contrast_paths()
    routes={**local,**{f'R{int(name[1:])+3}':tuple((x+4,y+4) for x,y in path)
                      for name,path in local.items()}}
    group_candidates=[]
    def builder(solver):
        group_candidates.clear()
        for names in (('R1','R2','R3'),('R4','R5','R6')):
            sub=WaitDuplicationSolver({name:solver.paths[solver.names.index(name)] for name in names},
                                      {name:solver.footprints[solver.names.index(name)] for name in names})
            first=sub.solve()
            candidates,_=make_candidates(sub,first,budget=12)
            group_candidates.append([result for _,result in candidates])
        return parallel_group_result(solver,[choices[0] for choices in group_candidates])
    model=create_controlled_model(routes,grid=make_grid(8),initial_headings={'R4':90},
                                  merge_sweeps=True,event_release=True,collision_check=collision,
                                  stop_on_collision=True,plan_builder=builder)[0]
    manager=model.controller.traffic_manager
    solver=WaitDuplicationSolver(manager.roadmap.paths,manager.footprints)
    combined=[parallel_group_result(solver,choice).tuple_path for choice in product(*group_candidates)]
    model.waiting_reference_plans=tuple(dict.fromkeys(combined))
    return model


def outcome(engine,model,steps,reason=None):
    manager=model.controller.traffic_manager
    complete=(manager.done and not manager.active_tasks and not manager.command_buffer
              and not manager.awaiting and not model.collision_history)
    goals=all(record['released_progress'].segment==len(manager.roadmap.paths[name].vertices)-1
              for name,record in manager.roadmap.robots.items())
    complete=complete and goals
    last_arrivals={event['robot']:event['time'] for event in manager.roadmap.history
                   if event['event']=='arrived'}
    completion_times={name:(last_arrivals.get(name,0. if len(path.vertices)==1 else None))
                      for name,path in manager.roadmap.paths.items()}
    if complete and any(value is None for value in completion_times.values()):
        raise AssertionError('Completed path is missing its final arrival record.')
    mean_completion=(sum(completion_times.values())/len(completion_times) if complete else None)
    return dict(completed=complete,makespan=engine.tNow if complete else None,
                mean_completion=mean_completion,completion_times=completion_times,
                stopped_at=engine.tNow,contacts=len(model.collision_history),steps=steps,
                reason=None if complete else (reason or 'stalled'),commands=manager.next_command_id)


def rollout(engine,model,budget=200000):
    manager=model.controller.traffic_manager
    steps=0
    while isfinite(model.tNext):
        if manager.pending_choice:
            raise AssertionError('Nested lookahead is forbidden in a rollout.')
        engine.execute_one_step()
        steps+=1
        if steps>=budget:
            return outcome(engine,model,steps,'event_budget')
    return outcome(engine,model,steps)


def run_model(model,*,lookahead=True,interval=inf,rollout_rules=DEFAULT_RULES,fixed_rule='fifo',
              objective='makespan'):
    if objective not in ('makespan','mean_completion'):
        raise ValueError('Unknown objective: '+str(objective))
    if not rollout_rules or any(rule not in RULES for rule in rollout_rules):
        raise ValueError('rollout_rules must contain supported rule names.')
    controller=attach(model,lookahead,selection_rule=fixed_rule)
    engine=TESEnginBase(model,TimeIntervalContinuous=inf,TimeIntervalAnimation=interval)
    engine.bVerbose=False
    decisions=[]
    steps=0
    started=perf_counter()
    while isfinite(model.tNext) or controller.traffic_manager.pending_choice:
        manager=controller.traffic_manager
        if manager.pending_choice:
            assert lookahead
            candidates=manager.pending_choice
            assert len(candidates)>=2
            assert all(manager.tasks[i][1] in manager.wait_since for i in candidates)
            decision_started=perf_counter()
            capture_started=perf_counter()
            snapshot=engine.capture_snapshot()
            capture_seconds=perf_counter()-capture_started
            virtual=replace(snapshot,t_next_continuous=inf,t_next_animation=inf)
            original_collision=model.collision_check
            original_interval=engine.TimeIntervalAnimation
            branches=[]
            try:
                model.collision_check=False
                engine.TimeIntervalAnimation=inf
                for rule in dict.fromkeys(rollout_rules):
                    branch_started=perf_counter()
                    engine.restore_snapshot(virtual)
                    manager.lookahead_enabled=False
                    manager.selection_rule=rule
                    index=manager.select_by_rule(candidates)
                    history_start=len(manager.choice_history)
                    commit(engine,controller,index)
                    result=rollout(engine,model)
                    branches.append(dict(task=index,robot=manager.tasks[index][1],rule=rule,
                                         subsequent_choices=deepcopy(manager.choice_history[history_start+1:]),
                                         wall_seconds=perf_counter()-branch_started,**result))
            finally:
                engine.TimeIntervalAnimation=original_interval
                model.collision_check=original_collision
                engine.restore_snapshot(snapshot)
            feasible=[row for row in branches if row['completed']]
            selected=min(feasible,key=lambda row:(row[objective],rollout_rules.index(row['rule']))) if feasible else None
            decisions.append(dict(time=engine.tNow,candidates=[manager.tasks[i][1] for i in candidates],
                                  waiting_since={manager.tasks[i][1]:manager.wait_since[manager.tasks[i][1]] for i in candidates},
                                  fifo=manager.tasks[candidates[0]][1],
                                  selected=selected['robot'] if selected else None,
                                  selected_rule=selected['rule'] if selected else None,
                                  differs_from_fifo=selected is not None and selected['task']!=candidates[0],
                                  capture_seconds=capture_seconds,
                                  wall_seconds=perf_counter()-decision_started,branches=branches))
            if selected is None:
                result=outcome(engine,model,steps,'no_completed_rollout')
                break
            commit(engine,controller,selected['task'])
        else:
            engine.execute_one_step()
            steps+=1
            if steps>=200000:
                result=outcome(engine,model,steps,'event_budget')
                break
    else:
        result=outcome(engine,model,steps)
    result.update(wall_seconds=perf_counter()-started,decisions=decisions,
                  lookahead_calls=len(decisions),rollouts=sum(len(d['branches']) for d in decisions),
                  decision_seconds=sum(d['wall_seconds'] for d in decisions),
                  max_decision_seconds=max((d['wall_seconds'] for d in decisions),default=0),
                  choice_history=controller.traffic_manager.choice_history,
                  nested_lookahead=False,rollout_horizon='system_completion',
                  rollout_policy='same_fixed_rule_for_current_and_all_future_contentions',
                  rollout_rules=list(rollout_rules),fixed_rule=fixed_rule,objective=objective)
    return result


def build(case,collision=False):
    if case=='waiting_six_two_groups':
        return build_six(collision)
    if case in ('waiting_three','waiting_three_rule_contrast'):
        routes=PATHS if case=='waiting_three' else rule_contrast_paths()
        model=create_controlled_model(routes,merge_sweeps=True,event_release=True,
                                      collision_check=collision,stop_on_collision=True)[0]
    else:
        source=import_module('examples.RMFS.Tests.'+case)
        if case in ('10_03_RobotPath_EventRelease','20_03_RobotPath_EventRelease'):
            model=source.create_model(collision_check=collision,stop_on_collision=True)[0]
        else:
            model=source.create_model(merge_sweeps=True,event_release=True,
                                      collision_check=collision,stop_on_collision=True)[0]
    manager=model.controller.traffic_manager
    solver=WaitDuplicationSolver(manager.roadmap.paths,manager.footprints)
    candidates,_=make_candidates(solver,model.plan_result,budget=12)
    model.waiting_reference_plans=tuple(result.tuple_path for _,result in candidates)
    return model


def experiment(case,rollout_rules=DEFAULT_RULES,objective='makespan'):
    baseline=run_model(build(case),lookahead=False)
    lrpf=run_model(build(case),lookahead=False,fixed_rule='longest_remaining_distance')
    selected=run_model(build(case),lookahead=True,rollout_rules=rollout_rules,objective=objective)
    validation=run_model(build(case,True),lookahead=True,interval=.01,rollout_rules=rollout_rules,objective=objective)
    assert validation['completed']==selected['completed']
    if selected['completed']:
        assert isclose(validation['makespan'],selected['makespan'],rel_tol=0,abs_tol=1e-9), (validation['makespan'],selected['makespan'])
        assert isclose(validation['mean_completion'],selected['mean_completion'],rel_tol=0,abs_tol=1e-9)
    assert len(validation['decisions'])==len(selected['decisions'])
    assert all(a['selected']==b['selected'] and isclose(a['time'],b['time'],rel_tol=0,abs_tol=1e-9)
               for a,b in zip(validation['decisions'],selected['decisions']))
    assert [d['selected_rule'] for d in validation['decisions']]==[d['selected_rule'] for d in selected['decisions']]
    assert validation['contacts']==0
    if baseline['completed']:
        baseline_validation=run_model(build(case,True),lookahead=False,interval=.01)
        assert baseline_validation['completed'] and baseline_validation['contacts']==0
        assert isclose(baseline_validation['makespan'],baseline['makespan'],rel_tol=0,abs_tol=1e-9)
        assert isclose(baseline_validation['mean_completion'],baseline['mean_completion'],rel_tol=0,abs_tol=1e-9)
    else:
        baseline_validation=None
    lrpf_validation=run_model(build(case,True),lookahead=False,interval=.01,
                             fixed_rule='longest_remaining_distance')
    assert lrpf_validation['completed']==lrpf['completed'] and lrpf_validation['contacts']==0
    if lrpf['completed']:
        assert isclose(lrpf_validation['makespan'],lrpf['makespan'],rel_tol=0,abs_tol=1e-9)
        assert isclose(lrpf_validation['mean_completion'],lrpf['mean_completion'],rel_tol=0,abs_tol=1e-9)
    return dict(case=case,objective=objective,baseline=baseline,lookahead=selected,validation=validation,
                baseline_validation=baseline_validation,lrpf=lrpf,lrpf_validation=lrpf_validation,
                scope='experimental_uncommitted_priorities_not_production_reference_order',
                improvement_percent=(100*(baseline['makespan']-selected['makespan'])/baseline['makespan']
                                     if baseline['completed'] and selected['completed'] else None))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--examples',nargs='+',default=['waiting_three_rule_contrast','waiting_six_two_groups','10_03_RobotPath_EventRelease','20_03_RobotPath_EventRelease'])
    parser.add_argument('--rollout-rules',nargs='+',choices=RULES,default=DEFAULT_RULES)
    parser.add_argument('--objective',choices=('makespan','mean_completion'),default='makespan')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    args.output=args.output or Path(__file__).parent/'results'/('waiting_rule_comparison_'+args.objective+'.json')
    rows=[]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    for case in args.examples:
        row=experiment(case,tuple(args.rollout_rules),objective=args.objective)
        rows.append(row)
        args.output.write_text(json.dumps(rows,indent=2),encoding='utf-8')
        print(case,'FIFO',row['baseline']['makespan'],'LRPF',row['lrpf']['makespan'],'lookahead',row['lookahead']['makespan'],
              'mean completion',[row[key]['mean_completion'] for key in ('baseline','lrpf','lookahead')],
              'calls',row['lookahead']['lookahead_calls'],'rollouts',row['lookahead']['rollouts'],
              'reason',row['lookahead']['reason'],flush=True)


if __name__=='__main__':
    main()
