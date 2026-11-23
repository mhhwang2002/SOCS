"""Compare first valid witnesses with initial-snapshot makespan selection.

Both arms retain identical Plant, geometry, headings, safe merging and event
release. Only the reference tuple choice changes. Candidate budgets are finite;
reported improvements are best-of-generated-candidates, not global optimality.
Original uncontrolled animations and their files are never modified.
"""

import argparse
from dataclasses import replace
from hashlib import sha256
from importlib import import_module
from itertools import permutations
import json
from math import dist, inf
from pathlib import Path
from random import Random
from time import perf_counter

from socs.devs.TESEngin import TESEnginBase
from ..controlled import create_controlled_model
from ..path_geometry import TuplePath, sweep_2d, PathPosition, Subpath
from ..TrafficManager import TrafficManager
from ..wait_duplication import WaitDuplicationSolver
from ..sweep_operations import compact_tuple
from .incremental_routing import dag_witness, dependency_order
from .run_priority_lookahead import finish, ordered_witness


LARGE = ('05_01_RobotPath_TheoremSafeEndpoints', '06_01_RobotPath_Convoy',
         '08_02_RobotPath_InteriorTraffic', '10_03_RobotPath_EventRelease',
         '20_03_RobotPath_EventRelease')


def canonical(result, solver):
    indices = [result.tuple_path.object_names.index(name) for name in solver.names]
    result = replace(result, tuple_path=TuplePath(solver.names, tuple(
        tuple(row[i] for i in indices) for row in result.tuple_path.stages)),
        consumed_counts=tuple(tuple(state[i] for i in indices) for state in result.consumed_counts))
    solver.validate(result)
    return result


def requirements(solver):
    whole = {name: sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(path.vertices)-1)),shape,solver.policy)
             for name,path,shape in zip(solver.names,solver.paths,solver.footprints)}
    graph = {name:set() for name in solver.names}
    for i,(name,path,shape) in enumerate(zip(solver.names,solver.paths,solver.footprints)):
        for j,(other,old,other_shape) in enumerate(zip(solver.names,solver.paths,solver.footprints)):
            if i >= j:
                continue
            si = sweep_2d(path.point(0),shape,solver.policy).intersects(whole[other],solver.policy.tolerance)
            ei = sweep_2d(path.point(len(path.vertices)-1),shape,solver.policy).intersects(whole[other],solver.policy.tolerance)
            sj = sweep_2d(old.point(0),other_shape,solver.policy).intersects(whole[name],solver.policy.tolerance)
            ej = sweep_2d(old.point(len(old.vertices)-1),other_shape,solver.policy).intersects(whole[name],solver.policy.tolerance)
            if sj or ei:
                graph[name].add(other)
            if si or ej:
                graph[other].add(name)
    return graph


def make_candidates(solver, baseline, budget=12):
    if budget < 1:
        raise ValueError('Candidate budget must include the baseline.')
    plans = [('first_valid', baseline)]
    seen = {repr(baseline.tuple_path)}
    audit = dict(attempted=0,rejected=0,duplicates=0)
    names = solver.names
    lengths = {name:sum(dist(a,b) for a,b in zip(path.vertices,path.vertices[1:]))
               for name,path in zip(names,solver.paths)}
    orders = [names,tuple(reversed(names)),
              tuple(sorted(names,key=lambda name:(-lengths[name],name))),
              tuple(sorted(names,key=lambda name:(lengths[name],name)))]
    if len(names)<=3:
        orders.extend(permutations(names))
    else:
        rng=Random(20261120)
        for _ in range(8):
            orders.append(tuple(rng.sample(list(names),len(names))))
    orders=tuple(dict.fromkeys(orders))
    graph=requirements(solver)
    dag=dependency_order(graph) is not None
    for order in orders:
        alternative=WaitDuplicationSolver(
            {name:solver.paths[names.index(name)] for name in order},
            {name:solver.footprints[names.index(name)] for name in order},policy=solver.policy)
        builders=[]
        if len(names)<=3:
            builders.append(('bfs_tie_order',lambda: alternative.solve()))
        if dag:
            builders.append(('dag_ready_order',lambda: dag_witness(alternative,graph)))
        builders.append(('serial_priority',lambda: ordered_witness(alternative,order)))
        for label,build in builders:
            if len(plans)>=budget:
                return plans,audit
            audit['attempted']+=1
            try:
                result=build()
                if result.status!='feasible':
                    audit['rejected']+=1
                    continue
                result=canonical(result,solver)
            except ValueError:
                audit['rejected']+=1
                continue
            key=repr(result.tuple_path)
            if key in seen:
                audit['duplicates']+=1
                continue
            seen.add(key)
            plans.append((label+':'+','.join(order),result))
    return plans,audit


def compare_example(name,budget=12):
    started=perf_counter()
    source=import_module('examples.RMFS.Tests.'+name)
    routes=source.PATHS
    headings=getattr(source,'INITIAL_HEADINGS',None)
    grid=getattr(source,'GRID',None)
    try:
        if len(routes)<=3:
            model,robots,sensors=create_controlled_model(
                routes,initial_headings=headings,merge_sweeps=True,event_release=True,collision_check=False)
        elif name in ('10_03_RobotPath_EventRelease','20_03_RobotPath_EventRelease'):
            # These wrappers already fix merging/release to True.
            model,robots,sensors=source.create_model(collision_check=False)
        else:
            model,robots,sensors=source.create_model(
                merge_sweeps=True,event_release=True,collision_check=False)
    except ValueError as error:
        if str(error).startswith('No wait-duplication'):
            return dict(example=name,robots=len(routes),status='no_plan_in_finite_model',
                        baseline_makespan=None,lookahead_makespan=None,reason=str(error))
        raise
    controller=model.controller
    original=controller.traffic_manager
    solver=WaitDuplicationSolver(controller.roadmap.paths,original.footprints)
    baseline=model.plan_result
    generation_started=perf_counter()
    candidates,audit=make_candidates(solver,baseline,budget)
    generation_seconds=perf_counter()-generation_started
    engine=TESEnginBase(model,TimeIntervalContinuous=inf,TimeIntervalAnimation=inf)
    engine.bVerbose=False
    snapshot=engine.capture_snapshot()
    results=[]
    for label,plan in candidates:
        branch_started=perf_counter()
        controller.traffic_manager=original
        engine.restore_snapshot(snapshot)
        assert engine.tNow==0 and all(robot.phase=='Wait' for robot in robots)
        _,blocks=compact_tuple(plan.tuple_path,original.footprints,solver.policy)
        controller.traffic_manager=TrafficManager(
            controller.roadmap,plan.tuple_path,event_release=True,
            footprints=original.footprints,command_blocks=blocks)
        controller.phase='Command' if controller.traffic_manager.command_buffer else 'Wait'
        assert controller.tNext==model.tNext==0
        outcome=finish(engine,model)
        results.append(dict(candidate=label,plan_hash=sha256(repr(plan.tuple_path).encode()).hexdigest(),
                            evaluation_seconds=perf_counter()-branch_started,**outcome))
    if not results[0]['completed']:
        raise AssertionError('Baseline failed: '+name)
    best=min((i for i,r in enumerate(results) if r['completed']),
             key=lambda i:(results[i]['makespan'],i))
    validations={}
    for index in dict.fromkeys((0,best)):
        witness=candidates[index][1]
        checked,check_robots,check_sensors=create_controlled_model(
            routes,initial_headings=headings,grid=grid,merge_sweeps=True,event_release=True,
            collision_check=True,stop_on_collision=True,
            plan_builder=lambda solver,witness=witness:witness)
        validation_engine=TESEnginBase(checked,TimeIntervalContinuous=inf,TimeIntervalAnimation=.01)
        validation_engine.bVerbose=False
        outcome=finish(validation_engine,checked)
        assert outcome['completed'] and outcome['contacts']==0,name
        assert abs(outcome['makespan']-results[index]['makespan'])<1e-9,name
        assert all(obj.stop_on_collision for obj in (*check_robots,*check_sensors))
        validations[str(index)]=outcome
    a,b=results[0]['makespan'],results[best]['makespan']
    return dict(example=name,robots=len(routes),status='completed',candidate_budget=budget,
                branches=len(results),candidates_audit=audit,generation_seconds=generation_seconds,
                baseline_makespan=a,lookahead_makespan=b,improvement_percent=100*(a-b)/a,
                selected=results[best]['candidate'],selected_index=best,
                evaluation_seconds=sum(r['evaluation_seconds'] for r in results),
                baseline_evaluation_seconds=results[0]['evaluation_seconds'],
                baseline_construction_seconds=baseline.elapsed_seconds,
                total_seconds=perf_counter()-started,results=results,validations=validations,
                merge_sweeps=True,event_release=True,collision_interval=.01,stop_on_collision=True,
                initial_snapshot_time=0,paths_changed=False,
                scope='initial_snapshot_best_of_generated_validated_tuples')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--examples',nargs='*')
    parser.add_argument('--candidate-budget',type=int,default=12)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results'/'lookahead_comparison.json')
    args=parser.parse_args()
    names=args.examples or ([p.stem for p in sorted(Path(__file__).parent.glob('0[23]_*.py'))
                             if '_Controlled' not in p.stem]+list(LARGE))
    report=dict(baseline='first_valid_tuple_without_makespan_comparison',
                treatment='best_evaluated_tuple_including_baseline',results=[])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    for name in names:
        row=compare_example(name,args.candidate_budget)
        report['results'].append(row)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(name,row['status'],row['baseline_makespan'],'->',row['lookahead_makespan'],
              'branches=',row.get('branches'),flush=True)
    completed=[r for r in report['results'] if r['status']=='completed']
    report['summary']=dict(cases=len(names),completed=len(completed),
        no_plan=len(names)-len(completed),
        improved=sum(r['improvement_percent']>1e-8 for r in completed),
        equal=sum(abs(r['improvement_percent'])<=1e-8 for r in completed),
        worse=sum(r['improvement_percent'] < -1e-8 for r in completed))
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(report['summary'],flush=True)


if __name__=='__main__':
    main()
