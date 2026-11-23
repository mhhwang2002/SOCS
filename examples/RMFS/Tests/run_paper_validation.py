"""Validate FIFO/LP at both decision types using complete tuple certificates.

Main runs are DEVS-only. A separate 0.01 s collision-stop replay must agree.
Every alternative certificate has the same fixed paths and is validated.
No timed forecast simulator is used by either control policy.
"""
import argparse
from importlib import import_module
import json
import sys
from math import atan2,degrees,inf,isfinite,isclose
from pathlib import Path
from time import perf_counter
from types import MethodType

from socs.devs.TESEngin import TESEnginBase
from ..controlled import create_controlled_model
from ..GridPath import make_grid
from ..MRobot import MRobot
from ..PSensor import PSensor
from ..wait_duplication import WaitDuplicationSolver
from .run_generated_policy import factory as policy_factory,ResidualPolicyManager,automatic_gates
from .generate_path_data import load_witness,measure
from .waiting_choice import stable_output


FIXTURES=(
 '02_01_RobotPath_Crossing_NoControl','02_02_RobotPath_LeftEdge_NoControl',
 '02_03_RobotPath_SameDirection_Collision','02_04_RobotPath_SameDirection_Clear',
 '02_06_RobotPath_OppositeDirection_Clear','02_07_RobotPath_OppositeDirection_EntryWait',
 '02_08_RobotPath_DisjointSweeps','02_09_RobotPath_FirstMoverMatters',
 '02_10_RobotPath_AlternatingProgress','02_11_RobotPath_SharedCorner',
 '03_02_RobotPath_ThreeFollowing_Collision','03_04_RobotPath_SweepInterference',
 '03_05_RobotPath_LocalSafe_Deadlock','05_01_RobotPath_TheoremSafeEndpoints',
 '06_01_RobotPath_Convoy','08_02_RobotPath_InteriorTraffic',
 '10_03_RobotPath_EventRelease','18_01_RobotPath_Convoy',
 '20_03_RobotPath_EventRelease','38_01_RobotPath_Convoy',
 '50_01_RobotPath_InteriorTraffic')


def saved_factory(data,index):
    policy_build,_=policy_factory(data,index)
    sample=next(s for s in data['samples'] if s['index']==index)
    routes={name:tuple(map(tuple,sample['routes'][name])) for name in sample['object_names']}
    cfg=data['config']
    headings={n:degrees(atan2(p[1][1]-p[0][1],p[1][0]-p[0][0])) for n,p in routes.items()}
    def build(policy,collision):
        model=policy_build('paper','fifo' if policy=='fifo' else 'length',collision=collision)
        return model,tuple(o for o in model.M if isinstance(o,MRobot)),tuple(o for o in model.M if isinstance(o,PSensor))
    return build,sample['metrics']


def fixture_factory(name):
    source=import_module('examples.RMFS.Tests.'+name)
    def build(policy,collision):
        opts=dict(collision_check=collision,stop_on_collision=True)
        if len(source.PATHS)<=3:
            result=create_controlled_model(source.PATHS,grid=getattr(source,'GRID',None),
                initial_headings=getattr(source,'INITIAL_HEADINGS',None),
                merge_sweeps=True,event_release=True,**opts)
        else:
            if name not in ('10_03_RobotPath_EventRelease','20_03_RobotPath_EventRelease'):
                opts.update(merge_sweeps=True,event_release=True)
            result=source.create_model(**opts)
        model,robots,sensors=result
        controller=model.controller;original=controller.traffic_manager
        solver=WaitDuplicationSolver(controller.roadmap.paths,original.footprints)
        metrics=measure(controller.roadmap.paths,next(iter(original.footprints.values())),original.policy)
        controller.traffic_manager=ResidualPolicyManager(controller.roadmap,original.tuple_path,
            pairs=(),residual_gates=automatic_gates(solver,metrics),mode='fifo' if policy=='fifo' else 'length',
            alternative_plans=(original.tuple_path,),event_release=True,
            footprints=original.footprints,command_blocks=original.command_blocks)
        return result
    return build


def run(build,policy,collision=False):
    started=perf_counter()
    model,robots,sensors=build(policy,collision)
    model.Delta_y=MethodType(stable_output,model)
    setup=perf_counter()-started
    engine=TESEnginBase(model,TimeIntervalContinuous=inf,
                       TimeIntervalAnimation=.01 if collision else inf)
    engine.bVerbose=False
    steps=0;max_waiting=0
    while isfinite(model.tNext):
        engine.execute_one_step();steps+=1
        manager=model.controller.traffic_manager
        max_waiting=max(max_waiting,len(manager.task_wait_since))
        if steps>2000000:
            raise RuntimeError('Event budget exhausted; no infeasibility conclusion.')
    manager=model.controller.traffic_manager
    assert manager.done and not manager.awaiting and not manager.active_tasks and not manager.command_buffer
    assert not model.collision_history and all(o.stop_on_collision for o in (*robots,*sensors))
    for robot,sensor in zip(robots,sensors):
        endpoint=manager.roadmap.paths[robot.name].vertices[-1]
        assert robot.phase=='Wait' and sensor.phase=='Idle'
        assert robot.position_at(engine.tNow)==endpoint
        assert manager.roadmap.robots[robot.name]['approved_path'] is None
    arrived={event['robot']:event['time'] for event in manager.roadmap.history if event['event']=='arrived'}
    # Every approved command must be emitted and acknowledged exactly once.
    lifecycle={}
    for event in manager.roadmap.history:
        if event['event'] in ('approved','issued','arrived'):
            lifecycle.setdefault(event['command_id'],[]).append(event['event'])
    assert all(v==['approved','issued','arrived'] for v in lifecycle.values())
    values=list(arrived.values())
    mean=sum(values)/len(values)
    return dict(completed=True,makespan=engine.tNow,mean_completion=mean,
        completion_std=(sum((v-mean)**2 for v in values)/len(values))**.5,
        completion_times=arrived,objects=len(robots),steps=steps,commands=manager.next_command_id,
        max_waiting=max_waiting,contacts=len(model.collision_history),
        setup_seconds=setup,total_seconds=perf_counter()-started,
        wait_actions=manager.wait_actions,waiting_choices=manager.choice_history,
        retained_certificates=len(manager.viable_plans),
        command_trace=[(e['time'],e['robot'],e['command_id']) for e in manager.roadmap.history if e['event']=='issued'])


def evaluate(name,build,metrics=None):
    if metrics is None:
        model,_,_=build('fifo',False)
        manager=model.controller.traffic_manager
        shapes=list(manager.footprints.values())
        assert all(shape==shapes[0] for shape in shapes)
        metrics=measure(manager.roadmap.paths,shapes[0],manager.policy)
    row=dict(case=name,geometry=metrics,policies={})
    for policy in ('fifo','lp'):
        print(name,policy,'starting',flush=True)
        result=run(build,policy)
        for choice in result['wait_actions']:
            expected=('wait_for_B' if policy=='lp' and choice['remaining_b_at_gate_m']>choice['remaining_a_m'] else 'send_now')
            assert choice['action']==expected,(name,policy,choice)
        checked=run(build,policy,True)
        assert checked['command_trace']==result['command_trace']
        for key in ('makespan','mean_completion','completion_std'):
            assert isclose(checked[key],result[key],abs_tol=1e-8,rel_tol=0),(name,policy,key)
        result['validation']=dict(completed=True,contacts=0,interval=.01,
            stop_on_collision=True,matching_trace=True,seconds=checked['total_seconds'])
        row['policies'][policy]=result
        print(name,policy,'PASS',round(result['makespan'],6),'seconds',round(result['total_seconds'],3),flush=True)
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results/paper_fifo_lp_validation.json')
    parser.add_argument('--include-200',action='store_true')
    parser.add_argument('--fixtures-only',action='store_true')
    parser.add_argument('--animate',choices=(*FIXTURES,'200_01_RobotPath_InteriorTraffic'))
    parser.add_argument('--animate-density',type=int,choices=(4,8,12,16),
                        help='animate a saved 12x12 grid case with this object count')
    parser.add_argument('--sample',type=int,default=1,help='one-based generated case number')
    parser.add_argument('--policy',choices=('fifo','lp'),default='fifo')
    args,viewer_args=parser.parse_known_args()
    if args.animate and args.animate_density:
        parser.error('Choose --animate or --animate-density.')
    if args.animate or args.animate_density:
        if args.animate_density:
            data_path=Path(__file__).parent/f'data/generated/density12/objects_{args.animate_density}.json'
            data=json.loads(data_path.read_text(encoding='utf-8'))
            sample=next((s for s in data['samples'] if s['index']==args.sample-1),None)
            if sample is None:parser.error('No such saved sample.')
            routes={n:tuple(map(tuple,p)) for n,p in sample['routes'].items()}
            grid=make_grid(data['config']['grid_size'],data['config']['spacing_m'])
            headings=None
            build,_=saved_factory(data,args.sample-1)
            title=f'Density12_{args.animate_density}_case{args.sample}'
        else:
            source=import_module('examples.RMFS.Tests.'+args.animate)
            routes=source.PATHS;grid=getattr(source,'GRID',None)
            headings=getattr(source,'INITIAL_HEADINGS',None)
            build=fixture_factory(args.animate)
            title=args.animate
        sys.argv=[sys.argv[0],*viewer_args]
        print(f'Paper controller: {args.policy.upper()} | {title}',flush=True)
        import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
            name=f'Paper_{args.policy.upper()}_{title}',no_rack=True,
            robot_paths=routes,grid=grid,
            initial_headings=headings,
            collision_check=False,stop_on_collision=True,
            hide_robot_names=len(routes)>=50,
            hide_motion_labels=len(routes)>=50,
            render_fps=10 if len(routes)>=50 else 30,
            model_factory=lambda **options:build(args.policy,options.get('collision_check',True)))
        return
    if viewer_args:
        parser.error('Unknown options: '+' '.join(viewer_args))
    report=dict(scope='validated_completion_certificates; FIFO_and_LP_at_both_decision_types; LP_compares_remaining_length_at_waiting_positions',
                collision_interval=.01,stop_on_collision=True,status='running',results=[])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    def save(row):
        report['results'].append(row)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    root=Path(__file__).parent/'data/generated'
    if not args.fixtures_only:
        for path in [*(root/'density12'/f'objects_{n}.json' for n in (4,8,12,16)),root/'complex.json',root/'random_trial.json']:
            data=json.loads(path.read_text(encoding='utf-8'))
            for sample in data['samples']:
                build,metrics=saved_factory(data,sample['index'])
                row=evaluate(f'{path.stem}_{sample["index"]+1}',build,metrics)
                row.update(dataset=str(path),path_hash=sample['path_hash'],grid_size=data['config']['grid_size'])
                save(row)
    for name in (*FIXTURES,*(('200_01_RobotPath_InteriorTraffic',) if args.include_200 else ())):
        save(evaluate(name,fixture_factory(name)))
    report['status']='complete'
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('COMPLETE',len(report['results']),'cases',args.output,flush=True)


if __name__=='__main__':
    main()
