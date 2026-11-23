"""Fresh paired FIFO measurements after the motion/sensor fixes.

Both arms use safe-prefix approach, sweep merging, identical fixed routes and
the same certificate. Only actual-clearance motion extension is toggled.
Timed runs are headless DEVS-only; separate stop-on-collision replays use .1 s.
"""
from argparse import ArgumentParser
from hashlib import sha256
from importlib import import_module
from json import dumps,loads
from math import inf,isfinite
from pathlib import Path
from statistics import mean,median,pstdev
from time import perf_counter
import csv
import platform
from types import MethodType

from socs.devs.TESEngin import TESEnginBase
from ..controlled import create_controlled_model
from ..interior_traffic import create_interior_model
from .incremental_routing import create_experiment_model
from .generate_path_data import measure
from .waiting_choice import stable_output
from .release_policy_comparison import attach_policy

GEOMETRY_CACHE = {}
POLICY_CACHE = {}


def cases():
    result = []
    from .run_wait_or_go import fixture
    from .run_priority_lookahead import ordered_witness
    from .run_waiting_lookahead import parallel_group_result
    from ..wait_duplication import WaitDuplicationSolver
    for fixture_name in ('short_A','medium_A','long_A','mixed_four'):
        def build(enabled,collision,fixture_name=fixture_name):
            routes,grid,pairs,headings = fixture(fixture_name)
            def witness(solver):
                groups = []
                for a,b,_,_ in pairs:
                    local = WaitDuplicationSolver({n:solver.paths[solver.names.index(n)] for n in (a,b)},
                        {n:solver.footprints[solver.names.index(n)] for n in (a,b)})
                    groups.append(ordered_witness(local,(a,b)))
                return parallel_group_result(solver,groups)
            return create_controlled_model(routes,grid=grid,initial_headings=headings,lift_up=False,
                plan_builder=witness,merge_sweeps=True,event_release=True,safe_prefix=True,
                motion_extension=enabled,collision_check=collision,stop_on_collision=True)
        result.append(('optional_'+fixture_name,dict(kind='optional_approach',seed=None),build))
    for number in ('03_08_RobotPath_FIFO_LocalPassage','03_09_RobotPath_FIFO_ReleaseExtension'):
        source = import_module('examples.RMFS.Tests.'+number)
        def build(enabled,collision,source=source):
            return create_controlled_model(source.PATHS,grid=source.GRID,lift_up=False,
                merge_sweeps=True,event_release=True,safe_prefix=True,
                motion_extension=enabled,collision_check=collision,stop_on_collision=True)
        result.append((number,dict(kind='handcrafted',seed=None),build))
    for size,count in ((10,10),(10,20),(12,30)):
        for seed in (20261119,20261120,20261121):
            def build(enabled,collision,size=size,count=count,seed=seed):
                return create_experiment_model(size=size,robot_count=count,seed=seed,lift_up=False,
                    merge_sweeps=True,event_release=True,safe_prefix=True,
                    motion_extension=enabled,collision_check=collision,stop_on_collision=True)
            result.append((f'dag_{size}x{size}_{count}_seed{seed}',
                           dict(kind='incremental_dag',seed=seed),build))
    for size in (4,10):
        def build(enabled,collision,size=size):
            return create_interior_model(size,lift_up=False,merge_sweeps=True,event_release=True,
                safe_prefix=True,motion_extension=enabled,collision_check=collision,stop_on_collision=True)
        result.append((f'interior_{size}x{size}_{size*size//2}',dict(kind='interior',seed=None),build))
    return result


def identity(model):
    c = model.controller
    payload = repr((c.initial_paths,c.traffic_manager.tuple_path,c.initial_headings,
                    c.traffic_manager.command_blocks,c.roadmap.footprints))
    return sha256(payload.encode()).hexdigest()


def run(build,enabled,collision,policy='fifo'):
    started = perf_counter()
    model,robots,sensors = build(enabled,collision)
    signature = identity(model)
    if signature not in GEOMETRY_CACHE:
        manager = model.controller.traffic_manager
        GEOMETRY_CACHE[signature] = measure(model.controller.roadmap.paths,
            next(iter(manager.footprints.values())),manager.policy)
    portfolio_size = attach_policy(model,policy,GEOMETRY_CACHE[signature],POLICY_CACHE)
    model.Delta_y = MethodType(stable_output,model)
    setup = perf_counter()-started
    engine = TESEnginBase(model,TimeIntervalContinuous=inf,
                         TimeIntervalAnimation=.1 if collision else inf)
    engine.bVerbose = False
    started = perf_counter()
    steps = 0
    while isfinite(model.tNext) and steps < 200000 and engine.tNow < 10000:
        engine.execute_one_step()
        steps += 1
    elapsed = perf_counter()-started
    manager = model.controller.traffic_manager
    goals = {name:path.vertices[-1] for name,path in model.controller.roadmap.paths.items()}
    complete = manager.done and not manager.awaiting and all(
        r.phase=='Wait' and r._variable.current_position==goals[r.name] for r in robots)
    arrivals = {name:next((e['time'] for e in model.controller.roadmap.history
                    if e['event']=='arrived' and e['robot']==name and e['position']==goal),None)
                for name,goal in goals.items()}
    counts = {key:sum(r._variable.motion_extension_counts[key] for r in robots)
              for key in robots[0]._variable.motion_extension_counts}
    reservations = sum(e['event']=='extension_reserved' for e in model.controller.roadmap.history)
    assert sum(counts.values()) == reservations, 'Approval and receipt counts differ.'
    assert enabled or reservations == 0
    summary = dict(complete=complete,contacts=len(model.collision_history),
        makespan_s=engine.tNow if complete else None,
        mean_completion_s=mean(arrivals.values()) if complete else None,
        completion_std_s=pstdev(arrivals.values()) if complete else None,
        completion_by_robot=arrivals,extension_counts=counts,extension_commands=reservations,
        intermediate_stops=sum(e['event']=='arrived' and e['position']!=goals[e['robot']]
            for e in model.controller.roadmap.history),wall_s=elapsed,setup_s=setup,steps=steps,
        input_sha256=signature,portfolio_size=portfolio_size,
        optional_waits=len(getattr(manager,'wait_actions',())),
        optional_holds=sum(r['action']=='wait_for_B' for r in getattr(manager,'wait_actions',())),
        arbitration_decisions=len(getattr(manager,'choice_history',())))
    return summary,model


def write_outputs(output,data):
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(dumps(data,indent=2)+'\n',encoding='utf-8')
    rows = []
    for case in data['results']:
        if 'off' not in case:
            continue
        if 'on' not in case:
            off = case['off']
            rows.append(dict(case=case['case'],policy=case['policy'],grid=case['grid'],robots=case['robots'],
                interference_intervals=case['geometry']['directed_pairwise_interval_count'],
                interference_length_m=case['geometry']['total_pairwise_interval_m'],
                makespan_s=off['makespan_s'],mean_completion_s=off['mean_completion_s'],
                completion_std_s=off['completion_std_s'],intermediate_stops=off['intermediate_stops'],
                extension_commands=off['extension_commands'],median_wall_ms=off['median_wall_s']*1000,
                optional_holds=off['optional_holds'],arbitrations=off['arbitration_decisions'],
                validated=case['validated']))
            continue
        off,on = case['off'],case['on']
        rows.append(dict(case=case['case'],policy=case['policy'],grid=case['grid'],robots=case['robots'],
            interference_intervals=case['geometry']['directed_pairwise_interval_count'],
            interference_length_m=case['geometry']['total_pairwise_interval_m'],
            makespan_off_s=off['makespan_s'],makespan_on_s=on['makespan_s'],
            makespan_reduction_pct=case['makespan_reduction_pct'],
            mean_off_s=off['mean_completion_s'],mean_on_s=on['mean_completion_s'],
            mean_reduction_pct=case['mean_reduction_pct'],
            std_off_s=off['completion_std_s'],std_on_s=on['completion_std_s'],
            stops_off=off['intermediate_stops'],stops_on=on['intermediate_stops'],
            extensions=on['extension_commands'],**on['extension_counts'],
            wall_off_ms=off['median_wall_s']*1000,wall_on_ms=on['median_wall_s']*1000,
            validated=case['validated']))
    if rows:
        with output.with_suffix('.csv').open('w',newline='',encoding='utf-8-sig') as file:
            writer = csv.DictWriter(file,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--case',action='append',help='substring filter; repeat to select more cases')
    parser.add_argument('--stop-only',action='store_true',help='Collect only the approved-destination stop baseline')
    parser.add_argument('--include-optional',action='store_true',help='Include additional optional-approach pilot fixtures')
    parser.add_argument('--extract-stop-from',type=Path,help='Retain only freshly measured OFF runs from a completed comparison')
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    if args.output is None:
        args.output = Path(__file__).parent/'results'/(
            'stop_only_baseline_20261121.json' if args.stop_only else 'release_extension_comparison_20261121.json')
    if args.extract_stop_from:
        data = loads(args.extract_stop_from.read_text(encoding='utf-8'))
        if data['status'] != 'complete':
            parser.error('Extraction requires a completed, validated collection.')
        retained = []
        for original in data['results']:
            case = {k:v for k,v in original.items() if k not in
                    ('on','makespan_reduction_pct','mean_reduction_pct')}
            assert case['off']['extension_commands'] == 0
            assert case['off']['collision_replay']['complete'] and case['off']['collision_replay']['contacts']==0
            assert case['validated']
            retained.append(case)
        data.update(results=retained,variation='motion extension OFF only; stop at each approved destination',
                    extraction_source=str(args.extract_stop_from),extension_results_included=False)
        write_outputs(args.output,data)
        print(f'Saved {len(retained)} stop-only policy cases; no extension measurements retained.')
        return
    if args.repeats < 1:
        parser.error('repeats must be positive')
    root = Path(__file__).parents[1]
    source_names = ('MRobot.py','PSensor.py','TrafficManager.py','RoadMap.py','RMFSController.py','controlled.py','motion.py','model.py')
    data = dict(status='running',date='2026-11-21',python=platform.python_version(),
        repeats=args.repeats,policies=['FIFO','LP'],safe_prefix=True,merge_sweeps=True,
        continuous_interval='infinity',timed_run_collision_check=False,
        collision_validation_interval_s=.1,stop_on_collision=True,
        variation='only motion_extension OFF versus ON',
        runtime_scope='engine execution excluding model construction, with RoadMap/event logging; median of repeated runs',
        interference_definition='directed pairwise intervals, counted on both participating paths; lengths in metres',
        priority_scope='FIFO/LP at optional approach waits and multiple-waiter arbitration; committed orders and certificates remain mandatory',
        same_time_order='zero-time controller Command first, remaining simultaneous models by name',
        source_sha256={name:sha256((root/name).read_bytes()).hexdigest() for name in source_names},results=[])
    if args.stop_only:
        data['variation'] = 'motion extension OFF only; stop at each approved destination'
    for name,description,build in [(n,dict(d,policy=p),b) for n,d,b in cases() for p in ('fifo','lp')]:
        if name.startswith('optional_') and not args.include_optional:
            continue
        if args.case and not any(pattern in name for pattern in args.case):
            continue
        print('Running '+name+' '+description['policy'],flush=True)
        case = dict(case=name,**description)
        signature = None
        validated = True
        for enabled,key in (((False,'off'),) if args.stop_only else ((False,'off'),(True,'on'))):
            measurements = []
            for repetition in range(args.repeats):
                result,model = run(build,enabled,False,description['policy'])
                if signature is None:
                    signature = result['input_sha256']
                    layout = model.controller.roadmap.layout['control_points']
                    coordinates = tuple(layout.values())
                    case['grid'] = f'{len({p[0] for p in coordinates})}x{len({p[1] for p in coordinates})}'
                    case['robots'] = len(model.controller.roadmap.paths)
                    case['paths_m'] = {n:p.vertices for n,p in model.controller.roadmap.paths.items()}
                    case['geometry'] = GEOMETRY_CACHE[signature]
                assert result['input_sha256']==signature,'Input changed between paired runs.'
                measurements.append(result)
            reference = measurements[0]
            for result in measurements[1:]:
                assert result['completion_by_robot']==reference['completion_by_robot']
                assert result['extension_counts']==reference['extension_counts']
            reference = dict(reference,median_wall_s=median(r['wall_s'] for r in measurements),
                             repeated_wall_s=[r['wall_s'] for r in measurements])
            replay,_ = run(build,enabled,True,description['policy'])
            assert replay['input_sha256']==signature
            matches = all(replay['completion_by_robot'][n] is not None and t is not None
                          and abs(replay['completion_by_robot'][n]-t)<1e-7
                          for n,t in reference['completion_by_robot'].items())
            passed = replay['complete'] and replay['contacts']==0 and matches
            validated &= passed
            reference['collision_replay'] = replay
            case[key] = reference
        case['validated'] = validated
        if args.stop_only:
            data['results'].append(case)
            write_outputs(args.output,data)
            print(f"  STOP {case['off']['makespan_s']}; validated={validated}",flush=True)
            continue
        for metric,label in (('makespan_s','makespan'),('mean_completion_s','mean')):
            a,b = case['off'][metric],case['on'][metric]
            case[label+'_reduction_pct'] = 100*(a-b)/a if a and b is not None else None
        data['results'].append(case)
        write_outputs(args.output,data)
        print(f"  OFF {case['off']['makespan_s']} -> ON {case['on']['makespan_s']}; "
              f"extensions={case['on']['extension_counts']}; validated={validated}",flush=True)
    data['status'] = 'complete' if all(c['validated'] for c in data['results']) else 'validation_failed'
    write_outputs(args.output,data)
    print(f"Saved {len(data['results'])} paired cases to {args.output}",flush=True)


if __name__ == '__main__':
    main()
