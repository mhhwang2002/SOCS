"""Twenty loaded robots on the RMFS warehouse: ten outbound and ten returns."""
import argparse
import json
from math import atan2,degrees,dist,isfinite
from pathlib import Path
from random import Random
import sys
from types import MethodType

from ..WarehouseLayout import load_layout,shortest_route
from ..controlled import create_controlled_model
from ..path_geometry import Path as MotionPath,PathPosition,Subpath,RectangularFootprint,GeometryPolicy,sweep_2d
from ..wait_duplication import WaitDuplicationSolver
from .generate_path_data import append_requirements,measure
from .incremental_routing import dag_witness
from .run_generated_policy import ResidualPolicyManager,automatic_gates
from .run_priority_lookahead import ordered_witness
from .run_paper_validation import evaluate
from .waiting_choice import stable_output

DATA=Path(__file__).parent/'data/generated/warehouse20_paths.json'


def generate_case(index,seed):
    layout=load_layout();rng=Random(seed+index*1000003)
    shape=RectangularFootprint(1.5,1.5);policy=GeometryPolicy()
    rack_nodes=set(layout['warehouse_racks'].values())
    for attempt in range(1,101):
        incoming_cells=[];return_cells=[]
        for cluster in range(10):
            a,b=rng.sample([cell for cell in layout['warehouse_racks'] if cell[0]==cluster],2)
            incoming_cells.append(a);return_cells.append(b)
        if index%3==0:
            incoming_order=list(range(10));return_order=list(reversed(range(10)))
        elif index%3==1:
            incoming_order=list(reversed(range(10)));return_order=list(range(10))
        else:
            incoming_order=rng.sample(list(range(10)),10);return_order=rng.sample(list(range(10)),10)
        routes={};assignments={}
        for i,(station,position) in enumerate(layout['warehouse_stations'].items()):
            cell=return_cells[return_order[i]];name=f'R{i+1:02}'
            target=layout['warehouse_racks'][cell]
            routes[name]=shortest_route(layout,position,target,rack_nodes)
            assignments[name]=dict(role='station_to_rack',station=station,rack_cell=cell,lift_up=True)
        for i,(station,position) in enumerate(layout['warehouse_stations'].items()):
            cell=incoming_cells[incoming_order[i]];name=f'R{i+11:02}'
            start=layout['warehouse_racks'][cell]
            routes[name]=shortest_route(layout,start,position,rack_nodes)
            assignments[name]=dict(role='rack_to_station',station=station,rack_cell=cell,lift_up=True)
        paths={n:MotionPath(n,p) for n,p in routes.items()};accepted={};sweeps={};graph={}
        empty=set(incoming_cells+return_cells)
        stationary={c:p for c,p in layout['warehouse_racks'].items() if c not in empty}
        reason=None
        for name,path in paths.items():
            whole=sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(path.vertices)-1)),shape,policy)
            if any(whole.intersects(sweep_2d(MotionPath('rack', (p,)).point(0),shape,policy),policy.tolerance)
                   for p in stationary.values()):
                reason='stationary_rack_interference';break
            next_graph,reason=append_requirements(name,path,whole,accepted,sweeps,graph,shape,policy)
            if reason:break
            graph=next_graph;accepted[name]=path;sweeps[name]=whole
        if reason:continue
        solver=WaitDuplicationSolver(paths,{n:shape for n in paths},policy=policy)
        witness=dag_witness(solver,graph);solver.validate(witness)
        return dict(index=index,seed=seed+index*1000003,attempts=attempt,
            routes=routes,assignments=assignments,empty_rack_cells=sorted(empty),
            requirements={n:sorted(v) for n,v in graph.items()},
            metrics=measure(paths,shape,policy),all_pair_F_one=True,acyclic_requirements=True,
            stationary_rack_sweeps_clear=True)
    raise RuntimeError(f'No admitted warehouse set {index+1} within the finite generation budget.')


def build_factory(case,portfolio_budget=6):
    layout=load_layout();layout['warehouse_empty_cells']=tuple(map(tuple,case['empty_rack_cells']))
    routes={n:tuple(map(tuple,p)) for n,p in case['routes'].items()}
    paths={n:MotionPath(n,p) for n,p in routes.items()}
    solver=WaitDuplicationSolver(paths)
    reference=dag_witness(solver,case['requirements']);portfolio=[reference]
    seen={repr(reference.tuple_path)};rng=Random(case['seed'])
    lengths={n:sum(dist(a,b) for a,b in zip(p,p[1:])) for n,p in routes.items()}
    for attempt in range(portfolio_budget*4):
        pending=set(routes);order=[]
        while pending:
            ready=[n for n in routes if n in pending and set(case['requirements'][n])<=set(order)]
            name=(min(ready,key=lambda n:(-lengths[n],n)) if attempt==0 else rng.choice(ready))
            pending.remove(name);order.append(name)
        option=ordered_witness(solver,order);key=repr(option.tuple_path)
        if key not in seen:portfolio.append(option);seen.add(key)
        if len(portfolio)>=portfolio_budget:break
    gates=automatic_gates(solver,case['metrics'])
    headings={n:degrees(atan2(p[1][1]-p[0][1],p[1][0]-p[0][0])) for n,p in routes.items()}
    def build(policy,collision):
        model,robots,sensors=create_controlled_model(routes,grid=layout,lift_up=True,
            initial_headings=headings,collision_check=collision,stop_on_collision=True,
            merge_sweeps=True,event_release=True,plan_builder=lambda _:reference)
        controller=model.controller;original=controller.traffic_manager
        controller.traffic_manager=ResidualPolicyManager(controller.roadmap,original.tuple_path,
            pairs=(),residual_gates=gates,mode='fifo' if policy=='fifo' else 'length',
            alternative_plans=tuple(p.tuple_path for p in portfolio),event_release=True,
            footprints=original.footprints,command_blocks=original.command_blocks)
        model.Delta_y=MethodType(stable_output,model)
        return model,robots,sensors
    return build,layout,routes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data',type=Path,default=DATA)
    parser.add_argument('--generate',action='store_true')
    parser.add_argument('--sets',type=int,default=5)
    parser.add_argument('--seed',type=int,default=20261120)
    parser.add_argument('--case',type=int,default=1)
    parser.add_argument('--policy',choices=('fifo','lp'),default='fifo')
    parser.add_argument('--validate',action='store_true')
    parser.add_argument('--capture-output',type=Path)
    parser.add_argument('--capture-time',type=float,default=0.)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results/warehouse20_validation.json')
    args,viewer_args=parser.parse_known_args()
    if args.generate:
        args.data.parent.mkdir(parents=True,exist_ok=True)
        cases=[]
        for i in range(args.sets):
            case=generate_case(i,args.seed);cases.append(case)
            args.data.write_text(json.dumps(dict(layout='warehouse_layout.json',cases=cases),indent=2),encoding='utf-8')
            print('Generated',i+1,'intervals',case['metrics']['directed_pairwise_interval_count'],
                  'sum_m',round(case['metrics']['total_pairwise_interval_m'],2),flush=True)
        if not args.validate:return
    cases=json.loads(args.data.read_text(encoding='utf-8'))['cases']
    if args.validate:
        rows=[];args.output.parent.mkdir(parents=True,exist_ok=True)
        for case in cases:
            build,_,_=build_factory(case)
            rows.append(evaluate(f'warehouse20_set_{case["index"]+1}',build,case['metrics']))
            args.output.write_text(json.dumps(dict(status='running',results=rows),indent=2),encoding='utf-8')
        args.output.write_text(json.dumps(dict(status='complete',results=rows),indent=2),encoding='utf-8')
        print(args.output,flush=True);return
    case=next((c for c in cases if c['index']==args.case-1),None)
    if case is None:parser.error('No such saved case.')
    build,layout,routes=build_factory(case)
    from importlib import import_module
    original_run=None
    if args.capture_output:
        from panda3d.core import loadPrcFileData,Filename
        from direct.showbase.ShowBase import ShowBase
        loadPrcFileData('','window-type offscreen')
        loadPrcFileData('','audio-library-name null')
        args.capture_output.parent.mkdir(parents=True,exist_ok=True)
        def capture(viewer):
            while isfinite(viewer.model.tNext) and viewer.model.tNext<=args.capture_time:
                viewer.engine.execute_one_step()
            target=args.capture_time if isfinite(viewer.model.tNext) else viewer.engine.tNow
            viewer.camera_view('iso');viewer.zoom(1.08);viewer.draw(target)
            viewer.toggle_info()
            for _ in range(4):viewer.graphicsEngine.renderFrame()
            assert not viewer.model.collision_history
            assert viewer.win.getScreenshot().write(Filename.fromOsSpecific(str(args.capture_output.resolve())))
            print(args.capture_output,flush=True)
        original_run=ShowBase.run;ShowBase.run=capture
    sys.argv=[sys.argv[0],*viewer_args]
    try:
        import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
            name=f'Warehouse20_set{args.case}_{args.policy.upper()}',no_rack=False,
            robot_paths=routes,grid=layout,collision_check=False,stop_on_collision=True,
            hide_robot_names=True,hide_motion_labels=True,hide_point_labels=True,render_fps=10,
            model_factory=lambda **options:build(args.policy,options.get('collision_check',False)))
    finally:
        if original_run is not None:ShowBase.run=original_run


if __name__=='__main__':main()
