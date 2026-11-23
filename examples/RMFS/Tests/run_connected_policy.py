"""New connected crossing/merge fixtures; existing demos/results unchanged.

Every full path shares a forward corridor with other objects. Each pair has
an approaching B and an A waiting before its merge, but optional waiting is
offered only when current approved geometry and a validated completion allow
it. Endpoints are in separate access/exit lanes, not parked on the corridor.
Both favorable and unfavorable policy outcomes are retained.
"""

import argparse
from importlib import import_module
import json
from math import dist
from pathlib import Path
from random import Random
from types import MethodType

from ..controlled import create_controlled_model
from ..GridPath import make_grid
from ..path_geometry import PathPosition,Subpath,sweep_2d
from ..wait_duplication import WaitDuplicationSolver
from .run_priority_lookahead import ordered_witness
from . import run_unified_policy as unified
from .waiting_choice import stable_output


CASES={'corridor4':(4,False),'corridor6':(6,False),'corridor8':(8,False),
       'corridor4_reverse_tails':(4,True),'corridor6_reverse_tails':(6,True),
       'corridor8_reverse_tails':(8,True)}


def fixture(case):
    count,reverse=CASES[case]
    size=max(20,2*count+4)
    exits=list(range(count+3,2*count+3))
    if reverse:
        exits.reverse()
    else:
        # Cross the long/short tail allocation within alternating arrival pairs.
        for group in range(0,count,4):
            exits[group:group+2]=reversed(exits[group:group+2])
    routes={}
    pairs=[]
    headings={}
    for i in range(count):
        name=f'R{i+1}'
        x=i+2
        first_y=6 if i%2==0 else 4
        route=tuple((x,y) for y in range(first_y,9))
        route+=tuple((q,8) for q in range(x+1,exits[i]+1))
        route+=tuple((exits[i],y) for y in range(9,size))
        routes[name]=route
        headings[name]=90.
        if i%2==1:
            # A reaches corridor entrance y=8. B approaches waiting point y=7.
            pairs.append((f'R{i}',name,PathPosition(2),PathPosition(3)))
    return routes,make_grid(size),tuple(pairs),headings


def witness_portfolio(solver,budget=16):
    names=solver.names
    lengths={name:sum(dist(a,b) for a,b in zip(path.vertices,path.vertices[1:]))
             for name,path in zip(names,solver.paths)}
    orders=[names,tuple(reversed(names)),
            tuple(sorted(names,key=lambda n:(-lengths[n],n))),
            tuple(sorted(names,key=lambda n:(lengths[n],n)))]
    # Adjacent pair swaps make local order choices explicit, without  n! plans.
    for i in range(0,len(names),2):
        order=list(reversed(names))
        a,b=order.index(names[i]),order.index(names[i+1])
        order[a],order[b]=order[b],order[a]
        orders.append(tuple(order))
    rng=Random(20261120)
    for _ in range(8):
        orders.append(tuple(rng.sample(list(names),len(names))))
    results=[]
    seen=set()
    for order in dict.fromkeys(orders):
        result=ordered_witness(solver,order)
        key=repr(result.tuple_path)
        if key not in seen:
            seen.add(key)
            results.append(result)
        if len(results)>=budget:
            break
    return results


def geometry_metadata(paths,shapes,policy):
    names=tuple(paths)
    whole={name:sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(path.vertices)-1)),shapes[name],policy)
           for name,path in paths.items()}
    edges=[(a,b) for i,a in enumerate(names) for b in names[i+1:]
           if whole[a].intersects(whole[b],policy.tolerance)]
    members={names[0]}
    while True:
        expanded=members|{a for a,b in edges if b in members}|{b for a,b in edges if a in members}
        if expanded==members:
            break
        members=expanded
    return dict(objects=len(names),interference_edges=edges,connected=len(members)==len(names),
                simple_paths=all(len(p.vertices)==len(set(p.vertices)) for p in paths.values()))


def build(case,policy,seed=0,collision=False):
    routes,grid,pairs,headings=fixture(case)
    portfolio=[]
    def builder(solver):
        portfolio.clear()
        portfolio.extend(witness_portfolio(solver))
        # A feasible initial synchronous approach requires downstream access
        # to remain available; the validated reverse order supplies that base.
        return portfolio[1]
    model,_,_=create_controlled_model(routes,grid=grid,initial_headings=headings,
                                      merge_sweeps=True,event_release=True,
                                      collision_check=collision,stop_on_collision=True,
                                      plan_builder=builder)
    controller=model.controller
    original=controller.traffic_manager
    controller.traffic_manager=unified.UnifiedPolicyManager(
        controller.roadmap,original.tuple_path,pairs=pairs,mode='lookahead' if policy.startswith('lookahead_') else policy,
        seed=seed,alternative_plans=tuple(p.tuple_path for p in portfolio),
        event_release=True,footprints=original.footprints,command_blocks=original.command_blocks)
    controller.add_input_ports('?priority','?wait_action')
    controller.delta_x=MethodType(unified.decision_input,controller)
    model.Delta_y=MethodType(stable_output,model)
    model.connected_metadata=geometry_metadata(controller.roadmap.paths,original.footprints,original.policy)
    model.connected_metadata.update(reference_candidates=len(portfolio),grid_points=grid['points_per_axis']**2)
    assert model.connected_metadata['connected'] and model.connected_metadata['simple_paths']
    return model


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',nargs='+',choices=tuple(CASES),default=tuple(CASES))
    parser.add_argument('--random-repetitions',type=int,default=20)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results'/'connected_policy.json')
    args=parser.parse_args()
    if args.random_repetitions<1:
        parser.error('random-repetitions must be positive')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rows=[]
    for case in args.cases:
        row=unified.experiment(case,args.random_repetitions,model_factory=build,progress=True)
        row['geometry']=build(case,'fifo').connected_metadata
        row['scope']='connected_shared_corridor; finite_validated_reference_portfolio'
        rows.append(row)
        args.output.write_text(json.dumps(rows,indent=2),encoding='utf-8')
    print(args.output,flush=True)


if __name__=='__main__':
    main()
