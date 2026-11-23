"""Three-size study: 4 objects plus 12/24 objects on a 20x20 point grid.

Large fixtures tile disjoint two-object crossing groups with real approach
commands and optional waits. Every object interferes with its partner, but
groups are independent. This tests load scaling, not a 200-object densely
connected warehouse. Local reference alternatives are factored: 2m stored
options represent 2**m combinations without enumerating the Cartesian product.
"""

import argparse
import json
from pathlib import Path

from ..controlled import create_controlled_model
from ..GridPath import make_grid
from ..path_geometry import PathPosition,Subpath,sweep_2d
from ..wait_duplication import WaitDuplicationSolver
from .run_priority_lookahead import ordered_witness
from .run_waiting_lookahead import parallel_group_result
from . import run_unified_policy as unified


class FactoredPolicyManager(unified.UnifiedPolicyManager):
    def __init__(self,*args,task_options,**kwargs):
        self.task_options=task_options
        super().__init__(*args,**kwargs)

    def _supporting_plans(self,index):
        all_options=self.viable_plans
        self.viable_plans=all_options & self.task_options[index]
        try:
            return super()._supporting_plans(index)
        finally:
            self.viable_plans=all_options

    def _issue(self,index,time):
        other_groups=self.viable_plans-self.task_options[index]
        super()._issue(index,time)
        self.viable_plans.update(other_groups)


def fixture(count):
    if count not in (12,24):
        raise ValueError('Supported sizes are 12 and 24.')
    routes={}
    pairs=[]
    headings={}
    for group in range(count//2):
        ox,oy=(group%4)*5,(group//4)*5
        # The long variant adds turns after the crossing. Neither path loops.
        a_path=(((0,0),(0,1),(1,1),(2,1)) if group%2==0 else
                ((0,0),(0,1),(1,1),(2,1),(2,2),(3,2),(3,3)))
        b_path=((1,3),(1,2),(1,1),(1,0),(2,0),(3,0))
        a,b=f'R{2*group+1}',f'R{2*group+2}'
        routes[a]=tuple((x+ox,y+oy) for x,y in a_path)
        routes[b]=tuple((x+ox,y+oy) for x,y in b_path)
        pairs.append((a,b,PathPosition(1),PathPosition(1)))
        headings[b]=90.  # finite entry turn: A reaches its gate before B
    return routes,make_grid(20),tuple(pairs),headings


def build(case,policy,seed=0,collision=False):
    if case=='mixed_four':
        return unified.build(case,policy,seed,collision)
    count=int(case.split('_')[1])
    routes,grid,pairs,headings=fixture(count)
    options=[]
    def builder(solver):
        options.clear()
        for a,b,_,_ in pairs:
            local=WaitDuplicationSolver({name:solver.paths[solver.names.index(name)] for name in (a,b)},
                                        {name:solver.footprints[solver.names.index(name)] for name in (a,b)})
            options.append([ordered_witness(local,order) for order in ((a,b),(b,a))])
        return parallel_group_result(solver,[group[0] for group in options])
    model,_,_=create_controlled_model(routes,grid=grid,initial_headings=headings,merge_sweeps=True,
                                      event_release=True,collision_check=collision,
                                      stop_on_collision=True,plan_builder=builder)
    controller=model.controller
    original=controller.traffic_manager
    tasks=original.tasks
    identities={(name,part.start,part.end):i for i,(_,name,part) in enumerate(tasks)}
    task_options={}
    predecessors=[]
    for group,(a,b,_,_) in enumerate(pairs):
        indices=[i for i,(_,name,_) in enumerate(tasks) if name in (a,b)]
        ids=frozenset((2*group,2*group+1))
        for i in indices:
            task_options[i]=ids
        conflicts=[(i,j) for i in indices for j in indices if tasks[i][1]!=tasks[j][1]
                   and original.task_sweeps[i].intersects(original.task_sweeps[j],original.policy.tolerance)]
        for reference in options[group]:
            rank={identities[name,part.start,part.end]:stage
                  for stage,row in enumerate(reference.tuple_path.stages)
                  for name,part in zip(reference.tuple_path.object_names,row) if not part.is_point}
            rows=[set() for _ in tasks]
            for i,j in conflicts:
                if rank[j]<rank[i]:
                    rows[i].add(j)
            predecessors.append(tuple(frozenset(row) for row in rows))
    # Factorization is allowed only because distinct groups' FULL sweeps are
    # disjoint. Check that premise, not just their centerline separation.
    full={name:sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(path.vertices)-1)),
                        original.footprints[name],original.policy)
          for name,path in controller.roadmap.paths.items()}
    owners={name:group for group,pair in enumerate(pairs) for name in pair[:2]}
    for i,name in enumerate(routes):
        for other in tuple(routes)[i+1:]:
            if owners[name]!=owners[other]:
                assert not full[name].intersects(full[other],original.policy.tolerance)
    controller.traffic_manager=FactoredPolicyManager(
        controller.roadmap,original.tuple_path,pairs=pairs,
        mode='lookahead' if policy.startswith('lookahead_') else policy,seed=seed,
        task_options=task_options,reference_predecessors=predecessors,
        event_release=True,footprints=original.footprints,command_blocks=original.command_blocks)
    controller.add_input_ports('?priority','?wait_action')
    from types import MethodType
    controller.delta_x=MethodType(unified.decision_input,controller)
    from .waiting_choice import stable_output
    model.Delta_y=MethodType(stable_output,model)
    model.scaling_metadata=dict(objects=count,grid_points=400,independent_groups=count//2,
                                stored_reference_options=count,
                                represented_reference_combinations=2**(count//2))
    return model


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',nargs='+',choices=('mixed_four','grid20_12','grid20_24'),
                        default=('mixed_four','grid20_12','grid20_24'))
    parser.add_argument('--random-repetitions',type=int,default=20)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results'/'large_policy.json')
    args=parser.parse_args()
    if args.random_repetitions<1:
        parser.error('random-repetitions must be positive')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rows=[]
    for case in args.cases:
        row=unified.experiment(case,args.random_repetitions,model_factory=build,progress=True)
        row['layout_scope']='independent_interaction_groups_for_load_scaling'
        rows.append(row)
        args.output.write_text(json.dumps(rows,indent=2),encoding='utf-8')
    print(args.output,flush=True)


if __name__=='__main__':
    main()
