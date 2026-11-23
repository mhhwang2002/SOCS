"""Seeded, policy-independent path datasets with measured sweep interference.

Accepted samples satisfy every pair's endpoint F test AND an acyclic serial
requirement graph, with an independently validated constructive tuple witness.
No controller performance or lookahead result is used for acceptance.
Counts refer to directed pairwise progress intervals, not distinct 2D regions.
Each unordered pair stores both directions, plus per-path merged-union data.
Budget exhaustion means not found within the budget, never infeasibility.
"""

import argparse
from collections import Counter
from dataclasses import asdict,dataclass
from hashlib import sha256
from heapq import heappop,heappush
import json
from math import dist,isfinite
from pathlib import Path as FilePath
from random import Random
from time import perf_counter

from ..GridPath import make_grid
from ..path_geometry import (Path,PathPosition,Subpath,RectangularFootprint,
                             GeometryPolicy,TuplePath,sweep_2d,interference_2d)
from ..wait_duplication import WaitDuplicationSolver,SearchResult
from .incremental_routing import dependency_order,dag_witness


@dataclass(frozen=True)
class Config:
    grid_size:int=12
    objects:int=4
    seed:int=20261120
    samples:int=5
    spacing_m:float=2.
    width_m:float=1.5
    depth_m:float=1.5
    safety_margin_m:float=0.
    tolerance_m:float=1e-9
    route_mode:str='shortest'
    endpoints:str='boundary_to_interior'
    min_edges:int=4
    max_edges:int=30
    min_turns:int=0
    waypoints:int=2
    goal_bias:float=.65
    min_intervals:int=0
    max_intervals:int|None=None
    min_mean_interval_m:float=0.
    max_mean_interval_m:float|None=None
    min_interval_m:float=0.
    max_interval_m:float|None=None
    min_interference_ratio:float=0.
    max_interference_ratio:float=1.
    connected:bool=False
    scenario_attempts:int=30
    candidates_per_object:int=100
    candidate_budget:int=10000

    def __post_init__(self):
        if self.grid_size<3 or self.objects<2 or self.samples<1:
            raise ValueError('Need grid_size >= 3, objects >= 2, samples >= 1.')
        if not 1<=self.min_edges<=self.max_edges<self.grid_size**2:
            raise ValueError('Require 1 <= min_edges <= max_edges < grid point count.')
        if self.min_turns<0 or self.min_turns>=self.max_edges:
            raise ValueError('min_turns must be between 0 and max_edges-1.')
        if self.route_mode not in ('shortest','detour','random'):
            raise ValueError('Unknown route mode.')
        if self.endpoints not in ('boundary_to_interior','boundary_to_boundary','any'):
            raise ValueError('Unknown endpoint mode.')
        if not 0<=self.goal_bias<=1 or self.waypoints<0:
            raise ValueError('Invalid goal bias or waypoint count.')
        for value in (self.spacing_m,self.width_m,self.depth_m):
            if not isfinite(value) or value<=0:
                raise ValueError('Spacing and footprint lengths must be positive finite values.')
        for value in (self.safety_margin_m,self.tolerance_m,self.min_mean_interval_m,self.min_interval_m):
            if not isfinite(value) or value<0:
                raise ValueError('Margins, tolerance and length bounds must be nonnegative finite values.')
        if self.min_intervals<0 or (self.max_intervals is not None and self.max_intervals<self.min_intervals):
            raise ValueError('Invalid interval count range.')
        if self.max_mean_interval_m is not None and (not isfinite(self.max_mean_interval_m) or self.max_mean_interval_m<self.min_mean_interval_m):
            raise ValueError('Invalid mean interval length range.')
        if self.max_interval_m is not None and (not isfinite(self.max_interval_m) or self.max_interval_m<self.min_interval_m):
            raise ValueError('Invalid individual interval length range.')
        if not 0<=self.min_interference_ratio<=self.max_interference_ratio<=1:
            raise ValueError('Interference ratio must be in [0,1].')
        if min(self.scenario_attempts,self.candidates_per_object,self.candidate_budget)<1:
            raise ValueError('Search budgets must be positive.')
        starts,goals=endpoint_pools(make_grid(self.grid_size),self.endpoints)
        if self.objects>min(len(starts),len(goals)) or 2*self.objects>self.grid_size**2:
            raise ValueError('Too many distinct starts/goals for this endpoint mode.')


def endpoint_pools(grid,mode):
    points=tuple(grid['control_points'])
    last=grid['points_per_axis']-1
    boundary=tuple(p for p in points if p[0] in (0,last) or p[1] in (0,last))
    inside=tuple(p for p in points if p not in boundary)
    return ((boundary,inside) if mode=='boundary_to_interior' else
            (boundary,boundary) if mode=='boundary_to_boundary' else (points,points))


def shortest(grid,start,goal,rng,forbidden=frozenset()):
    queue=[(abs(start[0]-goal[0])+abs(start[1]-goal[1]),rng.random(),0,start)]
    costs={start:0}; previous={}
    while queue:
        _,_,cost,node=heappop(queue)
        if cost!=costs[node]:
            continue
        if node==goal:
            route=[goal]
            while route[-1]!=start:
                route.append(previous[route[-1]])
            return tuple(reversed(route))
        for q in grid['adjacency'][node]:
            if q in forbidden:
                continue
            new=cost+1
            if new<costs.get(q,float('inf')):
                costs[q]=new; previous[q]=node
                h=abs(q[0]-goal[0])+abs(q[1]-goal[1])
                heappush(queue,(new+h,rng.random(),new,q))
    return None


def make_route(grid,start,goal,cfg,rng):
    if cfg.route_mode=='shortest':
        return shortest(grid,start,goal,rng)
    if cfg.route_mode=='detour':
        candidates=[p for p in grid['control_points'] if p not in (start,goal)]
        if cfg.waypoints>len(candidates):
            return None
        waypoints=rng.sample(candidates,cfg.waypoints)+[goal]
        route=(start,)
        for index,target in enumerate(waypoints):
            forbidden=set(route[:-1])|set(waypoints[index+1:])
            leg=shortest(grid,route[-1],target,rng,forbidden)
            if leg is None:
                return None
            route+=leg[1:]
            if len(route)-1>cfg.max_edges:
                return None
        return route
    route=[start]; seen={start}
    while len(route)-1<cfg.max_edges:
        if route[-1]==goal:
            return tuple(route)
        choices=[p for p in grid['adjacency'][route[-1]] if p not in seen
                 and abs(p[0]-goal[0])+abs(p[1]-goal[1])<=cfg.max_edges-len(route)]
        if not choices:
            return None
        if rng.random()<cfg.goal_bias:
            best=min(abs(p[0]-goal[0])+abs(p[1]-goal[1]) for p in choices)
            choices=[p for p in choices if abs(p[0]-goal[0])+abs(p[1]-goal[1])==best]
        point=rng.choice(choices)
        route.append(point);seen.add(point)
    return tuple(route) if route[-1]==goal else None


def turns(route):
    vectors=[(b[0]-a[0],b[1]-a[1]) for a,b in zip(route,route[1:])]
    return sum(a!=b for a,b in zip(vectors,vectors[1:]))


def append_requirements(name,path,sweep,paths,sweeps,graph,shape,policy):
    graph={n:set(deps) for n,deps in graph.items()}
    graph[name]=set()
    for other,old in paths.items():
        si=sweep_2d(path.point(0),shape,policy).intersects(sweeps[other],policy.tolerance)
        ei=sweep_2d(path.point(len(path.vertices)-1),shape,policy).intersects(sweeps[other],policy.tolerance)
        sj=sweep_2d(old.point(0),shape,policy).intersects(sweep,policy.tolerance)
        ej=sweep_2d(old.point(len(old.vertices)-1),shape,policy).intersects(sweep,policy.tolerance)
        if not ((not sj and not ei) or (not si and not ej)):
            return None,'pair_F_zero'
        if sj or ei:
            graph[name].add(other)
        if si or ej:
            graph[other].add(name)
    if dependency_order(graph) is None:
        return None,'cyclic_requirement'
    return graph,None


def encode_position(point):
    return [point.segment,point.ratio]


def interval_data(path,footprint,opponents,policy):
    data=interference_2d(path,footprint,opponents,policy)
    prefix=[0.]
    for a,b in zip(path.vertices,path.vertices[1:]):
        prefix.append(prefix[-1]+dist(a,b))
    def along(point):
        return prefix[point.segment]+(point.ratio*dist(path.vertices[point.segment],path.vertices[point.segment+1])
                                      if point.ratio else 0.)
    return [dict(start=encode_position(i.start),end=encode_position(i.end),
                 length_m=max(0.,along(i.end)-along(i.start))) for i in data.intervals]


def measure(paths,shape,policy):
    names=tuple(paths)
    full={name:sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(path.vertices)-1)),shape,policy)
          for name,path in paths.items()}
    pairs=[];lengths=[];edges=[]
    for i,a in enumerate(names):
        for b in names[i+1:]:
            ab=interval_data(paths[a],shape,{b:(paths[b],shape)},policy)
            ba=interval_data(paths[b],shape,{a:(paths[a],shape)},policy)
            if ab or ba:
                edges.append((a,b))
            flags=(sweep_2d(paths[a].point(0),shape,policy).intersects(full[b],policy.tolerance),
                   sweep_2d(paths[a].point(len(paths[a].vertices)-1),shape,policy).intersects(full[b],policy.tolerance),
                   sweep_2d(paths[b].point(0),shape,policy).intersects(full[a],policy.tolerance),
                   sweep_2d(paths[b].point(len(paths[b].vertices)-1),shape,policy).intersects(full[a],policy.tolerance))
            si,ei,sj,ej=flags
            pairs.append(dict(i=a,j=b,endpoint_flags=list(map(int,flags)),
                              F_ij=int((not sj and not ei) or (not si and not ej)),
                              i_intervals=ab,j_intervals=ba))
            lengths.extend(v['length_m'] for v in (*ab,*ba))
    per_path={}
    ratios=[]
    for name,path in paths.items():
        intervals=interval_data(path,shape,{n:(p,shape) for n,p in paths.items() if n!=name},policy)
        length=sum(dist(a,b) for a,b in zip(path.vertices,path.vertices[1:]))
        union=sum(v['length_m'] for v in intervals)
        ratio=min(1.,union/length) if length else 0.
        ratios.append(ratio)
        per_path[name]=dict(length_m=length,union_intervals=intervals,
                            union_interval_count=len(intervals),union_interference_m=union,interference_ratio=ratio)
    components=[];remaining=set(names)
    while remaining:
        start=next(n for n in names if n in remaining)
        found={start}
        while True:
            expanded=found|{a for a,b in edges if b in found}|{b for a,b in edges if a in found}
            if expanded==found:
                break
            found=expanded
        components.append([n for n in names if n in found]);remaining-=found
    return dict(directed_pairwise_interval_count=len(lengths),
                min_pairwise_interval_m=min(lengths,default=0.),
                mean_pairwise_interval_m=sum(lengths)/len(lengths) if lengths else 0.,
                max_pairwise_interval_m=max(lengths,default=0.),total_pairwise_interval_m=sum(lengths),
                per_path_union_interval_count=sum(v['union_interval_count'] for v in per_path.values()),
                mean_union_interference_ratio=sum(ratios)/len(ratios),
                interfering_pairs=len(edges),interference_edges=edges,components=components,
                max_interference_degree=max((sum(name in edge for edge in edges) for name in names),default=0),
                connected=len(components)==1,pairs=pairs,per_path=per_path)


def metric_rejection(metrics,cfg):
    count=metrics['directed_pairwise_interval_count']
    size=metrics['mean_pairwise_interval_m']
    ratio=metrics['mean_union_interference_ratio']
    if count<cfg.min_intervals or (cfg.max_intervals is not None and count>cfg.max_intervals):
        return 'interval_count_filter'
    if size<cfg.min_mean_interval_m or (cfg.max_mean_interval_m is not None and size>cfg.max_mean_interval_m):
        return 'interval_length_filter'
    if count and (metrics['min_pairwise_interval_m']<cfg.min_interval_m or
                  (cfg.max_interval_m is not None and metrics['max_pairwise_interval_m']>cfg.max_interval_m)):
        return 'individual_interval_length_filter'
    if not cfg.min_interference_ratio<=ratio<=cfg.max_interference_ratio:
        return 'interference_ratio_filter'
    if cfg.connected and not metrics['connected']:
        return 'connected_filter'
    return None


def generate_sample(cfg,index):
    sample_seed=cfg.seed+index*1000003
    rng=Random(sample_seed)
    grid=make_grid(cfg.grid_size,cfg.spacing_m)
    shape=RectangularFootprint(cfg.width_m,cfg.depth_m)
    policy=GeometryPolicy(tolerance=cfg.tolerance_m,safety_margin=cfg.safety_margin_m)
    starts,goals=endpoint_pools(grid,cfg.endpoints)
    reasons=Counter();log=[];candidate_count=0
    for attempt in range(1,cfg.scenario_attempts+1):
        paths={};routes={};sweeps={};requires={};used=set()
        for number in range(cfg.objects):
            name=f'R{number+1}'
            for _ in range(cfg.candidates_per_object):
                if candidate_count>=cfg.candidate_budget:
                    return None,dict(seed=sample_seed,status='budget_exhausted',candidates=candidate_count,
                                     rejection_counts=dict(reasons),attempt_log=log)
                candidate_count+=1
                left=[p for p in starts if p not in used]
                right=[p for p in goals if p not in used]
                if not left or not right:
                    reasons['endpoint_pool_exhausted']+=1
                    break
                start=rng.choice(left)
                eligible_goals=[p for p in right if p!=start]
                if not eligible_goals:
                    reasons['endpoint_pool_exhausted']+=1
                    break
                goal=rng.choice(eligible_goals)
                route=make_route(grid,start,goal,cfg,rng)
                reason=None
                if route is None:
                    reason='route_generation_failed'
                elif not cfg.min_edges<=len(route)-1<=cfg.max_edges:
                    reason='edge_count_filter'
                elif turns(route)<cfg.min_turns:
                    reason='turn_count_filter'
                else:
                    path=Path(name,tuple(grid['control_points'][p] for p in route))
                    full=sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(route)-1)),shape,policy)
                    graph,reason=append_requirements(name,path,full,paths,sweeps,requires,shape,policy)
                    if reason is None:
                        paths[name]=path;routes[name]=route;sweeps[name]=full;requires=graph
                        used.update((start,goal))
                        break
                reasons[reason]+=1
            else:
                reasons['object_candidate_budget']+=1
            if name not in paths:
                log.append(dict(attempt=attempt,status='incomplete_route_set',objects=len(paths)))
                break
        if len(paths)!=cfg.objects:
            continue
        metrics=measure(paths,shape,policy)
        reason=metric_rejection(metrics,cfg)
        if reason:
            reasons[reason]+=1
            log.append(dict(attempt=attempt,status=reason,
                            intervals=metrics['directed_pairwise_interval_count'],
                            mean_interval_m=metrics['mean_pairwise_interval_m']))
            continue
        solver=WaitDuplicationSolver(paths,{n:shape for n in paths},policy=policy)
        witness=dag_witness(solver,requires)
        solver.validate(witness)
        sample=dict(index=index,seed=sample_seed,object_names=tuple(paths),routes=routes,
                    paths_m={n:p.vertices for n,p in paths.items()},
                    turns={n:turns(route) for n,route in routes.items()},metrics=metrics,
                    requirements={n:sorted(deps) for n,deps in requires.items()},
                    serial_execution_order=dependency_order(requires),
                    all_pair_F_one=True,acyclic_requirements=True,
                    witness=dict(validated=True,consumed_counts=witness.consumed_counts,
                                 stages=[[dict(start=encode_position(part.start),end=encode_position(part.end))
                                          for part in row] for row in witness.tuple_path.stages]))
        sample['path_hash']=sha256(json.dumps(sample['paths_m'],sort_keys=True).encode()).hexdigest()
        return sample,dict(seed=sample_seed,status='accepted',candidates=candidate_count,
                           rejection_counts=dict(reasons),attempt_log=log)
    return None,dict(seed=sample_seed,status='scenario_budget_exhausted',candidates=candidate_count,
                     rejection_counts=dict(reasons),attempt_log=log)


def generate(cfg):
    started=perf_counter();samples=[];audit=[]
    for index in range(cfg.samples):
        sample,record=generate_sample(cfg,index)
        audit.append(record)
        if sample is not None:
            samples.append(sample)
    payload=dict(schema_version=1,generator='examples.RMFS.Tests.generate_path_data',config=asdict(cfg),
                 acceptance='simple_paths; pairwise_F=1; acyclic_requirements; explicit_metric_filters; validated_witness',
                 interval_count_definition='sum of connected progress intervals over ordered object pairs',
                 interval_size_definition='arc length in metres on the subject path, not 2D area',
                 ratio_definition='mean over objects of merged interference length / full path length',
                 no_policy_performance_filter=True,samples=samples,audit=audit,
                 summary=dict(requested=cfg.samples,accepted=len(samples),not_found=cfg.samples-len(samples)))
    payload['dataset_hash']=sha256(json.dumps(dict(config=payload['config'],samples=samples),sort_keys=True).encode()).hexdigest()
    payload['generation_seconds']=perf_counter()-started
    return payload


def load_witness(data,index=0):
    """Reconstruct geometry and independently validate an accepted JSON witness.

    No simulation is run. Object order is explicit and independent of JSON
    dictionary key ordering. Recomputed F/DAG checks do not trust the flags.
    """
    cfg=Config(**data['config'])
    sample=next((sample for sample in data['samples'] if sample['index']==index),None)
    if sample is None:
        raise ValueError('Requested sample was not accepted or is absent.')
    grid=make_grid(cfg.grid_size,cfg.spacing_m)
    names=tuple(sample['object_names'])
    if len(names)!=cfg.objects or len(set(names))!=len(names):
        raise ValueError('Invalid object order or count.')
    paths={name:Path(name,tuple(tuple(p) for p in sample['paths_m'][name])) for name in names}
    shape=RectangularFootprint(cfg.width_m,cfg.depth_m)
    policy=GeometryPolicy(tolerance=cfg.tolerance_m,safety_margin=cfg.safety_margin_m)
    accepted={};sweeps={};graph={}
    for name,path in paths.items():
        ids=tuple(tuple(point) for point in sample['routes'][name])
        if len(ids)!=len(set(ids)) or tuple(grid['control_points'][p] for p in ids)!=path.vertices:
            raise ValueError('Invalid simple grid path or inconsistent metre coordinates.')
        if any(b not in grid['adjacency'][a] for a,b in zip(ids,ids[1:])):
            raise ValueError('Path includes a nonadjacent grid edge.')
        full=sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(path.vertices)-1)),shape,policy)
        new_graph,reason=append_requirements(name,path,full,accepted,sweeps,graph,shape,policy)
        if reason:
            raise ValueError('Acceptance condition no longer holds: '+reason)
        graph=new_graph;accepted[name]=path;sweeps[name]=full
    solver=WaitDuplicationSolver(paths,{name:shape for name in names},policy=policy)
    stored=sample['witness']
    rows=tuple(tuple(Subpath(paths[name],PathPosition(*part['start']),PathPosition(*part['end']))
                     for name,part in zip(names,row)) for row in stored['stages'])
    result=SearchResult('feasible',TuplePath(names,rows),
                        tuple(tuple(c) for c in stored['consumed_counts']),
                        len(rows)+1,len(rows),0.,'Reconstructed from generated data.')
    solver.validate(result)
    return solver,result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    defaults=Config()
    for name,value in asdict(defaults).items():
        flag='--'+name.replace('_','-')
        if name=='connected':
            parser.add_argument(flag,action='store_true')
        elif name=='route_mode':
            parser.add_argument(flag,choices=('shortest','detour','random'),default=value)
        elif name=='endpoints':
            parser.add_argument(flag,choices=('boundary_to_interior','boundary_to_boundary','any'),default=value)
        else:
            kind=float if name in ('max_mean_interval_m','max_interval_m') else int if name=='max_intervals' else type(value)
            parser.add_argument(flag,type=kind,default=value)
    parser.add_argument('--output',type=FilePath,required=True)
    args=vars(parser.parse_args());destination=args.pop('output')
    try:
        cfg=Config(**args)
    except ValueError as error:
        parser.error(str(error))
    payload=generate(cfg)
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(payload,indent=2),encoding='utf-8')
    print(payload['summary'],'seconds',round(payload['generation_seconds'],3),flush=True)
    for sample in payload['samples']:
        m=sample['metrics']
        print(sample['index'],'intervals',m['directed_pairwise_interval_count'],
              'mean_m',round(m['mean_pairwise_interval_m'],3),
              'ratio',round(m['mean_union_interference_ratio'],3),'connected',m['connected'])
    print(destination)


if __name__=='__main__':
    main()
