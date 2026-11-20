"""Experimental incremental A* candidates -> pair checks -> DAG -> plant.

This module is intentionally outside RobotPathPlanner/TrafficManager. It uses
the proven serial-order sufficient condition; cyclic candidates are declined,
not declared infeasible. Accepted routes are never altered or rerouted.
"""

from collections import Counter, deque
from heapq import heappop, heappush
from itertools import pairwise
from math import atan2, degrees
from random import Random
from time import perf_counter

from ..GridPath import make_grid
from ..controlled import create_controlled_model
from ..path_geometry import Path, PathPosition, Subpath, TuplePath, sweep_2d
from ..wait_duplication import SearchResult


def astar(grid, start, goal, rng):
    heuristic = lambda p: abs(p[0]-goal[0]) + abs(p[1]-goal[1])
    queue = [(heuristic(start), rng.random(), 0, start)]
    costs, previous = {start: 0}, {}
    while queue:
        _, _, cost, node = heappop(queue)
        if cost != costs[node]:
            continue
        if node == goal:
            route = [goal]
            while route[-1] != start:
                route.append(previous[route[-1]])
            return tuple(reversed(route))
        for neighbor in grid["adjacency"][node]:
            candidate = cost + 1
            if candidate < costs.get(neighbor, float("inf")):
                costs[neighbor], previous[neighbor] = candidate, node
                heappush(queue, (candidate+heuristic(neighbor), rng.random(), candidate, neighbor))
    raise ValueError("No grid route exists.")


def dependency_order(requires):
    """i -> j means j first. Return execution order, or None on a cycle."""
    remaining = {name: len(deps) for name, deps in requires.items()}
    followers = {name: [] for name in requires}
    for name, deps in requires.items():
        for dependency in deps:
            followers[dependency].append(name)
    ready = deque(name for name, degree in remaining.items() if not degree)
    order = []
    while ready:
        name = ready.popleft()
        order.append(name)
        for follower in followers[name]:
            remaining[follower] -= 1
            if not remaining[follower]:
                ready.append(follower)
    return tuple(order) if len(order) == len(requires) else None


def common_path_is_unit(a, b):
    vertices = set(a) & set(b)
    edges = ({frozenset(e) for e in pairwise(a)} &
             {frozenset(e) for e in pairwise(b)})
    return not vertices or len(edges) == len(vertices)-1


def generate(size=10, robot_count=10, seed=20261119, max_attempts=1000):
    grid, rng = make_grid(size), Random(seed)
    last = size-1
    starts = [p for p in grid["control_points"]
              if (p[0] in (0,last)) != (p[1] in (0,last))]
    goals = [p for p in grid["control_points"] if 0<p[0]<last and 0<p[1]<last]
    if not 1 <= robot_count <= min(len(starts),len(goals)):
        raise ValueError("Robot count exceeds distinct boundary-start/interior-goal capacity.")
    routes, metric, sweeps, requires, log = {}, {}, {}, {}, []
    used_starts, used_goals = set(), set()
    for index in range(robot_count):
        name = f"R{index+1}"
        for attempt in range(1,max_attempts+1):
            start = rng.choice([p for p in starts if p not in used_starts])
            goal = rng.choice([p for p in goals if p not in used_goals])
            route = astar(grid,start,goal,rng)
            entry = dict(robot=name, attempt=attempt, start=start, goal=goal,
                         route=route, pair_checks=0, pairs=[])
            if len(route)-1 < min(4,size-1):
                entry["decision"]="too_short"; log.append(entry); continue
            path = Path(name,tuple(grid["control_points"][p] for p in route))
            sweep = sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(route)-1)))
            graph = {k:set(v) for k,v in requires.items()}
            graph[name] = set()
            rejected = None
            intersects = False
            for other, old in metric.items():
                entry["pair_checks"] += 1
                flags = (
                    sweep_2d(path.point(0)).intersects(sweeps[other],1e-9),
                    sweep_2d(path.point(len(route)-1)).intersects(sweeps[other],1e-9),
                    sweep_2d(old.point(0)).intersects(sweep,1e-9),
                    sweep_2d(old.point(len(old.vertices)-1)).intersects(sweep,1e-9))
                si,ei,sj,ej = flags
                unit = common_path_is_unit(route,routes[other])
                entry["pairs"].append(dict(other=other, flags=list(map(int,flags)), unit=unit))
                intersects |= sweep.intersects(sweeps[other],1e-9)
                if not ((not sj and not ei) or (not si and not ej)):
                    rejected = "unit_pair_infeasible" if unit else "nonunit_outside_serial_condition"
                    break
                if sj or ei:
                    graph[name].add(other)
                if si or ej:
                    graph[other].add(name)
            if rejected:
                entry["decision"]=rejected; log.append(entry); continue
            if routes and not intersects:
                entry["decision"]="no_interference_demo_filter"; log.append(entry); continue
            order = dependency_order(graph)
            if order is None:
                entry["decision"]="cyclic_requirement_not_adopted";log.append(entry);continue
            routes[name],metric[name],sweeps[name] = route,path,sweep
            requires = graph
            used_starts.add(start);used_goals.add(goal)
            entry.update(decision="accepted", execution_order=order)
            log.append(entry)
            break
        else:
            raise RuntimeError(f"Candidate budget exhausted at {name}; no infeasibility claim.")
    return dict(grid=grid,routes=routes,requires=requires,log=log,seed=seed,
                decisions=dict(Counter(e["decision"] for e in log)),
                pair_checks=sum(e["pair_checks"] for e in log))


def dag_witness(solver, requires):
    started = perf_counter()
    if dependency_order(requires) is None:
        raise ValueError("Cyclic requirements need a different planning method.")
    names=solver.names
    full = {name:sweep_2d(Subpath(path,PathPosition(0),PathPosition(len(path.vertices)-1)),shape,solver.policy)
            for name,path,shape in zip(names,solver.paths,solver.footprints)}
    pending,done=set(names),set()
    counts=[0]*len(names); states=[tuple(counts)];rows=[];batches=[]
    while pending:
        ready=[name for name in names if name in pending and set(requires[name])<=done]
        batch=[]
        for name in ready:
            if all(not full[name].intersects(full[other],solver.policy.tolerance) for other in batch):
                batch.append(name)
        if not batch:
            raise ValueError("No ready batch in an asserted DAG.")
        batches.append(tuple(batch))
        for _ in range(max(len(solver.words[names.index(name)]) for name in batch)):
            row=[]
            for i,name in enumerate(names):
                if name in batch and counts[i]<len(solver.words[i]):
                    row.append(solver.words[i][counts[i]]);counts[i]+=1
                else:
                    position = solver.words[i][counts[i]-1].end if counts[i] else PathPosition(0)
                    row.append(Subpath(solver.paths[i],position,position))
            rows.append(tuple(row));states.append(tuple(counts))
        pending.difference_update(batch);done.update(batch)
    result = SearchResult("feasible",TuplePath(names,tuple(rows)),tuple(states),len(states),len(rows),
                          perf_counter()-started,"Automatically constructed from acyclic requirements.")
    solver.validate(result)
    return result


def create_experiment_model(size=10, robot_count=10, seed=20261119, **options):
    data=generate(size,robot_count,seed)
    routes=data["routes"]
    headings={name:degrees(atan2(route[1][1]-route[0][1],route[1][0]-route[0][0]))
              for name,route in routes.items()}
    options.setdefault("initial_headings",headings)
    model,robots,sensors=create_controlled_model(
        routes,name=f"IncrementalDAG_{robot_count}",grid=data["grid"],
        plan_builder=lambda solver:dag_witness(solver,data["requires"]),**options)
    model.routing_experiment=data
    return model,robots,sensors


def run_example(robot_count, *, merge_sweeps=False, event_release=False):
    from importlib import import_module
    data=generate(robot_count=robot_count)
    print(f"Incremental A*: {robot_count} routes accepted; decisions={data['decisions']}; "
          f"pair checks={data['pair_checks']}")
    headings={name:degrees(atan2(route[1][1]-route[0][1],route[1][0]-route[0][0]))
              for name,route in data["routes"].items()}
    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name=f"IncrementalDAG_{robot_count}{'_Merged' if merge_sweeps else ''}{'_EventRelease' if event_release else ''}",no_rack=True,grid=data["grid"],
        robot_paths=data["routes"],initial_headings=headings,
        model_factory=lambda **opts:create_experiment_model(robot_count=robot_count,merge_sweeps=merge_sweeps,event_release=event_release,**opts),
        collision_check=True,stop_on_collision=True)
