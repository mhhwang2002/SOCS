"""Initial-snapshot priority lookahead on fixed paths, not a global optimizer.

No new DEVS model or production-controller policy is introduced. Both serial
reference orders are independently validated, then executed by the unchanged
TrafficManager with merging and event release. The same engine/Plant snapshot
is restored before installing each alternative reference plan at t=0, before
any Plant command is emitted. Mid-motion plan replacement is NOT supported.
"""

import argparse
import json
from math import atan2, degrees, dist, inf, isfinite
from pathlib import Path
from time import perf_counter

from socs.devs.TESEngin import TESEnginBase
from ..controlled import create_controlled_model
from ..TrafficManager import TrafficManager
from ..sweep_operations import compact_tuple
from .incremental_routing import dag_witness


CASES = {
    'long_horizontal': {
        'R1': ((0, 1), (1, 1), (2, 1), (3, 1)),
        'R2': ((1, 0), (1, 1), (1, 2)),
    },
    'long_vertical': {
        'R1': ((0, 1), (1, 1), (2, 1)),
        'R2': ((1, 0), (1, 1), (1, 2), (1, 3)),
    },
}


def ordered_witness(solver, order):
    if len(order) != len(solver.names) or set(order) != set(solver.names):
        raise ValueError('Order must contain every object exactly once.')
    requires = {name: set(order[:index]) for index, name in enumerate(order)}
    return dag_witness(solver, requires)


def finish(engine, model):
    steps = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        steps += 1
        if steps > 200000:
            return dict(completed=False, reason='event_budget', makespan=None, steps=steps)
    manager = model.controller.traffic_manager
    complete = (manager.done and not manager.active_tasks and not manager.awaiting
                and not manager.command_buffer and not model.collision_history
                and all(record['released_progress'].segment == len(model.controller.roadmap.paths[name].vertices)-1
                        for name, record in model.controller.roadmap.robots.items()))
    return dict(completed=complete, reason=None if complete else 'incomplete_or_collision',
                makespan=engine.tNow if complete else None, steps=steps,
                commands=manager.next_command_id, contacts=len(model.collision_history))


def experiment(case):
    routes = CASES[case]
    names = tuple(routes)
    headings = {name: degrees(atan2(route[1][1]-route[0][1], route[1][0]-route[0][0]))
                for name, route in routes.items()}
    model, robots, sensors = create_controlled_model(
        routes, initial_headings=headings, merge_sweeps=True, event_release=True,
        collision_check=False, plan_builder=lambda solver: ordered_witness(solver, names))
    engine = TESEnginBase(model, TimeIntervalContinuous=inf, TimeIntervalAnimation=inf)
    engine.bVerbose = False
    controller = model.controller
    original_manager = controller.traffic_manager
    snapshot = engine.capture_snapshot()
    # Use the same configured geometry as the actual controller.
    from ..wait_duplication import WaitDuplicationSolver
    solver = WaitDuplicationSolver(controller.roadmap.paths, original_manager.footprints)
    distance = {name: sum(dist(a, b) for a, b in zip(path.vertices, path.vertices[1:]))
                for name, path in controller.roadmap.paths.items()}
    longest = tuple(sorted(names, key=lambda name: (-distance[name], name)))
    # Arrivals are simultaneous in these fixtures; FIFO breaks ties by name.
    policies = {'id_fifo_tie': names, 'longest_remaining_first': longest,
                'opposite_order': tuple(reversed(names))}
    orders = tuple(dict.fromkeys(policies.values()))
    branches = []
    for order in orders:
        started = perf_counter()
        controller.traffic_manager = original_manager
        engine.restore_snapshot(snapshot)
        assert engine.tNow == 0 and all(robot.phase == 'Wait' for robot in robots)
        witness = ordered_witness(solver, order)
        _, blocks = compact_tuple(witness.tuple_path, original_manager.footprints, solver.policy)
        controller.traffic_manager = TrafficManager(
            controller.roadmap, witness.tuple_path, event_release=True,
            footprints=original_manager.footprints, command_blocks=blocks)
        controller.phase = 'Command' if controller.traffic_manager.command_buffer else 'Wait'
        # Clock structure is unchanged: each branch starts with Command at t=0.
        assert controller.tNext == model.tNext == 0
        outcome = finish(engine, model)
        branches.append(dict(order=list(order), policies=[k for k,v in policies.items() if v == order],
                             wall_seconds=perf_counter()-started, **outcome))
    feasible = [row for row in branches if row['completed']]
    if not feasible:
        return dict(case=case, branches=branches, selected=None)
    best = min(feasible, key=lambda row: (row['makespan'], row['order']))
    # Fresh Plant replay: 0.01 s validation, stop BOTH physical models on collision.
    checked, check_robots, check_sensors = create_controlled_model(
        routes, initial_headings=headings, merge_sweeps=True, event_release=True,
        collision_check=True, stop_on_collision=True,
        plan_builder=lambda solver: ordered_witness(solver, tuple(best['order'])))
    validation_engine = TESEnginBase(checked, TimeIntervalContinuous=inf, TimeIntervalAnimation=.01)
    validation_engine.bVerbose = False
    validation = finish(validation_engine, checked)
    assert validation['completed'] and validation['contacts'] == 0
    assert abs(validation['makespan']-best['makespan']) < 1e-9
    assert all(obj.stop_on_collision for obj in (*check_robots, *check_sensors))
    return dict(case=case, scope='best_of_evaluated_initial_reference_orders',
                routes=routes, initial_headings=headings, distance_m=distance,
                snapshot_time=snapshot.t_now, same_plant_ids=True,
                merge_sweeps=True, event_release=True, evaluation_animation_interval='infinity',
                evaluation_continuous_interval='infinity', evaluation_collision_check=False,
                branches=branches, selected=best['order'], selected_makespan=best['makespan'],
                validation_interval=.01, stop_on_collision=True, validation=validation)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=(*CASES, 'all'), default='all')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'results'/'priority_lookahead.json')
    args = parser.parse_args()
    rows = [experiment(case) for case in (CASES if args.case == 'all' else (args.case,))]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(rows, indent=2), encoding='utf-8')
    for row in rows:
        print(row['case'], [(r['order'], r['makespan']) for r in row['branches']],
              'selected=', row['selected'], flush=True)
    print(args.output)


if __name__ == '__main__':
    main()
